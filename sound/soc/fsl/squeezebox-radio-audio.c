// SPDX-License-Identifier: GPL-2.0
/* Logitech Squeezebox Radio (i.MX25) ASoC machine driver. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <sound/control.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/tlv320aic3x.h"


#include "imx-audmux.h"
#include "imx-ssi.h"

#define SQUEEZEBOX_MCLK_HZ 12000000

/*
 * Jack detection timing and codec bits, all from the legacy 2.6.26 drivers:
 * the headset detector was polled every quarter second with the shortest
 * sense and button debounce settings (codecs/tlv320aic3104.c:62,67,71,
 * 1463-1466), and the line-in GPIO was sampled half a second after an edge
 * (imx/baby-tlv320aic3104.c:51,434).
 */
#define SQUEEZEBOX_HP_POLL_INTERVAL	(HZ / 4)
#define SQUEEZEBOX_HP_SENSE_DEBOUNCE	0
#define SQUEEZEBOX_HP_PRESS_DEBOUNCE	0
#define SQUEEZEBOX_HP_DETECT_CTRL_A	(AIC3X_HEADSET_DETECT_ENABLED | \
					 (SQUEEZEBOX_HP_SENSE_DEBOUNCE << 2) | \
					 (SQUEEZEBOX_HP_PRESS_DEBOUNCE << 0))
/* Register 14 status bits (codecs/tlv320aic3104.c:1475-1476). */
#define SQUEEZEBOX_HP_HEADSET_DETECTED	BIT(4)
#define SQUEEZEBOX_HP_BUTTON_PRESSED	BIT(5)
#define SQUEEZEBOX_LINEIN_DEBOUNCE	(HZ / 2)

struct squeezebox_audio {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;
	struct gpio_desc *pa_shutdown;
	struct gpio_desc *pa_mute;
	struct gpio_desc *linein_detect;
	struct clk *mclk;
	struct snd_soc_component *codec_component;
	unsigned int endpoint;

	/* Jack detection, running between card late_probe and link exit. */
	struct snd_kcontrol *hp_switch_ctl;
	struct snd_kcontrol *hp_button_ctl;
	struct snd_kcontrol *linein_switch_ctl;
	struct delayed_work hp_work;
	struct delayed_work linein_work;
	int linein_irq;
	bool detect_running;
	bool hp_detect_armed;
	int hp_detected;
	int hp_button;
	int linein_present;
	bool linein_test;
};

static const char * const squeezebox_endpoints[] = {
	"Off",
	"Speaker",
	"Headphone",
	"FactoryTest",
};

static const struct soc_enum squeezebox_endpoint_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(squeezebox_endpoints),
			    squeezebox_endpoints);

static int squeezebox_endpoint_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = audio->endpoint;
	return 0;
}

static int squeezebox_endpoint_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);
	unsigned int endpoint = ucontrol->value.integer.value[0];
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);

	if (endpoint >= ARRAY_SIZE(squeezebox_endpoints))
		return -EINVAL;
	if (audio->endpoint == endpoint)
		return 0;

	audio->endpoint = endpoint;
	if (endpoint == 1 || endpoint == 3)
		snd_soc_dapm_enable_pin(dapm, "Speaker");
	else
		snd_soc_dapm_disable_pin(dapm, "Speaker");
	if (endpoint == 2 || endpoint == 3)
		snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	else
		snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_sync(dapm);

	return 1;
}

/*
 * Legacy "Headphone Switch" reported the headset type, hard-wired to 1 while
 * type detection was disabled (codecs/tlv320aic3104.c:400-407,1510).
 */
static int squeezebox_headphone_switch_get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = READ_ONCE(audio->hp_detected);

	return 0;
}

static int squeezebox_headphone_button_get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = READ_ONCE(audio->hp_button);

	return 0;
}

/* Legacy reported the debounced, inverted GPIO (imx/baby-tlv320aic3104.c:226). */
static int squeezebox_linein_switch_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = READ_ONCE(audio->linein_present);

	return 0;
}

/*
 * "Line In Test" routed the line-in PGAs straight to the headphone and line
 * outputs at 0 dB (codecs/tlv320aic3104.c:368-397: registers 46, 63, 81 and
 * 91 set to 0x80 when on and 0x00 when off).  Those bits are DAPM mixer
 * switches and bypass volumes in the mainline codec driver, so drive the
 * codec controls rather than the registers to keep DAPM in step.
 */
static const char * const squeezebox_linein_test_volumes[] = {
	"HP PGA Bypass Volume",		/* registers 46 and 63, bits 6:0 */
	"Line PGA Bypass Volume",	/* registers 81 and 91, bits 6:0 */
};

static const char * const squeezebox_linein_test_switches[] = {
	"Left HP Mixer PGAL Bypass Switch",	/* register 46, bit 7 */
	"Right HP Mixer PGAR Bypass Switch",	/* register 63, bit 7 */
	"Left Line Mixer PGAL Bypass Switch",	/* register 81, bit 7 */
	"Right Line Mixer PGAR Bypass Switch",	/* register 91, bit 7 */
};

struct squeezebox_ctl_buf {
	struct snd_ctl_elem_info info;
	struct snd_ctl_elem_value value;
};

/*
 * Set a codec control to 0 or to its maximum.  The bypass volumes are
 * inverted, so their maximum is register value 0, i.e. 0 dB.
 */
static int squeezebox_codec_ctl_put(struct snd_soc_card *card,
				    const char *name, bool to_max,
				    struct squeezebox_ctl_buf *buf)
{
	struct snd_kcontrol *kctl = snd_soc_card_get_kcontrol(card, name);
	long value = 0;
	int ret;

	if (!kctl)
		return -ENOENT;

	if (to_max) {
		memset(&buf->info, 0, sizeof(buf->info));
		buf->info.id = kctl->id;
		ret = kctl->info(kctl, &buf->info);
		if (ret)
			return ret;
		value = buf->info.value.integer.max;
	}

	memset(&buf->value, 0, sizeof(buf->value));
	buf->value.id = kctl->id;
	buf->value.value.integer.value[0] = value;
	buf->value.value.integer.value[1] = value;
	ret = kctl->put(kctl, &buf->value);
	if (ret < 0)
		return ret;
	if (ret > 0)
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &kctl->id);

	return 0;
}

static int squeezebox_linein_test_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = audio->linein_test;

	return 0;
}

static int squeezebox_linein_test_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);
	bool on = ucontrol->value.integer.value[0];
	struct squeezebox_ctl_buf *buf;
	int i, ret = 0;

	if (audio->linein_test == on)
		return 0;

	buf = kzalloc_obj(*buf);
	if (!buf)
		return -ENOMEM;

	/* Legacy wrote 0 dB in both states, with the route bit on or off. */
	for (i = 0; i < ARRAY_SIZE(squeezebox_linein_test_volumes) && !ret; i++)
		ret = squeezebox_codec_ctl_put(card,
					       squeezebox_linein_test_volumes[i],
					       true, buf);
	for (i = 0; i < ARRAY_SIZE(squeezebox_linein_test_switches) && !ret; i++)
		ret = squeezebox_codec_ctl_put(card,
					       squeezebox_linein_test_switches[i],
					       on, buf);
	kfree(buf);
	if (ret)
		return ret;

	audio->linein_test = on;

	return 1;
}

/*
 * Port of the legacy codec timer (codecs/tlv320aic3104.c:1447-1521).  When
 * not armed, the detector is switched off and on again to clear its sticky
 * state; it is then polled until the headset state changes, which re-arms it.
 * The button state is reported on every armed poll.
 */
static void squeezebox_hp_work(struct work_struct *work)
{
	struct squeezebox_audio *audio =
		container_of(to_delayed_work(work), struct squeezebox_audio,
			     hp_work);
	struct snd_soc_component *component = audio->codec_component;
	struct snd_card *snd_card = audio->card.snd_card;
	unsigned int val;
	int detected, button;

	/*
	 * At BIAS_OFF the codec has been soft-reset and its regmap is
	 * cache-only; register 13 is restored by regcache_sync on power-up.
	 */
	if (snd_soc_dapm_get_bias_level(snd_soc_component_to_dapm(component)) ==
	    SND_SOC_BIAS_OFF) {
		audio->hp_detect_armed = false;
		goto out;
	}

	if (!audio->hp_detect_armed) {
		if (snd_soc_component_write(component,
					    AIC3X_HEADSET_DETECT_CTRL_A, 0) ||
		    snd_soc_component_write(component,
					    AIC3X_HEADSET_DETECT_CTRL_A,
					    SQUEEZEBOX_HP_DETECT_CTRL_A))
			goto out;
		audio->hp_detect_armed = true;
		goto out;
	}

	/*
	 * The aic3x regmap caches register 14 (only the reset register is
	 * volatile), so a normal read returns the last written value.  Read
	 * the status bits from the hardware.
	 */
	if (regmap_read_bypassed(component->regmap,
				 AIC3X_HEADSET_DETECT_CTRL_B, &val))
		goto out;

	button = !!(val & SQUEEZEBOX_HP_BUTTON_PRESSED);
	if (button != audio->hp_button) {
		WRITE_ONCE(audio->hp_button, button);
		snd_ctl_notify(snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &audio->hp_button_ctl->id);
	}

	detected = !!(val & SQUEEZEBOX_HP_HEADSET_DETECTED);
	if (detected != audio->hp_detected) {
		WRITE_ONCE(audio->hp_detected, detected);
		snd_ctl_notify(snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &audio->hp_switch_ctl->id);
		audio->hp_detect_armed = false;
	}

out:
	queue_delayed_work(system_freezable_wq, &audio->hp_work,
			   SQUEEZEBOX_HP_POLL_INTERVAL);
}

static void squeezebox_linein_work(struct work_struct *work)
{
	struct squeezebox_audio *audio =
		container_of(to_delayed_work(work), struct squeezebox_audio,
			     linein_work);
	int value;

	value = gpiod_get_value_cansleep(audio->linein_detect);
	if (value < 0 || value == audio->linein_present)
		return;

	WRITE_ONCE(audio->linein_present, value);
	snd_ctl_notify(audio->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &audio->linein_switch_ctl->id);
}

/*
 * Legacy masked the interrupt on the first edge and sampled the GPIO once,
 * LINEIN_DEBOUNCE later (imx/baby-tlv320aic3104.c:400-436).  Queueing only
 * when not already pending gives the same single sample per burst.
 */
static irqreturn_t squeezebox_linein_irq(int irq, void *data)
{
	struct squeezebox_audio *audio = data;

	queue_delayed_work(system_freezable_wq, &audio->linein_work,
			   SQUEEZEBOX_LINEIN_DEBOUNCE);

	return IRQ_HANDLED;
}

static const struct snd_kcontrol_new squeezebox_controls[] = {
	SOC_ENUM_EXT("Endpoint", squeezebox_endpoint_enum,
		     squeezebox_endpoint_get, squeezebox_endpoint_put),
	SOC_SINGLE_EXT("Headphone Switch", 0, 0, 0, 0,
		       squeezebox_headphone_switch_get, NULL),
	SOC_SINGLE_EXT("Headphone Button", 0, 0, 0, 0,
		       squeezebox_headphone_button_get, NULL),
	SOC_SINGLE_BOOL_EXT("Line In Switch", 0,
			    squeezebox_linein_switch_get, NULL),
	SOC_SINGLE_BOOL_EXT("Line In Test", 0,
			    squeezebox_linein_test_get,
			    squeezebox_linein_test_put),
};


static int squeezebox_pa_event(struct snd_soc_dapm_widget *widget,
			       struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(widget->dapm);
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		gpiod_set_value_cansleep(audio->pa_shutdown, 1);
		usleep_range(2000, 2500);
		gpiod_set_value_cansleep(audio->pa_mute, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		gpiod_set_value_cansleep(audio->pa_mute, 1);
		usleep_range(2000, 2500);
		gpiod_set_value_cansleep(audio->pa_shutdown, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dapm_widget squeezebox_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", squeezebox_pa_event),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static const struct snd_soc_dapm_route squeezebox_routes[] = {
	{ "Speaker", NULL, "LLOUT" },
	{ "Speaker", NULL, "RLOUT" },
	{ "Headphone Jack", NULL, "HPLOUT" },
	{ "Headphone Jack", NULL, "HPROUT" },
	{ "LINE1L", NULL, "Line In Jack" },
	{ "LINE1R", NULL, "Line In Jack" },
};

static int squeezebox_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	audio->codec_component = component;
	/*
	 * Legacy left the codec "off, with power on" (D3hot only stopped the
	 * PLL, codecs/tlv320aic3104.c:1246-1257,1375-1376) so the headset
	 * detector ran while idle.  At BIAS_OFF aic3x soft-resets the codec,
	 * clearing the detector, and makes its regmap cache-only, so keep it
	 * at STANDBY when idle.
	 */
	snd_soc_dapm_set_idle_bias(snd_soc_component_to_dapm(component), true);
	/*
	 * Legacy polling used the codec's own headset detector: 2.5 V MICBIAS,
	 * detector B enabled, then status bit 4 read from detector B.
	 */
	ret = snd_soc_component_write(component, MICBIAS_CTRL,
				      AIC3X_MICBIAS_2_5V << MICBIAS_LEVEL_SHIFT);
	if (ret)
		return ret;
	ret = snd_soc_component_write(component, AIC3X_HEADSET_DETECT_CTRL_B,
				      AIC3X_HEADSET_DETECT_ENABLED);
	if (ret)
		return ret;
	ret = snd_soc_component_write(component, AIC3X_HEADSET_DETECT_CTRL_A,
				      AIC3X_HEADSET_DETECT_ENABLED);
	if (ret)
		return ret;

	/*
	 * The legacy default was Speaker; leave headphone unpowered until
	 * userspace selects the Headphone or FactoryTest endpoint.
	 */
	snd_soc_dapm_disable_pin(snd_soc_card_to_dapm(rtd->card), "Headphone Jack");
	snd_soc_dapm_sync(snd_soc_card_to_dapm(rtd->card));

	ret = snd_soc_component_write(component, DACL1_2_LLOPM_VOL, ROUTE_ON);
	if (ret)
		return ret;
	ret = snd_soc_component_write(component, DACR1_2_RLOPM_VOL, ROUTE_ON);
	if (ret)
		return ret;

	/*
	 * Legacy DEFAULT_VOL was 0 dB for every codec volume
	 * (codecs/tlv320aic3104.c:56); aic3x_init() uses -23.5 dB, and +16 dB
	 * for the ADC PGA (DEFAULT_GAIN).  Stock firmware runs with PCM and
	 * HP DAC at 0 dB (127 in amixer) and the PGA at 0 dB, so the same
	 * SqueezePlay volume was 23.5 dB quieter on mainline and line-in
	 * (capture loopback) 16 dB hotter.  Bits 6:0 are the level; bit 7
	 * (mute, output-mixer route) stays with aic3x_mute() and DAPM.
	 */
	ret = snd_soc_component_update_bits(component, LDAC_VOL, 0x7f, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component, RDAC_VOL, 0x7f, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component, DACL1_2_HPLOUT_VOL, 0x7f, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component, DACR1_2_HPROUT_VOL, 0x7f, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component, LADC_VOL, 0x7f, 0);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(component, RADC_VOL, 0x7f, 0);

	return ret < 0 ? ret : 0;
}

static void squeezebox_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(rtd->card);

	if (!audio->detect_running)
		return;

	/* Stop the IRQ first so it cannot requeue the line-in work. */
	audio->detect_running = false;
	disable_irq(audio->linein_irq);
	cancel_delayed_work_sync(&audio->linein_work);
	cancel_delayed_work_sync(&audio->hp_work);
}

/*
 * Start jack detection once the card controls exist.  It is stopped from the
 * link exit callback, which ASoC runs before the codec component and the
 * sound card are torn down, including when the codec unbinds first.
 */
static int squeezebox_late_probe(struct snd_soc_card *card)
{
	struct squeezebox_audio *audio = snd_soc_card_get_drvdata(card);
	int value;

	audio->hp_switch_ctl = snd_soc_card_get_kcontrol(card, "Headphone Switch");
	audio->hp_button_ctl = snd_soc_card_get_kcontrol(card, "Headphone Button");
	audio->linein_switch_ctl = snd_soc_card_get_kcontrol(card, "Line In Switch");
	if (!audio->hp_switch_ctl || !audio->hp_button_ctl ||
	    !audio->linein_switch_ctl)
		return -ENOENT;

	/* Legacy read the initial line-in level at probe (imx/baby-tlv320aic3104.c:568). */
	value = gpiod_get_value_cansleep(audio->linein_detect);
	if (value < 0)
		return value;

	audio->linein_present = value;
	audio->hp_detected = 0;
	audio->hp_button = 0;
	audio->hp_detect_armed = false;
	audio->linein_test = false;
	audio->detect_running = true;
	enable_irq(audio->linein_irq);
	/* The first legacy poll ran one interval after probe (codecs/tlv320aic3104.c:1386). */
	queue_delayed_work(system_freezable_wq, &audio->hp_work,
			   SQUEEZEBOX_HP_POLL_INTERVAL);

	return 0;
}

static int squeezebox_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	/*
	 * The legacy codec ignored clk_id.  Mainline uses it to select the
	 * physical source: 0 is MCLK, while 1 would incorrectly select GPIO2.
	 */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, SQUEEZEBOX_MCLK_HZ,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	/*
	 * Legacy forced the AIC3104 serial word length to 32 bits for every
	 * PCM format, and its SSI slot API left 32-bit slots in use.  Keep that
	 * wire format while accepting 16-bit PCM samples.  The codec frame
	 * always has two adjacent slots; for mono playback the codec copies
	 * the first slot to both DACs.
	 */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x3, 0x3, 2, 32);
	if (ret)
		return ret;

	/*
	 * The SSI driver accepts 16-bit sample slots only. The codec keeps its
	 * 32-bit serial framing internally, as in the legacy setup.  For mono
	 * fsl_ssi switches to normal mode and sends the first slot only, like
	 * the legacy 0xfffffffe slot mask.
	 */
	return snd_soc_dai_set_tdm_slot(cpu_dai,
		params_channels(params) == 1 ? 0x1 : 0x3,
		params_channels(params) == 1 ? 0x1 : 0x3, 2, 16);
}

static const struct snd_soc_ops squeezebox_ops = {
	.hw_params = squeezebox_hw_params,
};

static void squeezebox_disable_mclk(void *data)
{
	clk_disable_unprepare(data);
}

static int squeezebox_audio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct squeezebox_audio *audio;
	struct device_node *cpu_np, *codec_np;
	u32 int_port, ext_port;
	int ret;

	audio = devm_kzalloc(dev, sizeof(*audio), GFP_KERNEL);
	if (!audio)
		return -ENOMEM;

	audio->pa_shutdown = devm_gpiod_get(dev, "pa-shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(audio->pa_shutdown))
		return dev_err_probe(dev, PTR_ERR(audio->pa_shutdown), "PA shutdown GPIO\n");
	audio->pa_mute = devm_gpiod_get(dev, "pa-mute", GPIOD_OUT_HIGH);
	if (IS_ERR(audio->pa_mute))
		return dev_err_probe(dev, PTR_ERR(audio->pa_mute), "PA mute GPIO\n");
	audio->linein_detect = devm_gpiod_get(dev, "linein-detect", GPIOD_IN);
	if (IS_ERR(audio->linein_detect))
		return dev_err_probe(dev, PTR_ERR(audio->linein_detect),
				     "line-in detect GPIO\n");

	/*
	 * Legacy armed the opposite edge after every sample, i.e. it reacted
	 * to both edges (imx/baby-tlv320aic3104.c:409-416,569-575).  The IRQ
	 * stays disabled until late_probe.
	 */
	INIT_DELAYED_WORK(&audio->hp_work, squeezebox_hp_work);
	INIT_DELAYED_WORK(&audio->linein_work, squeezebox_linein_work);
	audio->linein_irq = gpiod_to_irq(audio->linein_detect);
	if (audio->linein_irq < 0)
		return dev_err_probe(dev, audio->linein_irq,
				     "line-in detect IRQ\n");
	ret = devm_request_any_context_irq(dev, audio->linein_irq,
					   squeezebox_linein_irq,
					   IRQF_TRIGGER_RISING |
					   IRQF_TRIGGER_FALLING |
					   IRQF_NO_AUTOEN,
					   "linein", audio);
	if (ret < 0)
		return dev_err_probe(dev, ret, "request line-in IRQ\n");

	audio->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(audio->mclk))
		return dev_err_probe(dev, PTR_ERR(audio->mclk), "MCLK\n");
	if (clk_get_rate(audio->mclk) != SQUEEZEBOX_MCLK_HZ)
		return dev_err_probe(dev, -EINVAL, "MCLK rate is not 12 MHz\n");
	ret = clk_prepare_enable(audio->mclk);
	if (ret)
		return dev_err_probe(dev, ret, "enable MCLK\n");
	ret = devm_add_action_or_reset(dev, squeezebox_disable_mclk, audio->mclk);
	if (ret)
		return ret;

	cpu_np = of_parse_phandle(dev->of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(dev->of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		of_node_put(cpu_np);
		of_node_put(codec_np);
		return dev_err_probe(dev, -EINVAL, "CPU DAI or codec missing\n");
	}
	ret = of_property_read_u32(dev->of_node, "fsl,mux-int-port", &int_port);
	ret |= of_property_read_u32(dev->of_node, "fsl,mux-ext-port", &ext_port);
	if (ret || !int_port || !ext_port) {
		of_node_put(cpu_np);
		of_node_put(codec_np);
		return dev_err_probe(dev, -EINVAL, "AUDMUX ports missing or zero\n");
	}
	int_port--;
	ext_port--;
	imx_audmux_v2_configure_port(int_port,
		IMX_AUDMUX_V2_PTCR_SYN | IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) | IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(ext_port), IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	imx_audmux_v2_configure_port(ext_port, IMX_AUDMUX_V2_PTCR_SYN,
		IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));

	audio->cpu.of_node = cpu_np;
	audio->codec.of_node = codec_np;
	audio->codec.dai_name = "tlv320aic3x-hifi";
	audio->platform.of_node = cpu_np;
	audio->link.name = "TLV320AIC3104";
	audio->link.stream_name = "HiFi";
	audio->link.cpus = &audio->cpu;
	audio->link.num_cpus = 1;
	audio->link.codecs = &audio->codec;
	audio->link.num_codecs = 1;
	audio->link.platforms = &audio->platform;
	audio->link.num_platforms = 1;
	audio->link.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBP_CFP;
	audio->link.ops = &squeezebox_ops;
	audio->link.init = squeezebox_init;
	audio->link.exit = squeezebox_exit;
	audio->card.dev = dev;
	audio->card.name = "baby";
	audio->card.owner = THIS_MODULE;
	audio->card.dai_link = &audio->link;
	audio->card.num_links = 1;
	audio->card.dapm_widgets = squeezebox_widgets;
	audio->card.num_dapm_widgets = ARRAY_SIZE(squeezebox_widgets);
	audio->card.dapm_routes = squeezebox_routes;
	audio->card.num_dapm_routes = ARRAY_SIZE(squeezebox_routes);
	audio->card.controls = squeezebox_controls;
	audio->card.num_controls = ARRAY_SIZE(squeezebox_controls);
	audio->card.late_probe = squeezebox_late_probe;
	audio->endpoint = 1;
	snd_soc_card_set_drvdata(&audio->card, audio);

	return devm_snd_soc_register_card(dev, &audio->card);
}

static const struct of_device_id squeezebox_audio_of_match[] = {
	{ .compatible = "logitech,squeezebox-radio-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, squeezebox_audio_of_match);

static struct platform_driver squeezebox_audio_driver = {
	.driver = { .name = "squeezebox-radio-audio", .of_match_table = squeezebox_audio_of_match },
	.probe = squeezebox_audio_probe,
};
module_platform_driver(squeezebox_audio_driver);

MODULE_DESCRIPTION("Logitech Squeezebox Radio ASoC machine driver");
MODULE_LICENSE("GPL");
