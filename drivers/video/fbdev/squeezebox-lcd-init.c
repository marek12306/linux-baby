// SPDX-License-Identifier: GPL-2.0-only
/*
 * Logitech Squeezebox Radio panel initializer.
 *
 * This preserves the active legacy RT bit-banged 9-bit panel protocol:
 * baby-base.patch plus the reset-timing and white-flash follow-ups.
 *
 * Two roles:
 * - "logitech,squeezebox-radio-panel-init": with imxfb, the sequence is sent
 *   once at probe; the RGB LCDC consumes its own DT mode description.
 * - "logitech,squeezebox-radio-lcd": DRM panel for imx-lcdc. The sequence is
 *   sent in prepare(), i.e. after the LCDC started scanning, and the
 *   backlight is enabled by drm_panel_enable() only afterwards. Legacy mx2fb
 *   did the same (LCDC clock, then panel init, then mxc_lcdc_bl;
 *   fix-lcd-white-flash.patch); lighting the backlight before the panel
 *   shows valid frames gives a white flash.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define SB_LCD_START		0x0100
#define SB_LCD_DELAY_MS	0x8000

struct sb_lcd {
	struct gpio_desc *reset;
	struct gpio_desc *mosi;
	struct gpio_desc *cs;
	struct gpio_desc *clk;
	struct gpio_desc *mode_mosi;
	struct gpio_desc *mode_miso;
	struct gpio_desc *slcdc_cs;
};

/* Active RT sequence after fix-lcd-white-flash.patch. */
static const u16 sb_lcd_init_sequence[] = {
	SB_LCD_START | 0xc0, 0x0a,
	SB_LCD_START | 0xc1, 0x06, 0x06, 0x06,
	SB_LCD_START | 0xc2, 0x55, 0x00,
	SB_LCD_START | 0xc3, 0x55, 0x00,
	SB_LCD_START | 0xc4, 0x55, 0x00,
	SB_LCD_START | 0xc5, 0x27, 0x34,
	SB_LCD_START | 0xc7, 0x40,
	SB_LCD_START | 0xe0, 0x0d, 0x0f, 0x13, 0x0c, 0x15, 0x17, 0x3b,
	0x8c, 0x4d, 0x0b, 0x1e, 0x25, 0x1c, 0x23, 0x37,
	SB_LCD_START | 0xe1, 0x0d, 0x1f, 0x26, 0x0a, 0x11, 0x13, 0x3a,
	0x37, 0x4c, 0x09, 0x1a, 0x23, 0x33, 0x37, 0x37,
	SB_LCD_START | 0x26, 0x10,
	SB_LCD_START | 0xb0, 0x0c,
	SB_LCD_START | 0xb1, 0x6a, 0x15,
	SB_LCD_START | 0xb2, 0x6a, 0x15,
	SB_LCD_START | 0xb3, 0x6a, 0x15,
	SB_LCD_START | 0xb6, 0x01, 0x04,
	SB_LCD_START | 0x29,
	SB_LCD_START | 0x11,
	SB_LCD_DELAY_MS | 200,
};

static void sb_lcd_send_word(struct sb_lcd *lcd, u16 word)
{
	unsigned int bit;

	if (word & SB_LCD_DELAY_MS) {
		msleep(word & ~SB_LCD_DELAY_MS);
		return;
	}

	/*
	 * The legacy stream stores start-of-command in bit 8, then inverts it
	 * before transfer: 0 on the wire is a command, 1 is parameter data.
	 */
	word ^= SB_LCD_START;
	if (!(word & SB_LCD_START)) {
		gpiod_set_value(lcd->cs, 1);
		udelay(10);
		gpiod_set_value(lcd->cs, 0);
		udelay(10);
	}

	udelay(1);
	gpiod_set_value(lcd->clk, 0);
	udelay(1);

	for (bit = 0; bit < 9; bit++) {
		gpiod_set_value(lcd->mosi, !!(word & SB_LCD_START));
		word <<= 1;
		udelay(1);
		gpiod_set_value(lcd->clk, 1);
		udelay(1);
		gpiod_set_value(lcd->clk, 0);
	}

	udelay(1);
}

static int sb_lcd_get_gpios(struct device *dev, struct sb_lcd *lcd)
{
	lcd->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->reset))
		return dev_err_probe(dev, PTR_ERR(lcd->reset), "reset GPIO\n");

	lcd->mosi = devm_gpiod_get(dev, "mosi", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->mosi))
		return dev_err_probe(dev, PTR_ERR(lcd->mosi), "MOSI GPIO\n");

	lcd->cs = devm_gpiod_get(dev, "cs", GPIOD_OUT_HIGH);
	if (IS_ERR(lcd->cs))
		return dev_err_probe(dev, PTR_ERR(lcd->cs), "chip-select GPIO\n");

	lcd->clk = devm_gpiod_get(dev, "clk", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->clk))
		return dev_err_probe(dev, PTR_ERR(lcd->clk), "clock GPIO\n");

	lcd->mode_mosi = devm_gpiod_get(dev, "mode-mosi", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->mode_mosi))
		return dev_err_probe(dev, PTR_ERR(lcd->mode_mosi),
				     "LCD mode MOSI GPIO\n");

	lcd->mode_miso = devm_gpiod_get(dev, "mode-miso", GPIOD_OUT_HIGH);
	if (IS_ERR(lcd->mode_miso))
		return dev_err_probe(dev, PTR_ERR(lcd->mode_miso),
				     "LCD mode MISO GPIO\n");

	lcd->slcdc_cs = devm_gpiod_get(dev, "slcdc-cs", GPIOD_OUT_LOW);
	if (IS_ERR(lcd->slcdc_cs))
		return dev_err_probe(dev, PTR_ERR(lcd->slcdc_cs),
				     "SLCDC chip-select GPIO\n");

	if (gpiod_cansleep(lcd->reset) || gpiod_cansleep(lcd->mosi) ||
	    gpiod_cansleep(lcd->cs) || gpiod_cansleep(lcd->clk) ||
	    gpiod_cansleep(lcd->mode_mosi) || gpiod_cansleep(lcd->mode_miso) ||
	    gpiod_cansleep(lcd->slcdc_cs))
		return dev_err_probe(dev, -EINVAL,
				     "9-bit protocol requires non-sleeping GPIOs\n");

	return 0;
}

static void sb_lcd_init_panel(struct sb_lcd *lcd)
{
	unsigned int i;

	/* reset-gpios is active low: deassert, assert, then deassert. */
	gpiod_set_value(lcd->reset, 0);
	udelay(11);
	gpiod_set_value(lcd->reset, 1);
	udelay(11);
	gpiod_set_value(lcd->reset, 0);
	msleep(6);

	/*
	 * The active cyan-lines follow-up selects the RGB LCD mode using
	 * CSPI1 MOSI=0 and MISO=1. On revision 7, base setup also holds the
	 * unused SLCDC chip-select low.
	 */
	gpiod_set_value(lcd->mode_mosi, 0);
	gpiod_set_value(lcd->mode_miso, 1);
	gpiod_set_value(lcd->slcdc_cs, 0);

	for (i = 0; i < ARRAY_SIZE(sb_lcd_init_sequence); i++)
		sb_lcd_send_word(lcd, sb_lcd_init_sequence[i]);

	gpiod_set_value(lcd->cs, 1);
}

static int sb_lcd_init_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sb_lcd lcd;
	int ret;

	ret = sb_lcd_get_gpios(dev, &lcd);
	if (ret)
		return ret;

	sb_lcd_init_panel(&lcd);
	dev_info(dev, "applied legacy panel initialization\n");

	return 0;
}

#if IS_ENABLED(CONFIG_DRM_PANEL)
struct sb_panel {
	struct drm_panel base;
	struct sb_lcd lcd;
	struct drm_display_mode mode;
	u32 bus_flags;
	struct device_node *backlight_np;
};

static struct sb_panel *to_sb_panel(struct drm_panel *panel)
{
	return container_of(panel, struct sb_panel, base);
}

static int sb_panel_prepare(struct drm_panel *panel)
{
	struct sb_panel *p = to_sb_panel(panel);

	/*
	 * The backlight belongs to the LCDC, whose probe needs this panel, so
	 * it is looked up at the first prepare rather than at probe.
	 */
	if (!panel->backlight && p->backlight_np) {
		panel->backlight = of_find_backlight_by_node(p->backlight_np);
		if (!panel->backlight)
			dev_warn(panel->dev, "backlight not found\n");
	}

	sb_lcd_init_panel(&p->lcd);

	return 0;
}

static int sb_panel_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct sb_panel *p = to_sb_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &p->mode);
	if (!mode)
		return -ENOMEM;

	mode->type |= DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	connector->display_info.bus_flags = p->bus_flags;

	return 1;
}

static const struct drm_panel_funcs sb_panel_funcs = {
	.prepare = sb_panel_prepare,
	.get_modes = sb_panel_get_modes,
};

static int sb_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct display_timing timing;
	struct videomode vm;
	struct sb_panel *p;
	int ret;

	p = devm_drm_panel_alloc(dev, struct sb_panel, base, &sb_panel_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(p))
		return PTR_ERR(p);

	/*
	 * Not of_get_drm_panel_display_mode(): it also requires width-mm and
	 * height-mm, and the panel's physical size is unknown.
	 */
	ret = of_get_display_timing(dev->of_node, "panel-timing", &timing);
	if (ret)
		return dev_err_probe(dev, ret, "panel-timing\n");
	videomode_from_timing(&timing, &vm);
	drm_display_mode_from_videomode(&vm, &p->mode);
	drm_bus_flags_from_videomode(&vm, &p->bus_flags);

	ret = sb_lcd_get_gpios(dev, &p->lcd);
	if (ret)
		return ret;

	p->backlight_np = of_parse_phandle(dev->of_node, "backlight", 0);
	platform_set_drvdata(pdev, p);
	drm_panel_add(&p->base);

	return 0;
}

static void sb_panel_remove(struct platform_device *pdev)
{
	struct sb_panel *p = platform_get_drvdata(pdev);

	drm_panel_remove(&p->base);
	if (p->base.backlight)
		put_device(&p->base.backlight->dev);
	of_node_put(p->backlight_np);
}
#endif

static int sb_lcd_probe(struct platform_device *pdev)
{
#if IS_ENABLED(CONFIG_DRM_PANEL)
	if (of_device_is_compatible(pdev->dev.of_node,
				    "logitech,squeezebox-radio-lcd"))
		return sb_panel_probe(pdev);
#endif
	return sb_lcd_init_probe(pdev);
}

static void sb_lcd_remove(struct platform_device *pdev)
{
#if IS_ENABLED(CONFIG_DRM_PANEL)
	if (platform_get_drvdata(pdev))
		sb_panel_remove(pdev);
#endif
}

static const struct of_device_id sb_lcd_of_match[] = {
	{ .compatible = "logitech,squeezebox-radio-panel-init" },
#if IS_ENABLED(CONFIG_DRM_PANEL)
	{ .compatible = "logitech,squeezebox-radio-lcd" },
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, sb_lcd_of_match);

static struct platform_driver sb_lcd_driver = {
	.probe = sb_lcd_probe,
	.remove = sb_lcd_remove,
	.driver = {
		.name = "squeezebox-lcd-init",
		.of_match_table = sb_lcd_of_match,
	},
};
module_platform_driver(sb_lcd_driver);

MODULE_DESCRIPTION("Logitech Squeezebox Radio legacy LCD panel initializer");
MODULE_LICENSE("GPL");
