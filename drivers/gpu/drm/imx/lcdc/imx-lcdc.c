// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: 2020 Marian Cichy <M.Cichy@pengutronix.de>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>
#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/spinlock.h>

#define IMX21LCDC_LSSAR         0x0000 /* LCDC Screen Start Address Register */
#define IMX21LCDC_LSR           0x0004 /* LCDC Size Register */
#define IMX21LCDC_LVPWR         0x0008 /* LCDC Virtual Page Width Register */
#define IMX21LCDC_LCPR          0x000C /* LCDC Cursor Position Register */
#define IMX21LCDC_LCWHB         0x0010 /* LCDC Cursor Width Height and Blink Register*/
#define IMX21LCDC_LCCMR         0x0014 /* LCDC Color Cursor Mapping Register */
#define IMX21LCDC_LPCR          0x0018 /* LCDC Panel Configuration Register */
#define IMX21LCDC_LHCR          0x001C /* LCDC Horizontal Configuration Register */
#define IMX21LCDC_LVCR          0x0020 /* LCDC Vertical Configuration Register */
#define IMX21LCDC_LPOR          0x0024 /* LCDC Panning Offset Register */
#define IMX21LCDC_LSCR          0x0028 /* LCDC Sharp Configuration Register */
#define IMX21LCDC_LPCCR         0x002C /* LCDC PWM Contrast Control Register */
#define IMX21LCDC_LDCR          0x0030 /* LCDC DMA Control Register */
#define IMX21LCDC_LRMCR         0x0034 /* LCDC Refresh Mode Control Register */
#define IMX21LCDC_LICR          0x0038 /* LCDC Interrupt Configuration Register */
#define IMX21LCDC_LIER          0x003C /* LCDC Interrupt Enable Register */
#define IMX21LCDC_LISR          0x0040 /* LCDC Interrupt Status Register */
#define IMX21LCDC_LGWSAR        0x0050 /* LCDC Graphic Window Start Address Register */
#define IMX21LCDC_LGWSR         0x0054 /* LCDC Graph Window Size Register */
#define IMX21LCDC_LGWVPWR       0x0058 /* LCDC Graphic Window Virtual Page Width Register */
#define IMX21LCDC_LGWPOR        0x005C /* LCDC Graphic Window Panning Offset Register */
#define IMX21LCDC_LGWPR         0x0060 /* LCDC Graphic Window Position Register */
#define IMX21LCDC_LGWCR         0x0064 /* LCDC Graphic Window Control Register */
#define IMX21LCDC_LGWDCR        0x0068 /* LCDC Graphic Window DMA Control Register */
#define IMX21LCDC_LAUSCR        0x0080 /* LCDC AUS Mode Control Register */
#define IMX21LCDC_LAUSCCR       0x0084 /* LCDC AUS Mode Cursor Control Register */
#define IMX21LCDC_BGLUT         0x0800 /* Background Lookup Table */
#define IMX21LCDC_GWLUT         0x0C00 /* Graphic Window Lookup Table */

#define IMX21LCDC_LCPR_CC0 BIT(30) /* Cursor Control Bit 0 */
#define IMX21LCDC_LCPR_CC1 BIT(31) /* Cursor Control Bit 1 */

/* Values HSYNC, VSYNC and Framesize Register */
#define IMX21LCDC_LHCR_HWIDTH		GENMASK(31, 26)
#define IMX21LCDC_LHCR_HFPORCH		GENMASK(15, 8)		/* H_WAIT_1 in the i.MX25 Reference manual */
#define IMX21LCDC_LHCR_HBPORCH		GENMASK(7, 0)		/* H_WAIT_2 in the i.MX25 Reference manual */

#define IMX21LCDC_LVCR_VWIDTH		GENMASK(31, 26)
#define IMX21LCDC_LVCR_VFPORCH		GENMASK(15, 8)		/* V_WAIT_1 in the i.MX25 Reference manual */
#define IMX21LCDC_LVCR_VBPORCH		GENMASK(7, 0)		/* V_WAIT_2 in the i.MX25 Reference manual */

#define IMX21LCDC_LSR_XMAX		GENMASK(25, 20)
#define IMX21LCDC_LSR_YMAX		GENMASK(9, 0)

/* Values for LPCR Register */
#define IMX21LCDC_LPCR_PCD		GENMASK(5, 0)
#define IMX21LCDC_LPCR_SHARP		BIT(6)
#define IMX21LCDC_LPCR_SCLKSEL		BIT(7)
#define IMX21LCDC_LPCR_ACD		GENMASK(14, 8)
#define IMX21LCDC_LPCR_ACDSEL		BIT(15)
#define IMX21LCDC_LPCR_REV_VS		BIT(16)
#define IMX21LCDC_LPCR_SWAP_SEL		BIT(17)
#define IMX21LCDC_LPCR_END_SEL		BIT(18)
#define IMX21LCDC_LPCR_SCLKIDLE		BIT(19)
#define IMX21LCDC_LPCR_OEPOL		BIT(20)
#define IMX21LCDC_LPCR_CLKPOL		BIT(21)
#define IMX21LCDC_LPCR_LPPOL		BIT(22)
#define IMX21LCDC_LPCR_FLMPOL		BIT(23)
#define IMX21LCDC_LPCR_PIXPOL		BIT(24)
#define IMX21LCDC_LPCR_BPIX		GENMASK(27, 25)
#define IMX21LCDC_LPCR_PBSIZ		GENMASK(29, 28)
#define IMX21LCDC_LPCR_COLOR		BIT(30)
#define IMX21LCDC_LPCR_TFT		BIT(31)

/* Values for LPCCR Register */
#define IMX21LCDC_LPCCR_PW		GENMASK(7, 0)

#define INTR_EOF BIT(1) /* VBLANK Interrupt Bit */

#define BPP_RGB565	0x05
#define BPP_XRGB8888	0x07

#define LCDC_MIN_XRES 64
#define LCDC_MIN_YRES 64

#define LCDC_MAX_XRES 1024
#define LCDC_MAX_YRES 1024

struct imx_lcdc {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector *connector;
	void __iomem *base;

	struct clk *clk_ipg;
	struct clk *clk_ahb;
	struct clk *clk_per;

	/* fsl,lpccr; 0 if the board does not use the contrast PWM. */
	u32 lpccr;
	/* Protects lpccr and enabled; LPCCR is written only while clocked. */
	spinlock_t lpccr_lock;
	bool enabled;
	struct gpio_desc *bl_low_power;
};

static const u32 imx_lcdc_formats[] = {
	DRM_FORMAT_RGB565, DRM_FORMAT_XRGB8888,
};

static inline struct imx_lcdc *imx_lcdc_from_drmdev(struct drm_device *drm)
{
	return container_of(drm, struct imx_lcdc, drm);
}

static void imx_lcdc_set_enabled(struct imx_lcdc *lcdc, bool enabled)
{
	spin_lock(&lcdc->lpccr_lock);
	lcdc->enabled = enabled;
	if (enabled && lcdc->lpccr)
		writel(lcdc->lpccr, lcdc->base + IMX21LCDC_LPCCR);
	spin_unlock(&lcdc->lpccr_lock);
}

static unsigned int imx_lcdc_get_format(unsigned int drm_format)
{
	switch (drm_format) {
	default:
		DRM_WARN("Format not supported - fallback to XRGB8888\n");
		fallthrough;

	case DRM_FORMAT_XRGB8888:
		return BPP_XRGB8888;

	case DRM_FORMAT_RGB565:
		return BPP_RGB565;
	}
}

/* Size, sync and line-stride registers; the caller keeps the PER clock off. */
static void imx_lcdc_write_mode(struct imx_lcdc *lcdc,
				const struct drm_display_mode *mode,
				const struct drm_framebuffer *fb)
{
	u32 lvcr, lhcr;
	u32 framesize;

	/* Framesize */
	framesize = FIELD_PREP(IMX21LCDC_LSR_XMAX, mode->hdisplay >> 4) |
		FIELD_PREP(IMX21LCDC_LSR_YMAX, mode->vdisplay);
	writel(framesize, lcdc->base + IMX21LCDC_LSR);

	/* HSYNC */
	lhcr = FIELD_PREP(IMX21LCDC_LHCR_HFPORCH, mode->hsync_start - mode->hdisplay - 1) |
		FIELD_PREP(IMX21LCDC_LHCR_HWIDTH, mode->hsync_end - mode->hsync_start - 1) |
		FIELD_PREP(IMX21LCDC_LHCR_HBPORCH, mode->htotal - mode->hsync_end - 3);
	writel(lhcr, lcdc->base + IMX21LCDC_LHCR);

	/* VSYNC */
	lvcr = FIELD_PREP(IMX21LCDC_LVCR_VFPORCH, mode->vsync_start - mode->vdisplay) |
		FIELD_PREP(IMX21LCDC_LVCR_VWIDTH, mode->vsync_end - mode->vsync_start) |
		FIELD_PREP(IMX21LCDC_LVCR_VBPORCH, mode->vtotal - mode->vsync_end);
	writel(lvcr, lcdc->base + IMX21LCDC_LVCR);

	/* Virtual Page Width */
	writel(fb->pitches[0] / 4, lcdc->base + IMX21LCDC_LVPWR);
}

static void imx_lcdc_pipe_enable(struct drm_simple_display_pipe *pipe,
				 struct drm_crtc_state *crtc_state,
				 struct drm_plane_state *plane_state)
{
	int ret;
	int clk_div;
	int bpp;
	struct imx_lcdc *lcdc = imx_lcdc_from_drmdev(pipe->crtc.dev);
	struct drm_display_mode *mode = &pipe->crtc.mode;
	struct drm_display_info *disp_info = &lcdc->connector->display_info;
	const int hsync_pol = (mode->flags & DRM_MODE_FLAG_PHSYNC) ? 0 : 1;
	const int vsync_pol = (mode->flags & DRM_MODE_FLAG_PVSYNC) ? 0 : 1;
	const int data_enable_pol =
		(disp_info->bus_flags & DRM_BUS_FLAG_DE_HIGH) ? 0 : 1;
	const int clk_pol =
		(disp_info->bus_flags & DRM_BUS_FLAG_PIXDATA_DRIVE_POSEDGE) ? 0 : 1;

	clk_div = DIV_ROUND_CLOSEST_ULL(clk_get_rate(lcdc->clk_per),
					mode->clock * 1000);
	bpp = imx_lcdc_get_format(plane_state->fb->format->format);

	writel(FIELD_PREP(IMX21LCDC_LPCR_PCD, clk_div - 1) |
	       FIELD_PREP(IMX21LCDC_LPCR_LPPOL, hsync_pol) |
	       FIELD_PREP(IMX21LCDC_LPCR_FLMPOL, vsync_pol) |
	       FIELD_PREP(IMX21LCDC_LPCR_OEPOL, data_enable_pol) |
	       FIELD_PREP(IMX21LCDC_LPCR_TFT, 1) |
	       FIELD_PREP(IMX21LCDC_LPCR_COLOR, 1) |
	       FIELD_PREP(IMX21LCDC_LPCR_PBSIZ, 3) |
	       FIELD_PREP(IMX21LCDC_LPCR_BPIX, bpp) |
	       FIELD_PREP(IMX21LCDC_LPCR_SCLKSEL, 1) |
	       FIELD_PREP(IMX21LCDC_LPCR_PIXPOL, 0) |
	       FIELD_PREP(IMX21LCDC_LPCR_CLKPOL, clk_pol),
	       lcdc->base + IMX21LCDC_LPCR);

	/* 0px panning offset */
	writel(0x00000000, lcdc->base + IMX21LCDC_LPOR);

	/* disable hardware cursor */
	writel(0, lcdc->base + IMX21LCDC_LCPR);

	/* The LSSAR register specifies the LCD screen start address (SSA). */
	writel(drm_fb_dma_get_gem_addr(plane_state->fb, plane_state, 0),
	       lcdc->base + IMX21LCDC_LSSAR);
	imx_lcdc_write_mode(lcdc, mode, plane_state->fb);

	/*
	 * The controller starts scanning once its clocks run, so program it
	 * completely with all clocks off, then enable ipg, ahb and per, as
	 * imxfb does. On i.MX25, an LCDC stalled by clocks enabled before
	 * programming (no EOF interrupt, panel stays white) did not recover
	 * when the mode was written with ipg and ahb running, but did with
	 * this order.
	 */
	ret = clk_prepare_enable(lcdc->clk_ipg);
	if (ret) {
		dev_err(pipe->crtc.dev->dev, "Cannot enable ipg clock: %pe\n", ERR_PTR(ret));
		return;
	}
	ret = clk_prepare_enable(lcdc->clk_ahb);
	if (ret) {
		dev_err(pipe->crtc.dev->dev, "Cannot enable ahb clock: %pe\n", ERR_PTR(ret));

		clk_disable_unprepare(lcdc->clk_ipg);

		return;
	}
	ret = clk_prepare_enable(lcdc->clk_per);
	if (ret) {
		dev_err(pipe->crtc.dev->dev, "Cannot enable per clock: %pe\n", ERR_PTR(ret));

		clk_disable_unprepare(lcdc->clk_ahb);
		clk_disable_unprepare(lcdc->clk_ipg);

		return;
	}

	imx_lcdc_set_enabled(lcdc, true);

	/* Enable VBLANK Interrupt */
	writel(INTR_EOF, lcdc->base + IMX21LCDC_LIER);
}

static void imx_lcdc_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct imx_lcdc *lcdc = imx_lcdc_from_drmdev(pipe->crtc.dev);
	struct drm_crtc *crtc = &lcdc->pipe.crtc;
	struct drm_pending_vblank_event *event;

	/* Disable VBLANK Interrupt */
	writel(0, lcdc->base + IMX21LCDC_LIER);

	imx_lcdc_set_enabled(lcdc, false);

	clk_disable_unprepare(lcdc->clk_per);
	clk_disable_unprepare(lcdc->clk_ahb);
	clk_disable_unprepare(lcdc->clk_ipg);

	spin_lock_irq(&lcdc->drm.event_lock);
	event = crtc->state->event;
	if (event) {
		crtc->state->event = NULL;
		drm_crtc_send_vblank_event(crtc, event);
	}
	spin_unlock_irq(&lcdc->drm.event_lock);
}

static int imx_lcdc_pipe_check(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *plane_state,
			       struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *mode = &crtc_state->mode;
	const struct drm_display_mode *old_mode = &pipe->crtc.state->mode;

	if (mode->hdisplay < LCDC_MIN_XRES || mode->hdisplay > LCDC_MAX_XRES ||
	    mode->vdisplay < LCDC_MIN_YRES || mode->vdisplay > LCDC_MAX_YRES ||
	    mode->hdisplay % 0x10) { /* must be multiple of 16 */
		drm_err(pipe->crtc.dev, "unsupported display mode (%u x %u)\n",
			mode->hdisplay, mode->vdisplay);
		return -EINVAL;
	}

	crtc_state->mode_changed =
		old_mode->hdisplay != mode->hdisplay ||
		old_mode->vdisplay != mode->vdisplay;

	return 0;
}

static void imx_lcdc_pipe_update(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *old_state)
{
	struct imx_lcdc *lcdc = imx_lcdc_from_drmdev(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event = crtc->state->event;
	struct drm_plane_state *new_state = pipe->plane.state;
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_framebuffer *old_fb = old_state->fb;

	writel(drm_fb_dma_get_gem_addr(fb, new_state, 0),
	       lcdc->base + IMX21LCDC_LSSAR);

	/*
	 * pipe_enable() programs a pipe enabled by this commit. A format or
	 * stride change on a running pipe needs PER off while LPCR and LVPWR
	 * change.
	 */
	if (!drm_atomic_crtc_needs_modeset(crtc->state) && old_fb &&
	    (old_fb->format != fb->format ||
	     old_fb->pitches[0] != fb->pitches[0])) {
		u32 lpcr;

		clk_disable_unprepare(lcdc->clk_per);

		lpcr = readl(lcdc->base + IMX21LCDC_LPCR);
		lpcr &= ~IMX21LCDC_LPCR_BPIX;
		lpcr |= FIELD_PREP(IMX21LCDC_LPCR_BPIX,
				   imx_lcdc_get_format(fb->format->format));
		writel(lpcr, lcdc->base + IMX21LCDC_LPCR);
		writel(fb->pitches[0] / 4, lcdc->base + IMX21LCDC_LVPWR);

		if (clk_prepare_enable(lcdc->clk_per))
			drm_err(crtc->dev, "Cannot enable per clock\n");
	}

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);

		if (crtc->state->active && drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);

		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static const struct drm_simple_display_pipe_funcs imx_lcdc_pipe_funcs = {
	.enable = imx_lcdc_pipe_enable,
	.disable = imx_lcdc_pipe_disable,
	.check = imx_lcdc_pipe_check,
	.update = imx_lcdc_pipe_update,
};

static const struct drm_mode_config_funcs imx_lcdc_mode_config_funcs = {
	/*
	 * The LCDC scans out continuously from memory, so framebuffers need no
	 * dirty handling and fbdev emulation maps the scanout buffer directly.
	 */
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs imx_lcdc_mode_config_helpers = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail_rpm,
};

DEFINE_DRM_GEM_DMA_FOPS(imx_lcdc_drm_fops);

static struct drm_driver imx_lcdc_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &imx_lcdc_drm_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name = "imx-lcdc",
	.desc = "i.MX LCDC driver",
};

static const struct of_device_id imx_lcdc_of_dev_id[] = {
	{
		.compatible = "fsl,imx21-lcdc",
	},
	{
		.compatible = "fsl,imx25-lcdc",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_lcdc_of_dev_id);

#if IS_ENABLED(CONFIG_DRM_IMX_LCDC_BACKLIGHT)
/*
 * Freescale 2.6 mxc_lcdc_bl semantics, as used by SqueezePlay: brightness is
 * the LPCCR pulse width; bl_power 4 turns the backlight off, and any non-zero
 * bl_power drives the panel's low-current (dim) GPIO.
 *
 * BL_CORE_FBBLANK is set by backlight_disable() when the panel is disabled
 * and cleared by backlight_enable() once it is initialized again. It keeps
 * the backlight dark meanwhile, whatever userspace writes to bl_power
 * (SqueezePlay's automatic brightness does so continuously).
 */
static int imx_lcdc_bl_update_status(struct backlight_device *bl)
{
	struct imx_lcdc *lcdc = bl_get_data(bl);
	u32 level = bl->props.brightness;

	if (bl->props.power == BACKLIGHT_POWER_OFF ||
	    bl->props.state & (BL_CORE_FBBLANK | BL_CORE_SUSPENDED))
		level = 0;

	spin_lock(&lcdc->lpccr_lock);
	lcdc->lpccr = (lcdc->lpccr & ~IMX21LCDC_LPCCR_PW) |
		      FIELD_PREP(IMX21LCDC_LPCCR_PW, level);
	if (lcdc->enabled)
		writel(lcdc->lpccr, lcdc->base + IMX21LCDC_LPCCR);
	spin_unlock(&lcdc->lpccr_lock);

	gpiod_set_value_cansleep(lcdc->bl_low_power,
				 bl->props.power != BACKLIGHT_POWER_ON);

	return 0;
}

static int imx_lcdc_bl_get_brightness(struct backlight_device *bl)
{
	struct imx_lcdc *lcdc = bl_get_data(bl);

	return FIELD_GET(IMX21LCDC_LPCCR_PW, READ_ONCE(lcdc->lpccr));
}

static const struct backlight_ops imx_lcdc_bl_ops = {
	.update_status = imx_lcdc_bl_update_status,
	.get_brightness = imx_lcdc_bl_get_brightness,
};

static int imx_lcdc_init_backlight(struct imx_lcdc *lcdc)
{
	struct device *dev = lcdc->drm.dev;
	/*
	 * Start dark: the panel enables the backlight through
	 * drm_panel_enable() once it shows valid frames, as legacy enabled
	 * mxc_lcdc_bl only after the display (fix-lcd-white-flash.patch).
	 * The fsl,lpccr pulse width becomes the initial brightness, and
	 * bl_power reads 0 as on legacy.
	 */
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = FIELD_MAX(IMX21LCDC_LPCCR_PW),
		.brightness = FIELD_GET(IMX21LCDC_LPCCR_PW, lcdc->lpccr),
		.power = BACKLIGHT_POWER_ON,
		.state = BL_CORE_FBBLANK,
	};
	struct backlight_device *bl;

	/* Only boards that configure the contrast PWM use it as backlight. */
	if (!lcdc->lpccr)
		return 0;

	lcdc->lpccr &= ~IMX21LCDC_LPCCR_PW;

	lcdc->bl_low_power = devm_gpiod_get_optional(dev, "backlight-low-power",
						     GPIOD_OUT_LOW);
	if (IS_ERR(lcdc->bl_low_power))
		return dev_err_probe(dev, PTR_ERR(lcdc->bl_low_power),
				     "backlight low-power GPIO\n");

	/* SqueezePlay opens /sys/class/backlight/mxc_lcdc_bl.0. */
	bl = devm_backlight_device_register(dev, "mxc_lcdc_bl.0", dev, lcdc,
					    &imx_lcdc_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");

	return 0;
}
#else
static int imx_lcdc_init_backlight(struct imx_lcdc *lcdc)
{
	return 0;
}
#endif

static irqreturn_t imx_lcdc_irq_handler(int irq, void *arg)
{
	struct imx_lcdc *lcdc = arg;
	struct drm_crtc *crtc = &lcdc->pipe.crtc;
	unsigned int status;

	status = readl(lcdc->base + IMX21LCDC_LISR);

	if (status & INTR_EOF) {
		drm_crtc_handle_vblank(crtc);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int imx_lcdc_probe(struct platform_device *pdev)
{
	struct imx_lcdc *lcdc;
	struct drm_device *drm;
	struct drm_bridge *bridge;
	int irq;
	int ret;
	struct device *dev = &pdev->dev;

	lcdc = devm_drm_dev_alloc(dev, &imx_lcdc_drm_driver,
				  struct imx_lcdc, drm);
	if (IS_ERR(lcdc))
		return PTR_ERR(lcdc);

	drm = &lcdc->drm;

	lcdc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(lcdc->base))
		return dev_err_probe(dev, PTR_ERR(lcdc->base), "Cannot get IO memory\n");

	bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
	if (IS_ERR(bridge))
		return dev_err_probe(dev, PTR_ERR(bridge), "Failed to find bridge\n");

	/* Get Clocks */
	lcdc->clk_ipg = devm_clk_get(dev, "ipg");
	if (IS_ERR(lcdc->clk_ipg))
		return dev_err_probe(dev, PTR_ERR(lcdc->clk_ipg), "Failed to get %s clk\n", "ipg");

	lcdc->clk_ahb = devm_clk_get(dev, "ahb");
	if (IS_ERR(lcdc->clk_ahb))
		return dev_err_probe(dev, PTR_ERR(lcdc->clk_ahb), "Failed to get %s clk\n", "ahb");

	lcdc->clk_per = devm_clk_get(dev, "per");
	if (IS_ERR(lcdc->clk_per))
		return dev_err_probe(dev, PTR_ERR(lcdc->clk_per), "Failed to get %s clk\n", "per");

	spin_lock_init(&lcdc->lpccr_lock);
	device_property_read_u32(dev, "fsl,lpccr", &lcdc->lpccr);

	ret = imx_lcdc_init_backlight(lcdc);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "Cannot set DMA Mask\n");

	/* Modeset init */
	ret = drmm_mode_config_init(drm);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot initialize mode configuration structure\n");

	/* CRTC, Plane, Encoder */
	ret = drm_simple_display_pipe_init(drm, &lcdc->pipe,
					   &imx_lcdc_pipe_funcs,
					   imx_lcdc_formats,
					   ARRAY_SIZE(imx_lcdc_formats), NULL, NULL);
	if (ret < 0)
		return dev_err_probe(drm->dev, ret, "Cannot setup simple display pipe\n");

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret < 0)
		return dev_err_probe(drm->dev, ret, "Failed to initialize vblank\n");

	ret = drm_bridge_attach(&lcdc->pipe.encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return dev_err_probe(drm->dev, ret, "Cannot attach bridge\n");

	lcdc->connector = drm_bridge_connector_init(drm, &lcdc->pipe.encoder);
	if (IS_ERR(lcdc->connector))
		return dev_err_probe(drm->dev, PTR_ERR(lcdc->connector), "Cannot init bridge connector\n");

	/*
	 * The LCDC controller does not have an enable bit. The
	 * controller starts directly when the clocks are enabled.
	 * If the clocks are enabled when the controller is not yet
	 * programmed with proper register values (enabled at the
	 * bootloader, for example) then it just goes into some undefined
	 * state.
	 * To avoid this issue, let's enable and disable LCDC IPG clock
	 * so that we force some kind of 'reset' to the LCDC block. As in
	 * imxfb, PER and AHB are not toggled: enabling them on the
	 * unprogrammed controller stalls an i.MX25 LCDC.
	 */

	ret = clk_prepare_enable(lcdc->clk_ipg);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot enable ipg clock\n");
	clk_disable_unprepare(lcdc->clk_ipg);

	drm->mode_config.min_width = LCDC_MIN_XRES;
	drm->mode_config.max_width = LCDC_MAX_XRES;
	drm->mode_config.min_height = LCDC_MIN_YRES;
	drm->mode_config.max_height = LCDC_MAX_YRES;
	drm->mode_config.preferred_depth = 16;
	drm->mode_config.funcs = &imx_lcdc_mode_config_funcs;
	drm->mode_config.helper_private = &imx_lcdc_mode_config_helpers;

	drm_mode_config_reset(drm);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		return ret;
	}

	ret = devm_request_irq(dev, irq, imx_lcdc_irq_handler, 0, "imx-lcdc", lcdc);
	if (ret < 0)
		return dev_err_probe(drm->dev, ret, "Failed to install IRQ handler\n");

	platform_set_drvdata(pdev, drm);

	ret = drm_dev_register(&lcdc->drm, 0);
	if (ret)
		return dev_err_probe(dev, ret, "Cannot register device\n");

	drm_client_setup(drm, NULL);

	return 0;
}

static void imx_lcdc_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	drm_atomic_helper_shutdown(drm);
}

static void imx_lcdc_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static struct platform_driver imx_lcdc_driver = {
	.driver = {
		.name = "imx-lcdc",
		.of_match_table = imx_lcdc_of_dev_id,
	},
	.probe = imx_lcdc_probe,
	.remove = imx_lcdc_remove,
	.shutdown = imx_lcdc_shutdown,
};
module_platform_driver(imx_lcdc_driver);

MODULE_AUTHOR("Marian Cichy <M.Cichy@pengutronix.de>");
MODULE_DESCRIPTION("Freescale i.MX LCDC driver");
MODULE_LICENSE("GPL");
