// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) Kamil Gołda <kamil.golda@protonmail.com>
// Copyright (c) Yedaya Katsman <yedaya.ka@gmail.com>
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) The Linux Foundation. All rights reserved.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct s6e8fc0_ctx {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data *supplies;
	struct gpio_desc *reset_gpio;
	const struct s6e8fc0_panel_desc *desc;
	bool dsi_attached;
	bool prepared;
	/* The bootloader hand-off (skip first reset+init) has been consumed. */
	bool handed_off;
	/* prepare() reset the panel and enable() must run the init sequence. */
	bool needs_on;
	/* The panel is lit (by the bootloader or our init); disable() must off it. */
	bool enabled;
};

struct s6e8fc0_panel_desc {
	const struct drm_display_mode *mode;
	const struct regulator_bulk_data *supplies;
	unsigned int num_supplies;
	void (*reset)(struct s6e8fc0_ctx *ctx);
	int (*on)(struct s6e8fc0_ctx *ctx);
	int (*off)(struct s6e8fc0_ctx *ctx);
	unsigned long mode_flags;
	unsigned int max_brightness;
	unsigned int default_brightness;
	bool skip_dsi_attach;
	bool skip_prepare;
};

static const struct regulator_bulk_data s6e8fc0_supplies[] = {
	{ .supply = "vdd" },
	{ .supply = "vci" },
};

static const struct regulator_bulk_data s6e8fc0_a40_supplies[] = {
	{ .supply = "vdd3p0" },
};

static inline
struct s6e8fc0_ctx *to_s6e8fc0_ctx(struct drm_panel *panel)
{
	return container_of_const(panel, struct s6e8fc0_ctx, panel);
}

static void s6e8fc0_m1906f9_reset(struct s6e8fc0_ctx *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(12000, 13000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

#define s6e8fc0_test_key_on_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define s6e8fc0_test_key_off_lvl2(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define s6e8fc0_test_key_on_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define s6e8fc0_test_key_off_lvl3(ctx) \
	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)

static int s6e8fc0_m1906f9_on(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	s6e8fc0_test_key_on_lvl3(&dsi_ctx);

	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0000);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x20);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04, 0xed);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed,
				     0xe4, 0x08, 0x96, 0xa4, 0x2a, 0x72, 0xe2,
				     0xca, 0x00);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);
	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	s6e8fc0_test_key_on_lvl3(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x01, 0x81, 0x04);
	s6e8fc0_test_key_off_lvl2(&dsi_ctx);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int s6e8fc0_m1906f9_off(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static void s6e8fc0_a40_reset(struct s6e8fc0_ctx *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
}

static int s6e8fc0_a40_on(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	u8 brightness[] = {
		MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
		ctx->desc->default_brightness >> 8,
		ctx->desc->default_brightness & 0xff,
	};

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	s6e8fc0_test_key_on_lvl3(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xed, 0x00, 0x4c, 0x40);
	s6e8fc0_test_key_off_lvl2(&dsi_ctx);
	s6e8fc0_test_key_off_lvl3(&dsi_ctx);

	s6e8fc0_test_key_on_lvl2(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x07, 0xf6);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf6, 0x87, 0x2d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x17, 0xbe);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbe,
				     0x5c, 0x00, 0x00, 0x00, 0x20, 0x07,
				     0x22, 0x31, 0x52, 0x00, 0x00, 0x00,
				     0x00, 0x00, 0x00, 0xe0, 0x0b, 0x7d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf7, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2,
				     0x0a, 0x04, 0xd8, 0xd8, 0xba, 0xba);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x04, 0xbf);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xbf, 0x04);
	s6e8fc0_test_key_off_lvl2(&dsi_ctx);

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, brightness, ARRAY_SIZE(brightness));
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x28);
	mipi_dsi_msleep(&dsi_ctx, 100);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int s6e8fc0_a40_off(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

/*
 * Bootloader hand-off: on this board the bootloader leaves the panel fully
 * initialised and lit (continuous splash), and the DSIM/DECON drivers keep
 * that pipeline live.  On the *first* prepare we therefore must not re-run the
 * panel-on sequence — the panel is already up.  The hand-off is one-shot;
 * after it is consumed, off/on cycles blank the panel by putting it to sleep
 * (display-off + sleep-in) and wake it again (exit-sleep + display-on) over the
 * still-live link, without ever resetting it.  Set to 0 to run the panel-on
 * sequence from the very first prepare instead of trusting the bootloader.
 */
static bool skip_init = true;
module_param(skip_init, bool, 0644);
MODULE_PARM_DESC(skip_init, "Trust the bootloader-initialised panel on first prepare (one-shot hand-off)");

static int s6e8fc0_prepare(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (ctx->desc->skip_prepare) {
		dev_info(dev, "probe-only mode, blocking panel prepare\n");
		return -EAGAIN;
	}

	ret = regulator_bulk_enable(ctx->desc->num_supplies, ctx->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	/*
	 * Consume the one-shot bootloader hand-off: the panel is already lit and
	 * initialised, so don't reset it or re-run the init sequence this once.
	 */
	if (skip_init && !ctx->handed_off) {
		ctx->handed_off = true;
		ctx->needs_on = false;
		dev_info(dev, "bootloader hand-off: preserving live panel, skipping reset + init\n");
		ctx->prepared = true;
		return 0;
	}

	/*
	 * Sleep/resume off/on: don't reset the panel.  The DSIM link and DECON
	 * stay live across the cycle, so the DDIC keeps its full
	 * (bootloader-programmed) register configuration through a display-off +
	 * sleep-in; enable() just wakes it with exit-sleep + display-on.  A
	 * hardware reset would wipe that config, and this driver does not
	 * reprogram the gamma/ELVSS/source registers a fully reset DDIC needs.
	 */
	ctx->needs_on = true;
	ctx->prepared = true;

	return 0;
}

static int s6e8fc0_enable(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/*
	 * Wake the panel over the continuously-running video link.  Skipped on
	 * the bootloader hand-off, where the panel is already lit.
	 */
	if (ctx->needs_on) {
		ret = ctx->desc->on(ctx);
		if (ret < 0) {
			dev_err(dev, "Failed to enable panel: %d\n", ret);
			return ret;
		}
		ctx->needs_on = false;
	}

	ctx->enabled = true;

	return 0;
}

static int s6e8fc0_disable(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	if (!ctx->enabled)
		return 0;

	/* Display-off + sleep-in over the still-running video link. */
	ret = ctx->desc->off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to disable panel: %d\n", ret);

	/*
	 * The panel is now actually asleep, so the next enable() must send the
	 * exit-sleep + display-on DCS regardless of how it got here (the
	 * bootloader hand-off in prepare() only ever applies once, before the
	 * first real disable()). Without this, needs_on stays false from that
	 * one-shot hand-off and every enable() after the first blank silently
	 * skips waking the panel, leaving it asleep for the rest of the boot.
	 */
	ctx->needs_on = true;
	ctx->enabled = false;

	return 0;
}

static int s6e8fc0_unprepare(struct drm_panel *panel)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);

	if (!ctx->prepared)
		return 0;

	/*
	 * The panel was put to sleep (display-off + sleep-in) by disable() and is
	 * woken by the next prepare()/enable(); it is never reset or fully powered
	 * down, so there is no reset/teardown here.  Balance prepare()'s regulator
	 * enable (the rail itself stays up across the sleep).
	 */
	regulator_bulk_disable(ctx->desc->num_supplies, ctx->supplies);
	ctx->prepared = false;

	return 0;
}

static const struct drm_display_mode s6e8fc0_m1906f9_samsungp_mode = {
	.clock = (720 + 350 + 40 + 294) * (1560 + 17 + 2 + 5) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 350,
	.hsync_end = 720 + 350 + 40,
	.htotal = 720 + 350 + 40 + 294,
	.vdisplay = 1560,
	.vsync_start = 1560 + 17,
	.vsync_end = 1560 + 17 + 2,
	.vtotal = 1560 + 17 + 2 + 5,
	.width_mm = 65,
	.height_mm = 140,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static const struct drm_display_mode s6e8fc0_a40_mode = {
	.clock = (1080 + 14 + 30 + 10) * (2340 + 5 + 25 + 2) * 60 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 14,
	.hsync_end = 1080 + 14 + 30,
	.htotal = 1080 + 14 + 30 + 10,
	.vdisplay = 2340,
	.vsync_start = 2340 + 5,
	.vsync_end = 2340 + 5 + 25,
	.vtotal = 2340 + 5 + 25 + 2,
	.width_mm = 63,
	.height_mm = 135,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int s6e8fc0_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct s6e8fc0_ctx *ctx = to_s6e8fc0_ctx(panel);

	return drm_connector_helper_get_modes_fixed(connector, ctx->desc->mode);
}

static const struct drm_panel_funcs s6e8fc0_panel_funcs = {
	.prepare = s6e8fc0_prepare,
	.enable = s6e8fc0_enable,
	.disable = s6e8fc0_disable,
	.unprepare = s6e8fc0_unprepare,
	.get_modes = s6e8fc0_get_modes,
};

static int s6e8fc0_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static int s6e8fc0_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops s6e8fc0_bl_ops = {
	.update_status = s6e8fc0_bl_update_status,
	.get_brightness = s6e8fc0_bl_get_brightness,
};

static struct backlight_device *
s6e8fc0_create_backlight(struct s6e8fc0_ctx *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = ctx->desc->default_brightness,
		.max_brightness = ctx->desc->max_brightness,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &s6e8fc0_bl_ops, &props);
}

static const struct s6e8fc0_panel_desc s6e8fc0_m1906f9_desc = {
	.mode = &s6e8fc0_m1906f9_samsungp_mode,
	.supplies = s6e8fc0_supplies,
	.num_supplies = ARRAY_SIZE(s6e8fc0_supplies),
	.reset = s6e8fc0_m1906f9_reset,
	.on = s6e8fc0_m1906f9_on,
	.off = s6e8fc0_m1906f9_off,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.max_brightness = 1023,
	.default_brightness = 512,
};

static const struct s6e8fc0_panel_desc s6e8fc0_a40_desc = {
	.mode = &s6e8fc0_a40_mode,
	.supplies = s6e8fc0_a40_supplies,
	.num_supplies = ARRAY_SIZE(s6e8fc0_a40_supplies),
	.reset = s6e8fc0_a40_reset,
	.on = s6e8fc0_a40_on,
	.off = s6e8fc0_a40_off,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
		      MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.max_brightness = 1023,
	.default_brightness = 512,
};

static int s6e8fc0_m1906f9_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct s6e8fc0_ctx *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e8fc0_ctx, panel,
				   &s6e8fc0_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->desc = of_device_get_match_data(dev);
	if (!ctx->desc)
		return -ENODEV;

	ret = devm_regulator_bulk_get_const(dev,
					    ctx->desc->num_supplies,
					    ctx->desc->supplies,
					    &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = ctx->desc->mode_flags;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = s6e8fc0_create_backlight(ctx);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	if (ctx->desc->skip_dsi_attach) {
		dev_info(dev, "probe-only mode, skipping panel DSI attach\n");
		return 0;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}
	ctx->dsi_attached = true;

	return 0;
}

static void s6e8fc0_remove(struct mipi_dsi_device *dsi)
{
	struct s6e8fc0_ctx *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	if (ctx->dsi_attached) {
		ret = mipi_dsi_detach(dsi);
		if (ret < 0)
			dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);
	}

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_s6e8fc0_of_match[] = {
	{ .compatible = "samsung,s6e8fc0-m1906f9", .data = &s6e8fc0_m1906f9_desc },
	{ .compatible = "samsung,s6e8fc0-a40", .data = &s6e8fc0_a40_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_s6e8fc0_of_match);

static struct mipi_dsi_driver s6e8fc0_driver = {
	.probe = s6e8fc0_m1906f9_probe,
	.remove = s6e8fc0_remove,
	.driver = {
		.name = "panel-samsung-s6e8fc0-m1906f9",
		.of_match_table = samsung_s6e8fc0_of_match,
	},
};
module_mipi_dsi_driver(s6e8fc0_driver);

MODULE_AUTHOR("Kamil Gołda <kamil.golda@protonmail.com>");
MODULE_DESCRIPTION("DRM driver for Samsung s6e8fc0 DSI controller");
MODULE_LICENSE("GPL");
