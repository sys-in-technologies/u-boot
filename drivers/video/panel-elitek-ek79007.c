// SPDX-License-Identifier: GPL-2.0+
/*
 * Elitek EK79007 MIPI DSI panel driver for U-Boot
 * Backported from Linux drivers/gpu/drm/panel/panel-ek79007.c
 *
 * Panel: 1024x600, 4-lane MIPI DSI, RGB888, video mode with sync pulse
 */

#include <dm.h>
#include <log.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <power/regulator.h>
#include <asm/gpio.h>
#include <linux/delay.h>

struct ek79007_priv {
	struct udevice *dev;
	struct gpio_desc reset;
	struct udevice *power;
};

/*
 * Pixel clock: 51.2 MHz
 * H: 1024 active + 160 front porch + 10 sync + 160 back porch = 1354 total
 * V:  600 active +  12 front porch +  1 sync +  23 back porch =  636 total
 */
static const struct display_timing ek79007_timing = {
	.pixelclock	= { 51200000, 51200000, 51200000 },
	.hactive	= { 1024, 1024, 1024 },
	.hfront_porch	= { 160, 160, 160 },
	.hsync_len	= { 10, 10, 10 },
	.hback_porch	= { 160, 160, 160 },
	.vactive	= { 600, 600, 600 },
	.vfront_porch	= { 12, 12, 12 },
	.vsync_len	= { 1, 1, 1 },
	.vback_porch	= { 23, 23, 23 },
};

/* Vendor-specific init commands: reg/value pairs */
static const u8 ek79007_init_cmds[][2] = {
	{ 0x80, 0xAC },
	{ 0x81, 0xB8 },
	{ 0x82, 0x09 },
	{ 0x83, 0x78 },
	{ 0x84, 0x7F },
	{ 0x85, 0xBB },
	{ 0x86, 0x70 },
};

static int ek79007_enable_backlight(struct udevice *dev)
{
	struct ek79007_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	struct mipi_dsi_device *dsi = plat->device;
	int ret, i;

	printf("Panel ek79007: powering up...\n");

	if (priv->power) {
		ret = regulator_set_enable(priv->power, true);
		if (ret < 0) {
			printf("Panel ek79007: failed to enable power: %d\n", ret);
			return ret;
		}
		mdelay(15);
	}

	/* Assert reset, then release and wait for panel to boot */
	dm_gpio_set_value(&priv->reset, 1);
	mdelay(15);
	dm_gpio_set_value(&priv->reset, 0);
	mdelay(120);
	//dm_gpio_set_value(&priv->reset, 1);
	//mdelay(120);

	printf("Panel ek79007: sending init commands...\n");

	ret = mipi_dsi_dcs_set_tear_off(dsi);
	if (ret < 0)
		printf("Panel ek79007: set_tear_off failed: %d\n", ret);

	for (i = 0; i < ARRAY_SIZE(ek79007_init_cmds); i++) {
		ret = mipi_dsi_dcs_write(dsi, ek79007_init_cmds[i][0],
					 &ek79007_init_cmds[i][1], 1);
		if (ret < 0) {
			printf("Panel ek79007: init cmd 0x%02x failed: %d\n",
			       ek79007_init_cmds[i][0], ret);
			return ret;
		}
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		printf("Panel ek79007: exit sleep mode failed: %d\n", ret);
		return ret;
	}
	mdelay(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		printf("Panel ek79007: set display on failed: %d\n", ret);
		return ret;
	}
	mdelay(20);

	printf("Panel ek79007: init complete.\n");
	return 0;
}

static int ek79007_get_display_timing(struct udevice *dev,
				      struct display_timing *timing)
{
	memcpy(timing, &ek79007_timing, sizeof(*timing));
	return 0;
}

static int ek79007_probe(struct udevice *dev)
{
	struct ek79007_priv *priv = dev_get_priv(dev);
	struct mipi_dsi_panel_plat *plat = dev_get_plat(dev);
	static struct mipi_dsi_device dsi_dev;
	int ret;

	printf("Panel ek79007: probing %s...\n", dev->name);

	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset,
				   GPIOD_IS_OUT);
	if (ret) {
		printf("Panel ek79007: failed to request reset-gpios: %d\n", ret);
		return ret;
	}

	ret = device_get_supply_regulator(dev, "power-supply", &priv->power);
	if (ret && ret != -ENOENT) {
		printf("Panel ek79007: failed to get power-supply: %d\n", ret);
		return ret;
	}

	plat->lanes = 4;
	plat->format = MIPI_DSI_FMT_RGB888;
	plat->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			   MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	memset(&dsi_dev, 0, sizeof(dsi_dev));
	dsi_dev.dev = dev;
	dsi_dev.lanes = plat->lanes;
	dsi_dev.format = plat->format;
	dsi_dev.mode_flags = plat->mode_flags;
	plat->device = &dsi_dev;

	printf("Panel ek79007: probe successful (lanes=%d).\n", plat->lanes);
	return 0;
}

static const struct panel_ops ek79007_ops = {
	.enable_backlight	= ek79007_enable_backlight,
	.get_display_timing	= ek79007_get_display_timing,
};

static const struct udevice_id ek79007_ids[] = {
	{ .compatible = "elitek,ek79007" },
	{ }
};

U_BOOT_DRIVER(ek79007) = {
	.name		= "ek79007",
	.id		= UCLASS_PANEL,
	.of_match	= ek79007_ids,
	.probe		= ek79007_probe,
	.ops		= &ek79007_ops,
	.priv_auto	= sizeof(struct ek79007_priv),
	.plat_auto	= sizeof(struct mipi_dsi_panel_plat),
};
