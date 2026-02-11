// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner LCD driver
 *
 * (C) Copyright 2017 Vasily Khoruzhick <anarsoul@gmail.com>
 */

#include <display.h>
#include <log.h>
#include <video_bridge.h>
#include <backlight.h>
#include <dsi_host.h>
#include <panel.h>
#include <dm.h>
#include <edid.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/lcdc.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <sunxi_gpio.h>

struct sunxi_lcd_priv {
	struct display_timing timing;
	int panel_bpp;
};

static void sunxi_lcdc_config_pinmux(void)
{
#ifdef CONFIG_MACH_SUN50I
	int pin;

	for (pin = SUNXI_GPD(0); pin <= SUNXI_GPD(21); pin++) {
		sunxi_gpio_set_cfgpin(pin, SUNXI_GPD_LCD0);
		sunxi_gpio_set_drv(pin, 3);
	}
#endif
}

static int sunxi_lcd_enable(struct udevice *dev, int bpp,
			    const struct display_timing *edid)
{
	printf("LCD: sunxi_lcd_enable start\n");
#ifndef CONFIG_SUNXI_GEN_NCAT2
	struct sunxi_ccm_reg * const ccm =
	       (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
#endif
	struct sunxi_lcdc_reg * const lcdc =
	       (struct sunxi_lcdc_reg *)SUNXI_LCD0_BASE;
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	struct udevice *backlight, *panel, *dsi_host;
	int clk_div, clk_double, ret;

#ifdef CONFIG_SUNXI_GEN_NCAT2
	/* D1/T113 has DPSS_TOP at 0xabc, gate bit 0, reset bit 16 */
	printf("LCD: Enabling DPSS_TOP clock/reset...\n");
	setbits_le32((u8 *)SUNXI_CCM_BASE + 0xabc, BIT(16));
	setbits_le32((u8 *)SUNXI_CCM_BASE + 0xabc, BIT(0));
	/* BUS_TCON_LCD0 at 0xb7c, gate bit 0, reset bit 16 */
	printf("LCD: Enabling TCON_LCD0 BUS clock/reset...\n");
	setbits_le32((u8 *)SUNXI_CCM_BASE + 0xb7c, BIT(16));
	setbits_le32((u8 *)SUNXI_CCM_BASE + 0xb7c, BIT(0));
	printf("LCD: Setting up TCON TOP mux...\n");
	sunxi_tcon_top_setup(0, 0); /* Mixer 0, TCON 0 */
#else
	/* Reset off */
	setbits_le32(&ccm->ahb_reset1_cfg, 1 << AHB_RESET_OFFSET_LCD0);
	/* Clock on */
	setbits_le32(&ccm->ahb_gate1, 1 << AHB_GATE_OFFSET_LCD0);
#endif

	printf("LCD: Initializing LCDC...\n");
	lcdc_init(lcdc);
	sunxi_lcdc_config_pinmux();
#ifndef CONFIG_SUNXI_GEN_NCAT2
	lcdc_pll_set(ccm, 0, edid->pixelclock.typ / 1000,
		     &clk_div, &clk_double, false);
#else
	/* For NCAT2, lcdc_pll_set's first arg is dummy, and we handle clocks manually */
	printf("LCD: Setting up PLL...\n");
	lcdc_pll_set(NULL, 0, edid->pixelclock.typ / 1000,
		     &clk_div, &clk_double, false);
#endif
	printf("LCD: Setting TCON0 mode...\n");
	lcdc_tcon0_mode_set(lcdc, edid, clk_div, false,
			    priv->panel_bpp, CONFIG_VIDEO_LCD_DCLK_PHASE,
			    IS_ENABLED(CONFIG_VIDEO_SUNXI_MIPI_DSI));
	printf("LCD: Enabling LCDC...\n");
	lcdc_enable(lcdc, priv->panel_bpp);

	if (IS_ENABLED(CONFIG_VIDEO_SUNXI_MIPI_DSI)) {
		struct mipi_dsi_device *dsi_dev;

		printf("LCD: MIPI DSI path enabled, finding devices...\n");
		ret = uclass_get_device(UCLASS_PANEL, 0, &panel);
		if (ret) {
			printf("LCD: MIPI DSI panel not found\n");
			goto skip_dsi;
		}

		ret = uclass_get_device(UCLASS_DSI_HOST, 0, &dsi_host);
		if (ret) {
			printf("LCD: MIPI DSI host not found\n");
			goto skip_dsi;
		}

		/* We need the mipi_dsi_device which is stored in panel's plat */
		struct mipi_dsi_panel_plat *plat = dev_get_plat(panel);
		dsi_dev = plat->device;

		if (!dsi_dev) {
			printf("LCD: Error: MIPI DSI device not initialized in panel plat\n");
			goto skip_dsi;
		}

		printf("LCD: Initializing DSI host with %d lanes...\n", dsi_dev->lanes);
		ret = dsi_host_init(dsi_host, dsi_dev, (struct display_timing *)edid, 4, NULL);
		if (ret) {
			printf("LCD: MIPI DSI host init failed: %d\n", ret);
			goto skip_dsi;
		}

		ret = mipi_dsi_attach(dsi_dev);
		if (ret) {
			printf("LCD: MIPI DSI attach failed: %d\n", ret);
			goto skip_dsi;
		}
		if (ret) {
			printf("LCD: MIPI DSI host init failed: %d\n", ret);
			goto skip_dsi;
		}

		printf("LCD: Enabling panel...\n");
		ret = panel_enable_backlight(panel);
		if (ret)
			printf("LCD: MIPI DSI panel enable failed: %d\n", ret);

		printf("LCD: Enabling DSI host...\n");
		ret = dsi_host_enable(dsi_host);
		if (ret)
			printf("LCD: MIPI DSI host enable failed: %d\n", ret);
	}

skip_dsi:
	printf("LCD: Checking for backlight...\n");
	ret = uclass_get_device(UCLASS_PANEL_BACKLIGHT, 0, &backlight);
	if (!ret) {
		printf("LCD: Enabling backlight via uclass...\n");
		backlight_enable(backlight);
	} else {
		/* Argon board: Backlight enable is PD17 */
		printf("LCD: Uclass backlight not found, trying manual GPIO PD17...\n");
		gpio_request(SUNXI_GPD(17), "backlight");
		gpio_direction_output(SUNXI_GPD(17), 1);
	}

	printf("LCD: sunxi_lcd_enable done\n");
	return 0;
}

static int sunxi_lcd_read_timing(struct udevice *dev,
				 struct display_timing *timing)
{
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);

	memcpy(timing, &priv->timing, sizeof(struct display_timing));

	return 0;
}

static int sunxi_lcd_probe(struct udevice *dev)
{
	struct udevice *cdev;
	struct sunxi_lcd_priv *priv = dev_get_priv(dev);
	int ret;
	int node, timing_node, val;

	printf("LCD: Probing %s...\n", dev->name);

#ifdef CONFIG_VIDEO_BRIDGE
	/* Try to get timings from bridge first */
	ret = uclass_get_device(UCLASS_VIDEO_BRIDGE, 0, &cdev);
	if (!ret) {
		u8 edid[EDID_SIZE];
		int channel_bpp;

		printf("LCD: Found video bridge, attempting attach...\n");
		ret = video_bridge_attach(cdev);
		if (ret) {
			debug("video bridge attach failed: %d\n", ret);
			return ret;
		}
		ret = video_bridge_read_edid(cdev, edid, EDID_SIZE);
		if (ret > 0) {
			ret = edid_get_timing(edid, ret,
					      &priv->timing, &channel_bpp);
			priv->panel_bpp = channel_bpp * 3;
			if (!ret) {
				printf("LCD: Got timing from bridge.\n");
				return ret;
			}
		}
	}
#endif

	/* Fallback to timings from DT if there's no bridge or
	 * if reading EDID failed
	 */
	ret = uclass_get_device(UCLASS_PANEL, 0, &cdev);
	if (ret) {
		printf("LCD: MIPI DSI panel not found: %d\n", ret);
		return ret;
	}

	printf("LCD: Found panel %s, getting timing...\n", cdev->name);
	ret = panel_get_display_timing(cdev, &priv->timing);
	if (!ret) {
		struct mipi_dsi_panel_plat *plat = dev_get_plat(cdev);
		if (plat && plat->device)
			priv->panel_bpp = mipi_dsi_pixel_format_to_bpp(plat->format);
		else
			priv->panel_bpp = 24; /* Default for DSI */
		
		printf("LCD: Got timing from panel driver, bpp: %d\n", priv->panel_bpp);
		return 0;
	}

	printf("LCD: Driver didn't provide timing, falling back to DT...\n");
	if (fdtdec_decode_display_timing(gd->fdt_blob, dev_of_offset(cdev),
					 0, &priv->timing)) {
		printf("LCD: Failed to decode display timing from DT\n");
		return -EINVAL;
	}
	timing_node = fdt_subnode_offset(gd->fdt_blob, dev_of_offset(cdev),
					 "display-timings");
	node = fdt_first_subnode(gd->fdt_blob, timing_node);
	val = fdtdec_get_int(gd->fdt_blob, node, "bits-per-pixel", -1);
	if (val != -1)
		priv->panel_bpp = val;
	else
		priv->panel_bpp = 18;

	return 0;
}

static const struct dm_display_ops sunxi_lcd_ops = {
	.read_timing = sunxi_lcd_read_timing,
	.enable = sunxi_lcd_enable,
};

static const struct udevice_id sunxi_lcd_ids[] = {
	{ .compatible = "allwinner,sun8i-a83t-tcon-lcd" },
	{ .compatible = "allwinner,sun50i-a64-tcon-lcd" },
	{ .compatible = "allwinner,sun20i-d1-tcon-lcd" },
	{ }
};

U_BOOT_DRIVER(sunxi_lcd) = {
	.name   = "sunxi_lcd",
	.id     = UCLASS_DISPLAY,
	.of_match = sunxi_lcd_ids,
	.ops    = &sunxi_lcd_ops,
	.probe  = sunxi_lcd_probe,
	.priv_auto	= sizeof(struct sunxi_lcd_priv),
};
