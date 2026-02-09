// SPDX-License-Identifier: GPL-2.0+
/*
 * MIPI DSI driver for Allwinner T113/D1
 */

#include <dm.h>
#include <display.h>
#include <video.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <sunxi_gpio.h>
#include <mipi_dsi.h>
#include <panel.h>

struct sun6i_mipi_dsi_priv {
	struct mipi_dsi_device device;
	struct mipi_dsi_host host;
	struct udevice *panel;
	void __iomem *regs;
};

static int sun6i_mipi_dsi_enable(struct udevice *dev, int panel_bpp,
				 const struct display_timing *timing)
{
	struct sun6i_mipi_dsi_priv *priv = dev_get_priv(dev);
	int ret;

	/* Initialize DSI Controller */
	/* Initialize D-PHY */
	
	/* Enable Panel */
	if (priv->panel) {
		ret = panel_enable_backlight(priv->panel);
		if (ret)
			return ret;
	}

	return 0;
}

static int sun6i_mipi_dsi_read_timing(struct udevice *dev,
				      struct display_timing *timing)
{
	struct sun6i_mipi_dsi_priv *priv = dev_get_priv(dev);
	int ret;

	if (priv->panel) {
		ret = panel_get_display_timing(priv->panel, timing);
		if (ret)
			return ret;
	}

	return 0;
}

static int sun6i_mipi_dsi_probe(struct udevice *dev)
{
	struct sun6i_mipi_dsi_priv *priv = dev_get_priv(dev);
	int ret;

	priv->regs = dev_read_addr_ptr(dev);

	/* Find panel */
	ret = uclass_get_device_by_phandle(UCLASS_PANEL, dev, "panel", &priv->panel);
	if (ret) {
		/* Try to find first panel */
		ret = uclass_first_device_err(UCLASS_PANEL, &priv->panel);
		if (ret) {
			printf("sun6i_mipi_dsi: No panel found\n");
			return ret;
		}
	}

	return 0;
}

static const struct dm_display_ops sun6i_mipi_dsi_ops = {
	.read_timing = sun6i_mipi_dsi_read_timing,
	.enable = sun6i_mipi_dsi_enable,
};

static const struct udevice_id sun6i_mipi_dsi_ids[] = {
	{ .compatible = "allwinner,sun20i-d1-mipi-dsi" },
	{ .compatible = "allwinner,sun50i-a100-mipi-dsi" },
	{ }
};

U_BOOT_DRIVER(sun6i_mipi_dsi) = {
	.name	= "sun6i_mipi_dsi",
	.id	= UCLASS_DISPLAY,
	.of_match = sun6i_mipi_dsi_ids,
	.ops	= &sun6i_mipi_dsi_ops,
	.probe	= sun6i_mipi_dsi_probe,
	.priv_auto = sizeof(struct sun6i_mipi_dsi_priv),
};
