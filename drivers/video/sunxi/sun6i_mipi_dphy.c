// SPDX-License-Identifier: GPL-2.0+
/*
 * MIPI D-PHY driver for Allwinner T113/D1
 */

#include <dm.h>

static const struct udevice_id sun6i_mipi_dphy_ids[] = {
	{ .compatible = "allwinner,sun20i-d1-mipi-dphy" },
	{ .compatible = "allwinner,sun50i-a100-mipi-dphy" },
	{ }
};

U_BOOT_DRIVER(sun6i_mipi_dphy) = {
	.name	= "sun6i_mipi_dphy",
	.id	= UCLASS_PHY,
	.of_match = sun6i_mipi_dphy_ids,
};
