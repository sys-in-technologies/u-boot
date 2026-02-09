// SPDX-License-Identifier: GPL-2.0+
/*
 * TCON TOP driver for Allwinner T113/D1
 */

#include <dm.h>

static const struct udevice_id sun8i_tcon_top_ids[] = {
	{ .compatible = "allwinner,sun20i-d1-tcon-top" },
	{ }
};

U_BOOT_DRIVER(sun8i_tcon_top) = {
	.name	= "sun8i_tcon_top",
	.id	= UCLASS_NOP,
	.of_match = sun8i_tcon_top_ids,
};
