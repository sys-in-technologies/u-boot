// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Jian
 */

#include <backlight.h>
#include <command.h>
#include <dm.h>
#include <log.h>
#include <asm/gpio.h>
#include <sunxi_gpio.h>

static int do_backlight(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct udevice *dev;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	ret = uclass_get_device(UCLASS_PANEL_BACKLIGHT, 0, &dev);
	if (ret) {
		/* Argon board fallback: Backlight enable is PD17 */
		if (!strcmp(argv[1], "on")) {
			gpio_request(SUNXI_GPD(17), "backlight");
			gpio_direction_output(SUNXI_GPD(17), 1);
			printf("Backlight (PD17) turned on.\n");
		} else if (!strcmp(argv[1], "off")) {
			gpio_request(SUNXI_GPD(17), "backlight");
			gpio_direction_output(SUNXI_GPD(17), 0);
			printf("Backlight (PD17) turned off.\n");
		} else {
			return CMD_RET_USAGE;
		}
		return 0;
	}

	if (!strcmp(argv[1], "on")) {
		ret = backlight_enable(dev);
	} else if (!strcmp(argv[1], "off")) {
		ret = backlight_set_brightness(dev, 0);
	} else {
		return CMD_RET_USAGE;
	}

	if (ret) {
		printf("Backlight control failed: %d\n", ret);
		return CMD_RET_FAILURE;
	}

	printf("Backlight turned %s.\n", argv[1]);
	return 0;
}

U_BOOT_CMD(
	backlight, 2, 0, do_backlight,
	"control the display backlight",
	"on/off - turn the backlight on or off"
);
