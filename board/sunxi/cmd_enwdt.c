#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <malloc.h>
#include <errno.h>
#include <linux/string.h>
#include <fdt_support.h>
#include <asm/arch/timer.h>

#define IOBASE                  0x02050000
#define WDOG_CTRL_REG           0x00B0
#define WDOG_MODE_REG           0x00B8

static int do_enwdt(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	int io_base = IOBASE;

	/* System 32K Select = LOSC */
	/* writel(0x16aa010f , 0x01c20310); */

	writel(0x16aa00b1, io_base + WDOG_MODE_REG); /* 16s */
	/* Restart the Watchdog */
	writel((0xA57<<1) | (1<<0), io_base + WDOG_CTRL_REG);

	return 0;
}

U_BOOT_CMD(enwdt,	1,	1,	do_enwdt,
		"\n enwdt  -- enable wdt\n",
		"disable after system startup\n");

