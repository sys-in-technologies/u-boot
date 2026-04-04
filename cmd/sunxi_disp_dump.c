// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner T113-S / NCAT2 display pipeline register dump command.
 * Output format matches the kernel's PRE-INIT FULL REGISTER DUMP
 * (commit 51d35125e) for direct diff-ability.
 *
 * Usage:  dispdump          — dump all display pipeline registers
 */

#include <command.h>
#include <linux/io.h>

static u32 rr(ulong addr)
{
	return readl((void __iomem *)addr);
}

/* Print 4 consecutive 32-bit registers per line: TAG[off] val val val val */
static void dump_block(ulong base, const char *tag, u32 size)
{
	u32 off;

	for (off = 0; off < size; off += 16)
		printf("%s[%03x] %08x %08x %08x %08x\n", tag, off,
		       rr(base + off),      rr(base + off + 4),
		       rr(base + off + 8),  rr(base + off + 12));
}

static int do_dispdump(struct cmd_tbl *cmdtp, int flag,
		       int argc, char *const argv[])
{
	printf("=== PRE-INIT FULL REGISTER DUMP ===\n");

	/* CCU bus gates */
	printf("CCU: 0xb4c=%08x 0xb7c=%08x 0xabc=%08x 0x60c=%08x\n",
	       rr(0x02001b4c), rr(0x02001b7c),
	       rr(0x02001abc), rr(0x0200160c));
	printf("CCU: PLL_VIDEO0(040)=%08x MIPI_DSI(b24)=%08x TCON_LCD0(b60)=%08x\n",
	       rr(0x02001040), rr(0x02001b24), rr(0x02001b60));
	printf("CCU: DE(600)=%08x DE_BUS(60c)=%08x\n",
	       rr(0x02001600), rr(0x0200160c));
	printf("CCU: PLL_PERIPH0(020)=%08x\n",
	       rr(0x02001020));

	/* DSI controller: 0x05450000, 0x200 bytes */
	dump_block(0x05450000, "DSI",  0x200);

	/* D-PHY: 0x05451000, 0x120 bytes */
	dump_block(0x05451000, "DPHY", 0x120);

	/* TCON LCD0: 0x05461000, 0x200 bytes */
	dump_block(0x05461000, "TCON", 0x200);

	/* TCON TOP: 0x05460000, 0x030 bytes */
	dump_block(0x05460000, "TCON_TOP", 0x030);

	/* DE2 Internal CCU: 0x05000000, 0x010 bytes */
	dump_block(0x05000000, "DE2_CCU", 0x010);

	/* DE2 Global: 0x05100000, 0x010 bytes */
	printf("DE2_GLB[000] %08x %08x %08x %08x\n",
	       rr(0x05100000), rr(0x05100004),
	       rr(0x05100008), rr(0x0510000c));

	/* DE2 Blender: 0x05101000, 0x100 bytes */
	dump_block(0x05101000, "DE2_BLD", 0x100);

	/* DE2 UI Channel 1: 0x05103000, 0x100 bytes */
	dump_block(0x05103000, "DE2_UI1", 0x100);

	/* DE2 VI Channel 0: 0x05102000, 0x100 bytes */
	dump_block(0x05102000, "DE2_VI0", 0x100);

	/* VEP sub-engines: FCE/BWS/LTI/PEAK/ASE */
	printf("VEP: FCE=%08x BWS=%08x LTI=%08x PEAK=%08x ASE=%08x\n",
	       rr(0x051A0000), rr(0x051A2000),
	       rr(0x051A4000), rr(0x051A6000),
	       rr(0x051A8000));

	/* FCC enable + CCSC00 */
	printf("FCC_EN=%08x CCSC00[050]=%08x %08x %08x %08x\n",
	       rr(0x051AA000),
	       rr(0x051AA050), rr(0x051AA054),
	       rr(0x051AA058), rr(0x051AA05c));

	/* DCSC enable */
	printf("DCSC_EN=%08x\n", rr(0x051B0000));

	/* CCSC01 (D1 layout): 0x051FA000, 0x100 bytes */
	dump_block(0x051FA000, "CCSC01", 0x100);

	printf("=== Done ===\n");
	return 0;
}

U_BOOT_CMD(
	dispdump, 1, 1, do_dispdump,
	"dump display pipeline registers (DE2/TCON/DSI/DPHY)",
	"\n    Dumps all display pipeline registers.\n"
	"    Output matches Linux kernel dump format for direct diff."
);
