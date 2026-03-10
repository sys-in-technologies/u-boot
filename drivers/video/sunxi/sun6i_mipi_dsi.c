// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Allwinnertech Co., Ltd.
 * Copyright (C) 2017-2018 Bootlin
 * Copyright (C) 2026 Jian
 */

#include <clk.h>
#include <dm.h>
#include <dsi_host.h>
#include <generic-phy.h>
#include <phy-mipi-dphy.h>
#include <log.h>
#include <mipi_dsi.h>
#include <reset.h>
#include <power/regulator.h>
#include <asm/arch/cpu.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iopoll.h>

#define SUN6I_DSI_CTL_REG		0x000
#define SUN6I_DSI_CTL_EN			BIT(0)

#define SUN6I_DSI_BASIC_CTL_REG		0x00c
#define SUN6I_DSI_BASIC_CTL_TRAIL_INV(n)		(((n) & 0xf) << 4)
#define SUN6I_DSI_BASIC_CTL_TRAIL_FILL		BIT(3)
#define SUN6I_DSI_BASIC_CTL_HBP_DIS		BIT(2)
#define SUN6I_DSI_BASIC_CTL_HSA_HSE_DIS		BIT(1)
#define SUN6I_DSI_BASIC_CTL_VIDEO_BURST		BIT(0)

#define SUN6I_DSI_BASIC_CTL0_REG	0x010
#define SUN6I_DSI_BASIC_CTL0_HS_EOTP_EN		BIT(18)
#define SUN6I_DSI_BASIC_CTL0_CRC_EN		BIT(17)
#define SUN6I_DSI_BASIC_CTL0_ECC_EN		BIT(16)
#define SUN6I_DSI_BASIC_CTL0_INST_ST		BIT(0)

#define SUN6I_DSI_BASIC_CTL1_REG	0x014
#define SUN6I_DSI_BASIC_CTL1_VIDEO_ST_DELAY(n)	(((n) & 0x1fff) << 4)
#define SUN6I_DSI_BASIC_CTL1_VIDEO_FILL		BIT(2)
#define SUN6I_DSI_BASIC_CTL1_VIDEO_PRECISION	BIT(1)
#define SUN6I_DSI_BASIC_CTL1_VIDEO_MODE		BIT(0)

#define SUN6I_DSI_BASIC_SIZE0_REG	0x018
#define SUN6I_DSI_BASIC_SIZE0_VBP(n)		(((n) & 0xfff) << 16)
#define SUN6I_DSI_BASIC_SIZE0_VSA(n)		((n) & 0xfff)

#define SUN6I_DSI_BASIC_SIZE1_REG	0x01c
#define SUN6I_DSI_BASIC_SIZE1_VT(n)		(((n) & 0xfff) << 16)
#define SUN6I_DSI_BASIC_SIZE1_VACT(n)		((n) & 0xfff)

#define SUN6I_DSI_INST_FUNC_REG(n)	(0x020 + (n) * 0x04)
#define SUN6I_DSI_INST_FUNC_INST_MODE(n)	(((n) & 0xf) << 28)
#define SUN6I_DSI_INST_FUNC_ESCAPE_ENTRY(n)	(((n) & 0xf) << 24)
#define SUN6I_DSI_INST_FUNC_TRANS_PACKET(n)	(((n) & 0xf) << 20)
#define SUN6I_DSI_INST_FUNC_LANE_CEN		BIT(4)
#define SUN6I_DSI_INST_FUNC_LANE_DEN(n)		((n) & 0xf)

#define SUN6I_DSI_INST_LOOP_SEL_REG	0x040

#define SUN6I_DSI_INST_LOOP_NUM_REG(n)	(0x044 + (n) * 0x10)
#define SUN6I_DSI_INST_LOOP_NUM_N1(n)		(((n) & 0xfff) << 16)
#define SUN6I_DSI_INST_LOOP_NUM_N0(n)		((n) & 0xfff)

#define SUN6I_DSI_INST_JUMP_SEL_REG	0x048

#define SUN6I_DSI_INST_JUMP_CFG_REG(n)	(0x04c + (n) * 0x04)
#define SUN6I_DSI_INST_JUMP_CFG_TO(n)		(((n) & 0xf) << 20)
#define SUN6I_DSI_INST_JUMP_CFG_POINT(n)	(((n) & 0xf) << 16)
#define SUN6I_DSI_INST_JUMP_CFG_NUM(n)		((n) & 0xffff)

#define SUN6I_DSI_TRANS_START_REG	0x060

#define SUN6I_DSI_TRANS_ZERO_REG	0x078

#define SUN6I_DSI_TCON_DRQ_REG		0x07c
#define SUN6I_DSI_TCON_DRQ_ENABLE_MODE		BIT(28)
#define SUN6I_DSI_TCON_DRQ_SET(n)		((n) & 0x3ff)

#define SUN6I_DSI_PIXEL_CTL0_REG	0x080
#define SUN6I_DSI_PIXEL_CTL0_PD_PLUG_DISABLE	BIT(16)
#define SUN6I_DSI_PIXEL_CTL0_FORMAT(n)		((n) & 0xf)

#define SUN6I_DSI_PIXEL_CTL1_REG	0x084

#define SUN6I_DSI_PIXEL_PH_REG		0x090
#define SUN6I_DSI_PIXEL_PH_ECC(n)		(((n) & 0xff) << 24)
#define SUN6I_DSI_PIXEL_PH_WC(n)		(((n) & 0xffff) << 8)
#define SUN6I_DSI_PIXEL_PH_VC(n)		(((n) & 3) << 6)
#define SUN6I_DSI_PIXEL_PH_DT(n)		((n) & 0x3f)

#define SUN6I_DSI_PIXEL_PF0_REG		0x098
#define SUN6I_DSI_PIXEL_PF0_CRC_FORCE(n)	((n) & 0xffff)

#define SUN6I_DSI_PIXEL_PF1_REG		0x09c
#define SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINEN(n)	(((n) & 0xffff) << 16)
#define SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINE0(n)	((n) & 0xffff)

#define SUN6I_DSI_SYNC_HSS_REG		0x0b0
#define SUN6I_DSI_SYNC_HSE_REG		0x0b4
#define SUN6I_DSI_SYNC_VSS_REG		0x0b8
#define SUN6I_DSI_SYNC_VSE_REG		0x0bc
#define SUN6I_DSI_BLK_HSA0_REG		0x0c0
#define SUN6I_DSI_BLK_HSA1_REG		0x0c4
#define SUN6I_DSI_BLK_PF(n)			(((n) & 0xffff) << 16)
#define SUN6I_DSI_BLK_PD(n)			((n) & 0xff)
#define SUN6I_DSI_BLK_HBP0_REG		0x0c8
#define SUN6I_DSI_BLK_HBP1_REG		0x0cc
#define SUN6I_DSI_BLK_HFP0_REG		0x0d0
#define SUN6I_DSI_BLK_HFP1_REG		0x0d4
#define SUN6I_DSI_BLK_HBLK0_REG		0x0e0
#define SUN6I_DSI_BLK_HBLK1_REG		0x0e4
#define SUN6I_DSI_BLK_VBLK0_REG		0x0e8
#define SUN6I_DSI_BLK_VBLK1_REG		0x0ec

#define SUN6I_DSI_BURST_LINE_REG	0x0f0
#define SUN6I_DSI_BURST_LINE_SYNC_POINT(n)	(((n) & 0xffff) << 16)
#define SUN6I_DSI_BURST_LINE_NUM(n)		((n) & 0xffff)

#define SUN6I_DSI_BURST_DRQ_REG		0x0f4
#define SUN6I_DSI_BURST_DRQ_EDGE1(n)		(((n) & 0xffff) << 16)
#define SUN6I_DSI_BURST_DRQ_EDGE0(n)		((n) & 0xffff)

#define SUN6I_DSI_CMD_CTL_REG		0x200
#define SUN6I_DSI_CMD_CTL_RX_OVERFLOW		BIT(26)
#define SUN6I_DSI_CMD_CTL_RX_FLAG		BIT(25)
#define SUN6I_DSI_CMD_CTL_TX_FLAG		BIT(9)

#define SUN6I_DSI_CMD_RX_REG(n)		(0x240 + (n) * 0x04)
#define SUN6I_DSI_DEBUG_DATA_REG	0x2f8
#define SUN6I_DSI_CMD_TX_REG(n)		(0x300 + (n) * 0x04)

#define SUN6I_DSI_SYNC_POINT		40
#define SUN6I_DSI_TCON_DIV		4

enum sun6i_dsi_start_inst {
	DSI_START_LPRX,
	DSI_START_LPTX,
	DSI_START_HSC,
	DSI_START_HSD,
};

enum sun6i_dsi_inst_id {
	DSI_INST_ID_LP11	= 0,
	DSI_INST_ID_TBA,
	DSI_INST_ID_HSC,
	DSI_INST_ID_HSD,
	DSI_INST_ID_LPDT,
	DSI_INST_ID_HSCEXIT,
	DSI_INST_ID_NOP,
	DSI_INST_ID_DLY,
	DSI_INST_ID_END		= 15,
};

enum sun6i_dsi_inst_mode {
	DSI_INST_MODE_STOP	= 0,
	DSI_INST_MODE_TBA,
	DSI_INST_MODE_HS,
	DSI_INST_MODE_ESCAPE,
	DSI_INST_MODE_HSCEXIT,
	DSI_INST_MODE_NOP,
};

enum sun6i_dsi_inst_escape {
	DSI_INST_ESCA_LPDT	= 0,
	DSI_INST_ESCA_ULPS,
	DSI_INST_ESCA_UN1,
	DSI_INST_ESCA_UN2,
	DSI_INST_ESCA_RESET,
	DSI_INST_ESCA_UN3,
	DSI_INST_ESCA_UN4,
	DSI_INST_ESCA_UN5,
};

enum sun6i_dsi_inst_packet {
	DSI_INST_PACK_PIXEL	= 0,
	DSI_INST_PACK_COMMAND,
};

struct sun6i_dsi_variant {
	bool has_mod_clk;
	bool set_mod_clk;
};

struct sun6i_dsi_priv {
	struct mipi_dsi_host host;
	struct mipi_dsi_device *device;
	void __iomem *regs;
	struct clk bus_clk;
	struct clk mod_clk;
	struct reset_ctl reset;
	struct udevice *vcc_dsi;
	struct phy dphy;
	const struct sun6i_dsi_variant *variant;
	struct display_timing timing;
};

static inline struct sun6i_dsi_priv *host_to_sun6i_dsi(struct mipi_dsi_host *host)
{
	return container_of(host, struct sun6i_dsi_priv, host);
}

static const u32 sun6i_dsi_ecc_array[] = {
	[0] = (BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(5) | BIT(7) | BIT(10) |
	       BIT(11) | BIT(13) | BIT(16) | BIT(20) | BIT(21) | BIT(22) |
	       BIT(23)),
	[1] = (BIT(0) | BIT(1) | BIT(3) | BIT(4) | BIT(6) | BIT(8) | BIT(10) |
	       BIT(12) | BIT(14) | BIT(17) | BIT(20) | BIT(21) | BIT(22) |
	       BIT(23)),
	[2] = (BIT(0) | BIT(2) | BIT(3) | BIT(5) | BIT(6) | BIT(9) | BIT(11) |
	       BIT(12) | BIT(15) | BIT(18) | BIT(20) | BIT(21) | BIT(22)),
	[3] = (BIT(1) | BIT(2) | BIT(3) | BIT(7) | BIT(8) | BIT(9) | BIT(13) |
	       BIT(14) | BIT(15) | BIT(19) | BIT(20) | BIT(21) | BIT(23)),
	[4] = (BIT(4) | BIT(5) | BIT(6) | BIT(7) | BIT(8) | BIT(9) | BIT(16) |
	       BIT(17) | BIT(18) | BIT(19) | BIT(20) | BIT(22) | BIT(23)),
	[5] = (BIT(10) | BIT(11) | BIT(12) | BIT(13) | BIT(14) | BIT(15) |
	       BIT(16) | BIT(17) | BIT(18) | BIT(19) | BIT(21) | BIT(22) |
	       BIT(23)),
};

static u32 sun6i_dsi_ecc_compute(unsigned int data)
{
	int i;
	u8 ecc = 0;

	for (i = 0; i < ARRAY_SIZE(sun6i_dsi_ecc_array); i++) {
		u32 field = sun6i_dsi_ecc_array[i];
		bool init = false;
		u8 val = 0;
		int j;

		for (j = 0; j < 24; j++) {
			if (!(BIT(j) & field))
				continue;

			if (!init) {
				val = (BIT(j) & data) ? 1 : 0;
				init = true;
			} else {
				val ^= (BIT(j) & data) ? 1 : 0;
			}
		}

		ecc |= val << i;
	}

	return ecc;
}

static u16 sun6i_dsi_crc_compute(u8 const *buffer, size_t len)
{
	u16 crc = 0xffff;
	int i, j;

	for (i = 0; i < len; i++) {
		crc ^= buffer[i];
		for (j = 0; j < 8; j++) {
			if (crc & 1)
				crc = (crc >> 1) ^ 0x8408;
			else
				crc >>= 1;
		}
	}

	return ~crc;
}

static u32 sun6i_dsi_build_sync_pkt(u8 dt, u8 vc, u8 d0, u8 d1)
{
	u32 val = dt & 0x3f;

	val |= (vc & 3) << 6;
	val |= (d0 & 0xff) << 8;
	val |= (d1 & 0xff) << 16;
	val |= sun6i_dsi_ecc_compute(val) << 24;

	return val;
}

static u32 sun6i_dsi_build_blk0_pkt(u8 vc, u16 wc)
{
	return sun6i_dsi_build_sync_pkt(MIPI_DSI_BLANKING_PACKET, vc,
					wc & 0xff, wc >> 8);
}

static u32 sun6i_dsi_build_blk1_pkt(u16 pd, u16 crc, size_t len)
{
	u32 val = SUN6I_DSI_BLK_PD(pd);

	return val | SUN6I_DSI_BLK_PF(crc);
}

static void sun6i_dsi_inst_abort(struct sun6i_dsi_priv *dsi)
{
	clrbits_le32(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG, SUN6I_DSI_BASIC_CTL0_INST_ST);
}

static void sun6i_dsi_inst_commit(struct sun6i_dsi_priv *dsi)
{
	setbits_le32(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG, SUN6I_DSI_BASIC_CTL0_INST_ST);
}

static int sun6i_dsi_inst_wait_for_completion(struct sun6i_dsi_priv *dsi)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG,
				  val, !(val & SUN6I_DSI_BASIC_CTL0_INST_ST),
				  100000);
	if (ret)
		printf("DSI: Instruction completion timeout! (CTL0: 0x%08x)\n", val);
	return ret;
}

static void sun6i_dsi_inst_setup(struct sun6i_dsi_priv *dsi,
				 enum sun6i_dsi_inst_id id,
				 enum sun6i_dsi_inst_mode mode,
				 bool clock, u8 data,
				 enum sun6i_dsi_inst_packet packet,
				 enum sun6i_dsi_inst_escape escape)
{
	writel(SUN6I_DSI_INST_FUNC_INST_MODE(mode) |
	       SUN6I_DSI_INST_FUNC_ESCAPE_ENTRY(escape) |
	       SUN6I_DSI_INST_FUNC_TRANS_PACKET(packet) |
	       (clock ? SUN6I_DSI_INST_FUNC_LANE_CEN : 0) |
	       SUN6I_DSI_INST_FUNC_LANE_DEN(data),
	       dsi->regs + SUN6I_DSI_INST_FUNC_REG(id));
}

static void sun6i_dsi_inst_init(struct sun6i_dsi_priv *dsi,
				struct mipi_dsi_device *device)
{
	u8 lanes_mask = BIT(device->lanes) - 1;

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LP11, DSI_INST_MODE_STOP,
			     true, lanes_mask, 0, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_TBA, DSI_INST_MODE_TBA,
			     false, 1, 0, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSC, DSI_INST_MODE_HS,
			     true, 0, DSI_INST_PACK_PIXEL, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSD, DSI_INST_MODE_HS,
			     false, lanes_mask, DSI_INST_PACK_PIXEL, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LPDT, DSI_INST_MODE_ESCAPE,
			     false, 1, DSI_INST_PACK_COMMAND,
			     DSI_INST_ESCA_LPDT);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSCEXIT, DSI_INST_MODE_HSCEXIT,
			     true, 0, 0, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_NOP, DSI_INST_MODE_STOP,
			     false, lanes_mask, 0, 0);

	sun6i_dsi_inst_setup(dsi, DSI_INST_ID_DLY, DSI_INST_MODE_NOP,
			     true, lanes_mask, 0, 0);

	writel(SUN6I_DSI_INST_JUMP_CFG_POINT(DSI_INST_ID_NOP) |
	       SUN6I_DSI_INST_JUMP_CFG_TO(DSI_INST_ID_HSCEXIT) |
	       SUN6I_DSI_INST_JUMP_CFG_NUM(1),
	       dsi->regs + SUN6I_DSI_INST_JUMP_CFG_REG(0));
}

static u16 sun6i_dsi_get_video_start_delay(struct sun6i_dsi_priv *dsi,
					   struct display_timing *timing)
{
	u32 vtotal = timing->vactive.typ + timing->vfront_porch.typ +
		     timing->vback_porch.typ + timing->vsync_len.typ;
	u32 vactive_start = timing->vactive.typ + timing->vfront_porch.typ;
	u16 delay = vtotal - (vactive_start - timing->vactive.typ) + 1;

	if (delay > vtotal)
		delay = delay % vtotal;

	return max((u16)delay, (u16)1);
}

static void sun6i_dsi_setup_burst(struct sun6i_dsi_priv *dsi,
				  struct display_timing *timing)
{
	struct mipi_dsi_device *device = dsi->device;
	u32 val = 0;

	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		unsigned int Bpp = mipi_dsi_pixel_format_to_bpp(device->format) / 8;
		u32 htotal = timing->hactive.typ + timing->hfront_porch.typ +
			     timing->hback_porch.typ + timing->hsync_len.typ;
		u16 line_num = htotal * Bpp / device->lanes;
		u16 edge0, edge1;
		u32 hbp = htotal - (timing->hactive.typ + timing->hfront_porch.typ);

		edge1 = SUN6I_DSI_SYNC_POINT;
		edge1 += (timing->hactive.typ + hbp + 20) * Bpp / device->lanes;
		if (edge1 > line_num)
			edge1 = line_num;

		edge0 = edge1;
		edge0 += (timing->hactive.typ + 40) * SUN6I_DSI_TCON_DIV / 8;
		if (edge0 > line_num)
			edge0 = edge0 - line_num;
		else
			edge0 = 1;

		writel(SUN6I_DSI_BURST_DRQ_EDGE0(edge0) |
		       SUN6I_DSI_BURST_DRQ_EDGE1(edge1),
		       dsi->regs + SUN6I_DSI_BURST_DRQ_REG);

		writel(SUN6I_DSI_BURST_LINE_NUM(line_num) |
		       SUN6I_DSI_BURST_LINE_SYNC_POINT(SUN6I_DSI_SYNC_POINT),
		       dsi->regs + SUN6I_DSI_BURST_LINE_REG);

		val = SUN6I_DSI_TCON_DRQ_ENABLE_MODE;
	} else if (timing->hfront_porch.typ > 20) {
		u16 drq = timing->hfront_porch.typ - 20;

		drq *= mipi_dsi_pixel_format_to_bpp(device->format);
		drq /= 32;

		val = (SUN6I_DSI_TCON_DRQ_ENABLE_MODE |
		       SUN6I_DSI_TCON_DRQ_SET(drq));
	}

	writel(val, dsi->regs + SUN6I_DSI_TCON_DRQ_REG);
}

static void sun6i_dsi_setup_inst_loop(struct sun6i_dsi_priv *dsi,
				      struct display_timing *timing)
{
	struct mipi_dsi_device *device = dsi->device;
	u16 delay = 50 - 1;

	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		u32 htotal = timing->hactive.typ + timing->hfront_porch.typ +
			     timing->hback_porch.typ + timing->hsync_len.typ;
		u32 hsync_porch = (htotal - timing->hactive.typ) * 150;

		delay = (hsync_porch / ((timing->pixelclock.typ / 1000) * 8));
		if (delay > 50)
			delay -= 50;
		else
			delay = 0;
	}

	writel(2 << (4 * DSI_INST_ID_LP11) |
	       3 << (4 * DSI_INST_ID_DLY),
	       dsi->regs + SUN6I_DSI_INST_LOOP_SEL_REG);

	writel(SUN6I_DSI_INST_LOOP_NUM_N0(50 - 1) |
	       SUN6I_DSI_INST_LOOP_NUM_N1(delay),
	       dsi->regs + SUN6I_DSI_INST_LOOP_NUM_REG(0));
	writel(SUN6I_DSI_INST_LOOP_NUM_N0(50 - 1) |
	       SUN6I_DSI_INST_LOOP_NUM_N1(delay),
	       dsi->regs + SUN6I_DSI_INST_LOOP_NUM_REG(1));
}

static void sun6i_dsi_setup_format(struct sun6i_dsi_priv *dsi,
				   struct display_timing *timing)
{
	struct mipi_dsi_device *device = dsi->device;
	u32 val = SUN6I_DSI_PIXEL_PH_VC(device->channel);
	u8 dt, fmt;
	u16 wc;

	switch (device->format) {
	case MIPI_DSI_FMT_RGB888:
		dt = MIPI_DSI_PACKED_PIXEL_STREAM_24;
		fmt = 8;
		break;
	case MIPI_DSI_FMT_RGB666:
		dt = MIPI_DSI_PIXEL_STREAM_3BYTE_18;
		fmt = 9;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		dt = MIPI_DSI_PACKED_PIXEL_STREAM_18;
		fmt = 10;
		break;
	case MIPI_DSI_FMT_RGB565:
		dt = MIPI_DSI_PACKED_PIXEL_STREAM_16;
		fmt = 11;
		break;
	default:
		return;
	}
	val |= SUN6I_DSI_PIXEL_PH_DT(dt);

	wc = timing->hactive.typ * mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	val |= SUN6I_DSI_PIXEL_PH_WC(wc);
	val |= sun6i_dsi_ecc_compute(val) << 24;

	writel(val, dsi->regs + SUN6I_DSI_PIXEL_PH_REG);

	writel(SUN6I_DSI_PIXEL_PF0_CRC_FORCE(0xffff),
	       dsi->regs + SUN6I_DSI_PIXEL_PF0_REG);

	writel(SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINE0(0xffff) |
	       SUN6I_DSI_PIXEL_PF1_CRC_INIT_LINEN(0xffff),
	       dsi->regs + SUN6I_DSI_PIXEL_PF1_REG);

	writel(SUN6I_DSI_PIXEL_CTL0_PD_PLUG_DISABLE |
	       SUN6I_DSI_PIXEL_CTL0_FORMAT(fmt),
	       dsi->regs + SUN6I_DSI_PIXEL_CTL0_REG);
}

static void sun6i_dsi_setup_timings(struct sun6i_dsi_priv *dsi,
				    struct display_timing *timing)
{
	struct mipi_dsi_device *device = dsi->device;
	int Bpp = mipi_dsi_pixel_format_to_bpp(device->format) / 8;
	u16 hbp = 0, hfp = 0, hsa = 0, hblk = 0;
	u32 basic_ctl = 0;
	u8 buffer[256]; /* Max blanking size */
	u16 crc;

	if (device->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
		hblk = timing->hactive.typ * Bpp;
		basic_ctl = SUN6I_DSI_BASIC_CTL_VIDEO_BURST |
			    SUN6I_DSI_BASIC_CTL_HSA_HSE_DIS |
			    SUN6I_DSI_BASIC_CTL_HBP_DIS;

		if (device->lanes == 4)
			basic_ctl |= SUN6I_DSI_BASIC_CTL_TRAIL_FILL |
				     SUN6I_DSI_BASIC_CTL_TRAIL_INV(0xc);
	} else {
		hsa = max(10, (int)(timing->hsync_len.typ * Bpp - 10));
		hbp = max(6, (int)(timing->hback_porch.typ * Bpp - 6));
		hfp = max(16, (int)(timing->hfront_porch.typ * Bpp - 16));
		hblk = max(10, (int)((timing->hactive.typ + timing->hfront_porch.typ + timing->hback_porch.typ + timing->hsync_len.typ - timing->hsync_len.typ) * Bpp - 10));
	}

	writel(basic_ctl, dsi->regs + SUN6I_DSI_BASIC_CTL_REG);

	writel(sun6i_dsi_build_sync_pkt(MIPI_DSI_H_SYNC_START,
					device->channel,
					0, 0),
	       dsi->regs + SUN6I_DSI_SYNC_HSS_REG);

	writel(sun6i_dsi_build_sync_pkt(MIPI_DSI_H_SYNC_END,
					device->channel,
					0, 0),
	       dsi->regs + SUN6I_DSI_SYNC_HSE_REG);

	writel(sun6i_dsi_build_sync_pkt(MIPI_DSI_V_SYNC_START,
					device->channel,
					0, 0),
	       dsi->regs + SUN6I_DSI_SYNC_VSS_REG);

	writel(sun6i_dsi_build_sync_pkt(MIPI_DSI_V_SYNC_END,
					device->channel,
					0, 0),
	       dsi->regs + SUN6I_DSI_SYNC_VSE_REG);

	writel(SUN6I_DSI_BASIC_SIZE0_VSA(timing->vsync_len.typ) |
	       SUN6I_DSI_BASIC_SIZE0_VBP(timing->vback_porch.typ),
	       dsi->regs + SUN6I_DSI_BASIC_SIZE0_REG);

	writel(SUN6I_DSI_BASIC_SIZE1_VACT(timing->vactive.typ) |
	       SUN6I_DSI_BASIC_SIZE1_VT(timing->vactive.typ + timing->vfront_porch.typ + timing->vback_porch.typ + timing->vsync_len.typ),
	       dsi->regs + SUN6I_DSI_BASIC_SIZE1_REG);

	/* Prepare a blank buffer for CRC calculation */
	memset(buffer, 0, sizeof(buffer));

	/* sync */
	crc = sun6i_dsi_crc_compute(buffer, hsa);
	writel(sun6i_dsi_build_blk0_pkt(device->channel, hsa), dsi->regs + SUN6I_DSI_BLK_HSA0_REG);
	writel(sun6i_dsi_build_blk1_pkt(0, crc, hsa), dsi->regs + SUN6I_DSI_BLK_HSA1_REG);

	/* backporch */
	crc = sun6i_dsi_crc_compute(buffer, hbp);
	writel(sun6i_dsi_build_blk0_pkt(device->channel, hbp), dsi->regs + SUN6I_DSI_BLK_HBP0_REG);
	writel(sun6i_dsi_build_blk1_pkt(0, crc, hbp), dsi->regs + SUN6I_DSI_BLK_HBP1_REG);

	/* frontporch */
	crc = sun6i_dsi_crc_compute(buffer, hfp);
	writel(sun6i_dsi_build_blk0_pkt(device->channel, hfp), dsi->regs + SUN6I_DSI_BLK_HFP0_REG);
	writel(sun6i_dsi_build_blk1_pkt(0, crc, hfp), dsi->regs + SUN6I_DSI_BLK_HFP1_REG);

	/* hblk */
	crc = sun6i_dsi_crc_compute(buffer, hblk);
	writel(sun6i_dsi_build_blk0_pkt(device->channel, hblk), dsi->regs + SUN6I_DSI_BLK_HBLK0_REG);
	writel(sun6i_dsi_build_blk1_pkt(0, crc, hblk), dsi->regs + SUN6I_DSI_BLK_HBLK1_REG);

	/* vblk */
	writel(sun6i_dsi_build_blk0_pkt(device->channel, 0), dsi->regs + SUN6I_DSI_BLK_VBLK0_REG);
	writel(sun6i_dsi_build_blk1_pkt(0, 0xffff, 0), dsi->regs + SUN6I_DSI_BLK_VBLK1_REG);
}

static int sun6i_dsi_start(struct sun6i_dsi_priv *dsi,
			   enum sun6i_dsi_start_inst func)
{
	switch (func) {
	case DSI_START_LPTX:
		writel(DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
		       DSI_INST_ID_END  << (4 * DSI_INST_ID_LPDT),
		       dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG);
		break;
	case DSI_START_LPRX:
		writel(DSI_INST_ID_LPDT << (4 * DSI_INST_ID_LP11) |
		       DSI_INST_ID_DLY  << (4 * DSI_INST_ID_LPDT) |
		       DSI_INST_ID_TBA  << (4 * DSI_INST_ID_DLY) |
		       DSI_INST_ID_END  << (4 * DSI_INST_ID_TBA),
		       dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG);
		break;
	case DSI_START_HSC:
		writel(DSI_INST_ID_HSC  << (4 * DSI_INST_ID_LP11) |
		       DSI_INST_ID_END  << (4 * DSI_INST_ID_HSC),
		       dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG);
		break;
	case DSI_START_HSD:
		writel(DSI_INST_ID_NOP  << (4 * DSI_INST_ID_LP11) |
		       DSI_INST_ID_HSD  << (4 * DSI_INST_ID_NOP) |
		       DSI_INST_ID_DLY  << (4 * DSI_INST_ID_HSD) |
		       DSI_INST_ID_NOP  << (4 * DSI_INST_ID_DLY) |
		       DSI_INST_ID_END  << (4 * DSI_INST_ID_HSCEXIT),
		       dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG);
		break;
	default:
		writel(DSI_INST_ID_END  << (4 * DSI_INST_ID_LP11),
		       dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG);
		break;
	}

	sun6i_dsi_inst_abort(dsi);
	sun6i_dsi_inst_commit(dsi);

	if (func == DSI_START_HSC)
		clrbits_le32(dsi->regs + SUN6I_DSI_INST_FUNC_REG(DSI_INST_ID_LP11),
			     SUN6I_DSI_INST_FUNC_LANE_CEN);

	return 0;
}

static u32 sun6i_dsi_dcs_build_pkt_hdr(struct sun6i_dsi_priv *dsi,
				       const struct mipi_dsi_msg *msg)
{
	u32 pkt = msg->type;

	if (msg->type == MIPI_DSI_DCS_LONG_WRITE) {
		pkt |= ((msg->tx_len) & 0xffff) << 8;
		pkt |= (((msg->tx_len) >> 8) & 0xffff) << 16;
	} else {
		pkt |= (((u8 *)msg->tx_buf)[0] << 8);
		if (msg->tx_len > 1)
			pkt |= (((u8 *)msg->tx_buf)[1] << 16);
	}

	pkt |= sun6i_dsi_ecc_compute(pkt) << 24;

	return pkt;
}

static int sun6i_dsi_dcs_write_short(struct sun6i_dsi_priv *dsi,
				     const struct mipi_dsi_msg *msg)
{
	int ret;
	u32 pkt = sun6i_dsi_dcs_build_pkt_hdr(dsi, msg);

	debug("DSI_LP: SHORT type=0x%02x len=%zu pkt=0x%08x data=",
	      msg->type, msg->tx_len, pkt);
	if (msg->tx_buf && msg->tx_len > 0) {
		const u8 *data = msg->tx_buf;
		for (size_t i = 0; i < msg->tx_len && i < 4; i++)
			debug("%02x ", data[i]);
	}
	debug("\n");

	writel(pkt, dsi->regs + SUN6I_DSI_CMD_TX_REG(0));
	clrsetbits_le32(dsi->regs + SUN6I_DSI_CMD_CTL_REG, 0xff, (4 - 1));

	sun6i_dsi_start(dsi, DSI_START_LPTX);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		printf("DSI: write_short: wait for completion failed: %d\n", ret);
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	return msg->tx_len;
}

static int sun6i_dsi_dcs_write_long(struct sun6i_dsi_priv *dsi,
				    const struct mipi_dsi_msg *msg)
{
	int ret, len = 0;
	u32 val;
	u8 *tx_buf = (u8 *)msg->tx_buf;
	u16 crc;
	u8 bounce[256]; /* DCS long packets are usually small */

	debug("DSI_LP: LONG type=0x%02x len=%zu data=", msg->type, msg->tx_len);
	if (tx_buf && msg->tx_len > 0) {
		for (size_t i = 0; i < msg->tx_len && i < 16; i++)
			debug("%02x ", tx_buf[i]);
		if (msg->tx_len > 16)
			debug("...");
	}
	debug("\n");

	if (msg->tx_len + 2 > sizeof(bounce))
		return -EINVAL;

	writel(sun6i_dsi_dcs_build_pkt_hdr(dsi, msg),
	       dsi->regs + SUN6I_DSI_CMD_TX_REG(0));

	memcpy(bounce, tx_buf, msg->tx_len);
	crc = sun6i_dsi_crc_compute(tx_buf, msg->tx_len);
	memcpy(bounce + msg->tx_len, &crc, sizeof(crc));
	len = msg->tx_len + sizeof(crc);

	for (int i = 0; i < len; i += 4) {
		val = 0;
		memcpy(&val, bounce + i, min((size_t)4, (size_t)(len - i)));
		writel(val, dsi->regs + SUN6I_DSI_CMD_TX_REG(1 + i / 4));
	}

	clrsetbits_le32(dsi->regs + SUN6I_DSI_CMD_CTL_REG, 0xfff, len + 4 - 1);

	sun6i_dsi_start(dsi, DSI_START_LPTX);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	return msg->tx_len;
}

static int sun6i_dsi_dcs_read(struct sun6i_dsi_priv *dsi,
			      const struct mipi_dsi_msg *msg)
{
	u32 val;
	int ret;
	u8 byte0;

	writel(sun6i_dsi_dcs_build_pkt_hdr(dsi, msg),
	       dsi->regs + SUN6I_DSI_CMD_TX_REG(0));
	writel((4 - 1), dsi->regs + SUN6I_DSI_CMD_CTL_REG);

	sun6i_dsi_start(dsi, DSI_START_LPRX);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		sun6i_dsi_inst_abort(dsi);
		return ret;
	}

	val = readl(dsi->regs + SUN6I_DSI_CMD_CTL_REG);
	if (val & SUN6I_DSI_CMD_CTL_RX_OVERFLOW)
		return -EIO;

	val = readl(dsi->regs + SUN6I_DSI_CMD_RX_REG(0));
	byte0 = val & 0xff;
	if (byte0 == MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT)
		return -EIO;

	((u8 *)msg->rx_buf)[0] = (val >> 8);

	return 1;
}

static ssize_t sun6i_dsi_transfer(struct mipi_dsi_host *host,
				  const struct mipi_dsi_msg *msg)
{
	struct sun6i_dsi_priv *dsi = host_to_sun6i_dsi(host);
	int ret;

	debug("DSI: transfer type=0x%02x len=%zu\n", msg->type, msg->tx_len);

	ret = sun6i_dsi_inst_wait_for_completion(dsi);
	if (ret < 0) {
		printf("DSI: transfer: wait for completion failed: %d\n", ret);
		sun6i_dsi_inst_abort(dsi);
	}

	writel(SUN6I_DSI_CMD_CTL_RX_OVERFLOW |
	       SUN6I_DSI_CMD_CTL_RX_FLAG |
	       SUN6I_DSI_CMD_CTL_TX_FLAG,
	       dsi->regs + SUN6I_DSI_CMD_CTL_REG);

	switch (msg->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
		ret = sun6i_dsi_dcs_write_short(dsi, msg);
		break;

	case MIPI_DSI_DCS_LONG_WRITE:
		ret = sun6i_dsi_dcs_write_long(dsi, msg);
		break;

	case MIPI_DSI_DCS_READ:
		if (msg->rx_len == 1) {
			ret = sun6i_dsi_dcs_read(dsi, msg);
			break;
		}
		/* Fall through */

	default:
		printf("DSI: transfer: unsupported message type 0x%02x\n", msg->type);
		ret = -EINVAL;
	}

	if (ret < 0)
		debug("DSI: transfer failed: %d\n", ret);

	return ret;
}

static int sun6i_dsi_attach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct sun6i_dsi_priv *dsi = host_to_sun6i_dsi(host);

	dsi->device = device;
	return 0;
}

static int sun6i_dsi_detach(struct mipi_dsi_host *host,
			    struct mipi_dsi_device *device)
{
	struct sun6i_dsi_priv *dsi = host_to_sun6i_dsi(host);

	dsi->device = NULL;
	return 0;
}

static const struct mipi_dsi_host_ops sun6i_dsi_mipi_host_ops = {
	.attach		= sun6i_dsi_attach,
	.detach		= sun6i_dsi_detach,
	.transfer	= sun6i_dsi_transfer,
};

static int sun6i_dsi_init(struct udevice *dev,
			  struct mipi_dsi_device *device,
			  struct display_timing *timings,
			  unsigned int max_data_lanes,
			  const struct mipi_dsi_phy_ops *phy_ops)
{
	struct sun6i_dsi_priv *dsi = dev_get_priv(dev);
	u16 delay;

	printf("DSI: init starting\n");
	dsi->device = device;
	memcpy(&dsi->timing, timings, sizeof(struct display_timing));
	dsi->host.dev = (struct device *)dev;
	dsi->host.ops = &sun6i_dsi_mipi_host_ops;
	device->host = &dsi->host;

	if (dsi->vcc_dsi) {
		printf("DSI: Enabling vcc-dsi-supply...\n");
		regulator_set_enable(dsi->vcc_dsi, true);
		mdelay(10);
	}

	printf("DSI: enabling block...\n");
	writel(SUN6I_DSI_CTL_EN, dsi->regs + SUN6I_DSI_CTL_REG);

	writel(SUN6I_DSI_BASIC_CTL0_ECC_EN | SUN6I_DSI_BASIC_CTL0_CRC_EN,
	       dsi->regs + SUN6I_DSI_BASIC_CTL0_REG);

	writel(10, dsi->regs + SUN6I_DSI_TRANS_START_REG);
	writel(0, dsi->regs + SUN6I_DSI_TRANS_ZERO_REG);

	printf("DSI: inst_init...\n");
	sun6i_dsi_inst_init(dsi, device);

	writel(0xff, dsi->regs + SUN6I_DSI_DEBUG_DATA_REG);

	delay = sun6i_dsi_get_video_start_delay(dsi, timings);
	writel(SUN6I_DSI_BASIC_CTL1_VIDEO_ST_DELAY(delay) |
	       SUN6I_DSI_BASIC_CTL1_VIDEO_FILL |
	       SUN6I_DSI_BASIC_CTL1_VIDEO_PRECISION |
	       SUN6I_DSI_BASIC_CTL1_VIDEO_MODE,
	       dsi->regs + SUN6I_DSI_BASIC_CTL1_REG);

	printf("DSI: setup_burst...\n");
	sun6i_dsi_setup_burst(dsi, timings);
	printf("DSI: setup_inst_loop...\n");
	sun6i_dsi_setup_inst_loop(dsi, timings);
	printf("DSI: setup_format...\n");
	sun6i_dsi_setup_format(dsi, timings);
	printf("DSI: setup_timings...\n");
	sun6i_dsi_setup_timings(dsi, timings);

	/* PHY initialization */
	struct phy_configure_opts_mipi_dphy cfg = {0};
	int ret;

	ret = phy_mipi_dphy_get_default_config(timings->pixelclock.typ,
					       mipi_dsi_pixel_format_to_bpp(device->format),
					       device->lanes, &cfg);
	if (ret) {
		printf("DSI: Failed to get default DPHY config: %d\n", ret);
		return ret;
	}

	printf("DSI: Configuring DPHY with %d lanes, bitrate %lu Hz\n", cfg.lanes, cfg.hs_clk_rate);
	generic_phy_init(&dsi->dphy);
	generic_phy_set_mode(&dsi->dphy, PHY_MODE_MIPI_DPHY, 0);
	ret = generic_phy_configure(&dsi->dphy, &cfg);
	if (ret)
		printf("DSI: generic_phy_configure failed: %d\n", ret);
	generic_phy_power_on(&dsi->dphy);

	printf("DSI: init done\n");
	return 0;
}

static int sun6i_dsi_enable(struct udevice *dev)
{
	struct sun6i_dsi_priv *dsi = dev_get_priv(dev);

	printf("DSI: enable starting (starting HSC then HSD)...\n");
	sun6i_dsi_start(dsi, DSI_START_HSC);
	udelay(1000);
	sun6i_dsi_start(dsi, DSI_START_HSD);

	printf("DSI: HSD started. Immediate register dump:\n");
	printf("DSI: CTL=0x%08x BASIC_CTL=0x%08x BASIC_CTL0=0x%08x BASIC_CTL1=0x%08x\n",
	       readl(dsi->regs + SUN6I_DSI_CTL_REG),
	       readl(dsi->regs + SUN6I_DSI_BASIC_CTL_REG),
	       readl(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG),
	       readl(dsi->regs + SUN6I_DSI_BASIC_CTL1_REG));
	printf("DSI: INST_JUMP_SEL=0x%08x INST_JUMP_CFG=0x%08x\n",
	       readl(dsi->regs + SUN6I_DSI_INST_JUMP_SEL_REG),
	       readl(dsi->regs + SUN6I_DSI_INST_JUMP_CFG_REG(0)));

	/* Wait 500ms then re-read to check if INST_ST (bit 0) is still set.
	 * If BASIC_CTL0 bit 0 = 0 after delay, the HSD loop terminated. */
	mdelay(500);
	{
		u32 ctl0 = readl(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG);
		u32 ctl1 = readl(dsi->regs + SUN6I_DSI_BASIC_CTL1_REG);
		printf("DSI: After 500ms: BASIC_CTL0=0x%08x BASIC_CTL1=0x%08x\n",
		       ctl0, ctl1);
		if (ctl0 & SUN6I_DSI_BASIC_CTL0_INST_ST)
			printf("DSI: INST_ST=1 -> HSD loop still running (good)\n");
		else
			printf("DSI: INST_ST=0 -> HSD loop TERMINATED (pipeline broken!)\n");
	}

	return 0;
}

static const struct dsi_host_ops sun6i_dsi_ops = {
	.init = sun6i_dsi_init,
	.enable = sun6i_dsi_enable,
};

static int sun6i_dsi_probe(struct udevice *dev)
{
	struct sun6i_dsi_priv *dsi = dev_get_priv(dev);
	int ret;

	printf("DSI: Probing %s...\n", dev->name);

	dsi->variant = (const struct sun6i_dsi_variant *)dev_get_driver_data(dev);

	dsi->regs = dev_read_addr_ptr(dev);
	if (!dsi->regs) {
		printf("DSI: Failed to get register address\n");
		return -EINVAL;
	}

	if (dsi->variant->has_mod_clk) {
#ifdef CONFIG_SUNXI_GEN_NCAT2
		/*
		 * T113-S/D1: The DSI controller's mod clock comes from
		 * TCON_TOP's DSI gate (CLK_TCON_TOP_DSI), which passes
		 * through CLK_TCON_LCD0. This is already enabled by
		 * sunxi_tcon_top_setup(). Register 0xb24 is the D-PHY
		 * mod clock, NOT the DSI controller mod clock — that is
		 * handled by the D-PHY driver.
		 */
#else
		ret = clk_get_by_name(dev, "mod", &dsi->mod_clk);
		if (!ret) {
			if (dsi->variant->set_mod_clk)
				clk_set_rate(&dsi->mod_clk, 297000000);
			clk_enable(&dsi->mod_clk);
		}
#endif
	}

#ifdef CONFIG_SUNXI_GEN_NCAT2
	/* T113-S/D1: DSI BUS gate/reset at 0xb4c */
	printf("DSI: Enabling bus gate/reset (NCAT2)...\n");
	setbits_le32((u8 *)SUNXI_CCM_BASE + 0xb4c, BIT(16) | BIT(0));
#endif

	if (dsi->variant->has_mod_clk)
		ret = clk_get_by_name(dev, "bus", &dsi->bus_clk);
	else
		ret = clk_get_by_index(dev, 0, &dsi->bus_clk);

	if (ret) {
		printf("DSI: Failed to get bus clock: %d\n", ret);
		/* Don't return error yet for NCAT2 as we handle it above */
#ifndef CONFIG_SUNXI_GEN_NCAT2
		return ret;
#endif
	}

	ret = clk_enable(&dsi->bus_clk);
	if (ret) {
		printf("DSI: Failed to enable bus clock: %d\n", ret);
#ifndef CONFIG_SUNXI_GEN_NCAT2
		return ret;
#endif
	}

	ret = reset_get_by_index(dev, 0, &dsi->reset);
	if (!ret) {
		printf("DSI: De-asserting reset...\n");
		reset_deassert(&dsi->reset);
	}

	ret = generic_phy_get_by_name(dev, "dphy", &dsi->dphy);
	if (ret) {
		printf("DSI: Failed to get dphy: %d\n", ret);
		return ret;
	}

	ret = device_get_supply_regulator(dev, "vcc-dsi-supply", &dsi->vcc_dsi);
	if (ret && ret != -ENOENT) {
		printf("DSI: Failed to get vcc-dsi-supply: %d\n", ret);
		return ret;
	}

	printf("DSI: Probe successful.\n");
	return 0;
}

static const struct sun6i_dsi_variant sun6i_a31_mipi_dsi_variant = {
	.has_mod_clk	= true,
	.set_mod_clk	= true,
};

static const struct sun6i_dsi_variant sun50i_a64_mipi_dsi_variant = {
};

static const struct sun6i_dsi_variant sun50i_a100_mipi_dsi_variant = {
	.has_mod_clk	= true,
};

static const struct udevice_id sun6i_dsi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-mipi-dsi", .data = (ulong)&sun6i_a31_mipi_dsi_variant },
	{ .compatible = "allwinner,sun50i-a64-mipi-dsi", .data = (ulong)&sun50i_a64_mipi_dsi_variant },
	{ .compatible = "allwinner,sun50i-a100-mipi-dsi", .data = (ulong)&sun50i_a100_mipi_dsi_variant },
	{ .compatible = "allwinner,sun20i-d1-mipi-dsi", .data = (ulong)&sun50i_a100_mipi_dsi_variant },
	{ }
};

static int sun6i_dsi_bind(struct udevice *dev)
{
	return dm_scan_fdt_dev(dev);
}

U_BOOT_DRIVER(sun6i_mipi_dsi) = {
	.name		= "sun6i_mipi_dsi",
	.id		= UCLASS_DSI_HOST,
	.of_match	= sun6i_dsi_ids,
	.bind		= sun6i_dsi_bind,
	.probe		= sun6i_dsi_probe,
	.ops		= &sun6i_dsi_ops,
	.priv_auto	= sizeof(struct sun6i_dsi_priv),
};
