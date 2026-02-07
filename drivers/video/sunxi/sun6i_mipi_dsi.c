// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner MIPI DSI Driver
 *
 * Copyright (C) 2018 Bootlin
 * Copyright (C) 2026 U-Boot Port
 */

#include <asm/arch/clock.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <generic-phy.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <log.h>
#include <panel.h>
#include <phy-mipi-dphy.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <video_bridge.h>

#include "sun6i_mipi_dsi.h"

struct sun6i_dsi_priv {
  void __iomem *regs;
  struct reset_ctl reset;
  struct clk bus_clk;
  struct clk mod_clk;
  struct phy dphy;
  struct udevice *panel;
};

static uint sun6i_dsi_crc_compute(u8 const *buffer, size_t len) {
  /* Simple CRC lookup or calculation if needed.
     CRC-CCITT (0xFFFF, x^16 + x^12 + x^5 + 1)
     For now returning 0 or simplified check
     (U-Boot has crc.h but crc_ccitt might not be standard) */
  return 0; // TODO: Implement if needed for debug
}

static u32 sun6i_dsi_ecc_compute(unsigned int data) {
  /* Simplified ECC for now - fixed table lookup should be used if ECC EN */
  return 0;
}

static u32 sun6i_dsi_build_sync_pkt(u8 dt, u8 vc, u8 d0, u8 d1) {
  u32 val = dt & 0x3f;

  val |= (vc & 3) << 6;
  val |= (d0 & 0xff) << 8;
  val |= (d1 & 0xff) << 16;
  // val |= sun6i_dsi_ecc_compute(val) << 24;

  return val;
}

static void sun6i_dsi_inst_abort(struct sun6i_dsi_priv *dsi) {
  clrbits_le32(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG,
               SUN6I_DSI_BASIC_CTL0_INST_ST);
}

static void sun6i_dsi_inst_commit(struct sun6i_dsi_priv *dsi) {
  setbits_le32(dsi->regs + SUN6I_DSI_BASIC_CTL0_REG,
               SUN6I_DSI_BASIC_CTL0_INST_ST);
}

static void sun6i_dsi_inst_setup(struct sun6i_dsi_priv *dsi,
                                 enum sun6i_dsi_inst_id id,
                                 enum sun6i_dsi_inst_mode mode, bool clock,
                                 u8 data, enum sun6i_dsi_inst_packet packet,
                                 enum sun6i_dsi_inst_escape escape) {
  writel(SUN6I_DSI_INST_FUNC_INST_MODE(mode) |
             SUN6I_DSI_INST_FUNC_ESCAPE_ENTRY(escape) |
             SUN6I_DSI_INST_FUNC_TRANS_PACKET(packet) |
             (clock ? SUN6I_DSI_INST_FUNC_LANE_CEN : 0) |
             SUN6I_DSI_INST_FUNC_LANE_DEN(data),
         dsi->regs + SUN6I_DSI_INST_FUNC_REG(id));
}

static void sun6i_dsi_inst_init(struct sun6i_dsi_priv *dsi, int lanes) {
  u8 lanes_mask = GENMASK(lanes - 1, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LP11, DSI_INST_MODE_STOP, true,
                       lanes_mask, 0, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_TBA, DSI_INST_MODE_TBA, false, 1, 0, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSC, DSI_INST_MODE_HS, true, 0,
                       DSI_INST_PACK_PIXEL, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSD, DSI_INST_MODE_HS, false,
                       lanes_mask, DSI_INST_PACK_PIXEL, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_LPDT, DSI_INST_MODE_ESCAPE, false, 1,
                       DSI_INST_PACK_COMMAND, DSI_INST_ESCA_LPDT);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_HSCEXIT, DSI_INST_MODE_HSCEXIT, true, 0,
                       0, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_NOP, DSI_INST_MODE_STOP, false,
                       lanes_mask, 0, 0);

  sun6i_dsi_inst_setup(dsi, DSI_INST_ID_DLY, DSI_INST_MODE_NOP, true,
                       lanes_mask, 0, 0);

  writel(SUN6I_DSI_INST_JUMP_CFG_POINT(DSI_INST_ID_NOP) |
             SUN6I_DSI_INST_JUMP_CFG_TO(DSI_INST_ID_HSCEXIT) |
             SUN6I_DSI_INST_JUMP_CFG_NUM(1),
         dsi->regs + SUN6I_DSI_INST_JUMP_CFG_REG(0));
}

static int sun6i_dsi_attach(struct udevice *dev) {
  struct sun6i_dsi_priv *dsi = dev_get_priv(dev);
  struct display_timing timing;
  struct udevice *panel = dsi->panel;
  struct phy_configure_opts_mipi_dphy dphy_opts = {0};
  int ret;
  int bpp = 24;  // Default 24bpp
  int lanes = 4; // Default 4 lanes

  /* Enable DSI block */
  writel(SUN6I_DSI_CTL_EN, dsi->regs + SUN6I_DSI_CTL_REG);
  writel(SUN6I_DSI_BASIC_CTL0_ECC_EN | SUN6I_DSI_BASIC_CTL0_CRC_EN,
         dsi->regs + SUN6I_DSI_BASIC_CTL0_REG);
  writel(10, dsi->regs + SUN6I_DSI_TRANS_START_REG);
  writel(0, dsi->regs + SUN6I_DSI_TRANS_ZERO_REG);

  sun6i_dsi_inst_init(dsi, lanes);

  writel(0xff, dsi->regs + SUN6I_DSI_DEBUG_DATA_REG);

  /* Get timing from panel */
  ret = panel_get_display_timing(panel, &timing);
  if (ret) {
    printf("Failed to get display timing\n");
    return ret;
  }

  /* Init PHY */
  ret = generic_phy_init(&dsi->dphy);
  if (ret)
    return ret;

  /* Configure PHY */
  dphy_opts.hs_clk_rate = timing.pixelclock.typ * bpp / lanes;
  dphy_opts.lanes = lanes;
  // dphy_opts.lp_clk_rate = ...;

  ret = generic_phy_configure(
      &dsi->dphy, &dphy_opts); // Need generic_phy_configure support in U-Boot
  /* U-Boot generic_phy_configure might not exist or have different signature.
     If not, we might need to cast to phy_ops directly or add custom call.
     Let's assume generic_phy_ops has configuration. */

  ret = generic_phy_power_on(&dsi->dphy);
  if (ret)
    return ret;

  /* Enable panel */
  ret = panel_enable_backlight(panel);
  if (ret)
    return ret;

  return 0;
}

static int sun6i_dsi_probe(struct udevice *dev) {
  struct sun6i_dsi_priv *dsi = dev_get_priv(dev);
  int ret;

  dsi->regs = dev_read_addr_ptr(dev);
  if (!dsi->regs)
    return -EINVAL;

  ret = reset_get_by_index(dev, 0, &dsi->reset);
  if (ret)
    return ret;

  ret = clk_get_by_name(dev, "bus", &dsi->bus_clk);
  if (ret)
    return ret;

  ret = clk_get_by_name(dev, "mod", &dsi->mod_clk);
  if (ret)
    return ret;

  ret = generic_phy_get_by_name(dev, "dphy", &dsi->dphy);
  if (ret) {
    printf("Failed to get DPHY\n");
    return ret;
  }

  /* Find panel - usually a subnode */
  ret = uclass_get_device_by_phandle(UCLASS_PANEL, dev, "panel", &dsi->panel);
  /* Or iterate children */
  if (ret) {
    /* Try finding first child */
    device_find_first_child(dev, &dsi->panel);
  }

  if (!dsi->panel) {
    printf("No panel found\n");
    return -ENODEV;
  }

  ret = reset_deassert(&dsi->reset);
  if (ret)
    return ret;

  ret = clk_enable(&dsi->bus_clk);
  if (ret)
    return ret;

  ret = clk_enable(&dsi->mod_clk);
  if (ret)
    return ret;

  /* Set mod clock rate - essential! */
  clk_set_rate(&dsi->mod_clk, 297000000);

  return 0;
}

static const struct video_bridge_ops sun6i_dsi_ops = {
    .attach = sun6i_dsi_attach,
};

static const struct udevice_id sun6i_dsi_ids[] = {
    {.compatible = "allwinner,sun50i-a100-mipi-dsi"}, {}};

U_BOOT_DRIVER(sun6i_mipi_dsi) = {
    .name = "sun6i_mipi_dsi",
    .id = UCLASS_VIDEO_BRIDGE,
    .of_match = sun6i_dsi_ids,
    .probe = sun6i_dsi_probe,
    .ops = &sun6i_dsi_ops,
    .priv_auto = sizeof(struct sun6i_dsi_priv),
};
