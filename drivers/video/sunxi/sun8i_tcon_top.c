// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner TCON TOP driver
 *
 * Copyright (C) 2018 Jernej Skrabec <jernej.skrabec@siol.net>
 * Copyright (C) 2026 U-Boot Port
 */

#include <asm/arch/clock.h>
#include <asm/io.h>
#include <clk.h>

#include <dm.h>
#include <dt-bindings/clock/sun8i-tcon-top.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <log.h>
#include <reset.h>

#define TCON_TOP_GATE_SRC_REG 0x00
#define TCON_TOP_PORT_SEL_REG 0x04

#define TCON_TOP_HDMI_SRC_MSK PERIPH_ID(30, 2)

#define TCON_TOP_PORT_DE0_MSK PERIPH_ID(0, 2)
#define TCON_TOP_PORT_DE1_MSK PERIPH_ID(4, 2)

#define TCON_TOP_TCON_TV0_GATE 28
#define TCON_TOP_TCON_TV1_GATE 29
#define TCON_TOP_TCON_DSI_GATE 30

struct sun8i_tcon_top_quirks {
  bool has_tcon_tv1;
  bool has_dsi;
};

struct sun8i_tcon_top_priv {
  void __iomem *regs;
  struct reset_ctl rst;
  struct clk bus_clk;
};

static int sun8i_tcon_top_bind(struct udevice *dev) { return 0; }

static int sun8i_tcon_top_probe(struct udevice *dev) {
  struct sun8i_tcon_top_priv *priv = dev_get_priv(dev);
  const struct sun8i_tcon_top_quirks *quirks;
  int ret;

  quirks = (const struct sun8i_tcon_top_quirks *)dev_get_driver_data(dev);

  priv->regs = dev_read_addr_ptr(dev);
  if (!priv->regs)
    return -EINVAL;

  ret = reset_get_by_index(dev, 0, &priv->rst);
  if (ret)
    return ret;

  ret = clk_get_by_name(dev, "bus", &priv->bus_clk);
  if (ret)
    return ret;

  ret = reset_deassert(&priv->rst);
  if (ret)
    return ret;

  ret = clk_enable(&priv->bus_clk);
  if (ret)
    return ret;

  /* Clear registers to default state */
  writel(0, priv->regs + TCON_TOP_PORT_SEL_REG);
  writel(0, priv->regs + TCON_TOP_GATE_SRC_REG);

  /* Enable DSI gate if present */
  if (quirks->has_dsi)
    setbits_le32(priv->regs + TCON_TOP_GATE_SRC_REG,
                 BIT(TCON_TOP_TCON_DSI_GATE));

  /* Enable TV0 gate */
  setbits_le32(priv->regs + TCON_TOP_GATE_SRC_REG, BIT(TCON_TOP_TCON_TV0_GATE));

  if (quirks->has_tcon_tv1)
    setbits_le32(priv->regs + TCON_TOP_GATE_SRC_REG,
                 BIT(TCON_TOP_TCON_TV1_GATE));

  return 0;
}

static const struct sun8i_tcon_top_quirks sun8i_r40_tcon_top_quirks = {
    .has_tcon_tv1 = true,
    .has_dsi = true,
};

static const struct sun8i_tcon_top_quirks sun20i_d1_tcon_top_quirks = {
    .has_dsi = true,
};

static const struct sun8i_tcon_top_quirks sun50i_h6_tcon_top_quirks = {
    /* Nothing special */
};

static const struct udevice_id sun8i_tcon_top_ids[] = {
    {.compatible = "allwinner,sun8i-r40-tcon-top",
     .data = (ulong)&sun8i_r40_tcon_top_quirks},
    {.compatible = "allwinner,sun20i-d1-tcon-top",
     .data = (ulong)&sun20i_d1_tcon_top_quirks},
    {.compatible = "allwinner,sun50i-h6-tcon-top",
     .data = (ulong)&sun50i_h6_tcon_top_quirks},
    {}};

U_BOOT_DRIVER(sun8i_tcon_top) = {
    .name = "sun8i_tcon_top",
    .id = UCLASS_MISC,
    .of_match = sun8i_tcon_top_ids,
    .bind = sun8i_tcon_top_bind,
    .probe = sun8i_tcon_top_probe,
    .priv_auto = sizeof(struct sun8i_tcon_top_priv),
    .flags = DM_FLAG_PRE_RELOC,
};
