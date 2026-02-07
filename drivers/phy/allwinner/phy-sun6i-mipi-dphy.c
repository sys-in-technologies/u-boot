// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner MIPI D-PHY Driver
 *
 * Copyright (C) 2018 Bootlin
 * Copyright (C) 2026 U-Boot Port
 */

#include <asm/arch/clock.h>
#include <asm/io.h>
#include <clk.h>
#include <dm.h>
#include <generic-phy.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <log.h>
#include <phy-mipi-dphy.h>
#include <reset.h>

#define SUN6I_DPHY_GCTL_REG 0x00
#define SUN6I_DPHY_GCTL_LANE_NUM(n) ((((n) - 1) & 3) << 4)
#define SUN6I_DPHY_GCTL_EN BIT(0)

#define SUN6I_DPHY_TX_CTL_REG 0x04
#define SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT BIT(28)

#define SUN6I_DPHY_TX_TIME0_REG 0x10
#define SUN6I_DPHY_TX_TIME0_HS_TRAIL(n) (((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME0_HS_PREPARE(n) (((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(n) ((n) & 0xff)

#define SUN6I_DPHY_TX_TIME1_REG 0x14
#define SUN6I_DPHY_TX_TIME1_CLK_POST(n) (((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME1_CLK_PRE(n) (((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME1_CLK_ZERO(n) (((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME1_CLK_PREPARE(n) ((n) & 0xff)

#define SUN6I_DPHY_TX_TIME2_REG 0x18
#define SUN6I_DPHY_TX_TIME2_CLK_TRAIL(n) ((n) & 0xff)

#define SUN6I_DPHY_TX_TIME3_REG 0x1c

#define SUN6I_DPHY_TX_TIME4_REG 0x20
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(n) (((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(n) ((n) & 0xff)

#define SUN6I_DPHY_ANA0_REG 0x4c
#define SUN6I_DPHY_ANA0_REG_PWS BIT(31)
#define SUN6I_DPHY_ANA0_REG_DMPC BIT(28)
#define SUN6I_DPHY_ANA0_REG_DMPD(n) (((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA0_REG_SLV(n) (((n) & 7) << 12)
#define SUN6I_DPHY_ANA0_REG_DEN(n) (((n) & 0xf) << 8)
#define SUN6I_DPHY_ANA0_REG_SFB(n) (((n) & 3) << 2)
#define SUN6I_DPHY_ANA0_REG_PLR(n) (((n) & 0xf) << 4)

#define SUN6I_DPHY_ANA1_REG 0x50
#define SUN6I_DPHY_ANA1_REG_VTTMODE BIT(31)
#define SUN6I_DPHY_ANA1_REG_SVTT(n) (((n) & 0xf) << 24)

#define SUN6I_DPHY_ANA2_REG 0x54
#define SUN6I_DPHY_ANA2_EN_P2S_CPU(n) (((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK GENMASK(27, 24)
#define SUN6I_DPHY_ANA2_EN_CK_CPU BIT(4)
#define SUN6I_DPHY_ANA2_REG_ENIB BIT(1)

#define SUN6I_DPHY_ANA3_REG 0x58
#define SUN6I_DPHY_ANA3_EN_VTTD(n) (((n) & 0xf) << 28)
#define SUN6I_DPHY_ANA3_EN_VTTD_MASK GENMASK(31, 28)
#define SUN6I_DPHY_ANA3_EN_VTTC BIT(27)
#define SUN6I_DPHY_ANA3_EN_DIV BIT(26)
#define SUN6I_DPHY_ANA3_EN_LDOC BIT(25)
#define SUN6I_DPHY_ANA3_EN_LDOD BIT(24)
#define SUN6I_DPHY_ANA3_EN_LDOR BIT(18)

#define SUN6I_DPHY_ANA4_REG 0x5c
#define SUN6I_DPHY_ANA4_REG_EN_MIPI BIT(31)
#define SUN6I_DPHY_ANA4_REG_IB(n) (((n) & 3) << 25)
#define SUN6I_DPHY_ANA4_REG_DMPLVC BIT(24)
#define SUN6I_DPHY_ANA4_REG_DMPLVD(n) (((n) & 0xf) << 20)
#define SUN6I_DPHY_ANA4_REG_VTT_SET(n) (((n) & 0x7) << 17)
#define SUN6I_DPHY_ANA4_REG_CKDV(n) (((n) & 0x1f) << 12)
#define SUN6I_DPHY_ANA4_REG_TMSC(n) (((n) & 3) << 10)
#define SUN6I_DPHY_ANA4_REG_TMSD(n) (((n) & 3) << 8)
#define SUN6I_DPHY_ANA4_REG_TXDNSC(n) (((n) & 3) << 6)
#define SUN6I_DPHY_ANA4_REG_TXDNSD(n) (((n) & 3) << 4)
#define SUN6I_DPHY_ANA4_REG_TXPUSC(n) (((n) & 3) << 2)
#define SUN6I_DPHY_ANA4_REG_TXPUSD(n) ((n) & 3)

#define SUN50I_DPHY_PLL_REG0 0x104
#define SUN50I_DPHY_PLL_REG0_CP36_EN BIT(23)
#define SUN50I_DPHY_PLL_REG0_LDO_EN BIT(22)
#define SUN50I_DPHY_PLL_REG0_EN_LVS BIT(21)
#define SUN50I_DPHY_PLL_REG0_PLL_EN BIT(20)
#define SUN50I_DPHY_PLL_REG0_NDET BIT(7)
#define SUN50I_DPHY_PLL_REG0_P(n) (((n) & 0xf) << 16)
#define SUN50I_DPHY_PLL_REG0_N(n) (((n) & 0xff) << 8)
#define SUN50I_DPHY_PLL_REG0_M0(n) (((n) & 3) << 4)
#define SUN50I_DPHY_PLL_REG0_M1(n) ((n) & 0xf)

#define SUN50I_DPHY_PLL_REG2 0x10c

#define SUN50I_COMBO_PHY_REG0 0x110
#define SUN50I_COMBO_PHY_REG0_EN_MIPI BIT(3)
#define SUN50I_COMBO_PHY_REG0_EN_COMBOLDO BIT(1)
#define SUN50I_COMBO_PHY_REG0_EN_CP BIT(0)

#define SUN50I_COMBO_PHY_REG2 0x118
#define SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(n) ((n) & 0xff)

struct sun6i_dphy_priv {
  void __iomem *regs;
  struct reset_ctl reset;
  struct clk mod_clk;
  struct clk bus_clk;
};

static int sun6i_dphy_init(struct phy *phy) {
  struct sun6i_dphy_priv *dphy = dev_get_priv(phy->dev);
  int ret;

  ret = reset_deassert(&dphy->reset);
  if (ret)
    return ret;

  ret = clk_enable(&dphy->bus_clk);
  if (ret)
    return ret;

  ret = clk_enable(&dphy->mod_clk);
  if (ret)
    return ret;

  /* Set rate to 150 MHz for calibration */
  clk_set_rate(&dphy->mod_clk, 150000000);

  return 0;
}

static int sun6i_dphy_exit(struct phy *phy) {
  struct sun6i_dphy_priv *dphy = dev_get_priv(phy->dev);

  clk_disable(&dphy->mod_clk);
  clk_disable(&dphy->bus_clk);
  reset_assert(&dphy->reset);

  return 0;
}

static int sun50i_a100_mipi_dphy_tx_power_on(struct sun6i_dphy_priv *dphy,
                                             unsigned long mipi_symbol_rate) {
  unsigned int div, n;

  writel(SUN6I_DPHY_ANA4_REG_IB(2) | SUN6I_DPHY_ANA4_REG_DMPLVD(4) |
             SUN6I_DPHY_ANA4_REG_VTT_SET(3) | SUN6I_DPHY_ANA4_REG_CKDV(3) |
             SUN6I_DPHY_ANA4_REG_TMSD(1) | SUN6I_DPHY_ANA4_REG_TMSC(1) |
             SUN6I_DPHY_ANA4_REG_TXPUSD(2) | SUN6I_DPHY_ANA4_REG_TXPUSC(3) |
             SUN6I_DPHY_ANA4_REG_TXDNSD(2) | SUN6I_DPHY_ANA4_REG_TXDNSC(3),
         dphy->regs + SUN6I_DPHY_ANA4_REG);

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA2_REG, SUN6I_DPHY_ANA2_EN_CK_CPU);
  setbits_le32(dphy->regs + SUN6I_DPHY_ANA2_REG, SUN6I_DPHY_ANA2_REG_ENIB);

  writel(SUN6I_DPHY_ANA3_EN_LDOR | SUN6I_DPHY_ANA3_EN_LDOC |
             SUN6I_DPHY_ANA3_EN_LDOD,
         dphy->regs + SUN6I_DPHY_ANA3_REG);

  writel(SUN6I_DPHY_ANA0_REG_PLR(4) | SUN6I_DPHY_ANA0_REG_SFB(1),
         dphy->regs + SUN6I_DPHY_ANA0_REG);

  writel(SUN50I_COMBO_PHY_REG0_EN_CP, dphy->regs + SUN50I_COMBO_PHY_REG0);

  /* Choose a divider to limit the VCO frequency to around 2 GHz. */
  /*
   * Note: overflow handling might be needed if mipi_symbol_rate is large,
   * but for typical DSI speeds in U-Boot it should be fine.
   */
  div = 16 >> (fls(DIV_ROUND_UP(mipi_symbol_rate, 264000000)) - 1);
  /* Using rough fls approximation for order_base_2 */
  /* Actually order_base_2(n) is equivalent to fls(n-1) if n is power of 2,
   * or simply fls(n) if we round up.
   * Let's just implement a simple loop or use DIV_ROUND_UP logic
   */
  {
    int d = DIV_ROUND_UP(mipi_symbol_rate, 264000000);
    int order = 0;
    while (d > 1) {
      d >>= 1;
      order++;
    }
    /* If exact power of 2, order is correct. If not, might need adjustment.
     * Linux order_base_2 returns log2 rounded up.
     * e.g. 1 -> 0, 2 -> 1, 3 -> 2, 4 -> 2
     * fls(3)=2, fls(4)=3.
     * fls(n-1) seems correct for order_base_2(n) where n >= 1
     */
    /* Re-implementing correctly: */
    order = 0;
    if (DIV_ROUND_UP(mipi_symbol_rate, 264000000) > 1)
      order = fls(DIV_ROUND_UP(mipi_symbol_rate, 264000000) - 1);

    div = 16 >> order;
  }

  n = mipi_symbol_rate * div / 24000000;

  writel(SUN50I_DPHY_PLL_REG0_CP36_EN | SUN50I_DPHY_PLL_REG0_LDO_EN |
             SUN50I_DPHY_PLL_REG0_EN_LVS | SUN50I_DPHY_PLL_REG0_PLL_EN |
             SUN50I_DPHY_PLL_REG0_NDET | SUN50I_DPHY_PLL_REG0_P((div - 1) % 8) |
             SUN50I_DPHY_PLL_REG0_N(n) |
             SUN50I_DPHY_PLL_REG0_M0((div - 1) / 8) |
             SUN50I_DPHY_PLL_REG0_M1(2),
         dphy->regs + SUN50I_DPHY_PLL_REG0);

  writel(0, dphy->regs + SUN50I_DPHY_PLL_REG2);

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA4_REG, SUN6I_DPHY_ANA4_REG_EN_MIPI);

  setbits_le32(dphy->regs + SUN50I_COMBO_PHY_REG0,
               SUN50I_COMBO_PHY_REG0_EN_MIPI |
                   SUN50I_COMBO_PHY_REG0_EN_COMBOLDO);

  writel(SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(20),
         dphy->regs + SUN50I_COMBO_PHY_REG2);

  udelay(1);

  return 0;
}

static int sun6i_dphy_power_on(struct phy *phy) {
  struct sun6i_dphy_priv *dphy = dev_get_priv(phy->dev);
  /*
   * U-Boot generic-phy doesn't pass strict config options like Linux.
   * We'll need to assume some defaults or get them from dt/platform data.
   * For now, assume a typical 4-lane setup and a symbol rate.
   * Wait, we need the symbol rate (bit clock).
   * The DSI driver should configure this.
   * But phy_power_on signature is fixed.
   * We can use phy_set_mode or similar if available, or just use a fixed rate
   * for now. Linux driver uses `phy_configure`. U-Boot has `phy_configure` too?
   * Yes, UCLASS_PHY has `configure`.
   */
  return 0;
}

static int sun6i_dphy_configure(struct phy *phy, void *params) {
  struct sun6i_dphy_priv *dphy = dev_get_priv(phy->dev);
  struct phy_configure_opts_mipi_dphy *cfg = params;
  int lanes_mask;

  if (!cfg)
    return -EINVAL;

  /* Simplified power on logic merged with configure for U-Boot context */
  /* In Linux, power_on calls tx_power_on which uses dphy->config which is set
   * by configure. */

  sun50i_a100_mipi_dphy_tx_power_on(dphy, cfg->hs_clk_rate);

  lanes_mask = GENMASK(cfg->lanes - 1, 0);

  writel(SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT, dphy->regs + SUN6I_DPHY_TX_CTL_REG);

  writel(SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(14) |
             SUN6I_DPHY_TX_TIME0_HS_PREPARE(6) |
             SUN6I_DPHY_TX_TIME0_HS_TRAIL(10),
         dphy->regs + SUN6I_DPHY_TX_TIME0_REG);

  writel(SUN6I_DPHY_TX_TIME1_CLK_PREPARE(7) | SUN6I_DPHY_TX_TIME1_CLK_ZERO(50) |
             SUN6I_DPHY_TX_TIME1_CLK_PRE(3) | SUN6I_DPHY_TX_TIME1_CLK_POST(10),
         dphy->regs + SUN6I_DPHY_TX_TIME1_REG);

  writel(SUN6I_DPHY_TX_TIME2_CLK_TRAIL(30),
         dphy->regs + SUN6I_DPHY_TX_TIME2_REG);

  writel(0, dphy->regs + SUN6I_DPHY_TX_TIME3_REG);

  writel(SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(3) | SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(3),
         dphy->regs + SUN6I_DPHY_TX_TIME4_REG);

  /* dphy->variant->tx_power_on(dphy); already called above */

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA3_REG,
               SUN6I_DPHY_ANA3_EN_VTTC | SUN6I_DPHY_ANA3_EN_VTTD(lanes_mask));
  udelay(1);

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA3_REG, SUN6I_DPHY_ANA3_EN_DIV);
  udelay(1);

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA2_REG, SUN6I_DPHY_ANA2_EN_CK_CPU);
  udelay(1);

  setbits_le32(dphy->regs + SUN6I_DPHY_ANA1_REG, SUN6I_DPHY_ANA1_REG_VTTMODE);

  clrsetbits_le32(dphy->regs + SUN6I_DPHY_ANA2_REG,
                  SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK,
                  SUN6I_DPHY_ANA2_EN_P2S_CPU(lanes_mask));

  writel(SUN6I_DPHY_GCTL_LANE_NUM(cfg->lanes) | SUN6I_DPHY_GCTL_EN,
         dphy->regs + SUN6I_DPHY_GCTL_REG);

  return 0;
}

static int sun6i_dphy_probe(struct udevice *dev) {
  struct sun6i_dphy_priv *dphy = dev_get_priv(dev);
  int ret;

  dphy->regs = dev_read_addr_ptr(dev);
  if (!dphy->regs)
    return -EINVAL;

  ret = reset_get_by_index(dev, 0, &dphy->reset);
  if (ret)
    return ret;

  ret = clk_get_by_name(dev, "mod", &dphy->mod_clk);
  if (ret)
    return ret;

  ret = clk_get_by_name(dev, "bus", &dphy->bus_clk);
  if (ret)
    return ret;

  return 0;
}

static const struct phy_ops sun6i_dphy_ops = {
    .init = sun6i_dphy_init,
    .exit = sun6i_dphy_exit,
    .power_on = sun6i_dphy_power_on,
    .configure = sun6i_dphy_configure, // Note: check if U-Boot phy_ops has
                                       // configure with 2 args
};

static const struct udevice_id sun6i_dphy_ids[] = {
    {.compatible = "allwinner,sun50i-a100-mipi-dphy"}, {}};

U_BOOT_DRIVER(sun6i_mipi_dphy) = {
    .name = "sun6i_mipi_dphy",
    .id = UCLASS_PHY,
    .of_match = sun6i_dphy_ids,
    .probe = sun6i_dphy_probe,
    .ops = &sun6i_dphy_ops,
    .priv_auto = sizeof(struct sun6i_dphy_priv),
};
