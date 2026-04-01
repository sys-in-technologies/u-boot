// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Allwinnertech Co., Ltd.
 * Copyright (C) 2017-2018 Bootlin
 * Copyright (C) 2026 Project Splash - Backported to U-Boot
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <clk.h>
#include <div64.h>
#include <dm.h>
#include <generic-phy.h>
#include <log.h>
#include <reset.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>
#include <linux/log2.h>

#include <phy-mipi-dphy.h>

#define SUN6I_DPHY_GCTL_REG		0x00
#define SUN6I_DPHY_GCTL_LANE_NUM(n)		((((n) - 1) & 3) << 4)
#define SUN6I_DPHY_GCTL_EN			BIT(0)

#define SUN6I_DPHY_TX_CTL_REG		0x04
#define SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT	BIT(28)

#define SUN6I_DPHY_TX_TIME0_REG		0x10
#define SUN6I_DPHY_TX_TIME0_HS_TRAIL(n)		(((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME0_HS_PREPARE(n)	(((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME1_REG		0x14
#define SUN6I_DPHY_TX_TIME1_CLK_POST(n)		(((n) & 0xff) << 24)
#define SUN6I_DPHY_TX_TIME1_CLK_PRE(n)		(((n) & 0xff) << 16)
#define SUN6I_DPHY_TX_TIME1_CLK_ZERO(n)		(((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME1_CLK_PREPARE(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME2_REG		0x18
#define SUN6I_DPHY_TX_TIME2_CLK_TRAIL(n)	((n) & 0xff)

#define SUN6I_DPHY_TX_TIME3_REG		0x1c

#define SUN6I_DPHY_TX_TIME4_REG		0x20
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(n)	(((n) & 0xff) << 8)
#define SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(n)	((n) & 0xff)

#define SUN6I_DPHY_ANA0_REG		0x4c
#define SUN6I_DPHY_ANA0_REG_PWS			BIT(31)
#define SUN6I_DPHY_ANA0_REG_DMPC		BIT(28)
#define SUN6I_DPHY_ANA0_REG_DMPD(n)		(((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA0_REG_SLV(n)		(((n) & 7) << 12)
#define SUN6I_DPHY_ANA0_REG_DEN(n)		(((n) & 0xf) << 8)
#define SUN6I_DPHY_ANA0_REG_PLR(n)		(((n) & 0xf) << 4)
#define SUN6I_DPHY_ANA0_REG_SFB(n)		(((n) & 3) << 2)

#define SUN6I_DPHY_ANA1_REG		0x50
#define SUN6I_DPHY_ANA1_REG_VTTMODE		BIT(31)
#define SUN6I_DPHY_ANA1_REG_CSMPS(n)		(((n) & 3) << 28)
#define SUN6I_DPHY_ANA1_REG_SVTT(n)		(((n) & 0xf) << 24)

#define SUN6I_DPHY_ANA2_REG		0x54
#define SUN6I_DPHY_ANA2_EN_P2S_CPU(n)		(((n) & 0xf) << 24)
#define SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK		GENMASK(27, 24)
#define SUN6I_DPHY_ANA2_EN_CK_CPU		BIT(4)
#define SUN6I_DPHY_ANA2_REG_ENIB		BIT(1)

#define SUN6I_DPHY_ANA3_REG		0x58
#define SUN6I_DPHY_ANA3_EN_VTTD(n)		(((n) & 0xf) << 28)
#define SUN6I_DPHY_ANA3_EN_VTTD_MASK		GENMASK(31, 28)
#define SUN6I_DPHY_ANA3_EN_VTTC			BIT(27)
#define SUN6I_DPHY_ANA3_EN_DIV			BIT(26)
#define SUN6I_DPHY_ANA3_EN_LDOC			BIT(25)
#define SUN6I_DPHY_ANA3_EN_LDOD			BIT(24)
#define SUN6I_DPHY_ANA3_EN_LDOR			BIT(18)

#define SUN6I_DPHY_ANA4_REG		0x5c
#define SUN6I_DPHY_ANA4_REG_EN_MIPI		BIT(31)
#define SUN6I_DPHY_ANA4_REG_IB(n)		(((n) & 3) << 25)
#define SUN6I_DPHY_ANA4_REG_DMPLVC		BIT(24)
#define SUN6I_DPHY_ANA4_REG_DMPLVD(n)		(((n) & 0xf) << 20)
#define SUN6I_DPHY_ANA4_REG_VTT_SET(n)		(((n) & 0x7) << 17)
#define SUN6I_DPHY_ANA4_REG_CKDV(n)		(((n) & 0x1f) << 12)
#define SUN6I_DPHY_ANA4_REG_TMSC(n)		(((n) & 3) << 10)
#define SUN6I_DPHY_ANA4_REG_TMSD(n)		(((n) & 3) << 8)
#define SUN6I_DPHY_ANA4_REG_TXDNSC(n)		(((n) & 3) << 6)
#define SUN6I_DPHY_ANA4_REG_TXDNSD(n)		(((n) & 3) << 4)
#define SUN6I_DPHY_ANA4_REG_TXPUSC(n)		(((n) & 3) << 2)
#define SUN6I_DPHY_ANA4_REG_TXPUSD(n)		((n) & 3)

#define SUN50I_DPHY_PLL_REG0		0x104
#define SUN50I_DPHY_PLL_REG0_CP36_EN		BIT(23)
#define SUN50I_DPHY_PLL_REG0_LDO_EN		BIT(22)
#define SUN50I_DPHY_PLL_REG0_EN_LVS		BIT(21)
#define SUN50I_DPHY_PLL_REG0_PLL_EN		BIT(20)
#define SUN50I_DPHY_PLL_REG0_P(n)		(((n) & 0xf) << 16)
#define SUN50I_DPHY_PLL_REG0_N(n)		(((n) & 0xff) << 8)
#define SUN50I_DPHY_PLL_REG0_NDET		BIT(7)
#define SUN50I_DPHY_PLL_REG0_M0(n)		(((n) & 3) << 4)
#define SUN50I_DPHY_PLL_REG0_M1(n)		((n) & 0xf)

#define SUN50I_DPHY_PLL_REG1		0x108

#define SUN50I_DPHY_PLL_REG2		0x10c

#define SUN50I_COMBO_PHY_REG0		0x110
#define SUN50I_COMBO_PHY_REG0_EN_MIPI		BIT(3)
#define SUN50I_COMBO_PHY_REG0_EN_COMBOLDO	BIT(1)
#define SUN50I_COMBO_PHY_REG0_EN_CP		BIT(0)

#define SUN50I_COMBO_PHY_REG2		0x118
#define SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(n)	((n) & 0xff)

enum sun6i_dphy_type {
	SUN6I_DPHY_VARIANT_A31,
	SUN6I_DPHY_VARIANT_A100,
};

struct sun6i_dphy_priv {
	void __iomem *regs;
	struct clk bus_clk;
	struct clk mod_clk;
	struct reset_ctl reset;
	struct phy_configure_opts_mipi_dphy config;
	enum sun6i_dphy_type variant;
};

static void sun6i_dphy_write(struct sun6i_dphy_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->regs + reg);
}

static u32 sun6i_dphy_read(struct sun6i_dphy_priv *priv, u32 reg)
{
	return readl(priv->regs + reg);
}

static void sun6i_dphy_update_bits(struct sun6i_dphy_priv *priv, u32 reg,
				   u32 mask, u32 val)
{
	u32 tmp = sun6i_dphy_read(priv, reg);

	tmp &= ~mask;
	tmp |= val & mask;
	sun6i_dphy_write(priv, reg, tmp);
}

static void sun6i_a31_mipi_dphy_tx_power_on(struct sun6i_dphy_priv *priv)
{
	u8 lanes_mask = GENMASK(priv->config.lanes - 1, 0);

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA0_REG,
			 SUN6I_DPHY_ANA0_REG_PWS |
			 SUN6I_DPHY_ANA0_REG_DMPC |
			 SUN6I_DPHY_ANA0_REG_SLV(7) |
			 SUN6I_DPHY_ANA0_REG_DMPD(lanes_mask) |
			 SUN6I_DPHY_ANA0_REG_DEN(lanes_mask));

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA1_REG,
			 SUN6I_DPHY_ANA1_REG_CSMPS(1) |
			 SUN6I_DPHY_ANA1_REG_SVTT(7));

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA4_REG,
			 SUN6I_DPHY_ANA4_REG_CKDV(1) |
			 SUN6I_DPHY_ANA4_REG_TMSC(1) |
			 SUN6I_DPHY_ANA4_REG_TMSD(1) |
			 SUN6I_DPHY_ANA4_REG_TXDNSC(1) |
			 SUN6I_DPHY_ANA4_REG_TXDNSD(1) |
			 SUN6I_DPHY_ANA4_REG_TXPUSC(1) |
			 SUN6I_DPHY_ANA4_REG_TXPUSD(1) |
			 SUN6I_DPHY_ANA4_REG_DMPLVC |
			 SUN6I_DPHY_ANA4_REG_DMPLVD(lanes_mask));

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA2_REG,
			 SUN6I_DPHY_ANA2_REG_ENIB);
	udelay(5);

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA3_REG,
			 SUN6I_DPHY_ANA3_EN_LDOR |
			 SUN6I_DPHY_ANA3_EN_LDOC |
			 SUN6I_DPHY_ANA3_EN_LDOD);
	udelay(1);
}

static void sun50i_a100_mipi_dphy_tx_power_on(struct sun6i_dphy_priv *priv)
{
	unsigned long mipi_symbol_rate = priv->config.hs_clk_rate;
	unsigned int div, n;

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA4_REG,
			 SUN6I_DPHY_ANA4_REG_IB(2) |
			 SUN6I_DPHY_ANA4_REG_DMPLVD(4) |
			 SUN6I_DPHY_ANA4_REG_VTT_SET(3) |
			 SUN6I_DPHY_ANA4_REG_CKDV(3) |
			 SUN6I_DPHY_ANA4_REG_TMSD(1) |
			 SUN6I_DPHY_ANA4_REG_TMSC(1) |
			 SUN6I_DPHY_ANA4_REG_TXPUSD(2) |
			 SUN6I_DPHY_ANA4_REG_TXPUSC(3) |
			 SUN6I_DPHY_ANA4_REG_TXDNSD(2) |
			 SUN6I_DPHY_ANA4_REG_TXDNSC(3));

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA2_REG,
			       SUN6I_DPHY_ANA2_EN_CK_CPU,
			       SUN6I_DPHY_ANA2_EN_CK_CPU);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA2_REG,
			       SUN6I_DPHY_ANA2_REG_ENIB,
			       SUN6I_DPHY_ANA2_REG_ENIB);

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA3_REG,
			 SUN6I_DPHY_ANA3_EN_LDOR |
			 SUN6I_DPHY_ANA3_EN_LDOC |
			 SUN6I_DPHY_ANA3_EN_LDOD);

	sun6i_dphy_write(priv, SUN6I_DPHY_ANA0_REG,
			 SUN6I_DPHY_ANA0_REG_PLR(4) |
			 SUN6I_DPHY_ANA0_REG_SFB(1));

	/* Clean start for Combo PHY registers */
	sun6i_dphy_write(priv, 0x114, 0); /* COMBO_PHY_REG1 */
	sun6i_dphy_write(priv, 0x118, 20); /* COMBO_PHY_REG2: HS_STOP_DLY */

	sun6i_dphy_write(priv, SUN50I_COMBO_PHY_REG0,
			 SUN50I_COMBO_PHY_REG0_EN_CP);

	/* Choose a divider to limit the VCO frequency to around 2 GHz. */
	div = 16 >> order_base_2(DIV_ROUND_UP(mipi_symbol_rate, 264000000));
	n = mipi_symbol_rate * div / 24000000;

	sun6i_dphy_write(priv, SUN50I_DPHY_PLL_REG0,
			 SUN50I_DPHY_PLL_REG0_CP36_EN |
			 SUN50I_DPHY_PLL_REG0_LDO_EN |
			 SUN50I_DPHY_PLL_REG0_EN_LVS |
			 SUN50I_DPHY_PLL_REG0_PLL_EN |
			 SUN50I_DPHY_PLL_REG0_NDET |
			 SUN50I_DPHY_PLL_REG0_P((div - 1) % 8) |
			 SUN50I_DPHY_PLL_REG0_N(n) |
			 SUN50I_DPHY_PLL_REG0_M0((div - 1) / 8) |
			 SUN50I_DPHY_PLL_REG0_M1(2));

	/* Disable sigma-delta modulation. */
	sun6i_dphy_write(priv, SUN50I_DPHY_PLL_REG2, 0);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA4_REG,
			       SUN6I_DPHY_ANA4_REG_EN_MIPI,
			       SUN6I_DPHY_ANA4_REG_EN_MIPI);

	sun6i_dphy_update_bits(priv, SUN50I_COMBO_PHY_REG0,
			       SUN50I_COMBO_PHY_REG0_EN_MIPI |
			       SUN50I_COMBO_PHY_REG0_EN_COMBOLDO,
			       SUN50I_COMBO_PHY_REG0_EN_MIPI |
			       SUN50I_COMBO_PHY_REG0_EN_COMBOLDO);

	sun6i_dphy_write(priv, SUN50I_COMBO_PHY_REG2,
			 SUN50I_COMBO_PHY_REG2_HS_STOP_DLY(20));
	udelay(1);
}

static int sun6i_dphy_init(struct phy *phy)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(phy->dev);
	int ret;

	log_debug("DPHY: Initializing...\n");

	/* Enable bus clock for register access */
	ret = clk_enable(&priv->bus_clk);
	if (ret) {
		dev_err(phy->dev, "Failed to enable bus clock: %d\n", ret);
		return ret;
	}

	/* Deassert reset */
	ret = reset_deassert(&priv->reset);
	if (ret) {
		dev_err(phy->dev, "Failed to deassert reset: %d\n", ret);
		clk_disable(&priv->bus_clk);
		return ret;
	}

#ifdef CONFIG_SUNXI_GEN_NCAT2
	/*
	 * Configure CLK_MIPI_DSI (CCU 0xb24) before enabling.
	 * U-Boot's CCU driver only supports gate control, not mux/divider,
	 * so we must configure the register directly.
	 *
	 * From Linux: mipi_dsi_parents[1] = pll_periph0 (600 MHz)
	 * Target: 150 MHz for D-PHY digital core
	 * Register: 0x81000003 (mux=1, M=3, gate will be set by clk_enable)
	 */
	{
		u32 pll_periph0 = 600000000;
		u32 target_rate = 150000000;
		u32 m_div = pll_periph0 / target_rate; /* = 4 */

		/* Configure mux and divider (gate will be controlled by clock framework) */
		writel((1 << 24) | (m_div - 1), (u8 *)SUNXI_CCM_BASE + 0xb24);

		log_debug("DPHY: CLK_MIPI_DSI configured: mux=1 (pll_periph0), M=%u, rate=%u MHz\n",
		       m_div, pll_periph0 / m_div / 1000000);
	}
#endif

	/* Enable module clock (this will set bit 31 of 0xb24) */
	ret = clk_enable(&priv->mod_clk);
	if (ret) {
		dev_err(phy->dev, "Failed to enable mod clock: %d\n", ret);
		reset_assert(&priv->reset);
		clk_disable(&priv->bus_clk);
		return ret;
	}

#ifdef CONFIG_SUNXI_GEN_NCAT2
	log_debug("DPHY: CLK_MIPI_DSI final value = 0x%08x\n",
	       readl((u8 *)SUNXI_CCM_BASE + 0xb24));
#endif

	return 0;
}

static int sun6i_dphy_configure(struct phy *phy, void *params)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(phy->dev);
	struct phy_configure_opts_mipi_dphy *cfg = params;
	int ret;

	log_debug("DPHY: Configuring (priv=%p, %d lanes, bitrate %lu)...\n", priv, cfg->lanes, cfg->hs_clk_rate);
	ret = phy_mipi_dphy_config_validate(cfg);
	if (ret)
		return ret;

	memcpy(&priv->config, cfg, sizeof(priv->config));
	log_debug("DPHY: lanes stored in priv: %d\n", priv->config.lanes);

	return 0;
}

static int sun6i_dphy_power_on(struct phy *phy)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(phy->dev);
	u8 lanes_mask = GENMASK(priv->config.lanes - 1, 0);
	u64 ui_ps, hs_trail_ps, clk_pre_ps;
	u32 hs_prepare, hs_trail, clk_prepare, clk_zero, clk_pre, clk_post, clk_trail;

	/*
	 * The DPHY mod clock is fixed at 150 MHz → period = 6667 ps/cycle.
	 * TX timing registers count in units of mod_clk cycles.
	 *
	 * Timing values in priv->config come from phy_mipi_dphy_get_default_config
	 * in picoseconds. Convert: reg = DIV_ROUND_UP(time_ps, 6667).
	 *
	 * Two exceptions computed directly from UI:
	 *  - hs_trail: the framework uses n=4 (reverse-direction HS); TX uses n=1.
	 *  - clk_pre:  the framework minimum (8000 ps) is below 8×UI for low bit rates.
	 */
#define DPHY_MOD_CLK_PS  6667ULL  /* 1 / 150 MHz in picoseconds */

	ui_ps = 1000000000000ULL / priv->config.hs_clk_rate;

	/* hs_trail (n=1 TX forward): max(8×UI, 60 ns + 4×UI) */
	hs_trail_ps = max_t(u64, 8 * ui_ps, 60000ULL + 4 * ui_ps);
	/* clk_pre: at least 8×UI per spec (framework floor is 8000 ps) */
	clk_pre_ps  = max_t(u64, (u64)priv->config.clk_pre, 8 * ui_ps);

	hs_prepare  = DIV_ROUND_UP(priv->config.hs_prepare,  DPHY_MOD_CLK_PS);
	hs_trail    = DIV_ROUND_UP_ULL(hs_trail_ps,           DPHY_MOD_CLK_PS);
	clk_prepare = DIV_ROUND_UP(priv->config.clk_prepare, DPHY_MOD_CLK_PS);
	clk_zero    = DIV_ROUND_UP(priv->config.clk_zero,    DPHY_MOD_CLK_PS);
	clk_pre     = DIV_ROUND_UP_ULL(clk_pre_ps,            DPHY_MOD_CLK_PS);
	clk_post    = DIV_ROUND_UP(priv->config.clk_post,    DPHY_MOD_CLK_PS);
	clk_trail   = DIV_ROUND_UP(priv->config.clk_trail,   DPHY_MOD_CLK_PS);

	log_debug("DPHY: Powering on (priv=%p, %d lanes)...\n", priv, priv->config.lanes);
	log_debug("DPHY: Timings (mod_clk cycles): hs_prep=%u trail=%u clk_prep=%u zero=%u pre=%u post=%u trail=%u\n",
	       hs_prepare, hs_trail, clk_prepare, clk_zero, clk_pre, clk_post, clk_trail);

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_CTL_REG,
			 SUN6I_DPHY_TX_CTL_HS_TX_CLK_CONT);

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_TIME0_REG,
			 SUN6I_DPHY_TX_TIME0_LP_CLK_DIV(14) |
			 SUN6I_DPHY_TX_TIME0_HS_PREPARE(hs_prepare) |
			 SUN6I_DPHY_TX_TIME0_HS_TRAIL(hs_trail));

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_TIME1_REG,
			 SUN6I_DPHY_TX_TIME1_CLK_PREPARE(clk_prepare) |
			 SUN6I_DPHY_TX_TIME1_CLK_ZERO(clk_zero) |
			 SUN6I_DPHY_TX_TIME1_CLK_PRE(clk_pre) |
			 SUN6I_DPHY_TX_TIME1_CLK_POST(clk_post));

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_TIME2_REG,
			 SUN6I_DPHY_TX_TIME2_CLK_TRAIL(clk_trail));

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_TIME3_REG, 0);

	sun6i_dphy_write(priv, SUN6I_DPHY_TX_TIME4_REG,
			 SUN6I_DPHY_TX_TIME4_HS_TX_ANA0(3) |
			 SUN6I_DPHY_TX_TIME4_HS_TX_ANA1(3));

	/* Variant-specific analog power-on sequence */
	if (priv->variant == SUN6I_DPHY_VARIANT_A100)
		sun50i_a100_mipi_dphy_tx_power_on(priv);
	else
		sun6i_a31_mipi_dphy_tx_power_on(priv);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA3_REG,
			       SUN6I_DPHY_ANA3_EN_VTTC |
			       SUN6I_DPHY_ANA3_EN_VTTD_MASK,
			       SUN6I_DPHY_ANA3_EN_VTTC |
			       SUN6I_DPHY_ANA3_EN_VTTD(lanes_mask));
	udelay(1);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA3_REG,
			       SUN6I_DPHY_ANA3_EN_DIV,
			       SUN6I_DPHY_ANA3_EN_DIV);
	udelay(1);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA2_REG,
			       SUN6I_DPHY_ANA2_EN_CK_CPU,
			       SUN6I_DPHY_ANA2_EN_CK_CPU);
	udelay(1);

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA1_REG,
			       SUN6I_DPHY_ANA1_REG_VTTMODE,
			       SUN6I_DPHY_ANA1_REG_VTTMODE);

#ifdef CONFIG_SUNXI_GEN_NCAT2
	/*
	 * T113-S/D1 Quirk: If IC version (bits 0-2 of 0x03000024) is > 0,
	 * bit 5 of ANA1 must be set.
	 */
	if ((readl((u8 *)0x03000024) & 0x7) > 0)
		sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA1_REG, BIT(5), BIT(5));
#endif

	sun6i_dphy_update_bits(priv, SUN6I_DPHY_ANA2_REG,
			       SUN6I_DPHY_ANA2_EN_P2S_CPU_MASK,
			       SUN6I_DPHY_ANA2_EN_P2S_CPU(lanes_mask));

	sun6i_dphy_write(priv, SUN6I_DPHY_GCTL_REG,
			 SUN6I_DPHY_GCTL_LANE_NUM(priv->config.lanes) |
			 SUN6I_DPHY_GCTL_EN);

	return 0;
}

static int sun6i_dphy_power_off(struct phy *phy)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(phy->dev);

	sun6i_dphy_write(priv, SUN6I_DPHY_GCTL_REG, 0);
	sun6i_dphy_write(priv, SUN6I_DPHY_ANA0_REG, 0);
	sun6i_dphy_write(priv, SUN6I_DPHY_ANA1_REG, 0);
	sun6i_dphy_write(priv, SUN6I_DPHY_ANA2_REG, 0);
	sun6i_dphy_write(priv, SUN6I_DPHY_ANA3_REG, 0);
	sun6i_dphy_write(priv, SUN6I_DPHY_ANA4_REG, 0);

	return 0;
}

static int sun6i_dphy_exit(struct phy *phy)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(phy->dev);

	clk_disable(&priv->mod_clk);
	reset_assert(&priv->reset);
	clk_disable(&priv->bus_clk);

	return 0;
}

static int sun6i_dphy_probe(struct udevice *dev)
{
	struct sun6i_dphy_priv *priv = dev_get_priv(dev);
	int ret;

	log_debug("DPHY: Probing %s...\n", dev->name);

	priv->regs = dev_read_addr_ptr(dev);
	if (!priv->regs)
		return -EINVAL;

	priv->variant = (enum sun6i_dphy_type)dev_get_driver_data(dev);

	/* Get bus clock from device tree (for register access) */
	ret = clk_get_by_name(dev, "bus", &priv->bus_clk);
	if (ret) {
		log_debug("DPHY: Warning: Failed to get bus clock: %d\n", ret);
#ifndef CONFIG_SUNXI_GEN_NCAT2
		return ret;
#endif
	}

	/* Get module clock from device tree (for PHY operation) */
	ret = clk_get_by_name(dev, "mod", &priv->mod_clk);
	if (ret) {
		log_debug("DPHY: Warning: Failed to get mod clock: %d\n", ret);
#ifndef CONFIG_SUNXI_GEN_NCAT2
		return ret;
#endif
	}

	/* Get reset control from device tree */
	ret = reset_get_by_index(dev, 0, &priv->reset);
	if (ret) {
		log_debug("DPHY: Warning: Failed to get reset: %d\n", ret);
#ifndef CONFIG_SUNXI_GEN_NCAT2
		return ret;
#endif
	}

	log_debug("DPHY: Probe successful.\n");
	return 0;
}

static struct phy_ops sun6i_dphy_ops = {
	.init		= sun6i_dphy_init,
	.exit		= sun6i_dphy_exit,
	.configure	= sun6i_dphy_configure,
	.power_on	= sun6i_dphy_power_on,
	.power_off	= sun6i_dphy_power_off,
};

static const struct udevice_id sun6i_dphy_ids[] = {
	{
		.compatible = "allwinner,sun6i-a31-mipi-dphy",
		.data = SUN6I_DPHY_VARIANT_A31,
	},
	{
		.compatible = "allwinner,sun50i-a100-mipi-dphy",
		.data = SUN6I_DPHY_VARIANT_A100,
	},
	{
		.compatible = "allwinner,sun20i-d1-mipi-dphy",
		.data = SUN6I_DPHY_VARIANT_A100,
	},
	{ }
};

U_BOOT_DRIVER(sun6i_mipi_dphy) = {
	.name		= "sun6i_mipi_dphy",
	.id		= UCLASS_PHY,
	.of_match	= sun6i_dphy_ids,
	.probe		= sun6i_dphy_probe,
	.ops		= &sun6i_dphy_ops,
	.priv_auto	= sizeof(struct sun6i_dphy_priv),
};
