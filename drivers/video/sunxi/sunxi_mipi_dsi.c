#include <common.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <clk.h>
#include <reset.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <power/regulator.h>
#include <linux/delay.h>
#include <display.h>
#include <log.h>
#include <malloc.h>

/* Register definitions for Allwinner MIPI DSI controller */
#define SUN6I_DSI_CTL                     0x00
#define SUN6I_DSI_STATUS                  0x04
#define SUN6I_DSI_DATA_CTL                0x08
#define SUN6I_DSI_DATA_STATUS             0x0c
#define SUN6I_DSI_VBP                     0x10
#define SUN6I_DSI_VFP                     0x14
#define SUN6I_DSI_VSW                     0x18
#define SUN6I_DSI_HBP                     0x1c
#define SUN6I_DSI_HFP                     0x20
#define SUN6I_DSI_HSW                     0x24
#define SUN6I_DSI_PHY_TMR_LPCLK           0x28
#define SUN6I_DSI_PHY_TMR_HS              0x2c
#define SUN6I_DSI_PHY_CTL                 0x30
#define SUN6I_DSI_PHY_STATUS              0x34
#define SUN6I_DSI_INT_STA                 0x38
#define SUN6I_DSI_INT_EN                  0x3c
#define SUN6I_DSI_DCS_CMD                 0x40
#define SUN6I_DSI_GEN_PKT_HDR             0x44
#define SUN6I_DSI_GEN_PL_DATA             0x48
#define SUN6I_DSI_PHY_BIST                0x4c

/* Bit definitions */
#define CTL_EN                            BIT(0)
#define CTL_START                         BIT(1)
#define CTL_ULPS_ENTRY_EN                 BIT(2)
#define CTL_ULPS_EXIT_EN                  BIT(3)

#define DATA_CTL_TX_SIZE(n)               (((n) & 0x3ff) << 0)
#define DATA_CTL_TX_TYPE(n)               (((n) & 0x3f) << 16)
#define DATA_CTL_TX_START                 BIT(31)

#define INT_STA_PHY_READY                 BIT(0)
#define INT_STA_PHY_STOP                  BIT(1)
#define INT_STA_CMD_COMPLETED             BIT(2)
#define INT_STA_TIMEOUT                   BIT(3)

#define PHY_CTL_CAL_START                 BIT(0)
#define PHY_CTL_HS_RX_TIMEOUT_EN          BIT(1)
#define PHY_CTL_HS_TX_TIMEOUT_EN          BIT(2)
#define PHY_CTL_CLK_TX_LP_STOP_EN         BIT(4)
#define PHY_CTL_CLK_CONTINUOUS            BIT(5)

#define PHY_STATUS_CAL_DONE               BIT(0)
#define PHY_STATUS_CLK_LANE_LP            BIT(4)
#define PHY_STATUS_DATA_LANE0_LP          BIT(5)
#define PHY_STATUS_DATA_LANE1_LP          BIT(6)
#define PHY_STATUS_PHY_STOP               BIT(7)
#define PHY_STATUS_PHY_READY              BIT(8)

/* Timing values */
#define LPCLK_TMR_LPCLK(n)                ((n) & 0xffff)
#define LPCLK_TMR_CLKDIV(n)               (((n) & 0xff) << 16)

#define HS_TMR_HS_PREPARE(n)              ((n) & 0xff)
#define HS_TMR_HS_ZERO(n)                 (((n) & 0xff) << 8)
#define HS_TMR_HS_TRAIL(n)                (((n) & 0xff) << 16)
#define HS_TMR_HS_EXIT(n)                 (((n) & 0xff) << 24)

/* Helper macros */
#define SUN6I_DSI_WAIT_US                 1
#define SUN6I_DSI_TIMEOUT_US              100000  /* 100ms timeout */

/* Private data structure for the MIPI DSI controller */
struct sun6i_mipi_dsi {
	struct sun6i_mipi_dsi_reg *regs;
	struct reset_ctl *reset;
	struct clk *bus_clk;
	struct clk *mod_clk;
	struct udevice *phy;
	struct gpio_desc reset_gpio;
};

/* Register structure mapping */
struct sun6i_mipi_dsi_reg {
	u32 ctl;          /* 0x00 */
	u32 status;       /* 0x04 */
	u32 data_ctl;     /* 0x08 */
	u32 data_status;  /* 0x0c */
	u32 vbp;          /* 0x10 */
	u32 vfp;          /* 0x14 */
	u32 vsw;          /* 0x18 */
	u32 hbp;          /* 0x1c */
	u32 hfp;          /* 0x20 */
	u32 hsw;          /* 0x24 */
	u32 phy_tmr_lpclk; /* 0x28 */
	u32 phy_tmr_hs;   /* 0x2c */
	u32 phy_ctl;      /* 0x30 */
	u32 phy_status;   /* 0x34 */
	u32 int_sta;      /* 0x38 */
	u32 int_en;       /* 0x3c */
	u32 dcs_cmd;      /* 0x40 */
	u32 gen_pkt_hdr;  /* 0x44 */
	u32 gen_pl_data;  /* 0x48 */
	u32 phy_bist;     /* 0x4c */
};

/* Static function declarations */
static int sunxi_mipi_dsi_wait_for_completion(struct sun6i_mipi_dsi *dsi,
					      u32 mask, u32 expected_val);
static int sunxi_mipi_dsi_phy_init(struct sun6i_mipi_dsi *dsi);
static int sunxi_mipi_dsi_setup_timing(struct sun6i_mipi_dsi *dsi,
				       const struct display_timing *timing);
static int sunxi_mipi_dsi_gen_pkt_hdr_write(struct sun6i_mipi_dsi *dsi,
					    u32 val);
static int sunxi_mipi_dsi_send_cmd(struct sun6i_mipi_dsi *dsi,
				   const struct mipi_dsi_msg *msg);

/**
 * sunxi_mipi_dsi_wait_for_completion - Poll for register status
 * @dsi: DSI controller instance
 * @mask: Mask to apply to status register
 * @expected_val: Expected value after applying mask
 *
 * Polls the status register until the masked bits match expected value
 * or timeout occurs.
 */
static int sunxi_mipi_dsi_wait_for_completion(struct sun6i_mipi_dsi *dsi,
					      u32 mask, u32 expected_val)
{
	unsigned long timeout = timer_get_us() + SUN6I_DSI_TIMEOUT_US;
	u32 status;

	do {
		status = readl(&dsi->regs->status);
		if ((status & mask) == expected_val)
			return 0;
		udelay(SUN6I_DSI_WAIT_US);
	} while (timer_get_us() < timeout);

	debug("MIPI DSI: Timeout waiting for status 0x%x, got 0x%x\n", 
	      expected_val, status);
	return -ETIMEDOUT;
}

/**
 * sunxi_mipi_dsi_phy_init - Initialize the MIPI DSI PHY
 * @dsi: DSI controller instance
 *
 * Initialize the D-PHY for operation. This is done via polling
 * rather than interrupt driven as in the Linux implementation.
 */
static int sunxi_mipi_dsi_phy_init(struct sun6i_mipi_dsi *dsi)
{
	u32 val;

	/* Reset the PHY first */
	writel(0, &dsi->regs->phy_ctl);

	/* Set up timing parameters for the PHY */
	writel(LPCLK_TMR_LPCLK(0x10) | LPCLK_TMR_CLKDIV(0x20),
	       &dsi->regs->phy_tmr_lpclk);

	writel(HS_TMR_HS_PREPARE(0x8) | HS_TMR_HS_ZERO(0x10) |
	       HS_TMR_HS_TRAIL(0x8) | HS_TMR_HS_EXIT(0x10),
	       &dsi->regs->phy_tmr_hs);

	/* Start calibration */
	setbits_le32(&dsi->regs->phy_ctl, PHY_CTL_CAL_START);

	/* Wait for calibration to complete (polling-based approach) */
	return sunxi_mipi_dsi_wait_for_completion(dsi, PHY_STATUS_CAL_DONE,
						 PHY_STATUS_CAL_DONE);
}

/**
 * sunxi_mipi_dsi_setup_timing - Configure video timing parameters
 * @dsi: DSI controller instance
 * @timing: Display timing information
 */
static int sunxi_mipi_dsi_setup_timing(struct sun6i_mipi_dsi *dsi,
				       const struct display_timing *timing)
{
	writel(timing->vback_porch.typ, &dsi->regs->vbp);
	writel(timing->vfront_porch.typ, &dsi->regs->vfp);
	writel(timing->vsync_len.typ, &dsi->regs->vsw);

	writel(timing->hback_porch.typ, &dsi->regs->hbp);
	writel(timing->hfront_porch.typ, &dsi->regs->hfp);
	writel(timing->hsync_len.typ, &dsi->regs->hsw);

	return 0;
}

/**
 * sunxi_mipi_dsi_gen_pkt_hdr_write - Write a generic packet header
 * @dsi: DSI controller instance
 * @val: Value to write to the header register
 */
static int sunxi_mipi_dsi_gen_pkt_hdr_write(struct sun6i_mipi_dsi *dsi, u32 val)
{
	writel(val, &dsi->regs->gen_pkt_hdr);

	/* Poll for command completion instead of waiting for interrupt */
	return sunxi_mipi_dsi_wait_for_completion(dsi, INT_STA_CMD_COMPLETED,
						 INT_STA_CMD_COMPLETED);
}

/**
 * sunxi_mipi_dsi_send_cmd - Send a DSI command
 * @dsi: DSI controller instance
 * @msg: Message containing command details
 */
static int sunxi_mipi_dsi_send_cmd(struct sun6i_mipi_dsi *dsi,
				   const struct mipi_dsi_msg *msg)
{
	int i, ret;

	if (msg->tx_len > 0) {
		/* Write payload data */
		for (i = 0; i < msg->tx_len; i += 4) {
			u32 word = 0;

			memcpy(&word, msg->tx_buf + i, min(4, (int)(msg->tx_len - i)));
			writel(word, &dsi->regs->gen_pl_data);
		}
	}

	/* Write command header with polling for completion */
	ret = sunxi_mipi_dsi_gen_pkt_hdr_write(dsi,
		(msg->tx_len << 16) | 
		(msg->type << 8) | 
		(msg->channel));

	return ret;
}

/**
 * sunxi_mipi_dsi_attach - Attach to a MIPI DSI panel
 * @dev: Video bridge device
 */
static int sunxi_mipi_dsi_attach(struct udevice *dev)
{
	struct sun6i_mipi_dsi *dsi = dev_get_priv(dev);
	int ret;

	debug("sunxi_mipi_dsi: Attaching to panel\n");

	/* Initialize the D-PHY */
	ret = sunxi_mipi_dsi_phy_init(dsi);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to initialize PHY: %d\n", ret);
		return ret;
	}

	/* Enable controller */
	setbits_le32(&dsi->regs->ctl, CTL_EN);

	return 0;
}

/**
 * sunxi_mipi_dsi_enable - Enable the display output
 * @dev: Video bridge device
 * @panel_bpp: Bits per pixel for the panel
 * @timing: Display timing information
 */
static int sunxi_mipi_dsi_enable(struct udevice *dev, int panel_bpp,
				const struct display_timing *timing)
{
	struct sun6i_mipi_dsi *dsi = dev_get_priv(dev);
	int ret;

	debug("sunxi_mipi_dsi: Enabling with %d bpp\n", panel_bpp);

	/* Configure video timing */
	ret = sunxi_mipi_dsi_setup_timing(dsi, timing);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to setup timing: %d\n", ret);
		return ret;
	}

	/* Enable the controller */
	setbits_le32(&dsi->regs->ctl, CTL_START);

	return 0;
}

/**
 * sunxi_mipi_dsi_read_timing - Read display timing information
 * @dev: Video bridge device
 * @timing: Timing structure to populate
 */
static int sunxi_mipi_dsi_read_timing(struct udevice *dev,
				      struct display_timing *timing)
{
	/* For now, use timing from device tree or panel */
	/* This could be enhanced to read from panel if supported */
	debug("sunxi_mipi_dsi: Reading display timing\n");

	/* Return -ENOSYS to indicate we rely on other sources for timing */
	return -ENOSYS;
}

/**
 * sunxi_mipi_dsi_probe - Probe the MIPI DSI device
 * @dev: Device to probe
 */
static int sunxi_mipi_dsi_probe(struct udevice *dev)
{
	struct sun6i_mipi_dsi *dsi = dev_get_priv(dev);
	ofnode phynode;
	int ret;

	debug("sunxi_mipi_dsi: Probing MIPI DSI controller\n");

	/* Get register base */
	dsi->regs = (struct sun6i_mipi_dsi_reg *)dev_read_addr(dev);
	if (IS_ERR(dsi->regs)) {
		debug("sunxi_mipi_dsi: Failed to get register base\n");
		return PTR_ERR(dsi->regs);
	}

	/* Get and enable clocks */
	ret = clk_get_by_name(dev, "bus", &dsi->bus_clk);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to get bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_enable(dsi->bus_clk);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to enable bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_get_by_name(dev, "mod", &dsi->mod_clk);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to get mod clock: %d\n", ret);
		goto disable_bus_clk;
	}

	ret = clk_enable(dsi->mod_clk);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to enable mod clock: %d\n", ret);
		goto put_mod_clk;
	}

	/* Get and deassert reset */
	ret = reset_get_by_name(dev, "apb", &dsi->reset);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to get reset: %d\n", ret);
		goto disable_mod_clk;
	}

	ret = reset_deassert(dsi->reset);
	if (ret) {
		debug("sunxi_mipi_dsi: Failed to deassert reset: %d\n", ret);
		goto put_reset;
	}

	/* Try to get the PHY device */
	phynode = dev_read_subnode_by_name(dev, "phy");
	if (ofnode_valid(phynode)) {
		ret = uclass_get_device_by_ofnode(UCLASS_PHY, phynode, &dsi->phy);
		if (ret && ret != -ENOENT) {
			debug("sunxi_mipi_dsi: Failed to get PHY: %d\n", ret);
			goto assert_reset;
		}
	}

	/* Get reset GPIO if present */
	ret = gpio_request_by_name(dev, "reset-gpios", 0, &dsi->reset_gpio,
				   GPIOD_IS_OUT);
	if (ret && ret != -ENOENT) {
		debug("sunxi_mipi_dsi: Failed to get reset GPIO: %d\n", ret);
		goto assert_reset;
	}

	/* Initialize controller */
	writel(0, &dsi->regs->ctl);

	return 0;

assert_reset:
	reset_assert(dsi->reset);
put_reset:
	reset_free(dsi->reset);
disable_mod_clk:
	clk_disable(dsi->mod_clk);
put_mod_clk:
	clk_free(dsi->mod_clk);
disable_bus_clk:
	clk_disable(dsi->bus_clk);

	return ret;
}

/**
 * sunxi_mipi_dsi_remove - Remove the MIPI DSI device
 * @dev: Device to remove
 */
static int sunxi_mipi_dsi_remove(struct udevice *dev)
{
	struct sun6i_mipi_dsi *dsi = dev_get_priv(dev);

	/* Reset the controller */
	writel(0, &dsi->regs->ctl);

	/* Disable clocks */
	if (dsi->mod_clk)
		clk_disable(dsi->mod_clk);
	if (dsi->bus_clk)
		clk_disable(dsi->bus_clk);

	/* Assert reset */
	if (dsi->reset)
		reset_assert(dsi->reset);

	/* Free resources */
	if (dsi->mod_clk)
		clk_free(dsi->mod_clk);
	if (dsi->bus_clk)
		clk_free(dsi->bus_clk);
	if (dsi->reset)
		reset_free(dsi->reset);

	return 0;
}

static const struct video_bridge_ops sunxi_mipi_dsi_ops = {
	.attach = sunxi_mipi_dsi_attach,
	.enable = sunxi_mipi_dsi_enable,
	.read_timing = sunxi_mipi_dsi_read_timing,
};

static const struct udevice_id sunxi_mipi_dsi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-mipi-dsi" },
	{ .compatible = "allwinner,sun8i-t113-mipi-dsi" },
	{ }
};

U_BOOT_DRIVER(sunxi_mipi_dsi) = {
	.name = "sunxi_mipi_dsi",
	.id = UCLASS_VIDEO_BRIDGE,
	.of_match = sunxi_mipi_dsi_ids,
	.ops = &sunxi_mipi_dsi_ops,
	.probe = sunxi_mipi_dsi_probe,
	.remove = sunxi_mipi_dsi_remove,
	.priv_auto = sizeof(struct sun6i_mipi_dsi),
};