/*
 * Copyright (c) 2013, Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/iopoll.h>
#include <linux/platform_device.h>

#include <mach/msm_bus.h>

#include "ufshcd.h"
#include "unipro.h"
#include "ufs-msm.h"

/* vendor specific pre-defined parameters */
#define SLOW 1
#define FAST 2

#define UFS_MSM_LIMIT_NUM_LANES_RX	2
#define UFS_MSM_LIMIT_NUM_LANES_TX	2
#define UFS_MSM_LIMIT_HSGEAR_RX	UFS_HS_G2
#define UFS_MSM_LIMIT_HSGEAR_TX	UFS_HS_G2
#define UFS_MSM_LIMIT_PWMGEAR_RX	UFS_PWM_G4
#define UFS_MSM_LIMIT_PWMGEAR_TX	UFS_PWM_G4
#define UFS_MSM_LIMIT_RX_PWR_PWM	SLOW_MODE
#define UFS_MSM_LIMIT_TX_PWR_PWM	SLOW_MODE
#define UFS_MSM_LIMIT_RX_PWR_HS	FAST_MODE
#define UFS_MSM_LIMIT_TX_PWR_HS	FAST_MODE
#define UFS_MSM_LIMIT_HS_RATE		PA_HS_MODE_B
#define UFS_MSM_LIMIT_DESIRED_MODE	FAST

static struct msm_ufs_phy_calibration phy_cal_table_rate_A[] = {
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_POWER_DOWN_CONTROL,
	},
	{
		.cfg_value = 0xFF,
		.reg_offset = QSERDES_COM_PLL_CRCTRL,
	},
	{
		.cfg_value = 0x24,
		.reg_offset = QSERDES_COM_PLL_CNTRL,
	},
	{
		.cfg_value = 0x08,
		.reg_offset = QSERDES_COM_SYSCLK_EN_SEL,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_SYS_CLK_CTRL,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = QSERDES_COM_PLL_CLKEPDIV,
	},
	{
		.cfg_value = 0x82,
		.reg_offset = QSERDES_COM_DEC_START1,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = QSERDES_COM_DEC_START2,
	},
	{
		.cfg_value = 0x80,
		.reg_offset = QSERDES_COM_DIV_FRAC_START1,
	},
	{
		.cfg_value = 0x80,
		.reg_offset = QSERDES_COM_DIV_FRAC_START2,
	},
	{
		.cfg_value = 0x10,
		.reg_offset = QSERDES_COM_DIV_FRAC_START3,
	},
	{
		.cfg_value = 0xff,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP1,
	},
	{
		.cfg_value = 0x67,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP2,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP3,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP_EN,
	},
	{
		.cfg_value = 0x10,
		.reg_offset = QSERDES_COM_RESETSM_CNTRL,
	},
	{
		.cfg_value = 0x13,
		.reg_offset = QSERDES_COM_PLL_RXTXEPCLK_EN,
	},
	{
		.cfg_value = 0x43,
		.reg_offset = QSERDES_RX_PWM_CNTRL1(0),
	},
	{
		.cfg_value = 0x43,
		.reg_offset = QSERDES_RX_PWM_CNTRL1(1),
	},
	{
		.cfg_value = 0x22,
		.reg_offset = QSERDES_RX_CDR_CONTROL(0),
	},
	{
		.cfg_value = 0x12,
		.reg_offset = QSERDES_RX_CDR_CONTROL_HALF(0),
	},
	{
		.cfg_value = 0x2a,
		.reg_offset = QSERDES_RX_CDR_CONTROL_QUARTER(0),
	},
	{
		.cfg_value = 0x22,
		.reg_offset = QSERDES_RX_CDR_CONTROL(1),
	},
	{
		.cfg_value = 0x12,
		.reg_offset = QSERDES_RX_CDR_CONTROL_HALF(1),
	},
	{
		.cfg_value = 0x2a,
		.reg_offset = QSERDES_RX_CDR_CONTROL_QUARTER(1),
	},
	{
		.cfg_value = 0xC0,
		.reg_offset = QSERDES_RX_SIGDET_CNTRL(0),
	},
	{
		.cfg_value = 0xC0,
		.reg_offset = QSERDES_RX_SIGDET_CNTRL(1),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_SIGDET_CNTRL2(0),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_SIGDET_CNTRL2(1),
	},
	{
		.cfg_value = 0x50,
		.reg_offset = UFS_PHY_PWM_G1_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x28,
		.reg_offset = UFS_PHY_PWM_G2_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x10,
		.reg_offset = UFS_PHY_PWM_G3_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x08,
		.reg_offset = UFS_PHY_PWM_G4_CLK_DIVIDER,
	},
	{
		.cfg_value = 0xa8,
		.reg_offset = UFS_PHY_CORECLK_PWM_G1_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x54,
		.reg_offset = UFS_PHY_CORECLK_PWM_G2_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x2a,
		.reg_offset = UFS_PHY_CORECLK_PWM_G3_CLK_DIVIDER,
	},
	{
		.cfg_value = 0x15,
		.reg_offset = UFS_PHY_CORECLK_PWM_G4_CLK_DIVIDER,
	},
	{
		.cfg_value = 0xff,
		.reg_offset = UFS_PHY_OMC_STATUS_RDVAL,
	},
	{
		.cfg_value = 0x1f,
		.reg_offset = UFS_PHY_LINE_RESET_TIME,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = UFS_PHY_LINE_RESET_GRANULARITY,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = UFS_PHY_TSYNC_RSYNC_CNTL,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_PLL_CNTL,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = UFS_PHY_TX_LARGE_AMP_DRV_LVL,
	},
	{
		.cfg_value = 0x1a,
		.reg_offset = UFS_PHY_TX_SMALL_AMP_DRV_LVL,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = UFS_PHY_TX_LARGE_AMP_POST_EMP_LVL,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = UFS_PHY_TX_SMALL_AMP_POST_EMP_LVL,
	},
	{
		.cfg_value = 0x09,
		.reg_offset = UFS_PHY_CFG_CHANGE_CNT_VAL,
	},
	{
		.cfg_value = 0x30,
		.reg_offset = UFS_PHY_RX_SYNC_WAIT_TIME,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x08,
		.reg_offset = UFS_PHY_RX_MIN_SLEEP_NOCONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_MIN_STALL_NOCONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = UFS_PHY_RX_MIN_STALL_NOCONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x04,
		.reg_offset = UFS_PHY_TX_MIN_SAVE_CONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0xc8,
		.reg_offset = UFS_PHY_RX_MIN_SAVE_CONFIG_TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x10,
		.reg_offset = UFS_PHY_RX_PWM_BURST_CLOSURE_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RX_MIN_ACTIVATETIME_CAPABILITY,
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_RX_EQ_GAIN1(0),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_RX_EQ_GAIN2(0),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_RX_EQ_GAIN1(1),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = QSERDES_RX_RX_EQ_GAIN2(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_CDR_CONTROL3(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_CDR_CONTROL3(1),
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_COM_PLL_IP_SETI,
	},
	{
		.cfg_value = 0x3f,
		.reg_offset = QSERDES_COM_PLL_CP_SETI,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_COM_PLL_IP_SETP,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_COM_PLL_CP_SETP,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_RES_TRIM_OFFSET,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = QSERDES_COM_BGTC,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_PLL_AMP_OS,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = QSERDES_TX_TX_DRV_LVL(0),
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = QSERDES_TX_TX_DRV_LVL(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_BIST_MODE_LANENO(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_BIST_MODE_LANENO(1),
	},
	{
		.cfg_value = 0x04,
		.reg_offset = QSERDES_TX_TX_EMP_POST1_LVL(0),
	},
	{
		.cfg_value = 0x04,
		.reg_offset = QSERDES_TX_TX_EMP_POST1_LVL(1),
	},
	{
		.cfg_value = 0x05,
		.reg_offset = QSERDES_TX_HIGHZ_TRANSCEIVEREN_BIAS_EN(0),
	},
	{
		.cfg_value = 0x05,
		.reg_offset = QSERDES_TX_HIGHZ_TRANSCEIVEREN_BIAS_EN(1),
	},
	{
		.cfg_value = 0x07,
		.reg_offset = UFS_PHY_TIMER_100US_SYSCLK_STEPS_MSB,
	},
	{
		.cfg_value = 0x80,
		.reg_offset = UFS_PHY_TIMER_100US_SYSCLK_STEPS_LSB,
	},
	{
		.cfg_value = 0x27,
		.reg_offset = UFS_PHY_TIMER_20US_CORECLK_STEPS_MSB,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = UFS_PHY_TIMER_20US_CORECLK_STEPS_LSB,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = UFS_PHY_CONTROLSYM_ONE_HOT_DISABLE,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RETIME_BUFFER_EN,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = UFS_PHY_TX_HSGEAR_CAPABILITY,
	},
	{
		.cfg_value = 0x04,
		.reg_offset = UFS_PHY_TX_PWMGEAR_CAPABILITY,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = UFS_PHY_TX_AMPLITUDE_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_EXTERNALSYNC_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_HS_UNTERMINATED_LINE_DRIVE_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_LS_TERMINATED_LINE_DRIVE_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_REF_CLOCK_SHARED_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_TX_HIBERN8TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = UFS_PHY_RX_HSGEAR_CAPABILITY,
	},
	{
		.cfg_value = 0x04,
		.reg_offset = UFS_PHY_RX_PWMGEAR_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RX_HS_UNTERMINATED_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RX_LS_TERMINATED_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RX_REF_CLOCK_SHARED_CAPABILITY,
	},
	{
		.cfg_value = 0x48,
		.reg_offset = UFS_PHY_RX_HS_G1_SYNC_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = UFS_PHY_RX_HS_G1_PREPARE_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x09,
		.reg_offset = UFS_PHY_RX_LS_PREPARE_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x01,
		.reg_offset = UFS_PHY_RX_HIBERN8TIME_CAPABILITY,
	},
	{
		.cfg_value = 0x48,
		.reg_offset = UFS_PHY_RX_HS_G2_SYNC_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x48,
		.reg_offset = UFS_PHY_RX_HS_G3_SYNC_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = UFS_PHY_RX_HS_G2_PREPARE_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x0f,
		.reg_offset = UFS_PHY_RX_HS_G3_PREPARE_LENGTH_CAPABILITY,
	},
	{
		.cfg_value = 0x09,
		.reg_offset = QSERDES_TX_CLKBUF_ENABLE(0),
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_TX_RESET_TSYNC_EN(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_RES_CODE(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_SERDES_BYP_EN_OUT(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_REC_DETECT_LVL(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_PARRATE_REC_DETECT_IDLE_EN(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_TRAN_DRVR_EMP_EN(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_AUX_CONTROL(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_AUX_DATA_TCODE(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_RCLK_AUXDATA_SEL(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_EQ_CONTROL(0),
	},
	{
		.cfg_value = 0x73,
		.reg_offset = QSERDES_RX_RX_IQ_RXDET_EN(0),
	},
	{
		.cfg_value = 0x05,
		.reg_offset = QSERDES_RX_RX_TERM_HIGHZ_CM_AC_COUPLE(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_CDR_FREEZE_UP_DN(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_UFS_CNTRL(0),
	},
	{
		.cfg_value = 0x22,
		.reg_offset = QSERDES_RX_CDR_CONTROL_EIGHTH(0),
	},
	{
		.cfg_value = 0x0a,
		.reg_offset = QSERDES_RX_UCDR_FO_GAIN(0),
	},
	{
		.cfg_value = 0x06,
		.reg_offset = QSERDES_RX_UCDR_SO_GAIN(0),
	},
	{
		.cfg_value = 0x35,
		.reg_offset = QSERDES_RX_UCDR_SO_SATURATION_AND_ENABLE(0),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_UCDR_FO_TO_SO_DELAY(0),
	},
	{
		.cfg_value = 0x09,
		.reg_offset = QSERDES_TX_CLKBUF_ENABLE(1),
	},
	{
		.cfg_value = 0x01,
		.reg_offset = QSERDES_TX_RESET_TSYNC_EN(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_RES_CODE(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_SERDES_BYP_EN_OUT(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_REC_DETECT_LVL(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_PARRATE_REC_DETECT_IDLE_EN(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_TX_TRAN_DRVR_EMP_EN(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_AUX_CONTROL(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_AUX_DATA_TCODE(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_RCLK_AUXDATA_SEL(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_EQ_CONTROL(1),
	},
	{
		.cfg_value = 0x73,
		.reg_offset = QSERDES_RX_RX_IQ_RXDET_EN(1),
	},
	{
		.cfg_value = 0x05,
		.reg_offset = QSERDES_RX_RX_TERM_HIGHZ_CM_AC_COUPLE(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_CDR_FREEZE_UP_DN(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_UFS_CNTRL(1),
	},
	{
		.cfg_value = 0x22,
		.reg_offset = QSERDES_RX_CDR_CONTROL_EIGHTH(1),
	},
	{
		.cfg_value = 0x0a,
		.reg_offset = QSERDES_RX_UCDR_FO_GAIN(1),
	},
	{
		.cfg_value = 0x06,
		.reg_offset = QSERDES_RX_UCDR_SO_GAIN(1),
	},
	{
		.cfg_value = 0x35,
		.reg_offset = QSERDES_RX_UCDR_SO_SATURATION_AND_ENABLE(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_RX_UCDR_FO_TO_SO_DELAY(1),
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_CMN_MODE,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_IE_TRIM,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_IP_TRIM,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_CORE_CLK_IN_SYNC_SEL,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_BIAS_EN_CLKBUFLR_EN,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_PLL_TEST_UPDN_RESTRIMSTEP,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_FAUX_EN,
	},
};

static struct msm_ufs_phy_calibration phy_cal_table_rate_B[] = {
	{
		.cfg_value = 0x03,
		.reg_offset = QSERDES_COM_PLL_CLKEPDIV,
	},
	{
		.cfg_value = 0x98,
		.reg_offset = QSERDES_COM_DEC_START1,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = QSERDES_COM_DEC_START2,
	},
	{
		.cfg_value = 0x80,
		.reg_offset = QSERDES_COM_DIV_FRAC_START1,
	},
	{
		.cfg_value = 0x80,
		.reg_offset = QSERDES_COM_DIV_FRAC_START2,
	},
	{
		.cfg_value = 0x10,
		.reg_offset = QSERDES_COM_DIV_FRAC_START3,
	},
	{
		.cfg_value = 0x65,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP1,
	},
	{
		.cfg_value = 0x1E,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP2,
	},
	{
		.cfg_value = 0x00,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP3,
	},
	{
		.cfg_value = 0x03,
		.reg_offset = QSERDES_COM_PLLLOCK_CMP_EN,
	},
};

static struct msm_ufs_phy *msm_get_ufs_phy(struct device *dev)
{
	int err  = -EPROBE_DEFER;
	struct msm_ufs_phy *phy;
	struct device_node *node;

	if (list_empty(&phy_list))
		goto out;

	node = of_parse_phandle(dev->of_node, "ufs-phy", 0);
	if (!node) {
		err = -EINVAL;
		dev_err(dev, "%s: ufs-phy property not specified\n", __func__);
		goto out;
	}

	list_for_each_entry(phy, &phy_list, list) {
		if (phy->dev->of_node == node) {
			err = 0;
			break;
		}
	}

	of_node_put(node);
out:
	if (err)
		return ERR_PTR(err);
	return phy;
}

/* Turn ON M-PHY RMMI interface clocks */
static int msm_ufs_enable_phy_iface_clk(struct msm_ufs_phy *phy)
{
	int ret = 0;

	if (phy->is_iface_clk_enabled)
		goto out;

	ret = clk_prepare_enable(phy->tx_iface_clk);
	if (ret)
		goto out;
	ret = clk_prepare_enable(phy->rx_iface_clk);
	if (ret)
		goto disable_tx_iface_clk;

	phy->is_iface_clk_enabled = true;

disable_tx_iface_clk:
	if (ret)
		clk_disable_unprepare(phy->tx_iface_clk);
out:
	if (ret)
		dev_err(phy->dev, "%s: iface_clk enable failed %d\n",
				__func__, ret);
	return ret;
}

/* Turn OFF M-PHY RMMI interface clocks */
static void msm_ufs_disable_phy_iface_clk(struct msm_ufs_phy *phy)
{
	if (phy->is_iface_clk_enabled) {
		clk_disable_unprepare(phy->tx_iface_clk);
		clk_disable_unprepare(phy->rx_iface_clk);
		phy->is_iface_clk_enabled = false;
	}
}

static int msm_ufs_enable_phy_ref_clk(struct msm_ufs_phy *phy)
{
	int ret = 0;

	if (phy->is_ref_clk_enabled)
		goto out;

	/*
	 * reference clock is propagated in a daisy-chained manner from
	 * source to phy, so ungate them at each stage.
	 */
	ret = clk_prepare_enable(phy->ref_clk_src);
	if (ret) {
		dev_err(phy->dev, "%s: ref_clk_src enable failed %d\n",
				__func__, ret);
		goto out;
	}

	ret = clk_prepare_enable(phy->ref_clk_parent);
	if (ret) {
		dev_err(phy->dev, "%s: ref_clk_parent enable failed %d\n",
				__func__, ret);
		goto out_disable_src;
	}

	ret = clk_prepare_enable(phy->ref_clk);
	if (ret) {
		dev_err(phy->dev, "%s: ref_clk enable failed %d\n",
				__func__, ret);
		goto out_disable_parent;
	}

	phy->is_ref_clk_enabled = true;
	goto out;

out_disable_parent:
	clk_disable_unprepare(phy->ref_clk_parent);
out_disable_src:
	clk_disable_unprepare(phy->ref_clk_src);
out:
	return ret;
}

static void msm_ufs_disable_phy_ref_clk(struct msm_ufs_phy *phy)
{
	if (phy->is_ref_clk_enabled) {
		clk_disable_unprepare(phy->ref_clk);
		clk_disable_unprepare(phy->ref_clk_parent);
		clk_disable_unprepare(phy->ref_clk_src);
		phy->is_ref_clk_enabled = false;
	}
}


static int msm_ufs_phy_cfg_vreg(struct device *dev,
				struct msm_ufs_phy_vreg *vreg, bool on)
{
	int ret = 0;
	struct regulator *reg = vreg->reg;
	const char *name = vreg->name;
	int min_uV, uA_load;

	BUG_ON(!vreg);

	if (regulator_count_voltages(reg) > 0) {
		min_uV = on ? vreg->min_uV : 0;
		ret = regulator_set_voltage(reg, min_uV, vreg->max_uV);
		if (ret) {
			dev_err(dev, "%s: %s set voltage failed, err=%d\n",
					__func__, name, ret);
			goto out;
		}

		uA_load = on ? vreg->max_uA : 0;
		ret = regulator_set_optimum_mode(reg, uA_load);
		if (ret >= 0) {
			/*
			 * regulator_set_optimum_mode() returns new regulator
			 * mode upon success.
			 */
			ret = 0;
		} else {
			dev_err(dev, "%s: %s set optimum mode(uA_load=%d) failed, err=%d\n",
					__func__, name, uA_load, ret);
			goto out;
		}
	}
out:
	return ret;
}

static int msm_ufs_phy_enable_vreg(struct msm_ufs_phy *phy,
					struct msm_ufs_phy_vreg *vreg)
{
	struct device *dev = phy->dev;
	int ret = 0;

	if (!vreg || vreg->enabled)
		goto out;

	ret = msm_ufs_phy_cfg_vreg(dev, vreg, true);
	if (!ret)
		ret = regulator_enable(vreg->reg);

	if (!ret)
		vreg->enabled = true;
	else
		dev_err(dev, "%s: %s enable failed, err=%d\n",
				__func__, vreg->name, ret);
out:
	return ret;
}

static int msm_ufs_phy_disable_vreg(struct msm_ufs_phy *phy,
					struct msm_ufs_phy_vreg *vreg)
{
	struct device *dev = phy->dev;
	int ret = 0;

	if (!vreg || !vreg->enabled)
		goto out;

	ret = regulator_disable(vreg->reg);

	if (!ret) {
		/* ignore errors on applying disable config */
		msm_ufs_phy_cfg_vreg(dev, vreg, false);
		vreg->enabled = false;
	} else {
		dev_err(dev, "%s: %s disable failed, err=%d\n",
				__func__, vreg->name, ret);
	}
out:
	return ret;
}

static void msm_ufs_phy_calibrate(struct msm_ufs_phy *phy)
{
	struct msm_ufs_phy_calibration *tbl = phy_cal_table_rate_A;
	int tbl_size = ARRAY_SIZE(phy_cal_table_rate_A);
	int i;

	for (i = 0; i < tbl_size; i++)
		writel_relaxed(tbl[i].cfg_value, phy->mmio + tbl[i].reg_offset);

	if (UFS_MSM_LIMIT_HS_RATE == PA_HS_MODE_B) {
		tbl = phy_cal_table_rate_B;
		tbl_size = ARRAY_SIZE(phy_cal_table_rate_B);

		for (i = 0; i < tbl_size; i++)
			writel_relaxed(tbl[i].cfg_value,
					phy->mmio + tbl[i].reg_offset);
	}

	/* flush buffered writes */
	mb();
}

static int msm_ufs_host_clk_get(struct device *dev,
		const char *name, struct clk **clk_out)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "%s: failed to get %s err %d",
				__func__, name, err);
	} else {
		*clk_out = clk;
	}

	return err;
}

static int msm_ufs_host_clk_enable(struct device *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

static void msm_ufs_disable_lane_clks(struct msm_ufs_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	clk_disable_unprepare(host->tx_l1_sync_clk);
	clk_disable_unprepare(host->tx_l0_sync_clk);
	clk_disable_unprepare(host->rx_l1_sync_clk);
	clk_disable_unprepare(host->rx_l0_sync_clk);

	host->is_lane_clks_enabled = false;
}

static int msm_ufs_enable_lane_clks(struct msm_ufs_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (host->is_lane_clks_enabled)
		return 0;

	err = msm_ufs_host_clk_enable(dev,
			"rx_lane0_sync_clk", host->rx_l0_sync_clk);
	if (err)
		goto out;

	err = msm_ufs_host_clk_enable(dev,
			"rx_lane1_sync_clk", host->rx_l1_sync_clk);
	if (err)
		goto disable_rx_l0;

	err = msm_ufs_host_clk_enable(dev,
			"tx_lane0_sync_clk", host->tx_l0_sync_clk);
	if (err)
		goto disable_rx_l1;

	err = msm_ufs_host_clk_enable(dev,
			"tx_lane1_sync_clk", host->tx_l1_sync_clk);
	if (err)
		goto disable_tx_l0;

	host->is_lane_clks_enabled = true;
	goto out;

disable_tx_l0:
	clk_disable_unprepare(host->tx_l0_sync_clk);
disable_rx_l1:
	clk_disable_unprepare(host->rx_l1_sync_clk);
disable_rx_l0:
	clk_disable_unprepare(host->rx_l0_sync_clk);
out:
	return err;
}

static int msm_ufs_init_lane_clks(struct msm_ufs_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	err = msm_ufs_host_clk_get(dev,
			"rx_lane0_sync_clk", &host->rx_l0_sync_clk);
	if (err)
		goto out;

	err = msm_ufs_host_clk_get(dev,
			"rx_lane1_sync_clk", &host->rx_l1_sync_clk);
	if (err)
		goto out;

	err = msm_ufs_host_clk_get(dev,
			"tx_lane0_sync_clk", &host->tx_l0_sync_clk);
	if (err)
		goto out;

	err = msm_ufs_host_clk_get(dev,
			"tx_lane1_sync_clk", &host->tx_l1_sync_clk);
out:
	return err;
}

static int msm_ufs_enable_tx_lanes(struct ufs_hba *hba)
{
	int err;
	u32 tx_lanes;
	u32 val;
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;

	err = ufshcd_dme_get(hba,
			UIC_ARG_MIB(PA_CONNECTEDTXDATALANES), &tx_lanes);
	if (err) {
		dev_err(hba->dev, "%s: couldn't read PA_CONNECTEDTXDATALANES %d\n",
				__func__, err);
		goto out;
	}

	val = ~(MAX_U32 << tx_lanes);
	writel_relaxed(val, phy->mmio + UFS_PHY_TX_LANE_ENABLE);
	mb();
out:
	return err;
}

static int msm_ufs_check_hibern8(struct ufs_hba *hba)
{
	int err;
	u32 tx_fsm_val = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(HBRN8_POLL_TOUT_MS);

	do {
		err = ufshcd_dme_get(hba,
			UIC_ARG_MIB(MPHY_TX_FSM_STATE), &tx_fsm_val);
		if (err || tx_fsm_val == TX_FSM_HIBERN8)
			break;

		/* sleep for max. 200us */
		usleep_range(100, 200);
	} while (time_before(jiffies, timeout));

	/*
	 * we might have scheduled out for long during polling so
	 * check the state again.
	 */
	if (time_after(jiffies, timeout))
		err = ufshcd_dme_get(hba,
				UIC_ARG_MIB(MPHY_TX_FSM_STATE), &tx_fsm_val);

	if (err) {
		dev_err(hba->dev, "%s: unable to get TX_FSM_STATE, err %d\n",
				__func__, err);
	} else if (tx_fsm_val != TX_FSM_HIBERN8) {
		err = tx_fsm_val;
		dev_err(hba->dev, "%s: invalid TX_FSM_STATE = %d\n",
				__func__, err);
	}

	return err;
}

static inline void msm_ufs_phy_start_serdes(struct msm_ufs_phy *phy)
{
	u32 tmp;

	tmp = readl_relaxed(phy->mmio + UFS_PHY_PHY_START);
	tmp &= ~MASK_SERDES_START;
	tmp |= (1 << OFFSET_SERDES_START);
	writel_relaxed(tmp, phy->mmio + UFS_PHY_PHY_START);
	mb();
}

static int msm_ufs_phy_power_on(struct msm_ufs_phy *phy)
{
	int err;

	err = msm_ufs_phy_enable_vreg(phy, &phy->vdda_phy);
	if (err)
		goto out;

	/* vdda_pll also enables ref clock LDOs so enable it first */
	err = msm_ufs_phy_enable_vreg(phy, &phy->vdda_pll);
	if (err)
		goto out_disable_phy;

	err = msm_ufs_enable_phy_ref_clk(phy);
	if (err)
		goto out_disable_pll;

	err = msm_ufs_enable_phy_iface_clk(phy);
	if (err)
		goto out_disable_ref;

	goto out;

out_disable_ref:
	msm_ufs_disable_phy_ref_clk(phy);
out_disable_pll:
	msm_ufs_phy_disable_vreg(phy, &phy->vdda_pll);
out_disable_phy:
	msm_ufs_phy_disable_vreg(phy, &phy->vdda_phy);
out:
	return err;
}

static int msm_ufs_phy_power_off(struct msm_ufs_phy *phy)
{
	writel_relaxed(0x0, phy->mmio + UFS_PHY_POWER_DOWN_CONTROL);
	mb();

	msm_ufs_disable_phy_iface_clk(phy);
	msm_ufs_disable_phy_ref_clk(phy);

	msm_ufs_phy_disable_vreg(phy, &phy->vdda_pll);
	msm_ufs_phy_disable_vreg(phy, &phy->vdda_phy);

	return 0;
}

static inline void msm_ufs_assert_reset(struct ufs_hba *hba)
{
	ufshcd_rmwl(hba, MASK_UFS_PHY_SOFT_RESET,
			1 << OFFSET_UFS_PHY_SOFT_RESET, REG_UFS_CFG1);
	mb();
}

static inline void msm_ufs_deassert_reset(struct ufs_hba *hba)
{
	ufshcd_rmwl(hba, MASK_UFS_PHY_SOFT_RESET,
			0 << OFFSET_UFS_PHY_SOFT_RESET, REG_UFS_CFG1);
	mb();
}

static int msm_ufs_hce_enable_notify(struct ufs_hba *hba, bool status)
{
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;
	u32 val;
	int err = -EINVAL;

	switch (status) {
	case PRE_CHANGE:
		/* Assert PHY reset and apply PHY calibration values */
		msm_ufs_assert_reset(hba);

		/* provide 1ms delay to let the reset pulse propagate */
		usleep_range(1000, 1100);

		msm_ufs_phy_calibrate(phy);

		/* De-assert PHY reset and start serdes */
		msm_ufs_deassert_reset(hba);

		/*
		 * after reset deassertion, phy will need all ref clocks,
		 * voltage, current to settle down before starting serdes.
		 */
		usleep_range(1000, 1100);

		msm_ufs_phy_start_serdes(phy);

		/* poll for PCS_READY for max. 1sec */
		err = readl_poll_timeout(phy->mmio + UFS_PHY_PCS_READY_STATUS,
				val, (val & MASK_PCS_READY), 10, 1000000);
		if (err) {
			dev_err(phy->dev, "%s: phy init failed, %d\n",
					__func__, err);
			break;
		}
		/*
		 * The PHY PLL output is the source of tx/rx lane symbol clocks.
		 * Hence, enable the lane clocks only after PHY is initialized.
		 */
		err = msm_ufs_enable_lane_clks(host);
		break;
	case POST_CHANGE:
		/* check if UFS PHY moved from DISABLED to HIBERN8 */
		err = msm_ufs_check_hibern8(hba);
	default:
		break;
	}

	return err;
}

static int msm_ufs_link_startup_notify(struct ufs_hba *hba, bool status)
{
	unsigned long core_clk_rate = 0;
	u32 core_clk_cycles_per_100ms;

	switch (status) {
	case PRE_CHANGE:
		core_clk_rate = msm_ufs_cfg_timers(hba, 0, 0);
		core_clk_cycles_per_100ms =
			(core_clk_rate / MSEC_PER_SEC) * 100;
		ufshcd_writel(hba, core_clk_cycles_per_100ms,
					REG_UFS_PA_LINK_STARTUP_TIMER);
		break;
	case POST_CHANGE:
		msm_ufs_enable_tx_lanes(hba);
		break;
	default:
		break;
	}

	return 0;
}

static int msm_ufs_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;
	int ret = 0;

	if (!phy)
		return 0;

	if (ufshcd_is_link_off(hba)) {
		/*
		 * Disable the tx/rx lane symbol clocks before PHY is
		 * powered down as the PLL source should be disabled
		 * after downstream clocks are disabled.
		 */
		msm_ufs_disable_lane_clks(host);
		msm_ufs_phy_power_off(phy);
		goto out;
	}

	/* M-PHY RMMI interface clocks can be turned off */
	msm_ufs_disable_phy_iface_clk(phy);

	/*
	 * If UniPro link is not active, PHY ref_clk, main PHY analog power
	 * rail and low noise analog power rail for PLL can be switched off.
	 */
	if (!ufshcd_is_link_active(hba)) {
		msm_ufs_disable_phy_ref_clk(phy);
		msm_ufs_phy_disable_vreg(phy, &phy->vdda_phy);
		msm_ufs_phy_disable_vreg(phy, &phy->vdda_pll);
	}

out:
	return ret;
}

static int msm_ufs_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;

	if (!phy)
		return 0;

	return msm_ufs_phy_power_on(phy);
}

struct ufs_msm_dev_params {
	u32 pwm_rx_gear;	/* pwm rx gear to work in */
	u32 pwm_tx_gear;	/* pwm tx gear to work in */
	u32 hs_rx_gear;		/* hs rx gear to work in */
	u32 hs_tx_gear;		/* hs tx gear to work in */
	u32 rx_lanes;		/* number of rx lanes */
	u32 tx_lanes;		/* number of tx lanes */
	u32 rx_pwr_pwm;		/* rx pwm working pwr */
	u32 tx_pwr_pwm;		/* tx pwm working pwr */
	u32 rx_pwr_hs;		/* rx hs working pwr */
	u32 tx_pwr_hs;		/* tx hs working pwr */
	u32 hs_rate;		/* rate A/B to work in HS */
	u32 desired_working_mode;
};

/**
 * as every power mode, according to the UFS spec, have a defined
 * number that are not corresponed to their order or power
 * consumption (i.e 5, 2, 4, 1 respectively from low to high),
 * we need to map them into array, so we can scan it easily
 * in order to find the minimum required power mode.
 * also, we can use this routine to go the other way around,
 * and from array index, the fetch the correspond power mode.
 */
static int map_unmap_pwr_mode(u32 mode, bool is_pwr_to_arr)
{
	enum {SL_MD = 0, SLA_MD = 1, FS_MD = 2, FSA_MD = 3, UNDEF = 4};
	int ret = -EINVAL;

	if (is_pwr_to_arr) {
		switch (mode) {
		case SLOW_MODE:
			ret = SL_MD;
			break;
		case SLOWAUTO_MODE:
			ret = SLA_MD;
			break;
		case FAST_MODE:
			ret = FS_MD;
			break;
		case FASTAUTO_MODE:
			ret = FSA_MD;
			break;
		default:
			ret = UNDEF;
			break;
		}
	} else {
		switch (mode) {
		case SL_MD:
			ret = SLOW_MODE;
			break;
		case SLA_MD:
			ret = SLOWAUTO_MODE;
			break;
		case FS_MD:
			ret = FAST_MODE;
			break;
		case FSA_MD:
			ret = FASTAUTO_MODE;
			break;
		default:
			ret = -EINVAL;
			break;
		}
	}

	return ret;
}

#define NUM_OF_SUPPORTED_MODES	5
static int get_pwr_dev_param(struct ufs_msm_dev_params *msm_param,
				struct ufs_pa_layer_attr *dev_max,
				struct ufs_pa_layer_attr *dev_req)
{
	int arr[NUM_OF_SUPPORTED_MODES] = {0};
	int i;
	int min_power;
	int min_msm_gear;
	int min_dev_gear;
	bool is_max_dev_hs;
	bool is_max_msm_hs;

	/**
	 * mapping the max. supported power mode of the device
	 * and the max. pre-defined support power mode of the vendor
	 * in order to scan them easily
	 */
	arr[map_unmap_pwr_mode(dev_max->pwr_rx, true)]++;
	arr[map_unmap_pwr_mode(dev_max->pwr_tx, true)]++;

	if (msm_param->desired_working_mode == SLOW) {
		arr[map_unmap_pwr_mode(msm_param->rx_pwr_pwm, true)]++;
		arr[map_unmap_pwr_mode(msm_param->tx_pwr_pwm, true)]++;
	} else {
		arr[map_unmap_pwr_mode(msm_param->rx_pwr_hs, true)]++;
		arr[map_unmap_pwr_mode(msm_param->tx_pwr_hs, true)]++;
	}

	for (i = 0; i < NUM_OF_SUPPORTED_MODES; ++i) {
		if (arr[i] != 0)
			break;
	}

	/* no supported power mode found */
	if (i == NUM_OF_SUPPORTED_MODES) {
		return -EINVAL;
	} else {
		min_power = map_unmap_pwr_mode(i, false);
		if (min_power >= 0)
			dev_req->pwr_rx = dev_req->pwr_tx = min_power;
		else
			return -EINVAL;
	}

	/**
	 * we would like tx to work in the minimum number of lanes
	 * between device capability and vendor preferences.
	 * the same decision will be made for rx.
	 */
	dev_req->lane_tx = min_t(u32, dev_max->lane_tx, msm_param->tx_lanes);
	dev_req->lane_rx = min_t(u32, dev_max->lane_rx, msm_param->rx_lanes);

	if (dev_max->pwr_rx == SLOW_MODE ||
	    dev_max->pwr_rx == SLOWAUTO_MODE)
		is_max_dev_hs = false;
	else
		is_max_dev_hs = true;

	/* setting the device maximum gear */
	min_dev_gear = min_t(u32, dev_max->gear_rx, dev_max->gear_tx);

	/**
	 * setting the desired gear to be the minimum according to the desired
	 * power mode
	 */
	if (msm_param->desired_working_mode == SLOW) {
		is_max_msm_hs = false;
		min_msm_gear = min_t(u32, msm_param->pwm_rx_gear,
						msm_param->pwm_tx_gear);
	} else {
		is_max_msm_hs = true;
		min_msm_gear = min_t(u32, msm_param->hs_rx_gear,
						msm_param->hs_tx_gear);
	}

	/**
	 * if both device capabilities and vendor pre-defined preferences are
	 * both HS or both PWM then set the minimum gear to be the
	 * chosen working gear.
	 * if one is PWM and one is HS then the one that is PWM get to decide
	 * what the gear, as he is the one that also decided previously what
	 * pwr the device will be configured to.
	 */
	if ((is_max_dev_hs && is_max_msm_hs) ||
	    (!is_max_dev_hs && !is_max_msm_hs)) {
		dev_req->gear_rx = dev_req->gear_tx =
			min_t(u32, min_dev_gear, min_msm_gear);
	} else if (!is_max_dev_hs) {
		dev_req->gear_rx = dev_req->gear_tx = min_dev_gear;
	} else {
		dev_req->gear_rx = dev_req->gear_tx = min_msm_gear;
	}

	dev_req->hs_rate = msm_param->hs_rate;

	return 0;
}

static unsigned long
msm_ufs_cfg_timers(struct ufs_hba *hba, u32 gear, u32 hs)
{
	struct ufs_clk_info *clki;
	u32 core_clk_period_in_ns;
	u32 tx_clk_cycles_per_us = 0;
	unsigned long core_clk_rate = 0;

	static u32 pwm_fr_table[][2] = {
		{UFS_PWM_G1, 0x1},
		{UFS_PWM_G2, 0x1},
		{UFS_PWM_G3, 0x1},
		{UFS_PWM_G4, 0x1},
	};

	static u32 hs_fr_table_rA[][2] = {
		{UFS_HS_G1, 0x1F},
		{UFS_HS_G2, 0x3e},
	};

	if (gear == 0 && hs == 0) {
		gear = UFS_PWM_G1;
		hs = SLOWAUTO_MODE;
	}

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, "core_clk"))
			core_clk_rate = clk_get_rate(clki->clk);
	}

	/* If frequency is smaller than 1MHz, set to 1MHz */
	if (core_clk_rate < DEFAULT_CLK_RATE_HZ)
		core_clk_rate = DEFAULT_CLK_RATE_HZ;

	core_clk_period_in_ns = NSEC_PER_SEC / core_clk_rate;
	core_clk_period_in_ns <<= OFFSET_CLK_NS_REG;
	core_clk_period_in_ns &= MASK_CLK_NS_REG;

	switch (hs) {
	case FASTAUTO_MODE:
	case FAST_MODE:
		tx_clk_cycles_per_us = hs_fr_table_rA[gear-1][1];
		break;
	case SLOWAUTO_MODE:
	case SLOW_MODE:
		tx_clk_cycles_per_us = pwm_fr_table[gear-1][1];
		break;
	case UNCHANGED:
	default:
		pr_err("%s: power parameter not valid\n", __func__);
		return core_clk_rate;
	}

	/* this register 2 fields shall be written at once */
	ufshcd_writel(hba, core_clk_period_in_ns | tx_clk_cycles_per_us,
						REG_UFS_TX_SYMBOL_CLK_NS_US);
	return core_clk_rate;
}

static int msm_ufs_pwr_change_notify(struct ufs_hba *hba,
				     bool status,
				     struct ufs_pa_layer_attr *dev_max_params,
				     struct ufs_pa_layer_attr *dev_req_params)
{
	int val;
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;
	struct ufs_msm_dev_params ufs_msm_cap;
	int ret = 0;

	if (!dev_req_params) {
		pr_err("%s: incoming dev_req_params is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	switch (status) {
	case PRE_CHANGE:
		if (hba->quirks & UFSHCD_QUIRK_BROKEN_2_TX_LANES)
			ufs_msm_cap.tx_lanes = 1;
		else
			ufs_msm_cap.tx_lanes = UFS_MSM_LIMIT_NUM_LANES_TX;

		ufs_msm_cap.rx_lanes = UFS_MSM_LIMIT_NUM_LANES_RX;
		ufs_msm_cap.hs_rx_gear = UFS_MSM_LIMIT_HSGEAR_RX;
		ufs_msm_cap.hs_tx_gear = UFS_MSM_LIMIT_HSGEAR_TX;
		ufs_msm_cap.pwm_rx_gear = UFS_MSM_LIMIT_PWMGEAR_RX;
		ufs_msm_cap.pwm_tx_gear = UFS_MSM_LIMIT_PWMGEAR_TX;
		ufs_msm_cap.rx_pwr_pwm = UFS_MSM_LIMIT_RX_PWR_PWM;
		ufs_msm_cap.tx_pwr_pwm = UFS_MSM_LIMIT_TX_PWR_PWM;
		ufs_msm_cap.rx_pwr_hs = UFS_MSM_LIMIT_RX_PWR_HS;
		ufs_msm_cap.tx_pwr_hs = UFS_MSM_LIMIT_TX_PWR_HS;
		ufs_msm_cap.hs_rate = UFS_MSM_LIMIT_HS_RATE;
		ufs_msm_cap.desired_working_mode =
					UFS_MSM_LIMIT_DESIRED_MODE;

		ret = get_pwr_dev_param(&ufs_msm_cap, dev_max_params,
							dev_req_params);
		if (ret) {
			pr_err("%s: failed to determine capabilities\n",
					__func__);
			goto out;
		}

		break;
	case POST_CHANGE:
		msm_ufs_cfg_timers(hba, dev_req_params->gear_rx,
							dev_req_params->pwr_rx);

		val = ~(MAX_U32 << dev_req_params->lane_tx);
		writel_relaxed(val, phy->mmio + UFS_PHY_TX_LANE_ENABLE);
		mb();

		/* cache the power mode parameters to use internally */
		memcpy(&host->dev_req_params,
				dev_req_params, sizeof(*dev_req_params));
		msm_ufs_update_bus_bw_vote(host);
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	return ret;
}

#define UFS_HW_VER_MAJOR_SHFT	(28)
#define UFS_HW_VER_MAJOR_MASK	(0x000F << UFS_HW_VER_MAJOR_SHFT)
#define UFS_HW_VER_MINOR_SHFT	(16)
#define UFS_HW_VER_MINOR_MASK	(0x0FFF << UFS_HW_VER_MINOR_SHFT)
#define UFS_HW_VER_STEP_SHFT	(0)
#define UFS_HW_VER_STEP_MASK	(0xFFFF << UFS_HW_VER_STEP_SHFT)

/**
 * msm_ufs_advertise_quirks - advertise the known MSM UFS controller quirks
 * @hba: host controller instance
 *
 * MSM UFS host controller might have some non standard behaviours (quirks)
 * than what is specified by UFSHCI specification. Advertise all such
 * quirks to standard UFS host controller driver so standard takes them into
 * account.
 */
static void msm_ufs_advertise_quirks(struct ufs_hba *hba)
{
	u32 ver = ufshcd_readl(hba, REG_UFS_HW_VERSION);
	u8 major;
	u16 minor, step;

	major = (ver & UFS_HW_VER_MAJOR_MASK) >> UFS_HW_VER_MAJOR_SHFT;
	minor = (ver & UFS_HW_VER_MINOR_MASK) >> UFS_HW_VER_MINOR_SHFT;
	step = (ver & UFS_HW_VER_STEP_MASK) >> UFS_HW_VER_STEP_SHFT;

	/*
	 * Interrupt aggregation and HIBERN8 on UFS HW controller revision 1.1.0
	 * is broken.
	 */
	if ((major == 0x1) && (minor == 0x001) && (step == 0x0000))
		hba->quirks |= (UFSHCD_QUIRK_BROKEN_INTR_AGGR
			      | UFSHCD_QUIRK_BROKEN_HIBERN8
			      | UFSHCD_QUIRK_BROKEN_VER_REG_1_1
			      | UFSHCD_QUIRK_BROKEN_CAP_64_BIT_0
			      | UFSHCD_QUIRK_DELAY_BEFORE_DME_CMDS
			      | UFSHCD_QUIRK_BROKEN_2_TX_LANES
			      | UFSHCD_QUIRK_BROKEN_SUSPEND);
}

static int msm_ufs_get_bus_vote(struct msm_ufs_host *host,
		const char *speed_mode)
{
	struct device *dev = host->hba->dev;
	struct device_node *np = dev->of_node;
	int err;
	const char *key = "qcom,bus-vector-names";

	if (!speed_mode) {
		err = -EINVAL;
		goto out;
	}

	if (host->bus_vote.is_max_bw_needed && !!strcmp(speed_mode, "MIN"))
		err = of_property_match_string(np, key, "MAX");
	else
		err = of_property_match_string(np, key, speed_mode);

out:
	if (err < 0)
		dev_err(dev, "%s: Invalid %s mode %d\n",
				__func__, speed_mode, err);
	return err;
}

static int msm_ufs_set_bus_vote(struct msm_ufs_host *host, int vote)
{
	int err = 0;

	if (vote != host->bus_vote.curr_vote) {
		err = msm_bus_scale_client_update_request(
				host->bus_vote.client_handle, vote);
		if (err) {
			dev_err(host->hba->dev,
				"%s: msm_bus_scale_client_update_request() failed: bus_client_handle=0x%x, vote=%d, err=%d\n",
				__func__, host->bus_vote.client_handle,
				vote, err);
			goto out;
		}

		host->bus_vote.curr_vote = vote;
	}
out:
	return err;
}

static int msm_ufs_get_speed_mode(struct ufs_pa_layer_attr *p, char *result)
{
	int err = 0;
	int gear = max_t(u32, p->gear_rx, p->gear_tx);
	int lanes = max_t(u32, p->lane_rx, p->lane_tx);
	int pwr = max_t(u32, map_unmap_pwr_mode(p->pwr_rx, true),
			map_unmap_pwr_mode(p->pwr_tx, true));

	/* default to PWM Gear 1, Lane 1 if power mode is not initialized */
	if (!gear)
		gear = 1;

	if (!lanes)
		lanes = 1;

	if (!p->pwr_rx && !p->pwr_tx)
		pwr = 0;

	pwr = map_unmap_pwr_mode(pwr, false);
	if (pwr < 0) {
		err = pwr;
		goto out;
	}

	if (pwr == FAST_MODE || pwr == FASTAUTO_MODE)
		snprintf(result, BUS_VECTOR_NAME_LEN, "%s_R%s_G%d_L%d", "HS",
				p->hs_rate == PA_HS_MODE_B ? "B" : "A",
				gear, lanes);
	else
		snprintf(result, BUS_VECTOR_NAME_LEN, "%s_G%d_L%d",
				"PWM", gear, lanes);
out:
	return err;
}

static int msm_ufs_update_bus_bw_vote(struct msm_ufs_host *host)
{
	int vote;
	int err = 0;
	char mode[BUS_VECTOR_NAME_LEN];

	err = msm_ufs_get_speed_mode(&host->dev_req_params, mode);
	if (err)
		goto out;

	vote = msm_ufs_get_bus_vote(host, mode);
	if (vote >= 0)
		err = msm_ufs_set_bus_vote(host, vote);
	else
		err = vote;

out:
	if (err)
		dev_err(host->hba->dev, "%s: failed %d\n", __func__, err);
	else
		host->bus_vote.saved_vote = vote;
	return err;
}

static int msm_ufs_setup_clocks(struct ufs_hba *hba, bool on)
{
	struct msm_ufs_host *host = hba->priv;
	int err;
	int vote;

	if (on) {
		vote = host->bus_vote.saved_vote;
		if (vote == host->bus_vote.min_bw_vote)
			msm_ufs_update_bus_bw_vote(host);
	} else {
		vote = host->bus_vote.min_bw_vote;
	}

	err = msm_ufs_set_bus_vote(host, vote);
	if (err)
		dev_err(hba->dev, "%s: set bus vote failed %d\n",
				__func__, err);

	return err;
}

static ssize_t
show_ufs_to_mem_max_bus_bw(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct msm_ufs_host *host = hba->priv;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			host->bus_vote.is_max_bw_needed);
}

static ssize_t
store_ufs_to_mem_max_bus_bw(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct msm_ufs_host *host = hba->priv;
	uint32_t value;

	if (!kstrtou32(buf, 0, &value)) {
		host->bus_vote.is_max_bw_needed = !!value;
		msm_ufs_update_bus_bw_vote(host);
	}

	return count;
}

static int msm_ufs_bus_register(struct msm_ufs_host *host)
{
	int err;
	struct msm_bus_scale_pdata *bus_pdata;
	struct device *dev = host->hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;

	bus_pdata = msm_bus_cl_get_pdata(pdev);
	if (!bus_pdata) {
		dev_err(dev, "%s: failed to get bus vectors\n", __func__);
		err = -ENODATA;
		goto out;
	}

	err = of_property_count_strings(np, "qcom,bus-vector-names");
	if (err < 0 || err != bus_pdata->num_usecases) {
		dev_err(dev, "%s: qcom,bus-vector-names not specified correctly %d\n",
				__func__, err);
		goto out;
	}

	host->bus_vote.client_handle = msm_bus_scale_register_client(bus_pdata);
	if (!host->bus_vote.client_handle) {
		dev_err(dev, "%s: msm_bus_scale_register_client failed\n",
				__func__);
		err = -EFAULT;
		goto out;
	}

	/* cache the vote index for minimum and maximum bandwidth */
	host->bus_vote.min_bw_vote = msm_ufs_get_bus_vote(host, "MIN");
	host->bus_vote.max_bw_vote = msm_ufs_get_bus_vote(host, "MAX");

	host->bus_vote.max_bus_bw.show = show_ufs_to_mem_max_bus_bw;
	host->bus_vote.max_bus_bw.store = store_ufs_to_mem_max_bus_bw;
	sysfs_attr_init(&host->bus_vote.max_bus_bw.attr);
	host->bus_vote.max_bus_bw.attr.name = "max_bus_bw";
	host->bus_vote.max_bus_bw.attr.mode = S_IRUGO | S_IWUSR;
	err = device_create_file(dev, &host->bus_vote.max_bus_bw);
out:
	return err;
}

/**
 * msm_ufs_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up PHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int msm_ufs_init(struct ufs_hba *hba)
{
	int err;
	struct device *dev = hba->dev;
	struct msm_ufs_phy *phy = msm_get_ufs_phy(hba->dev);
	struct msm_ufs_host *host;

	if (IS_ERR(phy)) {
		err = PTR_ERR(phy);
		goto out;
	}

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host) {
		err = -ENOMEM;
		dev_err(dev, "%s: no memory for msm ufs host\n", __func__);
		goto out;
	}

	host->phy = phy;
	host->hba = hba;
	hba->priv = (void *)host;

	err = msm_ufs_bus_register(host);
	if (err)
		goto out_host_free;

	err = msm_ufs_phy_power_on(phy);
	if (err)
		goto out_unregister_bus;

	err = msm_ufs_init_lane_clks(host);
	if (err)
		goto out_disable_phy;

	msm_ufs_advertise_quirks(hba);
	if (hba->quirks & UFSHCD_QUIRK_BROKEN_SUSPEND) {
		/*
		 * During suspend keep the device and the link active
		 * but shut-off the system clocks.
		 */
		hba->rpm_lvl = UFS_PM_LVL_0;
		hba->spm_lvl = UFS_PM_LVL_0;

	} else if (hba->quirks & UFSHCD_QUIRK_BROKEN_HIBERN8) {
		/*
		 * During runtime suspend, keep link active but put device in
		 * sleep state.
		 * During system suspend, power off both link and device.
		 */
		hba->rpm_lvl = UFS_PM_LVL_2;
		hba->spm_lvl = UFS_PM_LVL_4;
	} else {
		/*
		 * During runtime & system suspend, put link in Hibern8 and
		 * device in sleep.
		 */
		hba->rpm_lvl = UFS_PM_LVL_3;
		hba->spm_lvl = UFS_PM_LVL_3;
	}

	goto out;

out_disable_phy:
	msm_ufs_phy_power_off(phy);
out_unregister_bus:
	msm_bus_scale_unregister_client(host->bus_vote.client_handle);
out_host_free:
	devm_kfree(dev, host);
	hba->priv = NULL;
out:
	return err;
}

static void msm_ufs_exit(struct ufs_hba *hba)
{
	struct msm_ufs_host *host = hba->priv;
	struct msm_ufs_phy *phy = host->phy;

	msm_bus_scale_unregister_client(host->bus_vote.client_handle);
	msm_ufs_disable_lane_clks(host);
	msm_ufs_phy_power_off(phy);
}

static int msm_ufs_phy_init_vreg(struct device *dev,
		struct msm_ufs_phy_vreg *vreg, const char *name)
{
	int err = 0;
	char prop_name[MAX_PROP_NAME];

	vreg->name = kstrdup(name, GFP_KERNEL);
	if (!vreg->name) {
		err = -ENOMEM;
		goto out;
	}

	vreg->reg = devm_regulator_get(dev, name);
	if (IS_ERR(vreg->reg)) {
		err = PTR_ERR(vreg->reg);
		dev_err(dev, "failed to get %s, %d\n", name, err);
		goto out;
	}

	if (dev->of_node) {
		snprintf(prop_name, MAX_PROP_NAME, "%s-max-microamp", name);
		err = of_property_read_u32(dev->of_node,
					prop_name, &vreg->max_uA);
		if (err && err != -EINVAL) {
			dev_err(dev, "%s: failed to read %s\n",
					__func__, prop_name);
			goto out;
		} else if (err == -EINVAL || !vreg->max_uA) {
			if (regulator_count_voltages(vreg->reg) > 0) {
				dev_err(dev, "%s: %s is mandatory\n",
						__func__, prop_name);
				goto out;
			}
			err = 0;
		}
	}

	if (!strcmp(name, "vdda-pll")) {
		vreg->max_uV = VDDA_PLL_MAX_UV;
		vreg->min_uV = VDDA_PLL_MIN_UV;
	} else if (!strcmp(name, "vdda-phy")) {
		vreg->max_uV = VDDA_PHY_MAX_UV;
		vreg->min_uV = VDDA_PHY_MIN_UV;
	}

out:
	if (err)
		kfree(vreg->name);
	return err;
}

static int msm_ufs_phy_clk_get(struct device *dev,
		const char *name, struct clk **clk_out)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "failed to get %s err %d", name, err);
	} else {
		*clk_out = clk;
	}

	return err;
}

static int msm_ufs_phy_probe(struct platform_device *pdev)
{
	struct msm_ufs_phy *phy;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int err = 0;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		err = -ENOMEM;
		dev_err(dev, "failed to allocate memory %d\n", err);
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phy->mmio = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy->mmio)) {
		err = PTR_ERR(phy->mmio);
		goto out;
	}

	err = msm_ufs_phy_clk_get(dev, "tx_iface_clk", &phy->tx_iface_clk);
	if (err)
		goto out;

	err = msm_ufs_phy_clk_get(dev, "rx_iface_clk", &phy->rx_iface_clk);
	if (err)
		goto out;

	err = msm_ufs_phy_clk_get(dev, "ref_clk_src", &phy->ref_clk_src);
	if (err)
		goto out;

	err = msm_ufs_phy_clk_get(dev, "ref_clk_parent", &phy->ref_clk_parent);
	if (err)
		goto out;

	err = msm_ufs_phy_clk_get(dev, "ref_clk", &phy->ref_clk);
	if (err)
		goto out;

	err = msm_ufs_phy_init_vreg(dev, &phy->vdda_pll, "vdda-pll");
	if (err)
		goto out;

	err = msm_ufs_phy_init_vreg(dev, &phy->vdda_phy, "vdda-phy");
	if (err)
		goto out;

	phy->dev = dev;
	dev_set_drvdata(dev, phy);

	list_add_tail(&phy->list, &phy_list);
out:
	return err;
}

static int msm_ufs_phy_remove(struct platform_device *pdev)
{
	struct msm_ufs_phy *phy = platform_get_drvdata(pdev);

	msm_ufs_phy_power_off(phy);
	list_del_init(&phy->list);
	kfree(phy->vdda_pll.name);
	kfree(phy->vdda_phy.name);

	return 0;
}

/**
 * struct ufs_hba_msm_vops - UFS MSM specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initializaiton.
 */
const struct ufs_hba_variant_ops ufs_hba_msm_vops = {
	.name                   = "msm",
	.init                   = msm_ufs_init,
	.exit                   = msm_ufs_exit,
	.setup_clocks           = msm_ufs_setup_clocks,
	.hce_enable_notify      = msm_ufs_hce_enable_notify,
	.link_startup_notify    = msm_ufs_link_startup_notify,
	.pwr_change_notify	= msm_ufs_pwr_change_notify,
	.suspend		= msm_ufs_suspend,
	.resume			= msm_ufs_resume,
};
EXPORT_SYMBOL(ufs_hba_msm_vops);

static const struct of_device_id msm_ufs_phy_of_match[] = {
	{.compatible = "qcom,ufsphy"},
	{},
};
MODULE_DEVICE_TABLE(of, msm_ufs_phy_of_match);

static struct platform_driver msm_ufs_phy_driver = {
	.probe = msm_ufs_phy_probe,
	.remove = msm_ufs_phy_remove,
	.driver = {
		.of_match_table = msm_ufs_phy_of_match,
		.name = "msm_ufs_phy",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(msm_ufs_phy_driver);

MODULE_DESCRIPTION("Qualcomm Universal Flash Storage (UFS) PHY");
MODULE_LICENSE("GPL v2");
