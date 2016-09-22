/*
 * Restructured unified display panel driver for Xperia Open Devices
 *                *** Common generic helpers ***
 *
 * This driver is based on various SoMC implementations found in
 * copyleft archives for various devices.
 *
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) Sony Mobile Communications Inc. All rights reserved.
 * Copyright (C) 2014-2016, AngeloGioacchino Del Regno <kholk11@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include "../mdss_mdp.h"
#include "../mdss_dsi.h"
#include "somc_panels.h"

int somc_panel_vreg_name_to_config(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		struct dss_vreg *config, char *name)
{
	struct dss_vreg *vreg_config = ctrl_pdata->panel_power_data.vreg_config;
	int num_vreg = ctrl_pdata->panel_power_data.num_vreg;
	int i = 0;
	int valid = -EINVAL;

	for (i = 0; i < num_vreg; i++) {
		if (!strcmp(name, vreg_config[i].vreg_name)) {
			*config = vreg_config[i];
			valid = 0;
			break;
		}
	}

	return valid;
}

int somc_panel_vreg_ctrl(
		struct mdss_dsi_ctrl_pdata *ctrl_pdata,
		char *vreg, bool enable)
{
	struct dss_vreg vreg_config;
	struct mdss_panel_power_seq *pw_seq = NULL;
	int valid = 0;
	int wait = 0;
	int ret = 0;

	valid = somc_panel_vreg_name_to_config(
			ctrl_pdata, &vreg_config, vreg);

	if (!valid) {
		if (enable) {
			ret = msm_dss_enable_vreg(&vreg_config, 1, 1);
			pw_seq = &ctrl_pdata->spec_pdata->on_seq;
		} else {
			ret = msm_dss_enable_vreg(&vreg_config, 1, 0);
			pw_seq = &ctrl_pdata->spec_pdata->off_seq;
		}

#ifdef CONFIG_SOMC_PANEL_INCELL
		if (!strcmp(vreg, "vdd"))
			wait = pw_seq->disp_vdd;
		else if (!strcmp(vreg, "vddio"))
			wait = pw_seq->disp_vddio;
		else if (!strcmp(vreg, "lab"))
			wait = pw_seq->disp_vsp;
		else if (!strcmp(vreg, "ibb"))
			wait = pw_seq->disp_vsn;
		else if (!strcmp(vreg, "touch-avdd"))
			wait = pw_seq->touch_avdd;
		else
			wait = 0;

		if (!ret && wait) {
			usleep_range(wait * 1000, wait * 1000 + 100);
		}
#endif
	}

	return ret;
}

void somc_panel_chg_fps_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	struct mdss_panel_specific_pdata *spec_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	u32 fps_cmds, fps_payload;
	char rtn;

	pinfo = &ctrl_pdata->panel_data.panel_info;
	spec_pdata = ctrl_pdata->spec_pdata;

	if (ctrl_pdata->panel_data.panel_info.mipi.mode == DSI_CMD_MODE) {
		if (spec_pdata->fps_cmds.cmd_cnt) {
			fps_cmds = pinfo->lcdc.chg_fps.rtn_pos.pos[0];
			fps_payload = pinfo->lcdc.chg_fps.rtn_pos.pos[1];
			rtn = CHANGE_PAYLOAD(fps_cmds, fps_payload);
			pr_debug("%s: change fps sequence --- rtn = 0x%x\n",
				__func__, rtn);
			mdss_dsi_panel_cmds_send(ctrl_pdata,
						&spec_pdata->fps_cmds);
		}
	}
}
