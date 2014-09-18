/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include "mdss.h"
#include "mdss_panel.h"
#include "mdss_dsi.h"

static unsigned char *mdss_dsi_base;

static int mdss_dsi_regulator_init(struct platform_device *pdev,
				   struct dsi_drv_cm_data *dsi_drv)
{
	int ret;

	dsi_drv->vdd_vreg = devm_regulator_get(&pdev->dev, "vdd");
	if (IS_ERR(dsi_drv->vdd_vreg)) {
		pr_err("could not get 8941_l22, rc = %ld\n",
				PTR_ERR(dsi_drv->vdd_vreg));
		return -ENODEV;
	}

	ret = regulator_set_voltage(dsi_drv->vdd_vreg, 3000000, 3000000);
	if (ret) {
		pr_err("vdd_vreg->set_voltage failed, rc=%d\n", ret);
		return -EINVAL;
	}

	dsi_drv->vdd_io_vreg = devm_regulator_get(&pdev->dev, "vdd_io");
	if (IS_ERR(dsi_drv->vdd_io_vreg)) {
		pr_err("could not get 8941_l12, rc = %ld\n",
				PTR_ERR(dsi_drv->vdd_io_vreg));
		return -ENODEV;
	}

	ret = regulator_set_voltage(dsi_drv->vdd_io_vreg, 1800000, 1800000);
	if (ret) {
		pr_err("vdd_io_vreg->set_voltage failed, rc=%d\n", ret);
		return -EINVAL;
	}

	dsi_drv->dsi_vreg = devm_regulator_get(&pdev->dev, "vreg");
	if (IS_ERR(dsi_drv->dsi_vreg)) {
		pr_err("could not get 8941_l2, rc = %ld\n",
				PTR_ERR(dsi_drv->dsi_vreg));
		return -ENODEV;
	}

	ret = regulator_set_voltage(dsi_drv->dsi_vreg, 1200000, 1200000);
	if (ret) {
		pr_err("dsi_vreg->set_voltage failed, rc=%d\n", ret);
		return -EINVAL;
	}

	return 0;
}

static int mdss_dsi_panel_power_on(struct mdss_panel_data *pdata, int enable)
{
	int ret;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	pr_debug("%s: enable=%d\n", __func__, enable);

	if (enable) {
		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).vdd_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).vdd_io_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).dsi_vreg, 100000);
		if (ret < 0) {
			pr_err("%s: dsi_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_enable((ctrl_pdata->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}
		msleep(20);
		wmb();

		ret = regulator_enable((ctrl_pdata->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}
		msleep(20);
		wmb();

		ret = regulator_enable((ctrl_pdata->shared_pdata).dsi_vreg);
		if (ret) {
			pr_err("%s: Failed to enable regulator.\n", __func__);
			return ret;
		}
		if (pdata->panel_info.panel_power_on == 0)
			mdss_dsi_panel_reset(pdata, 1);

	} else {

		mdss_dsi_panel_reset(pdata, 0);

		ret = regulator_disable((ctrl_pdata->shared_pdata).vdd_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl_pdata->shared_pdata).dsi_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_disable((ctrl_pdata->shared_pdata).vdd_io_vreg);
		if (ret) {
			pr_err("%s: Failed to disable regulator.\n", __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).vdd_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}

		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).vdd_io_vreg, 100);
		if (ret < 0) {
			pr_err("%s: vdd_io_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode
		  ((ctrl_pdata->shared_pdata).dsi_vreg, 100);
		if (ret < 0) {
			pr_err("%s: dsi_vreg set regulator mode failed.\n",
						       __func__);
			return ret;
		}
	}
	return 0;
}

static int mdss_dsi_ctrl_unprepare(struct mdss_panel_data *pdata)
{
	struct mdss_panel_info *pinfo;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int ret = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	pinfo = &pdata->panel_info;

	mdss_dsi_op_mode_config(DSI_CMD_MODE, pdata);

	ret = ctrl_pdata->off(pdata);
	if (ret) {
		pr_err("%s: Panel OFF failed\n", __func__);
		return ret;
	}

	return ret;
}

static int mdss_dsi_off(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (!pdata->panel_info.panel_power_on) {
		pr_warn("%s:%d Panel already off.\n", __func__, __LINE__);
		return -EPERM;
	}

	pdata->panel_info.panel_power_on = 0;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mdss_dsi_clk_disable(pdata);
	mdss_dsi_unprepare_clocks(ctrl_pdata);

	/* disable DSI controller */
	mdss_dsi_controller_cfg(0, pdata);

	ret = mdss_dsi_panel_power_on(pdata, 0);
	if (ret) {
		pr_err("%s: Panel power off failed\n", __func__);
		return ret;
	}

	pr_debug("%s-:\n", __func__);

	return ret;
}

int mdss_dsi_cont_splash_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	struct mipi_panel_info *mipi;

	pr_info("%s:%d DSI on for continuous splash.\n", __func__, __LINE__);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	mipi  = &pdata->panel_info.mipi;

	ret = mdss_dsi_panel_power_on(pdata, 1);
	if (ret) {
		pr_err("%s: Panel power on failed\n", __func__);
		return ret;
	}
	mdss_dsi_sw_reset(pdata);
	mdss_dsi_host_init(mipi, pdata);

	pdata->panel_info.panel_power_on = 1;

	mdss_dsi_op_mode_config(mipi->mode, pdata);

	pr_debug("%s-:End\n", __func__);
	return ret;
}


int mdss_dsi_on(struct mdss_panel_data *pdata)
{
	int ret = 0;
	u32 clk_rate;
	struct mdss_panel_info *pinfo;
	struct mipi_panel_info *mipi;
	u32 hbp, hfp, vbp, vfp, hspw, vspw, width, height;
	u32 ystride, bpp, data;
	u32 dummy_xres, dummy_yres;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	if (pdata->panel_info.panel_power_on) {
		pr_warn("%s:%d Panel already on.\n", __func__, __LINE__);
		return 0;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	pinfo = &pdata->panel_info;

	ret = mdss_dsi_panel_power_on(pdata, 1);
	if (ret) {
		pr_err("%s: Panel power on failed\n", __func__);
		return ret;
	}

	pdata->panel_info.panel_power_on = 1;

	mdss_dsi_phy_sw_reset((ctrl_pdata->ctrl_base));
	mdss_dsi_phy_init(pdata);

	mdss_dsi_prepare_clocks(ctrl_pdata);
	mdss_dsi_clk_enable(pdata);

	clk_rate = pdata->panel_info.clk_rate;
	clk_rate = min(clk_rate, pdata->panel_info.clk_max);

	hbp = pdata->panel_info.lcdc.h_back_porch;
	hfp = pdata->panel_info.lcdc.h_front_porch;
	vbp = pdata->panel_info.lcdc.v_back_porch;
	vfp = pdata->panel_info.lcdc.v_front_porch;
	hspw = pdata->panel_info.lcdc.h_pulse_width;
	vspw = pdata->panel_info.lcdc.v_pulse_width;
	width = pdata->panel_info.xres;
	height = pdata->panel_info.yres;

	mipi  = &pdata->panel_info.mipi;
	if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		dummy_xres = pdata->panel_info.lcdc.xres_pad;
		dummy_yres = pdata->panel_info.lcdc.yres_pad;

		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x24,
			((hspw + hbp + width + dummy_xres) << 16 |
			(hspw + hbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x28,
			((vspw + vbp + height + dummy_yres) << 16 |
			(vspw + vbp)));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x2C,
			(vspw + vbp + height + dummy_yres +
				vfp - 1) << 16 | (hspw + hbp +
				width + dummy_xres + hfp - 1));

		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x30, (hspw << 16));
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x34, 0);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x38, (vspw << 16));

	} else {		/* command mode */
		if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB666)
			bpp = 3;
		else if (mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
			bpp = 2;
		else
			bpp = 3;	/* Default format set to RGB888 */

		ystride = width * bpp + 1;

		/* DSI_COMMAND_MODE_MDP_STREAM_CTRL */
		data = (ystride << 16) | (mipi->vc << 8) | DTYPE_DCS_LWRITE;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x60, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x58, data);

		/* DSI_COMMAND_MODE_MDP_STREAM_TOTAL */
		data = height << 16 | width;
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x64, data);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0x5C, data);
	}

	mdss_dsi_sw_reset(pdata);
	mdss_dsi_host_init(mipi, pdata);

	if (mipi->force_clk_lane_hs) {
		u32 tmp;

		tmp = MIPI_INP((ctrl_pdata->ctrl_base) + 0xac);
		tmp |= (1<<28);
		MIPI_OUTP((ctrl_pdata->ctrl_base) + 0xac, tmp);
		wmb();
	}

	ret = ctrl_pdata->on(pdata);
	if (ret) {
		pr_err("%s: unable to initialize the panel\n", __func__);
		return ret;
	}

	mdss_dsi_op_mode_config(mipi->mode, pdata);

	pr_debug("%s-:End\n", __func__);
	return ret;
}

static int mdss_dsi_event_handler(struct mdss_panel_data *pdata,
				  int event, void *arg)
{
	int rc = 0;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	switch (event) {
	case MDSS_EVENT_UNBLANK:
		if (ctrl_pdata->on_cmds->ctrl_state == DSI_LP_MODE) {
			rc = mdss_dsi_on(pdata);
		} else {
			pr_debug("%s:event=%d, Dsi On not called: ctrl_state: %d\n",
				 __func__, event,
				 ctrl_pdata->on_cmds->ctrl_state);
			rc = -EINVAL;
		}
		break;
	case MDSS_EVENT_BLANK:
		if (ctrl_pdata->off_cmds->ctrl_state == DSI_HS_MODE) {
			rc = mdss_dsi_ctrl_unprepare(pdata);
		} else {
			pr_debug("%s:event=%d,Unprepare not called.Ctrl_state: %d\n",
				 __func__, event,
				 ctrl_pdata->on_cmds->ctrl_state);
			rc = -EINVAL;
		}
		break;
	case MDSS_EVENT_TIMEGEN_OFF:
		if (ctrl_pdata->off_cmds->ctrl_state == DSI_LP_MODE) {
			pr_debug("%s:event=%d, calling unprepare: ctrl_state: %d\n",
				 __func__, event,
				 ctrl_pdata->on_cmds->ctrl_state);
			rc = mdss_dsi_ctrl_unprepare(pdata);
		}
		rc = mdss_dsi_off(pdata);
		break;
	case MDSS_EVENT_CONT_SPLASH_FINISH:
		if (ctrl_pdata->on_cmds->ctrl_state == DSI_LP_MODE) {
			rc = mdss_dsi_cont_splash_on(pdata);
		} else {
			pr_debug("%s:event=%d, Dsi On not called: ctrl_state: %d\n",
				 __func__, event,
				 ctrl_pdata->on_cmds->ctrl_state);
			rc = -EINVAL;
		}
		break;
	default:
		pr_debug("%s: unhandled event=%d\n", __func__, event);
		break;
	}
	return rc;
}

static int mdss_dsi_ctrl_probe(struct platform_device *pdev)
{
	int rc = 0;
	u32 index;

	pr_debug("%s\n", __func__);

	if (pdev->dev.of_node) {
		struct resource *mdss_dsi_mres;
		const char *ctrl_name;

		ctrl_name = of_get_property(pdev->dev.of_node, "label", NULL);
		if (!ctrl_name)
			pr_info("%s:%d, DSI Ctrl name not specified\n",
						__func__, __LINE__);
		else
			pr_info("%s: DSI Ctrl name = %s\n",
				__func__, ctrl_name);

		rc = of_property_read_u32(pdev->dev.of_node,
					  "cell-index", &index);
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Cell-index not specified, rc=%d\n",
							__func__, rc);
			return rc;
		}

		if (index == 0)
			pdev->id = 1;
		else
			pdev->id = 2;

		mdss_dsi_mres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!mdss_dsi_mres) {
			pr_err("%s:%d unable to get the MDSS resources",
				       __func__, __LINE__);
			return -ENOMEM;
		}
		if (mdss_dsi_mres) {
			mdss_dsi_base = ioremap(mdss_dsi_mres->start,
				resource_size(mdss_dsi_mres));
			if (!mdss_dsi_base) {
				pr_err("%s:%d unable to remap dsi resources",
					       __func__, __LINE__);
				return -ENOMEM;
			}
		}

		rc = of_platform_populate(pdev->dev.of_node,
					NULL, NULL, &pdev->dev);
		if (rc) {
			dev_err(&pdev->dev,
				"%s: failed to add child nodes, rc=%d\n",
							__func__, rc);
			iounmap(mdss_dsi_base);
			return rc;
		}

		pr_debug("%s: Dsi Ctrl->%d initialized\n", __func__, index);
	}

	return 0;
}

static int mdss_dsi_ctrl_remove(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);
	iounmap(mdss_dsi_base);
	return 0;
}

struct device dsi_dev;

int mdss_dsi_retrieve_ctrl_resources(struct platform_device *pdev, int mode,
			    unsigned char **ctrl_base)
{
	int rc = 0;
	u32 index;
	struct resource *mdss_dsi_mres;

	rc = of_property_read_u32(pdev->dev.of_node, "cell-index", &index);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Cell-index not specified, rc=%d\n",
						__func__, rc);
		return rc;
	}

	if (index == 0) {
		if (mode != DISPLAY_1) {
			pr_err("%s:%d Panel->Ctrl mapping is wrong",
				       __func__, __LINE__);
			return -EPERM;
		}
	} else if (index == 1) {
		if (mode != DISPLAY_2) {
			pr_err("%s:%d Panel->Ctrl mapping is wrong",
				       __func__, __LINE__);
			return -EPERM;
		}
	} else {
		pr_err("%s:%d Unknown Ctrl mapped to panel",
			       __func__, __LINE__);
		return -EPERM;
	}

	mdss_dsi_mres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mdss_dsi_mres) {
		pr_err("%s:%d unable to get the DSI ctrl resources",
			       __func__, __LINE__);
		return -ENOMEM;
	}

	*ctrl_base = ioremap(mdss_dsi_mres->start,
		resource_size(mdss_dsi_mres));
	if (!(*ctrl_base)) {
		pr_err("%s:%d unable to remap dsi resources",
			       __func__, __LINE__);
		return -ENOMEM;
	}

	return 0;
}


int dsi_panel_device_register(struct platform_device *pdev,
			      struct mdss_panel_common_pdata *panel_data,
			      char backlight_ctrl)
{
	struct mipi_panel_info *mipi;
	int rc;
	u8 lanes = 0, bpp;
	u32 h_period, v_period, dsi_pclk_rate;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct device_node *dsi_ctrl_np = NULL;
	struct platform_device *ctrl_pdev = NULL;
	unsigned char *ctrl_addr;
	bool broadcast;
	bool cont_splash_enabled = false;

	h_period = ((panel_data->panel_info.lcdc.h_pulse_width)
			+ (panel_data->panel_info.lcdc.h_back_porch)
			+ (panel_data->panel_info.xres)
			+ (panel_data->panel_info.lcdc.h_front_porch));

	v_period = ((panel_data->panel_info.lcdc.v_pulse_width)
			+ (panel_data->panel_info.lcdc.v_back_porch)
			+ (panel_data->panel_info.yres)
			+ (panel_data->panel_info.lcdc.v_front_porch));

	mipi  = &panel_data->panel_info.mipi;

	panel_data->panel_info.type =
		((mipi->mode == DSI_VIDEO_MODE)
			? MIPI_VIDEO_PANEL : MIPI_CMD_PANEL);

	if (mipi->data_lane3)
		lanes += 1;
	if (mipi->data_lane2)
		lanes += 1;
	if (mipi->data_lane1)
		lanes += 1;
	if (mipi->data_lane0)
		lanes += 1;


	if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB888)
	    || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB666_LOOSE))
		bpp = 3;
	else if ((mipi->dst_format == DSI_CMD_DST_FORMAT_RGB565)
		 || (mipi->dst_format == DSI_VIDEO_DST_FORMAT_RGB565))
		bpp = 2;
	else
		bpp = 3;		/* Default format set to RGB888 */

	if (panel_data->panel_info.type == MIPI_VIDEO_PANEL &&
		!panel_data->panel_info.clk_rate) {
		h_period += panel_data->panel_info.lcdc.xres_pad;
		v_period += panel_data->panel_info.lcdc.yres_pad;

		if (lanes > 0) {
			panel_data->panel_info.clk_rate =
			((h_period * v_period * (mipi->frame_rate) * bpp * 8)
			   / lanes);
		} else {
			pr_err("%s: forcing mdss_dsi lanes to 1\n", __func__);
			panel_data->panel_info.clk_rate =
				(h_period * v_period
					 * (mipi->frame_rate) * bpp * 8);
		}
	}
	pll_divider_config.clk_rate = panel_data->panel_info.clk_rate;

	rc = mdss_dsi_clk_div_config(bpp, lanes, &dsi_pclk_rate);
	if (rc) {
		pr_err("%s: unable to initialize the clk dividers\n", __func__);
		return rc;
	}

	if ((dsi_pclk_rate < 3300000) || (dsi_pclk_rate > 103300000))
		dsi_pclk_rate = 35000000;
	mipi->dsi_pclk_rate = dsi_pclk_rate;

	ctrl_pdata = devm_kzalloc(&pdev->dev,
		sizeof(struct mdss_dsi_ctrl_pdata), GFP_KERNEL);
	if (!ctrl_pdata)
		return -ENOMEM;

	dsi_ctrl_np = of_parse_phandle(pdev->dev.of_node,
				       "qcom,dsi-ctrl-phandle", 0);
	if (!dsi_ctrl_np) {
		pr_err("%s: Dsi controller node not initialized\n", __func__);
		devm_kfree(&pdev->dev, ctrl_pdata);
		return -EPROBE_DEFER;
	}

	ctrl_pdev = of_find_device_by_node(dsi_ctrl_np);

	rc = mdss_dsi_regulator_init(ctrl_pdev, &(ctrl_pdata->shared_pdata));
	if (rc) {
		dev_err(&pdev->dev,
			"%s: failed to init regulator, rc=%d\n",
						__func__, rc);
		devm_kfree(&pdev->dev, ctrl_pdata);
		return rc;
	}

	broadcast = of_property_read_bool(pdev->dev.of_node,
					  "qcom,mdss-pan-broadcast-mode");
	if (broadcast)
		ctrl_pdata->shared_pdata.broadcast_enable = 1;

	ctrl_pdata->disp_en_gpio = of_get_named_gpio(pdev->dev.of_node,
						     "qcom,enable-gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_err("%s:%d, Disp_en gpio not specified\n",
						__func__, __LINE__);
	} else {
		rc = gpio_request(ctrl_pdata->disp_en_gpio, "disp_enable");
		if (rc) {
			pr_err("request reset gpio failed, rc=%d\n",
			       rc);
			gpio_free(ctrl_pdata->disp_en_gpio);
			return -ENODEV;
		}
	}

	ctrl_pdata->rst_gpio = of_get_named_gpio(pdev->dev.of_node,
						 "qcom,rst-gpio", 0);
	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_err("%s:%d, reset gpio not specified\n",
						__func__, __LINE__);
	} else {
		rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
		if (rc) {
			pr_err("request reset gpio failed, rc=%d\n",
				rc);
			gpio_free(ctrl_pdata->rst_gpio);
			gpio_free(ctrl_pdata->disp_en_gpio);
			return -ENODEV;
		}
	}

	if (mdss_dsi_clk_init(ctrl_pdev, ctrl_pdata)) {
		pr_err("%s: unable to initialize Dsi ctrl clks\n", __func__);
		devm_kfree(&pdev->dev, ctrl_pdata);
		return -EPERM;
	}

	if (mdss_dsi_retrieve_ctrl_resources(ctrl_pdev,
					     panel_data->panel_info.pdest,
					     &ctrl_addr)) {
		pr_err("%s: unable to get Dsi controller res\n", __func__);
		devm_kfree(&pdev->dev, ctrl_pdata);
		return -EPERM;
	}

	pr_debug("%s: ctrl base address: 0x%x\n", __func__, (int)ctrl_addr);
	ctrl_pdata->panel_data.event_handler = mdss_dsi_event_handler;

	ctrl_pdata->on_cmds = panel_data->dsi_panel_on_cmds;
	ctrl_pdata->off_cmds = panel_data->dsi_panel_off_cmds;

	memcpy(&((ctrl_pdata->panel_data).panel_info),
				&(panel_data->panel_info),
				       sizeof(struct mdss_panel_info));

	mdss_dsi_irq_handler_config(ctrl_pdata);
	(ctrl_pdata->panel_data).set_backlight = panel_data->bl_fnc;
	(ctrl_pdata->ctrl_base) = ctrl_addr;
	(ctrl_pdata->bl_ctrl) = backlight_ctrl;
	/*
	 * register in mdp driver
	 */

	cont_splash_enabled = of_property_read_bool(pdev->dev.of_node,
			"qcom,cont-splash-enabled");
	if (!cont_splash_enabled) {
		pr_info("%s:%d Continous splash flag not found.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
		ctrl_pdata->panel_data.panel_info.panel_power_on = 0;
	} else {
		pr_info("%s:%d Continous splash flag enabled.\n",
				__func__, __LINE__);

		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 1;
		ctrl_pdata->panel_data.panel_info.panel_power_on = 1;
	}


	if (ctrl_pdata->panel_data.panel_info.cont_splash_enabled) {
		mdss_dsi_prepare_clocks(ctrl_pdata);
		mdss_dsi_clk_enable(&(ctrl_pdata->panel_data));
	}

	rc = mdss_register_panel(ctrl_pdev, &(ctrl_pdata->panel_data));
	if (rc) {
		dev_err(&pdev->dev, "unable to register MIPI DSI panel\n");
		devm_kfree(&pdev->dev, ctrl_pdata);
		if (ctrl_pdata->rst_gpio)
			gpio_free(ctrl_pdata->rst_gpio);
		if (ctrl_pdata->disp_en_gpio)
			gpio_free(ctrl_pdata->disp_en_gpio);
		return rc;
	}

	ctrl_pdata->on = panel_data->on;
	ctrl_pdata->off = panel_data->off;

	pr_debug("%s: Panal data initialized\n", __func__);
	return 0;
}

static const struct of_device_id mdss_dsi_ctrl_dt_match[] = {
	{.compatible = "qcom,mdss-dsi-ctrl"},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_dsi_ctrl_dt_match);

static struct platform_driver mdss_dsi_ctrl_driver = {
	.probe = mdss_dsi_ctrl_probe,
	.remove = mdss_dsi_ctrl_remove,
	.shutdown = NULL,
	.driver = {
		.name = "mdss_dsi_ctrl",
		.of_match_table = mdss_dsi_ctrl_dt_match,
	},
};

static int mdss_dsi_register_driver(void)
{
	return platform_driver_register(&mdss_dsi_ctrl_driver);
}

static int __init mdss_dsi_driver_init(void)
{
	int ret;

	mdss_dsi_init();

	ret = mdss_dsi_register_driver();
	if (ret) {
		pr_err("mdss_dsi_register_driver() failed!\n");
		return ret;
	}

	return ret;
}
module_init(mdss_dsi_driver_init);

static void __exit mdss_dsi_driver_cleanup(void)
{
	iounmap(mdss_dsi_base);
	platform_driver_unregister(&mdss_dsi_ctrl_driver);
}
module_exit(mdss_dsi_driver_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DSI controller driver");
MODULE_AUTHOR("Chandan Uddaraju <chandanu@codeaurora.org>");
