/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#include "cpr3-regulator.h"

#define MSM8996_MMSS_FUSE_CORNERS	4

/**
 * struct cpr3_msm8996_mmss_fuses - MMSS specific fuse data for MSM8996
 * @init_voltage:	Initial (i.e. open-loop) voltage fuse parameter value
 *			for each fuse corner (raw, not converted to a voltage)
 * @cpr_fusing_rev:	CPR fusing revision fuse parameter value
 * @limitation:		CPR limitation select fuse parameter value
 *
 * This struct holds the values for all of the fuses read from memory.
 */
struct cpr3_msm8996_mmss_fuses {
	u64	init_voltage[MSM8996_MMSS_FUSE_CORNERS];
	u64	cpr_fusing_rev;
	u64	limitation;
};

/**
 * enum cpr3_msm8996_mmss_fuse_combo - fuse combinations supported by the MMSS
 *			CPR3 controller on MSM8996
 * %CPR3_MSM8996_MMSS_FUSE_COMBO_DEFAULT:	Initial default combination
 * %CPR3_MSM8996_MMSS_FUSE_COMBO_COUNT:		Defines the number of
 *						combinations supported
 *
 * This list will be expanded as new requirements are added.
 */
enum cpr3_msm8996_mmss_fuse_combo {
	CPR3_MSM8996_MMSS_FUSE_COMBO_DEFAULT = 0,
	CPR3_MSM8996_MMSS_FUSE_COMBO_COUNT
};

/*
 * MSM8996 MMSS fuse parameter locations:
 *
 * Structs are organized with the following dimensions:
 *	Outer: 0 to 3 for fuse corners from lowest to highest corner
 *	Inner: large enough to hold the longest set of parameter segments which
 *		fully defines a fuse parameter, +1 (for NULL termination).
 *		Each segment corresponds to a contiguous group of bits from a
 *		single fuse row.  These segments are concatentated together in
 *		order to form the full fuse parameter value.  The segments for
 *		a given parameter may correspond to different fuse rows.
 */
static const struct cpr3_fuse_param
msm8996_mmss_init_voltage_param[MSM8996_MMSS_FUSE_CORNERS][2] = {
	{{63, 55, 59}, {} },
	{{63, 50, 54}, {} },
	{{63, 45, 49}, {} },
	{{63, 40, 44}, {} },
};

static const struct cpr3_fuse_param msm8996_cpr_fusing_rev_param[] = {
	{39, 48, 50},
	{},
};

static const struct cpr3_fuse_param msm8996_cpr_limitation_param[] = {
	{41, 31, 32},
	{},
};

/*
 * Some initial msm8996 parts cannot be used in a meaningful way by software.
 * Other parts can only be used when operating with CPR disabled (i.e. at the
 * fused open-loop voltage) when no voltage interpolation is applied.  A fuse
 * parameter is provided so that software can properly handle these limitations.
 */
enum msm8996_cpr_limitation {
	MSM8996_CPR_LIMITATION_NONE = 0,
	MSM8996_CPR_LIMITATION_UNSUPPORTED = 2,
	MSM8996_CPR_LIMITATION_NO_CPR_OR_INTERPOLATION = 3,
};

/* Additional MSM8996 specific data: */

/* Open loop voltage fuse reference voltages in microvolts */
static const int msm8996_mmss_fuse_ref_volt[MSM8996_MMSS_FUSE_CORNERS] = {
	670000,
	745000,
	905000,
	1015000,
};

#define MSM8996_MMSS_FUSE_STEP_VOLT		10000
#define MSM8996_MMSS_VOLTAGE_FUSE_SIZE		5

#define MSM8996_MMSS_CPR_SENSOR_COUNT		35

#define MSM8996_MMSS_CPR_CLOCK_RATE		19200000

/**
 * cpr3_msm8996_mmss_read_fuse_data() - load MMSS specific fuse parameter values
 * @thread:		Pointer to the CPR3 thread
 *
 * This function allocates a cpr3_msm8996_mmss_fuses struct, fills it with
 * values read out of hardware fuses, and finally copies common fuse values
 * into the thread struct.
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_msm8996_mmss_read_fuse_data(struct cpr3_thread *thread)
{
	void __iomem *base = thread->ctrl->fuse_base;
	struct cpr3_msm8996_mmss_fuses *fuse;
	int i, rc;

	fuse = devm_kzalloc(thread->ctrl->dev, sizeof(*fuse), GFP_KERNEL);
	if (!fuse) {
		cpr3_err(thread, "could not allocate memory for fuse data\n");
		return -ENOMEM;
	}

	rc = cpr3_read_fuse_param(base, msm8996_cpr_fusing_rev_param,
				&fuse->cpr_fusing_rev);
	if (rc) {
		cpr3_err(thread, "Unable to read CPR fusing revision fuse, rc=%d\n",
			rc);
		return rc;
	}
	cpr3_info(thread, "CPR fusing revision = %llu\n", fuse->cpr_fusing_rev);

	rc = cpr3_read_fuse_param(base, msm8996_cpr_limitation_param,
				&fuse->limitation);
	if (rc) {
		cpr3_err(thread, "Unable to read CPR limitation fuse, rc=%d\n",
			rc);
		return rc;
	}
	cpr3_info(thread, "CPR limitation = %s\n",
		fuse->limitation == MSM8996_CPR_LIMITATION_UNSUPPORTED
		? "unsupported chip" : fuse->limitation
			  == MSM8996_CPR_LIMITATION_NO_CPR_OR_INTERPOLATION
		? "CPR disabled and no interpolation" : "none");

	for (i = 0; i < MSM8996_MMSS_FUSE_CORNERS; i++) {
		rc = cpr3_read_fuse_param(base,
			msm8996_mmss_init_voltage_param[i],
			&fuse->init_voltage[i]);
		if (rc) {
			cpr3_err(thread, "Unable to read fuse-corner %d initial voltage fuse, rc=%d\n",
				i, rc);
			return rc;
		}
	}

	thread->cpr_rev_fuse		= fuse->cpr_fusing_rev;
	thread->fuse_corner_count	= MSM8996_MMSS_FUSE_CORNERS;
	thread->platform_fuses		= fuse;
	thread->fuse_combo		= CPR3_MSM8996_MMSS_FUSE_COMBO_DEFAULT;

	return 0;
}

/**
 * cpr3_mmss_parse_corner_data() - parse MMSS corner data from device tree
 *				properties of the thread's device node
 * @thread:		Pointer to the CPR3 thread
 * @corner_sum:		Pointer which is output with the sum of the corner
 *			counts across all fuse combos
 * @combo_offset:	Pointer which is output with the array offset for the
 *			selected fuse combo
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_mmss_parse_corner_data(struct cpr3_thread *thread,
			int *corner_sum, int *combo_offset)
{
	int i, rc;
	u32 *temp;

	rc = cpr3_parse_common_corner_data(thread, corner_sum, combo_offset);
	if (rc) {
		cpr3_err(thread, "error reading corner data, rc=%d\n", rc);
		return rc;
	}

	temp = kzalloc(sizeof(*temp) * thread->corner_count * CPR3_RO_COUNT,
			GFP_KERNEL);
	if (!temp) {
		cpr3_err(thread, "memory allocation failed\n");
		return -ENOMEM;
	}

	rc = cpr3_parse_array_property(thread, "qcom,cpr-target-quotients",
			thread->corner_count * CPR3_RO_COUNT,
			*corner_sum * CPR3_RO_COUNT,
			*combo_offset * CPR3_RO_COUNT,
			temp);
	if (rc) {
		cpr3_err(thread, "could not load target quotients, rc=%d\n",
			rc);
		goto done;
	}

	for (i = 0; i < thread->corner_count; i++)
		memcpy(thread->corner[i].target_quot, &temp[i * CPR3_RO_COUNT],
			sizeof(*temp) * CPR3_RO_COUNT);

done:
	kfree(temp);
	return rc;
}

/**
 * cpr3_mmss_adjust_target_quotients() - adjust the target quotients for each
 *		corner according to device tree values
 * @thread:		Pointer to the CPR3 thread
 * @corner_sum:		Sum of the corner counts across all fuse combos
 * @combo_offset:	Array offset for the selected fuse combo
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_mmss_adjust_target_quotients(struct cpr3_thread *thread,
			int corner_sum, int combo_offset)
{
	int i, j, rc, quot_adjust;
	int *volt_adjust, *ro_scale;
	u32 prev_quot;

	if (!of_find_property(thread->of_node,
			"qcom,cpr-closed-loop-voltage-adjustment", NULL)) {
		/* No adjustment required. */
		return 0;
	} else if (!of_find_property(thread->of_node,
			"qcom,cpr-ro-scaling-factor", NULL)) {
		cpr3_err(thread, "qcom,cpr-ro-scaling-factor is required for closed-loop voltage adjustment, but is missing\n");
		return -EINVAL;
	}

	volt_adjust = kzalloc(sizeof(*volt_adjust) * thread->corner_count,
				GFP_KERNEL);
	ro_scale = kzalloc(sizeof(*ro_scale) * thread->corner_count
				* CPR3_RO_COUNT, GFP_KERNEL);
	if (!volt_adjust || !ro_scale) {
		cpr3_err(thread, "memory allocation failed\n");
		rc = -ENOMEM;
		goto done;
	}

	rc = cpr3_parse_array_property(thread,
		"qcom,cpr-closed-loop-voltage-adjustment",
		thread->corner_count, corner_sum, combo_offset, volt_adjust);
	if (rc) {
		cpr3_err(thread, "could not load closed-loop voltage adjustments, rc=%d\n",
			rc);
		goto done;
	}

	rc = cpr3_parse_array_property(thread, "qcom,cpr-ro-scaling-factor",
			thread->corner_count * CPR3_RO_COUNT,
			corner_sum * CPR3_RO_COUNT,
			combo_offset * CPR3_RO_COUNT,
			ro_scale);
	if (rc) {
		cpr3_err(thread, "could not load RO scaling factors, rc=%d\n",
			rc);
		goto done;
	}

	for (i = 0; i < thread->corner_count; i++) {
		for (j = 0; j < CPR3_RO_COUNT; j++) {
			if (thread->corner[i].target_quot[j]) {
				quot_adjust = cpr3_quot_adjustment(
						ro_scale[i * CPR3_RO_COUNT + j],
						volt_adjust[i]);
				if (quot_adjust) {
					prev_quot
					     = thread->corner[i].target_quot[j];
					thread->corner[i].target_quot[j]
					     += quot_adjust;
					cpr3_info(thread, "adjusted corner %d RO%d target quot: %u --> %u (%d uV)\n",
					     i, j, prev_quot,
					     thread->corner[i].target_quot[j],
					     volt_adjust[i]);
				}
			}
		}
	}

done:
	kfree(volt_adjust);
	kfree(ro_scale);
	return rc;
}

/**
 * cpr3_msm8996_mmss_calculate_open_loop_voltages() - calculate the open-loop
 *		voltage for each corner of a CPR3 thread
 * @thread:		Pointer to the CPR3 thread
 * @corner_sum:		Sum of the corner counts across all fuse combos
 * @combo_offset:	Array offset for the selected fuse combo
 *
 * If open-loop voltage interpolation is allowed in both device tree and in
 * hardware fuses, then this function calculates the open-loop voltage for a
 * given corner using linear interpolation.  This interpolation is performed
 * using the processor frequencies of the lower and higher Fmax corners along
 * with their fused open-loop voltages.
 *
 * If open-loop voltage interpolation is not allowed, then this function uses
 * the Fmax fused open-loop voltage for all of the corners associated with a
 * given fuse corner.
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_msm8996_mmss_calculate_open_loop_voltages(
			struct cpr3_thread *thread, int corner_sum,
			int combo_offset)
{
	struct device_node *node = thread->of_node;
	struct cpr3_msm8996_mmss_fuses *fuse = thread->platform_fuses;
	int rc = 0;
	bool allow_interpolation;
	u64 freq_low, volt_low, freq_high, volt_high;
	int i, j;
	int *fuse_volt;
	int *fmax_corner;

	fuse_volt = kzalloc(sizeof(*fuse_volt) * thread->fuse_corner_count,
				GFP_KERNEL);
	fmax_corner = kzalloc(sizeof(*fmax_corner) * thread->fuse_corner_count,
					GFP_KERNEL);
	if (!fuse_volt || !fmax_corner) {
		cpr3_err(thread, "unable to allocate temp memory\n");
		rc = -ENOMEM;
		goto done;
	}

	for (i = 0; i < thread->fuse_corner_count; i++) {
		fuse_volt[i] = cpr3_convert_open_loop_voltage_fuse(
			msm8996_mmss_fuse_ref_volt[i],
			MSM8996_MMSS_FUSE_STEP_VOLT, fuse->init_voltage[i],
			MSM8996_MMSS_VOLTAGE_FUSE_SIZE);
		cpr3_info(thread, "fuse_corner[%d] open-loop=%7d uV\n",
			i, fuse_volt[i]);
	}

	rc = cpr3_adjust_fused_open_loop_voltages(thread, fuse_volt);
	if (rc) {
		cpr3_err(thread, "fused open-loop voltage adjustment failed, rc=%d\n",
			rc);
		goto done;
	}

	allow_interpolation = of_property_read_bool(node,
				"qcom,allow-voltage-interpolation");

	for (i = 1; i < thread->fuse_corner_count; i++) {
		if (fuse_volt[i] < fuse_volt[i - 1]) {
			cpr3_err(thread, "voltage fuse[%d]=%d < fuse[%d]=%d uV; interpolation not possible\n",
				i, fuse_volt[i], i - 1, fuse_volt[i - 1]);
			allow_interpolation = false;
		}
	}

	if (fuse->limitation == MSM8996_CPR_LIMITATION_NO_CPR_OR_INTERPOLATION)
		allow_interpolation = false;

	if (!allow_interpolation) {
		/* Use fused open-loop voltage for lower frequencies. */
		for (i = 0; i < thread->corner_count; i++)
			thread->corner[i].open_loop_volt
				= fuse_volt[thread->corner[i].cpr_fuse_corner];
		goto done;
	}

	/* Determine highest corner mapped to each fuse corner */
	j = thread->fuse_corner_count - 1;
	for (i = thread->corner_count - 1; i >= 0; i--) {
		if (thread->corner[i].cpr_fuse_corner == j) {
			fmax_corner[j] = i;
			j--;
		}
	}
	if (j >= 0) {
		cpr3_err(thread, "invalid fuse corner mapping\n");
		rc = -EINVAL;
		goto done;
	}

	/*
	 * Interpolation is not possible for corners mapped to the lowest fuse
	 * corner so use the fuse corner value directly.
	 */
	for (i = 0; i <= fmax_corner[0]; i++)
		thread->corner[i].open_loop_volt = fuse_volt[0];

	/* Interpolate voltages for the higher fuse corners. */
	for (i = 1; i < thread->fuse_corner_count; i++) {
		freq_low = thread->corner[fmax_corner[i - 1]].proc_freq;
		volt_low = fuse_volt[i - 1];
		freq_high = thread->corner[fmax_corner[i]].proc_freq;
		volt_high = fuse_volt[i];

		for (j = fmax_corner[i - 1] + 1; j <= fmax_corner[i]; j++)
			thread->corner[j].open_loop_volt = cpr3_interpolate(
				freq_low, volt_low, freq_high, volt_high,
				thread->corner[j].proc_freq);
	}

done:
	if (rc == 0) {
		cpr3_debug(thread, "unadjusted per-corner open-loop voltages:\n");
		for (i = 0; i < thread->corner_count; i++)
			cpr3_debug(thread, "open-loop[%2d] = %d uV\n", i,
				thread->corner[i].open_loop_volt);

		rc = cpr3_adjust_open_loop_voltages(thread, corner_sum,
			combo_offset);
		if (rc)
			cpr3_err(thread, "open-loop voltage adjustment failed, rc=%d\n",
				rc);
	}

	kfree(fuse_volt);
	kfree(fmax_corner);
	return rc;
}

/**
 * cpr3_mmss_print_settings() - print out MMSS CPR configuration settings into
 *		the kernel log for debugging purposes
 * @thread:		Pointer to the CPR3 thread
 */
static void cpr3_mmss_print_settings(struct cpr3_thread *thread)
{
	struct cpr3_corner *corner;
	int i;

	cpr3_debug(thread, "Corner: Frequency (Hz), Fuse Corner, Floor (uV), Open-Loop (uV), Ceiling (uV)\n");
	for (i = 0; i < thread->corner_count; i++) {
		corner = &thread->corner[i];
		cpr3_debug(thread, "%3d: %10u, %2d, %7d, %7d, %7d\n",
			i, corner->proc_freq, corner->cpr_fuse_corner,
			corner->floor_volt, corner->open_loop_volt,
			corner->ceiling_volt);
	}
}

/**
 * cpr3_mmss_init_thread() - perform all steps necessary to initialize the
 *		configuration data for a CPR3 thread
 * @thread:		Pointer to the CPR3 thread
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_mmss_init_thread(struct cpr3_thread *thread)
{
	struct cpr3_msm8996_mmss_fuses *fuse;
	int corner_sum = 0;
	int combo_offset = 0;
	int rc;

	rc = cpr3_msm8996_mmss_read_fuse_data(thread);
	if (rc) {
		cpr3_err(thread, "unable to read CPR fuse data, rc=%d\n", rc);
		return rc;
	}

	fuse = thread->platform_fuses;
	if (fuse->limitation == MSM8996_CPR_LIMITATION_UNSUPPORTED) {
		cpr3_err(thread, "this chip requires an unsupported voltage\n");
		return -EPERM;
	} else if (fuse->limitation
			== MSM8996_CPR_LIMITATION_NO_CPR_OR_INTERPOLATION) {
		thread->ctrl->cpr_allowed_hw = false;
	}

	rc = cpr3_parse_common_thread_data(thread);
	if (rc) {
		cpr3_err(thread, "unable to read CPR thread data from device tree, rc=%d\n",
			rc);
		return rc;
	}

	rc = cpr3_mmss_parse_corner_data(thread, &corner_sum, &combo_offset);
	if (rc) {
		cpr3_err(thread, "unable to read CPR corner data from device tree, rc=%d\n",
			rc);
		return rc;
	}

	rc = cpr3_mmss_adjust_target_quotients(thread, corner_sum,
			combo_offset);
	if (rc) {
		cpr3_err(thread, "unable to adjust target quotients, rc=%d\n",
			rc);
		return rc;
	}

	rc = cpr3_msm8996_mmss_calculate_open_loop_voltages(thread, corner_sum,
			combo_offset);
	if (rc) {
		cpr3_err(thread, "unable to calculate open-loop voltages, rc=%d\n",
			rc);
		return rc;
	}

	rc = cpr3_limit_open_loop_voltages(thread);
	if (rc) {
		cpr3_err(thread, "unable to limit open-loop voltages, rc=%d\n",
			rc);
		return rc;
	}

	cpr3_open_loop_voltage_as_ceiling(thread);

	cpr3_mmss_print_settings(thread);

	return 0;
}

/**
 * cpr3_mmss_init_controller() - perform MMSS CPR3 controller specific
 *		initializations
 * @ctrl:		Pointer to the CPR3 controller
 *
 * Return: 0 on success, errno on failure
 */
static int cpr3_mmss_init_controller(struct cpr3_controller *ctrl)
{
	int rc;

	rc = cpr3_parse_common_ctrl_data(ctrl);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			cpr3_err(ctrl, "unable to parse common controller data, rc=%d\n",
				rc);
		return rc;
	}

	ctrl->sensor_count = MSM8996_MMSS_CPR_SENSOR_COUNT;

	/*
	 * MMSS only has one thread (0) so the zeroed array does not need
	 * further modification.
	 */
	ctrl->sensor_owner = devm_kzalloc(ctrl->dev,
		sizeof(*ctrl->sensor_owner) * ctrl->sensor_count, GFP_KERNEL);
	if (!ctrl->sensor_owner) {
		cpr3_err(ctrl, "memory allocation failed\n");
		return -ENOMEM;
	}

	ctrl->cpr_clock_rate = MSM8996_MMSS_CPR_CLOCK_RATE;

	ctrl->iface_clk = devm_clk_get(ctrl->dev, "iface_clk");
	if (IS_ERR(ctrl->iface_clk)) {
		rc = PTR_ERR(ctrl->iface_clk);
		if (rc != -EPROBE_DEFER)
			cpr3_err(ctrl, "unable request interface clock, rc=%d\n",
				rc);
		return rc;
	}

	ctrl->bus_clk = devm_clk_get(ctrl->dev, "bus_clk");
	if (IS_ERR(ctrl->bus_clk)) {
		rc = PTR_ERR(ctrl->bus_clk);
		if (rc != -EPROBE_DEFER)
			cpr3_err(ctrl, "unable request bus clock, rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

static int cpr3_mmss_regulator_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct cpr3_controller *ctrl = platform_get_drvdata(pdev);

	return cpr3_regulator_suspend(ctrl);
}

static int cpr3_mmss_regulator_resume(struct platform_device *pdev)
{
	struct cpr3_controller *ctrl = platform_get_drvdata(pdev);

	return cpr3_regulator_resume(ctrl);
}

static struct of_device_id cpr_regulator_match_table[] = {
	{ .compatible = "qcom,cpr3-msm8996-mmss-regulator", },
	{}
};

static int cpr3_mmss_regulator_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cpr3_controller *ctrl;
	int rc;

	if (!dev->of_node) {
		dev_err(dev, "Device tree node is missing\n");
		return -EINVAL;
	}

	ctrl = devm_kzalloc(dev, sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl) {
		dev_err(dev, "cpr3 controller memory allocation failed\n");
		return -ENOMEM;
	}

	ctrl->dev = dev;
	/* Set to false later if anything precludes CPR operation. */
	ctrl->cpr_allowed_hw = true;

	rc = of_property_read_string(dev->of_node, "qcom,cpr-ctrl-name",
					&ctrl->name);
	if (rc) {
		cpr3_err(ctrl, "unable to read qcom,cpr-ctrl-name, rc=%d\n",
			rc);
		return rc;
	}

	rc = cpr3_map_fuse_base(ctrl, pdev);
	if (rc) {
		cpr3_err(ctrl, "could not map fuse base address\n");
		return rc;
	}

	rc = cpr3_allocate_threads(ctrl, 0, 0);
	if (rc) {
		cpr3_err(ctrl, "failed to allocate CPR thread array, rc=%d\n",
			rc);
		return rc;
	}

	if (ctrl->thread_count != 1) {
		cpr3_err(ctrl, "thread node is missing\n");
		return -EINVAL;
	}

	rc = cpr3_mmss_init_controller(ctrl);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			cpr3_err(ctrl, "failed to initialize CPR controller parameters, rc=%d\n",
				rc);
		return rc;
	}

	rc = cpr3_mmss_init_thread(&ctrl->thread[0]);
	if (rc) {
		cpr3_err(&ctrl->thread[0], "thread initialization failed, rc=%d\n",
			rc);
		return rc;
	}

	platform_set_drvdata(pdev, ctrl);

	return cpr3_regulator_register(pdev, ctrl);
}

static int cpr3_mmss_regulator_remove(struct platform_device *pdev)
{
	struct cpr3_controller *ctrl = platform_get_drvdata(pdev);

	return cpr3_regulator_unregister(ctrl);
}

static struct platform_driver cpr3_mmss_regulator_driver = {
	.driver		= {
		.name		= "qcom,cpr3-mmss-regulator",
		.of_match_table	= cpr_regulator_match_table,
		.owner		= THIS_MODULE,
	},
	.probe		= cpr3_mmss_regulator_probe,
	.remove		= cpr3_mmss_regulator_remove,
	.suspend	= cpr3_mmss_regulator_suspend,
	.resume		= cpr3_mmss_regulator_resume,
};

static int cpr_regulator_init(void)
{
	return platform_driver_register(&cpr3_mmss_regulator_driver);
}

static void cpr_regulator_exit(void)
{
	platform_driver_unregister(&cpr3_mmss_regulator_driver);
}

MODULE_DESCRIPTION("CPR3 MMSS regulator driver");
MODULE_LICENSE("GPL v2");

arch_initcall(cpr_regulator_init);
module_exit(cpr_regulator_exit);
