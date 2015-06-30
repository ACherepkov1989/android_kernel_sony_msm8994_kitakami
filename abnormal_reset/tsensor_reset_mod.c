/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>

#define V1_MPM2_TSENS_Sn_MIN_MAX_STATUS_CTRL(n) (0x004A8008 + 0x4 * n)
#define V2_MPM2_TSENS_Sn_MIN_MAX_STATUS_CTRL(n) (0x004A800C + 0x4 * n)

static int tsense_reset;

static int tsense_crash_set(const char *val, struct kernel_param *kp);
module_param_call(tsense_reset, tsense_crash_set, param_get_int, &tsense_reset, 0644);

static int tsense_crash_set(const char *val, struct kernel_param *kp)
{
	int ret;
	void *mpm2_tsens_min_max_status_ctrl;
	phys_addr_t addr;

	if((ret = param_set_int(val, kp)))
		return ret;

	if (tsense_reset != 1)
		return -EPERM;

	if (of_machine_is_compatible("qcom,msm8916") ||
		of_machine_is_compatible("qcom,msm8939") ||
		of_machine_is_compatible("qcom,msm8909"))
		addr = V1_MPM2_TSENS_Sn_MIN_MAX_STATUS_CTRL(0);
	else
		addr = V2_MPM2_TSENS_Sn_MIN_MAX_STATUS_CTRL(0);

	mpm2_tsens_min_max_status_ctrl = ioremap(addr, 0x4);

	if (!mpm2_tsens_min_max_status_ctrl) {
		pr_err("Abnormal reset test: Unable to map mpm2 register space\n");
		return -ENOMEM;
	}

	__raw_writel(0x1405, mpm2_tsens_min_max_status_ctrl);
	mb();
	mdelay(1000);
	pr_err("Abnormal reset test failed\n");
	iounmap(mpm2_tsens_min_max_status_ctrl);
	return -EIO;
}

static int tsensor_reset_init(void)
{
	return 0;
}

static void tsensor_reset_exit(void)
{
}

module_init(tsensor_reset_init);
module_exit(tsensor_reset_exit);
MODULE_LICENSE("GPL v2");
