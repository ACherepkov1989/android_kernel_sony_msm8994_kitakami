/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

struct clk_pair {
	const char *dev;
	const char *clk;
};

static int bus_timeout_camera = 1;
static int bus_timeout_usb = 1;

static int bus_timeout_camera_set(const char *val, struct kernel_param *kp);
module_param_call(bus_timeout_camera, bus_timeout_camera_set, param_get_int,
						&bus_timeout_camera, 0644);

static int bus_timeout_usb_set(const char *val, struct kernel_param *kp);
module_param_call(bus_timeout_usb, bus_timeout_usb_set, param_get_int,
						&bus_timeout_usb, 0644);

static struct clk_pair bus_timeout_camera_clocks_on[] = {
	/*
	 * gcc_mmss_noc_cfg_ahb_clk should be on but right
	 * now this clock is on by default and not accessable.
	 * Update this table if gcc_mmss_noc_cfg_ahb_clk is
	 * ever not enabled by default!
	 */
	{
		.dev = "fda0c000.qcom,cci",
		.clk = "camss_top_ahb_clk",
	},
	{
		.dev = "fda10000.qcom,vfe",
		.clk = "iface_clk",
	},
};

static struct clk_pair bus_timeout_camera_clocks_off[] = {
	{
		.dev = "fda10000.qcom,vfe",
		.clk = "camss_vfe_vfe_clk",
	}
};

static struct clk_pair bus_timeout_usb_clocks_on[] = {
	{
		.dev = "f9a55000.usb",
		.clk = "iface_clk",
	},
	{
		.dev = "f9a55000.usb",
		.clk = "core_clk",
	},
	{
		.dev = "msm_otg",
		.clk = "iface_clk",
	},
	{
		.dev = "msm_otg",
		.clk = "core_clk",
	},
};

static struct clk_pair bus_timeout_usb_clocks_off[] = {
	{
		.dev = "f9a55000.usb",
		.clk = "core_clk",
	},
	{
		.dev = "msm_otg",
		.clk = "core_clk",
	},
};

static void bus_timeout_clk_access(struct clk_pair bus_timeout_clocks_off[],
				struct clk_pair bus_timeout_clocks_on[],
				int off_size, int on_size)
{
	int i;

	/*
	 * Yes, none of this cleans up properly but the goal here
	 * is to trigger a hang which is going to kill the rest of
	 * the system anyway
	 */

	for (i = 0; i < on_size; i++) {
		struct clk *this_clock;

		this_clock = clk_get_sys(bus_timeout_clocks_on[i].dev,
					bus_timeout_clocks_on[i].clk);
		if (!IS_ERR(this_clock))
			if (clk_prepare_enable(this_clock))
				pr_warn("Device %s: Clock %s not enabled",
					bus_timeout_clocks_on[i].clk,
					bus_timeout_clocks_on[i].dev);
	}

	for (i = 0; i < off_size; i++) {
		struct clk *this_clock;

		this_clock = clk_get_sys(bus_timeout_clocks_off[i].dev,
					 bus_timeout_clocks_off[i].clk);
		if (!IS_ERR(this_clock))
			clk_disable_unprepare(this_clock);
	}
}

static int bus_timeout_camera_set(const char *val, struct kernel_param *kp)
{
	int ret;
	uint32_t dummy;
	uint32_t address = 0xfda10000;
	void *hang_addr;
	struct regulator *r;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (bus_timeout_camera != 1)
		return -EPERM;

	hang_addr = ioremap(address, SZ_4K);
	r = regulator_get(NULL, "gdsc_vfe");
	ret = IS_ERR(r);
	if (!ret)
		regulator_enable(r);
	else {
		pr_err("Bus timeout test: Unable to get regulator reference\n");
		return ret;
	}
	bus_timeout_clk_access(bus_timeout_camera_clocks_off,
				bus_timeout_camera_clocks_on,
				ARRAY_SIZE(bus_timeout_camera_clocks_off),
				ARRAY_SIZE(bus_timeout_camera_clocks_on));
	dummy = readl_relaxed(hang_addr);
	mdelay(15000);
	pr_err("Bus timeout test failed\n");
	iounmap(hang_addr);
	return -EIO;
}

static int bus_timeout_usb_set(const char *val, struct kernel_param *kp)
{
	int ret;
	uint32_t dummy;
	uint32_t address = 0xf9a55000;
	void *hang_addr;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (bus_timeout_usb != 1)
		return -EPERM;

	hang_addr = ioremap(address, SZ_4K);
	bus_timeout_clk_access(bus_timeout_usb_clocks_off,
				bus_timeout_usb_clocks_on,
				ARRAY_SIZE(bus_timeout_usb_clocks_off),
				ARRAY_SIZE(bus_timeout_usb_clocks_on));
	dummy = readl_relaxed(hang_addr);
	mdelay(15000);
	pr_err("Bus timeout test failed: 0x%x\n", dummy);
	iounmap(hang_addr);
	return -EIO;
}

static int bus_timeout_init(void)
{
	return 0;
}

static void bus_timeout_exit(void)
{
}

module_init(bus_timeout_init);
module_exit(bus_timeout_exit);
MODULE_LICENSE("GPL v2");
