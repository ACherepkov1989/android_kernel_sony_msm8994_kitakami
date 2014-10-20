/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/mod_devicetable.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#define WDT0_RST	0x04
#define WDT0_EN		0x08
#define SEC_WDOG_RESET_OFFSET	0x0
#define SEC_WDOG_CTL_OFFSET	0x4
#define SEC_WDOG_BARK_OFFSET	0xc
#define SEC_WDOG_BITE_OFFSET	0x10
#define SEC_WDOG_BARK_VAL	0x7ffff
#define SEC_WDOG_BITE_VAL	0x20
#define SEC_WDOG_RESET_DO_RESET	0x1
#define SEC_WDOG_CTL_ENABLE	0x1

enum msm_target_index {
	DEFAULT = 0,
	MSM8916 = 1,
};

struct clk_pair {
	const char *compat;
	const char *dev;
	const char *clk;
};

static int bus_timeout_camera;
static int bus_timeout_usb;
static int pc_save;

struct bus_timeout_clks {
	struct clk_pair *clks_on;
	struct clk_pair *clks_off;
	uint32_t clks_on_sz;
	uint32_t clks_off_sz;
};

struct bus_timeout_data {
	const char *target;
	uint32_t ahb_bus_timeout;
	uint32_t sec_wdog;
	uint32_t mmss_vfe_base;
	uint32_t usb_otg_hs_base;
};

static int bus_timeout_camera_set(const char *val, struct kernel_param *kp);
module_param_call(bus_timeout_camera, bus_timeout_camera_set, param_get_int,
						&bus_timeout_camera, 0644);

static int bus_timeout_usb_set(const char *val, struct kernel_param *kp);
module_param_call(bus_timeout_usb, bus_timeout_usb_set, param_get_int,
						&bus_timeout_usb, 0644);

static int pc_save_set(const char *val, struct kernel_param *kp);
module_param_call(pc_save, pc_save_set, param_get_int, &pc_save, 0644);

struct completion pcsave_complete;

static struct clk_pair bus_timeout_camera_clocks_on[] = {
	/*
	 * gcc_mmss_noc_cfg_ahb_clk should be on but right
	 * now this clock is on by default and not accessable.
	 * Update this table if gcc_mmss_noc_cfg_ahb_clk is
	 * ever not enabled by default!
	 */
	{
		.compat = "qcom,cci",
		.dev = "fda0c000.qcom,cci",
		.clk = "camss_top_ahb_clk",
	},
	{
		.compat = "qcom,vfe40",
		.dev = "fda10000.qcom,vfe",
		.clk = "iface_clk",
	},
	{
		.compat = "qcom,vfe40",
		.dev = "fda10000.qcom,vfe",
		.clk = "camss_ahb_clk",
	}
};

static struct clk_pair bus_timeout_camera_clocks_off[] = {
	{
		.compat = "qcom,vfe40",
		.dev = "fda10000.qcom,vfe",
		.clk = "camss_vfe_vfe_clk",
	},
};

static struct bus_timeout_clks camera = {
	.clks_on = bus_timeout_camera_clocks_on,
	.clks_off = bus_timeout_camera_clocks_off,
	.clks_on_sz = ARRAY_SIZE(bus_timeout_camera_clocks_on),
	.clks_off_sz = ARRAY_SIZE(bus_timeout_camera_clocks_off),
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
		.compat = "qcom,hsusb-otg",
		.dev = "msm_otg",
		.clk = "iface_clk",
	},
	{
		.compat = "qcom,hsusb-otg",
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
		.compat = "qcom,hsusb-otg",
		.dev = "msm_otg",
		.clk = "core_clk",
	},
};

static struct bus_timeout_clks usb = {
	.clks_on = bus_timeout_usb_clocks_on,
	.clks_off = bus_timeout_usb_clocks_off,
	.clks_on_sz = ARRAY_SIZE(bus_timeout_usb_clocks_on),
	.clks_off_sz = ARRAY_SIZE(bus_timeout_usb_clocks_off),
};

static struct bus_timeout_data data[] = {
	[DEFAULT] = {
			.target = "msm8974",
			.ahb_bus_timeout = 0xfd4a0800,
			.sec_wdog = 0xfc4aa000,
			.mmss_vfe_base = 0xfda10000,
			.usb_otg_hs_base = 0xf9a55000,
		},
	[MSM8916] = {
			.target = "msm8916, msm8939, msm8909",
			.ahb_bus_timeout = 0x193c000,
			.sec_wdog = 0x4aa000,
			.mmss_vfe_base = 0x1b10000,
			.usb_otg_hs_base = 0x78d9000,
		},
};

static struct bus_timeout_data *bus_timeout_get_data(void)
{
	uint32_t index = DEFAULT;
	int rc;

	rc = of_machine_is_compatible("qcom,msm8916")
				|| of_machine_is_compatible("qcom,msm8939")
				|| of_machine_is_compatible("qcom,msm8909");
	if (rc)
		index = MSM8916;

	if (index >= ARRAY_SIZE(data)) {
		pr_err("Bus timeout test: index out of range\n");
		return NULL;
	}

	pr_info("Bus timeout test: target %s\n", data[index].target);
	return &data[index];
}

static void bus_timeout_clk_access(struct bus_timeout_clks *bus_timeout_clks)
{
	int i;
	struct device_node *node;
	struct clk *this_clock;

	if (!bus_timeout_clks) {
		pr_err("Bus timeout test: Missing clocks\n");
		return ;
	}

	/*
	 * Yes, none of this cleans up properly but the goal here
	 * is to trigger a hang which is going to kill the rest of
	 * the system anyway
	 */
	for (i = 0; i < bus_timeout_clks->clks_on_sz; i++) {
		this_clock = NULL;
		/*
		 * firstly, use the compatible string to find the clock.
		 * if failed, try to dev_id to find the clock from target
		 * related clk lookup table.
		 */
		if (bus_timeout_clks->clks_on[i].compat) {
			node = of_find_compatible_node(NULL, NULL,
							bus_timeout_clks->clks_on[i].compat);
			if (node)
				this_clock = of_clk_get_by_name(node,
								bus_timeout_clks->clks_on[i].clk);
		} else if (bus_timeout_clks->clks_on[i].dev) {
			this_clock = clk_get_sys(bus_timeout_clks->clks_on[i].dev,
							bus_timeout_clks->clks_on[i].clk);
		} else {
			pr_err("Bus timeout test: Invalid clock\n");
			continue;
		}
		if (!IS_ERR(this_clock)) {
			if (clk_prepare_enable(this_clock))
				pr_warn("Device %s: Clock %s not enabled",
					bus_timeout_clks->clks_on[i].dev,
					bus_timeout_clks->clks_on[i].clk);
		}
	}

	for (i = 0; i < bus_timeout_clks->clks_off_sz; i++) {
		this_clock = NULL;
		if (bus_timeout_clks->clks_off[i].compat) {
			node = of_find_compatible_node(NULL, NULL,
							bus_timeout_clks->clks_off[i].compat);
			if (node)
				this_clock = of_clk_get_by_name(node,
								bus_timeout_clks->clks_off[i].clk);
		} else if (bus_timeout_clks->clks_off[i].dev) {
			this_clock = clk_get_sys(bus_timeout_clks->clks_off[i].dev,
						bus_timeout_clks->clks_off[i].clk);
		} else {
			pr_err("Bus timeout test: Invalid clock\n");
			continue;
		}
		if (!IS_ERR(this_clock))
			clk_disable_unprepare(this_clock);
	}
}

static int bus_timeout_camera_set(const char *val, struct kernel_param *kp)
{
	int ret;
	uint32_t dummy;
	uint32_t address;
	void *hang_addr;
	struct regulator *r;
	struct bus_timeout_data *data = NULL;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (bus_timeout_camera != 1)
		return -EPERM;
	if (!(data = bus_timeout_get_data())) {
		pr_err("Bus timeout test: Unable to get target resource\n");
		return -EINVAL;
	}
	address = data->mmss_vfe_base;
	if (!address) {
		pr_err("Bus timeout test: Unable to get camera address\n");
		return -EINVAL;
	}
	hang_addr = ioremap(address, SZ_4K);
	r = regulator_get(NULL, "gdsc_vfe");
	ret = IS_ERR(r);
	if (!ret) {
		ret = regulator_enable(r);
		if (ret) {
			pr_err("Bus timeout test: regulator failed to enable\n");
			return ret;
		}
	} else {
		pr_err("Bus timeout test: Unable to get regulator reference\n");
		return ret;
	}
	bus_timeout_clk_access(&camera);
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
	uint32_t address;
	void *hang_addr;
	struct bus_timeout_data *data = NULL;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (bus_timeout_usb != 1)
		return -EPERM;
	if (!(data = bus_timeout_get_data())) {
		pr_err("Bus timeout test: Unable to get target resources\n");
		return -EINVAL;
	}
	address = data->usb_otg_hs_base;
	if (!address) {
		pr_err("Bus timeout test: Unable to get usb address\n");
		return -EINVAL;
	}
	hang_addr = ioremap(address, SZ_4K);
	bus_timeout_clk_access(&usb);
	dummy = readl_relaxed(hang_addr);
	mdelay(15000);
	pr_err("Bus timeout test failed: 0x%x\n", dummy);
	iounmap(hang_addr);
	return -EIO;
}

static const struct of_device_id apps_wdog_of_match[] = {
	{ .compatible	= "qcom,msm-watchdog" },
	{},
};

static void __iomem *wdogbase;
static void *sec_wdog_virt;
static void *global_bus_timeout_disable;
static void *hang_addr;

static int msm_apps_wdog_probe(void)
{
	int ret;
	struct device_node *node;
	uint32_t base;

	node = of_find_matching_node(NULL, apps_wdog_of_match);
	if (node) {
		ret = of_property_read_u32(node, "reg", &base);
		if (ret) {
			pr_err("PC-Save test: Cannot read wdog base\n");
			return ret;
		}
		wdogbase = ioremap(base, SZ_4K);
		if (!wdogbase) {
			pr_err("PC-save test: Cannot map wdog register space\n");
			return -ENOMEM;
		}
	} else {
		pr_info("PC-save test: wdog node not found\n");
		return -EINVAL;
	}
	return 0;
}

static void msm_apps_wdog_disable(void)
{
	__raw_writel(1, wdogbase + WDT0_RST);
	__raw_writel(0, wdogbase + WDT0_EN);
	mb();
}

static void pc_save_work(struct work_struct *work)
{
	uint32_t dummy;
	unsigned long flags;
	uint32_t sec_status;

	local_irq_save(flags);

	/* Disable sec watchdog */
	writel_relaxed(SEC_WDOG_RESET_DO_RESET, (sec_wdog_virt +
		       SEC_WDOG_RESET_OFFSET));
	sec_status = readl_relaxed(sec_wdog_virt + SEC_WDOG_CTL_OFFSET);
	writel_relaxed(sec_status & ~SEC_WDOG_CTL_ENABLE, (sec_wdog_virt +
		       SEC_WDOG_CTL_OFFSET));
	/* Make sure sec watchdog is disabled before continuing */
	mb();

	/* Disable apps watchdog */
	msm_apps_wdog_disable();

	/* Set sec watchdog bite time < bark time */
	writel_relaxed(SEC_WDOG_BARK_VAL, (sec_wdog_virt +
		       SEC_WDOG_BARK_OFFSET));
	/* 1ms bite time in sleep clock cycles */
	writel_relaxed(SEC_WDOG_BITE_VAL, (sec_wdog_virt +
		       SEC_WDOG_BITE_OFFSET));
	writel_relaxed(sec_status | SEC_WDOG_CTL_ENABLE, (sec_wdog_virt +
		       SEC_WDOG_CTL_OFFSET));
	/* Make sure sec watchdog is enabled before continuing */
	mb();

	/* Disable Bus timeout */
	writel_relaxed(0x0, global_bus_timeout_disable);
	/* Make sure bus timeout is disabled before continuing */
	mb();

	dummy = readl_relaxed(hang_addr);
	mb();

	mdelay(15000);

	local_irq_restore(flags);

	complete(&pcsave_complete);
}

DECLARE_WORK(pc_save_work_struct, pc_save_work);
static int pc_save_set(const char *val, struct kernel_param *kp)
{
	int ret;
	uint32_t address;
	struct regulator *r;
	struct bus_timeout_data *data = NULL;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (pc_save != 1)
		return -EPERM;

	init_completion(&pcsave_complete);

	ret = msm_apps_wdog_probe();
	if (ret)
		return ret;
	if (!(data = bus_timeout_get_data())) {
		pr_err("Bus timeout test: Unable to get target resouces\n");
		return -EINVAL;
	}

	address = data->mmss_vfe_base;
	if (!address) {
		pr_err("Bus timeout test: Unable to get camera address\n");
		return -EINVAL;
	}
	hang_addr = ioremap(address, SZ_4K);
	if (!hang_addr) {
		pr_err("PC-save test: unable to map hang address\n");
		return -ENOMEM;
	}
	if (!data->ahb_bus_timeout ||
		!(global_bus_timeout_disable =
			ioremap(data->ahb_bus_timeout, SZ_4K))) {
		pr_err("PC-save test: unable to map bus timeout disable\
					AHB_BUS_TIMEOUT register\n");
		iounmap(hang_addr);
		iounmap(wdogbase);
		return -ENOMEM;
	}
	r = regulator_get(NULL, "gdsc_vfe");
	ret = IS_ERR(r);
	if (!ret) {
		ret = regulator_enable(r);
		if (ret < 0) {
			pr_err("PC-save test: unable to enable regulator\n");
			return ret;
		}
	} else {
		pr_err("PC-save test: Unable to get regulator reference\n");
		return ret;
	}
	bus_timeout_clk_access(&camera);

	if (!data->sec_wdog ||
			!(sec_wdog_virt = ioremap(data->sec_wdog, SZ_4K))) {
		pr_err("unable to map sec wdog page\n");
		iounmap(hang_addr);
		iounmap(wdogbase);
		iounmap(global_bus_timeout_disable);
		return -ENOMEM;
	}

	schedule_work(&pc_save_work_struct);
	wait_for_completion(&pcsave_complete);
	pr_err("Bus timeout test failed\n");
	iounmap(hang_addr);
	iounmap(wdogbase);
	iounmap(global_bus_timeout_disable);
	iounmap(sec_wdog_virt);
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
