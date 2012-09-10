/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include <linux/regulator/consumer.h>

struct clk_pair {
	const char *dev;
	const char *clk;
};


static struct clk_pair bus_timeout_clocks_on[] = {
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


static struct clk_pair bus_timeout_clocks_off[] = {
	{
		.dev = "fda10000.qcom,vfe",
		.clk = "camss_vfe_vfe_clk",
	}
};



#define HANG_ADDRESS 0xfda10000

static int bus_timeout_init(void)
{
	void *hang_addr;
	int i;
	int dummy;
	struct regulator *r;

	hang_addr = ioremap(HANG_ADDRESS, SZ_4K);

	r = regulator_get(NULL, "gdsc_vfe");
	regulator_enable(r);

	/*
	 * Yes, none of this cleans up properly but the goal here
	 * is to trigger a hang which is going to kill the rest of
	 * the system anyway
	 */

	for (i = 0; i < ARRAY_SIZE(bus_timeout_clocks_on); i++) {
		struct clk *this_clock;

		this_clock = clk_get_sys(bus_timeout_clocks_on[i].dev,
					bus_timeout_clocks_on[i].clk);
		clk_prepare_enable(this_clock);
	}

	for (i = 0; i < ARRAY_SIZE(bus_timeout_clocks_off); i++) {
		struct clk *this_clock;

		this_clock = clk_get_sys(bus_timeout_clocks_off[i].dev,
					 bus_timeout_clocks_off[i].clk);

		clk_disable(this_clock);
	}

	dummy = readl_relaxed(hang_addr);

	mdelay(15000);

	BUG();

	return 0;
}

static void bus_timeout_exit(void)
{
}

module_init(bus_timeout_init);
module_exit(bus_timeout_exit);
MODULE_LICENSE("GPL v2");

