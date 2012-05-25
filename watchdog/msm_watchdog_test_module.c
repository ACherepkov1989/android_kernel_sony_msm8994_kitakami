/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>

struct completion timeout_complete;

static void timeout_work(struct work_struct *work)
{
	local_irq_disable();
	mdelay(15000);
	local_irq_enable();
	complete(&timeout_complete);
}

static DECLARE_WORK(timeout_work_struct, timeout_work);

static int msm_watchdog_test_init(void)
{
	init_completion(&timeout_complete);
	schedule_work_on(0, &timeout_work_struct);
	wait_for_completion(&timeout_complete);
	return -1;
}

module_init(msm_watchdog_test_init)
MODULE_LICENSE("GPL v2");
