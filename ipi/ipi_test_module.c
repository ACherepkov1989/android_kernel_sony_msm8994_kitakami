/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/printk.h>

#define MODULE_NAME "msm_ipi_test"

struct ipi_test_work_data {
	struct work_struct work;
	struct completion complete;
};

static struct ipi_test_work_data work_data;
static int ipi_test_array[NR_CPUS][NR_CPUS];

static u64 ipi_test_times;
static int ipi_test_result = -1;

static void ipi_test_call(void *data)
{
	long i, j;

	preempt_disable();
	i = (long)data;
	j = raw_smp_processor_id();
	ipi_test_array[i][j]++;
	preempt_enable();
}

static void ipi_test_work(struct work_struct *work)
{
	struct ipi_test_work_data *work_data = container_of(work,
				struct ipi_test_work_data, work);

	pr_debug("I am IPI test work item on cpu %d\n", raw_smp_processor_id());

	smp_call_function(ipi_test_call, (void*)(long)raw_smp_processor_id(), true);
	complete(&work_data->complete);
}

static int ipi_test_run(void)
{
	int cpu;
	int i, j;
	int err = 0;

	pr_debug("IPI test starting\n");
	ipi_test_result = -1;

	memset(ipi_test_array, 0, sizeof(ipi_test_array));

#ifdef CONFIG_HOTPLUG_CPU
	for_each_present_cpu(cpu) {
		if (!cpu_online(cpu)) {
			err = cpu_up(cpu);
			if (err) {
				pr_debug("IPI test up cpu %d was failed!\n", cpu);
				return err;
			}
		}
	}
#endif
	get_online_cpus();

	for (i = 0; i < ipi_test_times; i ++) {
		for_each_online_cpu(cpu) {
			INIT_WORK(&work_data.work, ipi_test_work);
			init_completion(&work_data.complete);
			schedule_work_on(cpu, &work_data.work);
			wait_for_completion(&work_data.complete);
		}
	}

	err = 0;
	for_each_online_cpu(i) {
		for_each_online_cpu(j) {
			pr_debug("IPI test: CPU %d -> %d: %d\n", i, j, ipi_test_array[i][j]);

			if (i != j && (ipi_test_array[i][j]) != ipi_test_times) {
				pr_alert("IPI test erred i = %d j = %d val = %d\n",
					 i, j, ipi_test_array[i][j]);
				err = 1;
			}
		}
	}

	put_online_cpus();

	return err;
}

static int set_ipi_test(void *data, u64 val)
{
	if (val > 0)
		ipi_test_times = val;
	else
		return -1;

	ipi_test_result = ipi_test_run();

	return 0;
}

static int get_ipi_test_result(void *data, u64 *val)
{
	*val = ipi_test_result;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ipitest_times_fops, NULL, set_ipi_test, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(ipitest_result_fops, get_ipi_test_result,
						NULL, "%llu\n");

static struct dentry *ipi_test_dir_dentry;

static int __init ipi_test_init(void)
{
	struct dentry *ipi_test_times_dentry;
	struct dentry *ipi_test_result_dentry;

	ipi_test_dir_dentry = debugfs_create_dir("ipi_test", 0);
	if (!ipi_test_dir_dentry) {
		pr_err("Failed to create the debugfs ipi_test file\n");
		goto out;
	}

	ipi_test_times_dentry = debugfs_create_file("test_times", 0666,
							ipi_test_dir_dentry, NULL, &ipitest_times_fops);
	if (!ipi_test_times_dentry) {
		pr_err("Failed to create the debugfs ipi_test file\n");
		goto out_rmdebugfs_dir;
	}

	ipi_test_result_dentry = debugfs_create_file("test_result", 0666,
							ipi_test_dir_dentry, NULL, &ipitest_result_fops);
	if (!ipi_test_result_dentry) {
		pr_err("Failed to create the debugfs ipi_test_result file\n");
		goto out_rmdebugfs;
	}

	return 0;

out_rmdebugfs:
	debugfs_remove(ipi_test_times_dentry);
out_rmdebugfs_dir:
	debugfs_remove_recursive(ipi_test_dir_dentry);
out:
	return -ENODEV;
}

static void __exit ipi_test_exit(void)
{
	debugfs_remove_recursive(ipi_test_dir_dentry);
}

module_init(ipi_test_init);
module_exit(ipi_test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM IPI test");
