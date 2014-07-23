/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <linux/atomic.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#define MODULE_NAME "hrtimer_test"

#define MS_TO_NS(x)	(x * 1000000)
#define SEC_TO_NS(x)	(x * 1000000000)

static struct hrtimer my_timer[2];
struct task_struct *k[16];
static atomic_t counter = ATOMIC_INIT(0);
static DEFINE_MUTEX(hrtimer_mutex);
static struct dentry *dent;
static int hrtimer_num = 2, loop = 1000, start = 0, kill_test = 0;

static ssize_t hrtimer_debug_read_stats(struct file *file, char __user *ubuf,
										size_t count, loff_t *ppos)
{
	char buf[20];
	int	ret = 0;

	ret += scnprintf(buf, sizeof(buf), "%d\n", counter.counter);
	ret = simple_read_from_buffer(ubuf, count, ppos, buf, ret);

	return ret;
}

static int set_hrtimer_num(void *data, u64 val)
{
	if (val != 0) {
		pr_debug("set_hrtimer_num: %llu\n", val);
		hrtimer_num = val;
	}
	return 0;
}

static int get_hrtimer_num(void *data, u64 *val)
{
	*val = hrtimer_num;
	return 0;
}

static int set_hrtimer_loop(void *data, u64 val)
{
	if (val != 0) {
		pr_debug("set_hrtimer_count: %llu\n", val);
		loop = val;
	}
	return 0;
}

static int get_hrtimer_loop(void *data, u64 *val)
{
	*val = loop;
	return 0;
}

static int hrtimer_test_start(void);
static int hrtimer_test_stop(void);
static int set_hrtimer_start(void *data, u64 val)
{
	start = val;
	if (val == 1) {
		pr_debug("set_hrtimer_start: %llu\n", val);
		kill_test = 0;
		hrtimer_test_start();
	} else if (val == 0) {
		kill_test = 1;
		hrtimer_test_stop();
	}
	return 0;
}

static int get_hrtimer_start(void *data, u64 *val)
{
	*val = start;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(hrtimer_num_ops,
	get_hrtimer_num, set_hrtimer_num, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(hrtimer_loop_ops,
	get_hrtimer_loop, set_hrtimer_loop, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(hrtimer_start_ops,
	get_hrtimer_start, set_hrtimer_start, "%llu\n");

const struct file_operations hrtimer_stats_ops = {
	.read = hrtimer_debug_read_stats,
};

static void creat_hrtimer_debugfs(void)
{
	/*
	* Create a simple debugfs with the name of "hrtimer-test",
	*
	* As this is a simple directory, no uevent will be sent to
	* userspace.
	*/
	struct dentry *dfile_status, *dfile_hrtimer_num, *dfile_loop, *dfile_start;

	dent = debugfs_create_dir("hrtimer-test", 0);
	if (IS_ERR(dent))
		return;

	dfile_status = debugfs_create_file("status", 0666, dent, 0, &hrtimer_stats_ops);
	if (!dfile_status || IS_ERR(dfile_status))
		debugfs_remove(dent);

	dfile_hrtimer_num = debugfs_create_file("timer_num", 0666, dent, 0, &hrtimer_num_ops);
	if (!dfile_hrtimer_num || IS_ERR(dfile_hrtimer_num))
		debugfs_remove(dent);

	dfile_loop = debugfs_create_file("count", 0666, dent, 0, &hrtimer_loop_ops);
	if (!dfile_loop || IS_ERR(dfile_loop))
		debugfs_remove(dent);

	dfile_start = debugfs_create_file("start", 0666, dent, 0, &hrtimer_start_ops);
	if (!dfile_start || IS_ERR(dfile_start))
		debugfs_remove(dent);

}

static enum hrtimer_restart my_hrtimer_callback(struct hrtimer *timer)
{
	pr_debug("hrtimer_callback: Jiffies (%lu).  Process Name(%s) PID(%d) timer expired: %lu\n",
			jiffies, timer->start_comm,
			timer->start_pid,
			(unsigned long)ktime_to_us(timer->_softexpires));
	atomic_dec(&counter);
	return HRTIMER_NORESTART;
}

static void wait_to_die(void)
{
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);
}

static int hrtimer_test_handler(void *data)
{
	unsigned long delay_in_ms = (unsigned long)data;
	ktime_t ktime;
	int ret;
	int i = loop, j;

	atomic_inc(&counter);
	while (i-- && !kill_test) {
		for(j = 0; j < hrtimer_num && !kill_test; j++) {
			mutex_lock(&hrtimer_mutex);
			ret = hrtimer_cancel(&my_timer[j]);
			if (ret == 1) {
				pr_debug("HANDLER: The Timer is canceled ...\n");
				atomic_dec(&counter);
			}
			hrtimer_set_expires(&my_timer[j], ns_to_ktime(0));
			ktime = ktime_set(0, MS_TO_NS(delay_in_ms));
			pr_debug("HANDLER: Starting timer fire in %ldms (jiffies:%lu); loop = %d \n",
					delay_in_ms, jiffies, i);

			atomic_inc(&counter);
			hrtimer_start(&my_timer[j], ktime, HRTIMER_MODE_REL);
			mutex_unlock(&hrtimer_mutex);
		}
		mdelay(20);
	}
	atomic_dec(&counter);
	wait_to_die();
	pr_info("HANDLER: Starting timer fire in %ldms finished\n",
			delay_in_ms);

	return 0;
}

static int hrtimer_test_start(void)
{
	ktime_t ktime;
	char thread_str[20] = "";
	unsigned int cpu, i;
	unsigned long init_delay_ms = 2;
	unsigned long delay_in_ms = 200L;

	ktime = ktime_set(0, MS_TO_NS(delay_in_ms));
	for(i = 0; i < hrtimer_num; i++) {
		hrtimer_init(&my_timer[i], CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		my_timer[i].function = &my_hrtimer_callback;
		atomic_inc(&counter);

		pr_info("PROBE: Starting timer to fire in %ldms (%lu)\n",
		delay_in_ms, jiffies);

		hrtimer_start(&my_timer[i], ktime, HRTIMER_MODE_REL);
	}

	for_each_present_cpu(cpu) {
#ifdef CONFIG_HOTPLUG_CPU
		/*Initial to bring all cpu cores online. It is to create thread for each CPU core and wait up afterwards.*/
		if (!cpu_online(cpu))
			cpu_up(cpu);
#endif

		snprintf(thread_str, sizeof(thread_str), "hrtimer_thread%d", cpu);
		k[cpu] = kthread_create(&hrtimer_test_handler, (void *)init_delay_ms,thread_str);
		kthread_bind(k[cpu],cpu);
		wake_up_process(k[cpu]);
		mdelay(init_delay_ms);
		init_delay_ms += 3;
	}

	return 0;
}

static int hrtimer_test_stop(void)
{
	unsigned int cpu;

	/* We need to destroy also the parked threads of offline cpus */
	for_each_possible_cpu(cpu) {
		if (k[cpu]) {
			kthread_stop(k[cpu]);
			k[cpu] = NULL;
		}
	}
	return 0;
}

static int hrtimer_test_remove(struct platform_device *pdev)
{
	hrtimer_test_stop();

	if (dent) {
		debugfs_remove_recursive(dent);
		dent = NULL;
	}
	kill_test = 0;
	return 0;
}

static int hrtimer_test_probe(struct platform_device *pdev)
{
	creat_hrtimer_debugfs();

	return 0;
}

static struct platform_driver hrtimer_test_driver = {
	.probe = hrtimer_test_probe,
	.remove = hrtimer_test_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device hrtimer_test_device = {
	.name = MODULE_NAME,
	.id   = -1,
};

static int init_hrtimer_test(void)
{
	platform_device_register(&hrtimer_test_device);
	return platform_driver_register(&hrtimer_test_driver);
}


static void exit_hrtimer_test(void)
{
	platform_driver_unregister(&hrtimer_test_driver);
	platform_device_unregister(&hrtimer_test_device);
}

MODULE_DESCRIPTION("HRTIMER TEST");
MODULE_LICENSE("GPL v2");
module_init(init_hrtimer_test);
module_exit(exit_hrtimer_test);
