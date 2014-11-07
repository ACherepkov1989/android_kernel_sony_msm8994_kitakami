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
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/moduleparam.h>
#include <linux/cpu.h>
#include <linux/debugfs.h>

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#define MODULE_NAME "ldrex_test"

struct monitor_lock {
	unsigned int lock[NR_CPUS];
	unsigned int count;
	unsigned int repeat_time;
	unsigned int cachable;
};

struct test_parameter {
	int cachable;
	unsigned int same_lock;
	unsigned int single_cpu;
	unsigned int lock_count;
	unsigned int repeat_time;
	unsigned int multi_thread;
	int wait_cpu;
};

static struct dentry *dent;
static struct monitor_lock * pmon_locks[NR_CPUS];
static struct task_struct* k[NR_CPUS][NR_CPUS];
static int start;
static struct device * ldrex_dev;

static void wait_to_die(void)
{
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop()) {
		schedule();
		set_current_state(TASK_INTERRUPTIBLE);
	}
	__set_current_state(TASK_RUNNING);
}

static void* do_alloc_cached(size_t size)
{
	return kmalloc(size, GFP_KERNEL);
}

static void* do_alloc_coherent(size_t size)
{
	dma_addr_t dma_addr; /* DMA address of aligned chunk    */
	struct device *pdev = NULL;
#ifdef CONFIG_ARM64
	pdev = ldrex_dev;
	if (!pdev)
		return NULL;
#endif
	return dma_alloc_coherent(pdev, size, &dma_addr, 0);
}

#if defined(CONFIG_ARM)
static unsigned int do_ldrex_strex(struct monitor_lock *pmon_lock)
{
	unsigned int result, new_result, tmp;

	__asm__ __volatile__(
		" ldrex %[result],[%[lock]]\n"
		" add %[new_result],%[result], #1\n"
		" strex	%[tmp], %[new_result], [%[lock]]\n"
		: [result] "=&r" (result), [new_result] "=&r" (new_result), [tmp] "=&r" (tmp)
		: [lock] "r" (&pmon_lock->lock[0])
		: "cc");
	return !(tmp);
}

static void do_ldrex(unsigned int * l)
{
	unsigned int result;

	__asm__ __volatile__(
		"ldrex %[result],[%[lock]]\n"
		: [result] "=&r" (result)
		: [lock] "r" (l)
		: "cc");
}
#elif defined(CONFIG_ARM64)
static unsigned int do_ldrex_strex(struct monitor_lock *pmon_lock)
{
        unsigned int result, new_result, tmp;

        __asm__ __volatile__(
                " ldxr %w[result],[%[lock]]\n"
                " add %w[new_result],%w[result], #1\n"
                " stxr %w[tmp], %w[new_result], [%[lock]]\n"
                : [result] "=&r" (result), [new_result] "=&r" (new_result), [tmp] "=&r" (tmp)
                : [lock] "r" (&pmon_lock->lock[0])
                : "cc");
	return !(tmp);
}

static void do_ldrex(unsigned int * l)
{
	unsigned int result;

        __asm__ __volatile__(
                "ldxr %w[result],[%[lock]]\n"
                : [result] "=&r" (result)
                : [lock] "r" (l)
                : "cc");
}
#else
static unsigned long do_ldrex_strex(struct monitor_lock *pmon_lock)
{
	return 0;
}

static void do_ldrex(unsigned int * l) {}
#endif

/*
 * Uncacheable: 4 LDREX on same memory location - same cpu
 */
static int ldrex_test_handler(void *data)
{
	struct monitor_lock* pmon_lock = (struct monitor_lock*)data;
	int i;
	int j;
	pmon_lock->count = pmon_lock->count > NR_CPUS ? NR_CPUS : pmon_lock->count;
	pr_debug("%s: Issuing %d different Ldrex repeat %d times on Thread %d\n",
		__func__, pmon_lock->count, pmon_lock->repeat_time, current->pid);

	for(i = 0; i < pmon_lock->count; i++)
	{
		for(j = 0; j < pmon_lock->repeat_time; j++)
		{
			do_ldrex(&pmon_lock->lock[i]);
		}
	}

	wait_to_die();

	return 0;
}

static int ldrex_wait_handler(void *data)
{
	struct monitor_lock* pmon_lock = (struct monitor_lock*)data;

	pr_debug("%s: Waiting Ldrex on Thread %d\n", __func__, current->pid);

	while (!do_ldrex_strex(pmon_lock))
		pr_debug("Waiting for lock in pid %d\n", current->pid);

	wait_to_die();

	return 0;
}

static int test_stop(void);
static int run_test(struct test_parameter t_param)
{
	int i;
	int j;
	unsigned int cpu;
	char thread_str[50] = "";
	int cachable = t_param.cachable;
	unsigned int same_lock = t_param.same_lock;
	unsigned int single_cpu = t_param.single_cpu;
	unsigned int lock_count = t_param.lock_count;
	unsigned int repeat_time = t_param.repeat_time;
	unsigned int multi_thread = t_param.multi_thread;
	int wait_cpu = t_param.wait_cpu;

	for(i = 0; i < ARRAY_SIZE(pmon_locks); i++)
	{
		if(cachable)
			pmon_locks[i] = do_alloc_cached(sizeof(struct monitor_lock));
		else
			pmon_locks[i] = do_alloc_coherent(sizeof(struct monitor_lock));

		if (!pmon_locks[i])
		{
			pr_err("Unable to allocate memory\n");
			goto err;
		}

		for(j = 0; j < ARRAY_SIZE(pmon_locks[i]->lock); j++)
			pmon_locks[i]->lock[j] = 1;
		pmon_locks[i]->count = lock_count;
		pmon_locks[i]->repeat_time = repeat_time;
		pmon_locks[i]->cachable = cachable;
	}

	for_each_present_cpu(cpu) {
                /*Initial to bring all cpu cores online. It is to create thread for each CPU core and wait up afterwards.*/
                if (!cpu_online(cpu))
                        cpu_up(cpu);

	}

	get_online_cpus();

	for_each_online_cpu(cpu) {
		if (wait_cpu == (int)cpu)
			continue;

		if(multi_thread){
			for(i=0; i<NR_CPUS; i++){
				snprintf(thread_str, sizeof(thread_str), "ldrext_test_w%d_%d", wait_cpu,i);
	                        k[cpu][i] = kthread_create(&ldrex_wait_handler, (void *)pmon_locks[cpu], thread_str);
				kthread_bind(k[cpu][i],cpu);
				wake_up_process(k[cpu][i]);
			}
		} else {
	                snprintf(thread_str, sizeof(thread_str), "ldrext_test_t%d", cpu);
	                if(same_lock)
				k[cpu][0] = kthread_create(&ldrex_test_handler, (void *)pmon_locks[0], thread_str);
			else
				k[cpu][0] = kthread_create(&ldrex_test_handler, (void *)pmon_locks[cpu], thread_str);
			kthread_bind(k[cpu][0],cpu);
			wake_up_process(k[cpu][0]);
		}
		if(single_cpu)
			break;

	}

	if(wait_cpu > 0 && cpu_present(wait_cpu))
	{
		mdelay(10);
		if (!cpu_online(wait_cpu))
			cpu_up(wait_cpu);
		snprintf(thread_str, sizeof(thread_str), "ldrext_test_w%d", wait_cpu);
		k[wait_cpu][0] = kthread_create(&ldrex_wait_handler, (void *)pmon_locks[wait_cpu], thread_str);
		kthread_bind(k[wait_cpu][0],wait_cpu);
		wake_up_process(k[wait_cpu][0]);
	}

        put_online_cpus();
	return 0;

err:
	test_stop();
	return -1;
}

static int test_stop(void)
{
	unsigned int cpu;
	int i;
	int num = ARRAY_SIZE(pmon_locks);

	for_each_online_cpu(cpu) {
		for(i=0; i<NR_CPUS; i++){
			if (k[cpu][i]) {
				kthread_stop(k[cpu][i]);
				k[cpu][i] = NULL;
			}
		}
	}

	for(i = 0; i < num && pmon_locks[i]; i++)
	{
		if(pmon_locks[i]->cachable){
			kfree(pmon_locks[i]);
		}else{
			dma_free_coherent(NULL, sizeof(struct monitor_lock), pmon_locks[i], 0);
		}
		pmon_locks[i] = NULL;
	}
	return 0;
}

enum case_type {
	STOP,
	CASE_1,
	CASE_2,
	CASE_3,
	CASE_4,
	CASE_5,
	CASE_6,
	CASE_7,
	CASE_8,
	CASE_9,
	CASE_10,
	CASE_11,
	CASE_MAX,
};

static int test_start(int case_num)
{
	struct test_parameter t_param;

	t_param.cachable = 1;
	t_param.same_lock = 0;
	t_param.single_cpu = 0;
	t_param.lock_count = NR_CPUS;
	t_param.repeat_time = NR_CPUS;
	t_param.wait_cpu = NR_CPUS-1;
	t_param.multi_thread = 0;


	switch(case_num) {
	case STOP:
		test_stop();
	case CASE_1:
		/*
		 * cacheable: 4 LDREX on same memory location - same cpu
		 */
		t_param.cachable = 1;
		t_param.same_lock = 1;
		t_param.single_cpu = 1;
		t_param.lock_count = 1;
		t_param.repeat_time = 4;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_2:
		/*
		 * cacheable: NR_CPUS LDREX on different memory location - same cpu
		 */
		t_param.cachable = 1;
		t_param.same_lock = 0;
		t_param.single_cpu = 1;
		t_param.lock_count = NR_CPUS;
		t_param.repeat_time = 1;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_3:
		/*
		 * Uncacheable: 4 LDREX on same memory location - same cpu
		 */
		t_param.cachable = 0;
		t_param.same_lock = 1;
		t_param.single_cpu = 1;
		t_param.lock_count = 1;
		t_param.repeat_time = 4;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_4:
		/*
		 * Uncacheable: 1 LDREX on different memory location - same cpu
		 */
		t_param.cachable = 0;
		t_param.same_lock = 0;
		t_param.single_cpu = 1;
		t_param.lock_count = NR_CPUS;
		t_param.repeat_time = 1;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_5:
		/*
		 * cacheable:  4 LDREX on same memory location for all present cores
		 */
		t_param.cachable = 1;
		t_param.same_lock = 1;
		t_param.single_cpu = 0;
		t_param.lock_count = 1;
		t_param.repeat_time = 4;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_6:
		/*
		 * cacheable: 1 LDREX on different memory location for all present cores
		 */
		t_param.cachable = 1;
		t_param.same_lock = 0;
		t_param.single_cpu = 0;
		t_param.lock_count = NR_CPUS;
		t_param.repeat_time = 1;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_7:
		/*
		 * Uncacheable:  2 LDREX on same memory location for all present cores
		 */
		t_param.cachable = 0;
		t_param.same_lock = 1;
		t_param.single_cpu = 0;
		t_param.lock_count = 1;
		t_param.repeat_time = 2;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
		break;
	case CASE_8:
		/*
		 * Uncacheable: 1 LDREX on different memory location for all present cores
		 */
		t_param.cachable = 0;
		t_param.same_lock = 0;
		t_param.single_cpu = 0;
		t_param.lock_count = NR_CPUS;
		t_param.repeat_time = 1;
		t_param.wait_cpu = -1;
		t_param.multi_thread = 0;
	case CASE_9:
		/*
		 * Uncacheable: 1 LDREX on same memory location for all present cores,
		 * except waiting thread for cpu NR_CPUS -1
		 */
		t_param.cachable = 0;
		t_param.same_lock = 1;
		t_param.single_cpu = 0;
		t_param.lock_count = 1;
		t_param.repeat_time = 1;
		t_param.wait_cpu = NR_CPUS-1;
		t_param.multi_thread = 0;
		break;
	case CASE_10:
		/*
		 * cacheable: 1 LDREX on different memory location for all present cores
		 * except waiting thread for cpu NR_CPUS -1
		 */
		t_param.cachable = 1;
		t_param.same_lock = 0;
		t_param.single_cpu = 0;
		t_param.lock_count = NR_CPUS;
		t_param.repeat_time = 1;
		t_param.wait_cpu = NR_CPUS-1;
		t_param.multi_thread = 0;
		break;
	case CASE_11:
		/*
		 * cacheable: LDREX and STREX for all present cores with multiple threads
		 */
                t_param.cachable = 1;
                t_param.same_lock = 0;
                t_param.single_cpu = 0;
                t_param.lock_count = 0;
                t_param.repeat_time = 0;
                t_param.wait_cpu = -1;
		t_param.multi_thread = 1;
		break;
	default:
		pr_err("wrong parameter for start case: %d\n", case_num);
	}

	return run_test(t_param);

}

static int set_test_case(void *data, u64 val)
{
	start = val;
	if (val > 0) {
		test_stop();
		return test_start(val);
	} else if (val == 0) {
		test_stop();
	}
	return 0;
}

static int get_test_case(void *data, u64 *val)
{
	*val = start;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(test_case_ops,
	get_test_case, set_test_case, "%llu\n");

/*
 * Create a simple debugfs with the name of "ldrex_test",
 *
 * As this is a simple directory, no uevent will be sent to
 * userspace.
 *
 * @return -1: fail ; 0 : success
 */
static int create_test_debugfs(void)
{

	struct dentry *dfile_case;

	dent = debugfs_create_dir(MODULE_NAME, 0);
	if (IS_ERR(dent))
		return -1;

	dfile_case = debugfs_create_file("test_case", 0666, dent, 0, &test_case_ops);
	if (!dfile_case || IS_ERR(dfile_case)){
		debugfs_remove_recursive(dent);
		return -1;
	}
	ldrex_dev = NULL;

	return 0;

}

static int ldrex_test_probe(struct platform_device *pdev)
{
	int create_result;
	create_result = create_test_debugfs();
	if (create_result != 0){
		pr_err("Fail to create test debugfs \n");
	}

	ldrex_dev = &pdev->dev;

	return 0;
}

static int ldrex_test_remove(struct platform_device *pdev)
{
	test_stop();
	if (dent && !IS_ERR(dent)) {
		debugfs_remove_recursive(dent);
		dent = NULL;
	}
	return 0;
}

static struct platform_driver ldrex_test_driver = {
	.probe = ldrex_test_probe,
	.remove = ldrex_test_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static void platform_ldrex_release(struct device* dev)
{
	return;
}

static struct platform_device ldrex_test_device = {
	.name = MODULE_NAME,
	.id   = -1,
	.dev  = {
		.release = platform_ldrex_release,
	}
};

static int init_ldrex_test(void)
{
	platform_device_register(&ldrex_test_device);
	return platform_driver_register(&ldrex_test_driver);
}
module_init(init_ldrex_test);

static void exit_ldrex_test(void)
{
	platform_driver_unregister(&ldrex_test_driver);
	platform_device_unregister(&ldrex_test_device);
}
module_exit(exit_ldrex_test);

MODULE_DESCRIPTION("LDREX test");
MODULE_LICENSE("GPL v2");
