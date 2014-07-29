/* Copyright (c) 2012-2014, Linux Foundation. All rights reserved.

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
#include <linux/preempt.h>
#include <linux/irq.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/slab.h>
#include <asm/barrier.h>
#include <asm/io.h>
#include <asm-generic/sizes.h>
#include <soc/qcom/scm.h>
#include <linux/of_platform.h>

#define REG_MPM2_8916_WDOG_BASE		0x4AA000
#define REG_MPM2_WDOG_BASE		0xFC4AA000

#define REG_OFFSET_MPM2_WDOG_RESET	0x0
#define REG_OFFSET_MPM2_WDOG_BITE_VAL	0x10

#define REG_VAL_WDOG_RESET_DO_RESET	0x1
#define REG_VAL_WDOG_BITE_VAL		0x400

#define SCM_SVC_SEC_WDOG_TRIG	0x8
#define SCM_SVC_SPIN_CPU	0xD

#define MDELAY_TIME		15000

static int apps_wdog_bite;
static int sec_wdog_bite;
static int sec_wdog_scm;
static int apps_wdog_bark;
static int cpu_cntxt_test;

static int apps_wdog_bite_set(const char *val, struct kernel_param *kp);
module_param_call(apps_wdog_bite, apps_wdog_bite_set, param_get_int,
						&apps_wdog_bite, 0644);

static int sec_wdog_bite_set(const char *val, struct kernel_param *kp);
module_param_call(sec_wdog_bite, sec_wdog_bite_set, param_get_int,
						&sec_wdog_bite, 0644);

static int sec_wdog_scm_set(const char *val, struct kernel_param *kp);
module_param_call(sec_wdog_scm, sec_wdog_scm_set, param_get_int,
						&sec_wdog_scm, 0644);

static int apps_wdog_bark_set(const char *val, struct kernel_param *kp);
module_param_call(apps_wdog_bark, apps_wdog_bark_set, param_get_int,
						&apps_wdog_bark, 0644);

static int apps_wdog_bark_cntxt_set(const char *val, struct kernel_param *kp);
module_param_call(cpu_cntxt_test, apps_wdog_bark_cntxt_set, param_get_int,
						&cpu_cntxt_test, 0644);

struct completion timeout_complete;
static atomic_t cause_bark;

static void keep_looping(int arg)
{
	int cpu = arg;
	pr_info("Executing on cpu %d\n",cpu);
	atomic_inc(&cause_bark);
	while(1);
}

/* Force the secure lockup of the CPU on which this task runs
 */
static void keep_looping_2(struct work_struct *work)
{
	int ret = 0;
	u32 argument = 0;
	pr_info("Forcing secure lockup of cpu 1 using SCM call\n");
	atomic_inc(&cause_bark);
	ret = scm_call(SCM_SVC_BOOT, SCM_SVC_SPIN_CPU, &argument,
					sizeof(argument), NULL, 0);
	pr_err("scm call failed!\n");
	while(1);
}

/* Disable premption on the CPUs this task runs so that we always
 * know the HLOS context for the current CPU
 */
static void cpu_cntx_work(struct work_struct *work)
{
	keep_looping(get_cpu());
}

static void apps_wdog_bark_cntxt_work(struct work_struct *work)
{
	int cpu = 0, i;
	struct work_struct **workq;
	int online_cpus = num_online_cpus();
	if (online_cpus != num_present_cpus()) {
		pr_err("Stop mpdecision and enable all CPUs\n");
		goto out;
	}
	workq = kmalloc(sizeof(struct work_struct *) * (online_cpus - 1),
			GFP_KERNEL);

	if (!workq)
		goto out;

	for_each_online_cpu(cpu) {
		if (cpu == 0)
			continue;
		workq[cpu - 1] = kmalloc(sizeof(struct work_struct),
					 GFP_KERNEL);
		if (!workq[cpu - 1]) {
			for(i = 1; i < cpu; i++)
				kfree(workq[i - 1]);
			kfree(workq);
			goto out;
		}
		if (cpu == 1)
			INIT_WORK(workq[cpu - 1], keep_looping_2);
		else
			INIT_WORK(workq[cpu - 1], cpu_cntx_work);
		schedule_work_on(cpu, workq[cpu - 1]);
	}
	while(atomic_read(&cause_bark) != online_cpus - 1)
		mdelay(500);
	pr_info("Apps watchdog bark for cpu context save test\n");
	preempt_disable();
	mdelay(MDELAY_TIME);
	preempt_enable();
out:
	complete(&timeout_complete);
}

static void timeout_work(struct work_struct *work)
{
	pr_info("apps watchdog bark\n");
	preempt_disable();
	mdelay(MDELAY_TIME);
	preempt_enable();
	complete(&timeout_complete);
}

#ifdef CONFIG_HOTPLUG_CPU
static void bring_other_cpus_down(void)
{
	int cpu;

	for_each_online_cpu(cpu) {
		if (cpu == 0)
			continue;
		cpu_down(cpu);
	}
}
#else
static void bring_other_cpus_down(void)
{
}
#endif

static void apps_bite_work(struct work_struct *work)
{
	bring_other_cpus_down();
	pr_info("apps watchdog bite\n");
	local_irq_disable();
	mdelay(MDELAY_TIME);
	local_irq_enable();
	pr_err("apps watchdog bite failed\n");
	complete(&timeout_complete);
}

static void sec_wdog_bite_work(struct work_struct *work)
{
	static void *sec_wdog_virt;
	int rc;

	bring_other_cpus_down();

	rc = of_machine_is_compatible("qcom,msm8916") ||
				of_machine_is_compatible("qcom,msm8939");
	if (rc)
		sec_wdog_virt = ioremap(REG_MPM2_8916_WDOG_BASE, SZ_4K);
	else
		sec_wdog_virt = ioremap(REG_MPM2_WDOG_BASE, SZ_4K);

	if (!sec_wdog_virt) {
		pr_info("unable to map sec wdog page\n");
		goto err;
	}
	writel_relaxed(REG_VAL_WDOG_RESET_DO_RESET,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_RESET);
	writel_relaxed(REG_VAL_WDOG_BITE_VAL,
		sec_wdog_virt + REG_OFFSET_MPM2_WDOG_BITE_VAL);
	mb();
	mdelay(MDELAY_TIME);
err:
	complete(&timeout_complete);
}

/*
 * If this returns operation failed
 */
static DECLARE_WORK(apps_bite_work_struct, apps_bite_work);
static int apps_wdog_bite_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val;

	old_val = apps_wdog_bite;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (apps_wdog_bite == 1) {
		init_completion(&timeout_complete);
		schedule_work_on(0, &apps_bite_work_struct);
		wait_for_completion(&timeout_complete);
	}
	return -EIO;
}

/*
 * If this returns operation failed
 */
static DECLARE_WORK(sec_wdog_bite_work_struct, sec_wdog_bite_work);
static int sec_wdog_bite_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val;

	old_val = sec_wdog_bite;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (sec_wdog_bite == 1) {
		init_completion(&timeout_complete);
		schedule_work_on(0, &sec_wdog_bite_work_struct);
		wait_for_completion(&timeout_complete);
	}
	return -EIO;
}

/*
 * If this returns operation failed
 */
static int sec_wdog_scm_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val;

	old_val = sec_wdog_scm;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (sec_wdog_scm == 1) {
		u8 trigger = 0;
		pr_info("sec watchdog bite\n");
		ret = scm_call(SCM_SVC_BOOT, SCM_SVC_SEC_WDOG_TRIG, &trigger,
						sizeof(trigger), NULL, 0);
		pr_err("Secure watchdog bite failed\n");
	}
	return ret;
}

/*
 * If this returns operation failed
 */
static DECLARE_WORK(timeout_work_struct, timeout_work);
static int apps_wdog_bark_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val;

	old_val = apps_wdog_bark;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (apps_wdog_bark == 1) {
		init_completion(&timeout_complete);
		schedule_work_on(0, &timeout_work_struct);
		wait_for_completion(&timeout_complete);
		pr_err("Failed to trigger apps bark\n");
	}
	return -EIO;
}

static DECLARE_WORK(apps_wdog_bark_cntxt_work_struct,
		    apps_wdog_bark_cntxt_work);
static int apps_wdog_bark_cntxt_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val;

	old_val = cpu_cntxt_test;
	ret = param_set_int(val, kp);
	if (ret)
		return ret;
	if (cpu_cntxt_test == 1) {
		atomic_set(&cause_bark, 0);
		init_completion(&timeout_complete);
		schedule_work_on(0, &apps_wdog_bark_cntxt_work_struct);
		wait_for_completion(&timeout_complete);
		pr_err("Failed cpu context test\n");
	}
	return -EIO;
}

static int msm_watchdog_test_init(void)
{
	return 0;
}

static void msm_watchdog_test_exit(void)
{
	return;
}

module_init(msm_watchdog_test_init);
module_exit(msm_watchdog_test_exit);
MODULE_LICENSE("GPL v2");
