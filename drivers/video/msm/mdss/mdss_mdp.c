/*
 * MDSS MDP Interface (used by framebuffer core)
 *
 * Copyright (c) 2007-2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/memory_alloc.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include <mach/board.h>
#include <mach/clk.h>
#include <mach/hardware.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/iommu.h>
#include <mach/iommu_domains.h>

#include "mdss.h"
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_debug.h"

struct mdss_data_type *mdss_res;

static DEFINE_SPINLOCK(mdp_lock);
static DEFINE_MUTEX(mdp_clk_lock);

#define MDP_BUS_VECTOR_ENTRY(ab_val, ib_val)		\
	{						\
		.src = MSM_BUS_MASTER_MDP_PORT0,	\
		.dst = MSM_BUS_SLAVE_EBI_CH0,		\
		.ab = (ab_val),				\
		.ib = (ib_val),				\
	}

static struct msm_bus_vectors mdp_bus_vectors[] = {
	MDP_BUS_VECTOR_ENTRY(0, 0),
	MDP_BUS_VECTOR_ENTRY(SZ_128M, SZ_256M),
	MDP_BUS_VECTOR_ENTRY(SZ_256M, SZ_512M),
};
static struct msm_bus_paths mdp_bus_usecases[ARRAY_SIZE(mdp_bus_vectors)];
static struct msm_bus_scale_pdata mdp_bus_scale_table = {
	.usecase = mdp_bus_usecases,
	.num_usecases = ARRAY_SIZE(mdp_bus_usecases),
	.name = "mdss_mdp",
};

struct mdss_iommu_map_type mdss_iommu_map[MDSS_IOMMU_MAX_DOMAIN] = {
	[MDSS_IOMMU_DOMAIN_UNSECURE] = {
		.client_name = "mdp_ns",
		.ctx_name = "mdp_0",
		.partitions = {
			{
				.start = SZ_128K,
				.size = SZ_1G - SZ_128K,
			},
		},
		.npartitions = 1,
	},
	[MDSS_IOMMU_DOMAIN_SECURE] = {
		.client_name = "mdp_secure",
		.ctx_name = "mdp_1",
		.partitions = {
			{
				.start = SZ_1G,
				.size = SZ_1G,
			},
		},
		.npartitions = 1,
	},
};

struct mdss_hw mdss_mdp_hw = {
	.hw_ndx = MDSS_HW_MDP,
	.ptr = NULL,
	.irq_handler = mdss_mdp_isr,
};

static DEFINE_SPINLOCK(mdss_lock);
struct mdss_hw *mdss_irq_handlers[MDSS_MAX_HW_BLK];

static void mdss_mdp_footswitch_ctrl(struct mdss_data_type *mdata, int on);
static int mdss_mdp_parse_dt(struct platform_device *pdev);
static int mdss_mdp_parse_dt_pipe(struct platform_device *pdev);
static int mdss_mdp_parse_dt_mixer(struct platform_device *pdev);
static int mdss_mdp_parse_dt_ctl(struct platform_device *pdev);
static int mdss_mdp_parse_dt_video_intf(struct platform_device *pdev);
static int mdss_mdp_parse_dt_handler(struct platform_device *pdev,
				      char *prop_name, u32 *offsets, int len);
static int mdss_mdp_parse_dt_prop_len(struct platform_device *pdev,
				       char *prop_name);

static inline int mdss_irq_dispatch(u32 hw_ndx, int irq, void *ptr)
{
	struct mdss_hw *hw;

	spin_lock(&mdss_lock);
	hw = mdss_irq_handlers[hw_ndx];
	spin_unlock(&mdss_lock);
	if (hw)
		return hw->irq_handler(irq, hw->ptr);

	return -ENODEV;
}

static irqreturn_t mdss_irq_handler(int irq, void *ptr)
{
	struct mdss_data_type *mdata = ptr;
	u32 intr = MDSS_MDP_REG_READ(MDSS_REG_HW_INTR_STATUS);

	if (!mdata)
		return IRQ_NONE;

	mdata->irq_buzy = true;

	if (intr & MDSS_INTR_MDP)
		mdss_irq_dispatch(MDSS_HW_MDP, irq, ptr);

	if (intr & MDSS_INTR_DSI0)
		mdss_irq_dispatch(MDSS_HW_DSI0, irq, ptr);

	if (intr & MDSS_INTR_DSI1)
		mdss_irq_dispatch(MDSS_HW_DSI1, irq, ptr);

	if (intr & MDSS_INTR_EDP)
		mdss_irq_dispatch(MDSS_HW_EDP, irq, ptr);

	if (intr & MDSS_INTR_HDMI)
		mdss_irq_dispatch(MDSS_HW_HDMI, irq, ptr);

	mdata->irq_buzy = false;

	return IRQ_HANDLED;
}


void mdss_enable_irq(struct mdss_hw *hw)
{
	unsigned long irq_flags;
	u32 ndx_bit;

	if (hw->hw_ndx >= MDSS_MAX_HW_BLK)
		return;

	ndx_bit = BIT(hw->hw_ndx);

	pr_debug("Enable HW=%d irq ena=%d mask=%x\n", hw->hw_ndx,
			mdss_res->irq_ena, mdss_res->irq_mask);

	spin_lock_irqsave(&mdss_lock, irq_flags);
	if (mdss_res->irq_mask & ndx_bit) {
		pr_debug("MDSS HW ndx=%d is already set, mask=%x\n",
				hw->hw_ndx, mdss_res->irq_mask);
	} else {
		mdss_irq_handlers[hw->hw_ndx] = hw;
		mdss_res->irq_mask |= ndx_bit;
		if (!mdss_res->irq_ena) {
			mdss_res->irq_ena = true;
			enable_irq(mdss_res->irq);
		}
	}
	spin_unlock_irqrestore(&mdss_lock, irq_flags);
}
EXPORT_SYMBOL(mdss_enable_irq);

void mdss_disable_irq(struct mdss_hw *hw)
{
	unsigned long irq_flags;
	u32 ndx_bit;

	if (hw->hw_ndx >= MDSS_MAX_HW_BLK)
		return;

	ndx_bit = BIT(hw->hw_ndx);

	pr_debug("Disable HW=%d irq ena=%d mask=%x\n", hw->hw_ndx,
			mdss_res->irq_ena, mdss_res->irq_mask);

	spin_lock_irqsave(&mdss_lock, irq_flags);
	if (!(mdss_res->irq_mask & ndx_bit)) {
		pr_warn("MDSS HW ndx=%d is NOT set, mask=%x, hist mask=%x\n",
			hw->hw_ndx, mdss_res->mdp_irq_mask,
			mdss_res->mdp_hist_irq_mask);
	} else {
		mdss_irq_handlers[hw->hw_ndx] = NULL;
		mdss_res->irq_mask &= ~ndx_bit;
		if (mdss_res->irq_mask == 0) {
			mdss_res->irq_ena = false;
			disable_irq_nosync(mdss_res->irq);
		}
	}
	spin_unlock_irqrestore(&mdss_lock, irq_flags);
}
EXPORT_SYMBOL(mdss_disable_irq);

void mdss_disable_irq_nosync(struct mdss_hw *hw)
{
	u32 ndx_bit;

	if (hw->hw_ndx >= MDSS_MAX_HW_BLK)
		return;

	ndx_bit = BIT(hw->hw_ndx);

	pr_debug("Disable HW=%d irq ena=%d mask=%x\n", hw->hw_ndx,
			mdss_res->irq_ena, mdss_res->irq_mask);

	spin_lock(&mdss_lock);
	if (!(mdss_res->irq_mask & ndx_bit)) {
		pr_warn("MDSS HW ndx=%d is NOT set, mask=%x, hist mask=%x\n",
			hw->hw_ndx, mdss_res->mdp_irq_mask,
			mdss_res->mdp_hist_irq_mask);
	} else {
		mdss_irq_handlers[hw->hw_ndx] = NULL;
		mdss_res->irq_mask &= ~ndx_bit;
		if (mdss_res->irq_mask == 0) {
			mdss_res->irq_ena = false;
			disable_irq_nosync(mdss_res->irq);
		}
	}
	spin_unlock(&mdss_lock);
}
EXPORT_SYMBOL(mdss_disable_irq_nosync);

static int mdss_mdp_bus_scale_register(struct mdss_data_type *mdata)
{
	if (!mdata->bus_hdl) {
		struct msm_bus_scale_pdata *bus_pdata = &mdp_bus_scale_table;
		int i;

		for (i = 0; i < bus_pdata->num_usecases; i++) {
			mdp_bus_usecases[i].num_paths = 1;
			mdp_bus_usecases[i].vectors = &mdp_bus_vectors[i];
		}

		mdata->bus_hdl = msm_bus_scale_register_client(bus_pdata);
		if (!mdata->bus_hdl) {
			pr_err("not able to get bus scale\n");
			return -ENOMEM;
		}

		pr_debug("register bus_hdl=%x\n", mdata->bus_hdl);
	}
	return 0;
}

static void mdss_mdp_bus_scale_unregister(struct mdss_data_type *mdata)
{
	pr_debug("unregister bus_hdl=%x\n", mdata->bus_hdl);

	if (mdata->bus_hdl)
		msm_bus_scale_unregister_client(mdata->bus_hdl);
}

int mdss_mdp_bus_scale_set_quota(u64 ab_quota, u64 ib_quota)
{
	static int current_bus_idx;
	int bus_idx;

	if (mdss_res->bus_hdl < 1) {
		pr_err("invalid bus handle %d\n", mdss_res->bus_hdl);
		return -EINVAL;
	}

	if ((ab_quota | ib_quota) == 0) {
		bus_idx = 0;
	} else {
		int num_cases = mdp_bus_scale_table.num_usecases;
		struct msm_bus_vectors *vect = NULL;

		bus_idx = (current_bus_idx % (num_cases - 1)) + 1;

		/* aligning to avoid performing updates for small changes */
		ab_quota = ALIGN(ab_quota, SZ_64M);
		ib_quota = ALIGN(ib_quota, SZ_64M);

		vect = mdp_bus_scale_table.usecase[current_bus_idx].vectors;
		if ((ab_quota == vect->ab) && (ib_quota == vect->ib)) {
			pr_debug("skip bus scaling, no change in vectors\n");
			return 0;
		}

		vect = mdp_bus_scale_table.usecase[bus_idx].vectors;
		vect->ab = ab_quota;
		vect->ib = ib_quota;

		pr_debug("bus scale idx=%d ab=%llu ib=%llu\n", bus_idx,
				vect->ab, vect->ib);
	}
	current_bus_idx = bus_idx;
	return msm_bus_scale_client_update_request(mdss_res->bus_hdl, bus_idx);
}

static inline u32 mdss_mdp_irq_mask(u32 intr_type, u32 intf_num)
{
	if (intr_type == MDSS_MDP_IRQ_INTF_UNDER_RUN ||
	    intr_type == MDSS_MDP_IRQ_INTF_VSYNC)
		intf_num = (intf_num - MDSS_MDP_INTF0) * 2;
	return 1 << (intr_type + intf_num);
}

int mdss_mdp_irq_enable(u32 intr_type, u32 intf_num)
{
	u32 irq;
	unsigned long irq_flags;
	int ret = 0;

	irq = mdss_mdp_irq_mask(intr_type, intf_num);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (mdss_res->mdp_irq_mask & irq) {
		pr_warn("MDSS MDP IRQ-0x%x is already set, mask=%x\n",
				irq, mdss_res->mdp_irq_mask);
		ret = -EBUSY;
	} else {
		pr_debug("MDP IRQ mask old=%x new=%x\n",
				mdss_res->mdp_irq_mask, irq);
		mdss_res->mdp_irq_mask |= irq;
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_INTR_CLEAR, irq);
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_INTR_EN,
				mdss_res->mdp_irq_mask);
		mdss_enable_irq(&mdss_mdp_hw);
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);

	return ret;
}
int mdss_mdp_hist_irq_enable(u32 irq)
{
	unsigned long irq_flags;
	int ret = 0;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (mdss_res->mdp_hist_irq_mask & irq) {
		pr_warn("MDSS MDP Hist IRQ-0x%x is already set, mask=%x\n",
				irq, mdss_res->mdp_hist_irq_mask);
		ret = -EBUSY;
	} else {
		pr_debug("MDP IRQ mask old=%x new=%x\n",
				mdss_res->mdp_hist_irq_mask, irq);
		mdss_res->mdp_hist_irq_mask |= irq;
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_HIST_INTR_CLEAR, irq);
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_HIST_INTR_EN,
				mdss_res->mdp_hist_irq_mask);
		mdss_enable_irq(&mdss_mdp_hw);
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);

	return ret;
}

void mdss_mdp_irq_disable(u32 intr_type, u32 intf_num)
{
	u32 irq;
	unsigned long irq_flags;

	irq = mdss_mdp_irq_mask(intr_type, intf_num);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (!(mdss_res->mdp_irq_mask & irq)) {
		pr_warn("MDSS MDP IRQ-%x is NOT set, mask=%x\n",
				irq, mdss_res->mdp_irq_mask);
	} else {
		mdss_res->mdp_irq_mask &= ~irq;

		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_INTR_EN,
				mdss_res->mdp_irq_mask);
		if ((mdss_res->mdp_irq_mask == 0) &&
			(mdss_res->mdp_hist_irq_mask == 0))
			mdss_disable_irq(&mdss_mdp_hw);
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
}

void mdss_mdp_hist_irq_disable(u32 irq)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (!(mdss_res->mdp_hist_irq_mask & irq)) {
		pr_warn("MDSS MDP IRQ-%x is NOT set, mask=%x\n",
				irq, mdss_res->mdp_hist_irq_mask);
	} else {
		mdss_res->mdp_hist_irq_mask &= ~irq;
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_HIST_INTR_EN,
				mdss_res->mdp_hist_irq_mask);
		if ((mdss_res->mdp_irq_mask == 0) &&
			(mdss_res->mdp_hist_irq_mask == 0))
			mdss_disable_irq(&mdss_mdp_hw);
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
}

void mdss_mdp_irq_disable_nosync(u32 intr_type, u32 intf_num)
{
	u32 irq;

	irq = mdss_mdp_irq_mask(intr_type, intf_num);

	spin_lock(&mdp_lock);
	if (!(mdss_res->mdp_irq_mask & irq)) {
		pr_warn("MDSS MDP IRQ-%x is NOT set, mask=%x\n",
				irq, mdss_res->mdp_irq_mask);
	} else {
		mdss_res->mdp_irq_mask &= ~irq;
		MDSS_MDP_REG_WRITE(MDSS_MDP_REG_INTR_EN,
				mdss_res->mdp_irq_mask);
		if ((mdss_res->mdp_irq_mask == 0) &&
			(mdss_res->mdp_hist_irq_mask == 0))
			mdss_disable_irq_nosync(&mdss_mdp_hw);
	}
	spin_unlock(&mdp_lock);
}

static inline struct clk *mdss_mdp_get_clk(u32 clk_idx)
{
	if (clk_idx < MDSS_MAX_CLK)
		return mdss_res->mdp_clk[clk_idx];
	return NULL;
}

static int mdss_mdp_clk_update(u32 clk_idx, u32 enable)
{
	int ret = -ENODEV;
	struct clk *clk = mdss_mdp_get_clk(clk_idx);

	if (clk) {
		pr_debug("clk=%d en=%d\n", clk_idx, enable);
		if (enable) {
			ret = clk_prepare_enable(clk);
		} else {
			clk_disable_unprepare(clk);
			ret = 0;
		}
	}
	return ret;
}

int mdss_mdp_vsync_clk_enable(int enable)
{
	int ret = 0;
	pr_debug("clk enable=%d\n", enable);
	mutex_lock(&mdp_clk_lock);
	if (mdss_res->vsync_ena != enable) {
		mdss_res->vsync_ena = enable;
		ret = mdss_mdp_clk_update(MDSS_CLK_MDP_VSYNC, enable);
	}
	mutex_unlock(&mdp_clk_lock);
	return ret;
}

void mdss_mdp_set_clk_rate(unsigned long min_clk_rate)
{
	unsigned long clk_rate;
	struct clk *clk = mdss_mdp_get_clk(MDSS_CLK_MDP_SRC);
	if (clk) {
		mutex_lock(&mdp_clk_lock);
		clk_rate = clk_round_rate(clk, min_clk_rate);
		if (IS_ERR_VALUE(clk_rate)) {
			pr_err("unable to round rate err=%ld\n", clk_rate);
		} else if (clk_rate != clk_get_rate(clk)) {
			if (IS_ERR_VALUE(clk_set_rate(clk, clk_rate)))
				pr_err("clk_set_rate failed\n");
			else
				pr_debug("mdp clk rate=%lu\n", clk_rate);
		}
		mutex_unlock(&mdp_clk_lock);
	} else {
		pr_err("mdp src clk not setup properly\n");
	}
}

unsigned long mdss_mdp_get_clk_rate(u32 clk_idx)
{
	unsigned long clk_rate = 0;
	struct clk *clk = mdss_mdp_get_clk(clk_idx);
	mutex_lock(&mdp_clk_lock);
	if (clk)
		clk_rate = clk_get_rate(clk);
	mutex_unlock(&mdp_clk_lock);

	return clk_rate;
}

static void mdss_mdp_clk_ctrl_update(struct mdss_data_type *mdata)
{
	int enable;

	mutex_lock(&mdp_clk_lock);
	enable = atomic_read(&mdata->clk_ref) > 0;
	if (mdata->clk_ena == enable) {
		mutex_unlock(&mdp_clk_lock);
		return;
	}
	mdata->clk_ena = enable;

	pr_debug("MDP CLKS %s\n", (enable ? "Enable" : "Disable"));
	mb();

	mdss_mdp_clk_update(MDSS_CLK_AHB, enable);
	mdss_mdp_clk_update(MDSS_CLK_AXI, enable);

	mdss_mdp_clk_update(MDSS_CLK_MDP_CORE, enable);
	mdss_mdp_clk_update(MDSS_CLK_MDP_LUT, enable);
	if (mdata->vsync_ena)
		mdss_mdp_clk_update(MDSS_CLK_MDP_VSYNC, enable);

	mutex_unlock(&mdp_clk_lock);
}

static void mdss_mdp_clk_ctrl_workqueue_handler(struct work_struct *work)
{
	struct mdss_data_type *mdata;

	mdata = container_of(work, struct mdss_data_type, clk_ctrl_worker);
	mdss_mdp_clk_ctrl_update(mdata);
}

void mdss_mdp_clk_ctrl(int enable, int isr)
{
	struct mdss_data_type *mdata = mdss_res;

	pr_debug("clk enable=%d isr=%d ref= %d\n", enable, isr,
			atomic_read(&mdata->clk_ref));

	if (enable == MDP_BLOCK_POWER_ON) {
		BUG_ON(isr);

		if (atomic_inc_return(&mdata->clk_ref) == 1)
			mdss_mdp_clk_ctrl_update(mdata);
	} else {
		BUG_ON(atomic_read(&mdata->clk_ref) == 0);

		if (atomic_dec_and_test(&mdata->clk_ref)) {
			if (isr)
				queue_work(mdata->clk_ctrl_wq,
						&mdata->clk_ctrl_worker);
			else
				mdss_mdp_clk_ctrl_update(mdata);
		}
	}


}

static inline int mdss_mdp_irq_clk_register(struct mdss_data_type *mdata,
					    char *clk_name, int clk_idx)
{
	struct clk *tmp;
	if (clk_idx >= MDSS_MAX_CLK) {
		pr_err("invalid clk index %d\n", clk_idx);
		return -EINVAL;
	}

	tmp = devm_clk_get(&mdata->pdev->dev, clk_name);
	if (IS_ERR(tmp)) {
		pr_err("unable to get clk: %s\n", clk_name);
		return PTR_ERR(tmp);
	}

	mdata->mdp_clk[clk_idx] = tmp;
	return 0;
}

static int mdss_mdp_irq_clk_setup(struct mdss_data_type *mdata)
{
	int ret;

	ret = devm_request_irq(&mdata->pdev->dev, mdata->irq, mdss_irq_handler,
			 IRQF_DISABLED,	"MDSS", mdata);
	if (ret) {
		pr_err("mdp request_irq() failed!\n");
		return ret;
	}
	disable_irq(mdata->irq);

	mdata->fs = devm_regulator_get(&mdata->pdev->dev, "vdd");
	if (IS_ERR_OR_NULL(mdata->fs)) {
		mdata->fs = NULL;
		pr_err("unable to get gdsc regulator\n");
		return -EINVAL;
	}
	mdata->fs_ena = false;

	if (mdss_mdp_irq_clk_register(mdata, "bus_clk", MDSS_CLK_AXI) ||
	    mdss_mdp_irq_clk_register(mdata, "iface_clk", MDSS_CLK_AHB) ||
	    mdss_mdp_irq_clk_register(mdata, "core_clk_src",
				      MDSS_CLK_MDP_SRC) ||
	    mdss_mdp_irq_clk_register(mdata, "core_clk",
				      MDSS_CLK_MDP_CORE) ||
	    mdss_mdp_irq_clk_register(mdata, "lut_clk", MDSS_CLK_MDP_LUT) ||
	    mdss_mdp_irq_clk_register(mdata, "vsync_clk", MDSS_CLK_MDP_VSYNC))
		return -EINVAL;

	mdss_mdp_set_clk_rate(MDP_CLK_DEFAULT_RATE);
	pr_debug("mdp clk rate=%ld\n", mdss_mdp_get_clk_rate(MDSS_CLK_MDP_SRC));

	return 0;
}

static int mdss_iommu_fault_handler(struct iommu_domain *domain,
		struct device *dev, unsigned long iova, int flags, void *token)
{
	pr_err("MDP IOMMU page fault: iova 0x%lx\n", iova);
	return 0;
}

int mdss_iommu_attach(struct mdss_data_type *mdata)
{
	struct iommu_domain *domain;
	struct mdss_iommu_map_type *iomap;
	int i;

	if (mdata->iommu_attached) {
		pr_warn("mdp iommu already attached\n");
		return 0;
	}

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		iomap = mdata->iommu_map + i;

		domain = msm_get_iommu_domain(iomap->domain_idx);
		if (!domain) {
			WARN(1, "could not attach iommu client %s to ctx %s\n",
				iomap->client_name, iomap->ctx_name);
			continue;
		}
		iommu_attach_device(domain, iomap->ctx);
	}

	mdata->iommu_attached = true;

	return 0;
}

int mdss_iommu_dettach(struct mdss_data_type *mdata)
{
	struct iommu_domain *domain;
	struct mdss_iommu_map_type *iomap;
	int i;

	if (!mdata->iommu_attached) {
		pr_debug("mdp iommu already dettached\n");
		return 0;
	}

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		iomap = mdata->iommu_map + i;

		domain = msm_get_iommu_domain(iomap->domain_idx);
		if (!domain) {
			pr_err("unable to get iommu domain(%d)\n",
				iomap->domain_idx);
			continue;
		}
		iommu_detach_device(domain, iomap->ctx);
	}

	mdata->iommu_attached = false;

	return 0;
}

int mdss_iommu_init(struct mdss_data_type *mdata)
{
	struct msm_iova_layout layout;
	struct iommu_domain *domain;
	struct mdss_iommu_map_type *iomap;
	int i;

	if (mdata->iommu_map) {
		pr_warn("iommu already initialized\n");
		return 0;
	}

	for (i = 0; i < MDSS_IOMMU_MAX_DOMAIN; i++) {
		iomap = &mdss_iommu_map[i];

		layout.client_name = iomap->client_name;
		layout.partitions = iomap->partitions;
		layout.npartitions = iomap->npartitions;
		layout.is_secure = (i == MDSS_IOMMU_DOMAIN_SECURE);

		iomap->domain_idx = msm_register_domain(&layout);
		if (IS_ERR_VALUE(iomap->domain_idx))
			return -EINVAL;

		domain = msm_get_iommu_domain(iomap->domain_idx);
		if (!domain) {
			pr_err("unable to get iommu domain(%d)\n",
				iomap->domain_idx);
			return -EINVAL;
		}
		iommu_set_fault_handler(domain, mdss_iommu_fault_handler, NULL);

		iomap->ctx = msm_iommu_get_ctx(iomap->ctx_name);
		if (!iomap->ctx) {
			pr_warn("unable to get iommu ctx(%s)\n",
				iomap->ctx_name);
			return -EINVAL;
		}
	}

	mdata->iommu_map = mdss_iommu_map;

	return 0;
}

static int mdss_mdp_debug_init(struct mdss_data_type *mdata)
{
	int rc;

	rc = mdss_debugfs_init(mdata);
	if (rc)
		return rc;

	mdss_debug_register_base(NULL, mdata->mdp_base, mdata->mdp_reg_size);

	return 0;
}

static int mdss_hw_init(struct mdss_data_type *mdata)
{
	int i, j;
	char *offset;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	mdata->mdp_rev = MDSS_MDP_REG_READ(MDSS_MDP_REG_HW_VERSION);
	pr_info_once("MDP Rev=%x\n", mdata->mdp_rev);

	if (mdata->hw_settings) {
		struct mdss_hw_settings *hws = mdata->hw_settings;

		while (hws->reg) {
			writel_relaxed(hws->val, hws->reg);
			hws++;
		}
	}

	for (i = 0; i < mdata->nmixers_intf; i++) {
		offset = mdata->mixer_intf[i].dspp_base +
				MDSS_MDP_REG_DSPP_HIST_LUT_BASE;
		for (j = 0; j < ENHIST_LUT_ENTRIES; j++)
			writel_relaxed(j, offset);

		/* swap */
		writel_relaxed(i, offset + 4);
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
	pr_debug("MDP hw init done\n");

	return 0;
}

static u32 mdss_mdp_res_init(struct mdss_data_type *mdata)
{
	u32 rc = 0;

	if (mdata->res_init) {
		pr_err("mdss resources already initialized\n");
		return -EPERM;
	}

	mdata->res_init = true;
	mdata->timeout = HZ/20;
	mdata->clk_ena = false;
	mdata->irq_mask = MDSS_MDP_DEFAULT_INTR_MASK;
	mdata->irq_ena = false;
	mdata->suspend = false;

	rc = mdss_mdp_irq_clk_setup(mdata);
	if (rc)
		return rc;

	mdata->clk_ctrl_wq = create_singlethread_workqueue("mdp_clk_wq");
	INIT_WORK(&mdata->clk_ctrl_worker, mdss_mdp_clk_ctrl_workqueue_handler);

	mdata->smp_mb_cnt = MDSS_MDP_SMP_MMB_BLOCKS;
	mdata->smp_mb_size = MDSS_MDP_SMP_MMB_SIZE;


	mdata->iclient = msm_ion_client_create(-1, mdata->pdev->name);
	if (IS_ERR_OR_NULL(mdata->iclient)) {
		pr_err("msm_ion_client_create() return error (%p)\n",
				mdata->iclient);
		mdata->iclient = NULL;
	}

	rc = mdss_iommu_init(mdata);

	return rc;
}

static int mdss_mdp_probe(struct platform_device *pdev)
{
	struct resource *res;
	int rc;
	struct mdss_data_type *mdata;

	if (!pdev->dev.of_node) {
		pr_err("MDP driver only supports device tree probe\n");
		return -ENOTSUPP;
	}

	if (mdss_res) {
		pr_err("MDP already initialized\n");
		return -EINVAL;
	}

	mdata = devm_kzalloc(&pdev->dev, sizeof(*mdata), GFP_KERNEL);
	if (mdata == NULL)
		return -ENOMEM;

	pdev->id = 0;
	mdata->pdev = pdev;
	platform_set_drvdata(pdev, mdata);
	mdss_res = mdata;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mdp_phys");
	if (!res) {
		pr_err("unable to get MDP base address\n");
		rc = -ENOMEM;
		goto probe_done;
	}

	mdata->mdp_reg_size = resource_size(res);
	mdata->mdp_base = devm_ioremap(&pdev->dev, res->start,
				       mdata->mdp_reg_size);
	if (unlikely(!mdata->mdp_base)) {
		pr_err("unable to map MDP base\n");
		rc = -ENOMEM;
		goto probe_done;
	}
	pr_info("MDP HW Base phy_Address=0x%x virt=0x%x\n",
		(int) res->start,
		(int) mdata->mdp_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vbif_phys");
	if (!res) {
		pr_err("unable to get MDSS VBIF base address\n");
		rc = -ENOMEM;
		goto probe_done;
	}

	mdata->vbif_base = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (unlikely(!mdata->vbif_base)) {
		pr_err("unable to map MDSS VBIF base\n");
		rc = -ENOMEM;
		goto probe_done;
	}
	pr_info("MDSS VBIF HW Base phy_Address=0x%x virt=0x%x\n",
		(int) res->start,
		(int) mdata->vbif_base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		pr_err("unable to get MDSS irq\n");
		rc = -ENOMEM;
		goto probe_done;
	}
	mdata->irq = res->start;

	/*populate hw iomem base info from device tree*/
	rc = mdss_mdp_parse_dt(pdev);
	if (rc) {
		pr_err("unable to parse device tree\n");
		goto probe_done;
	}

	rc = mdss_mdp_res_init(mdata);
	if (rc) {
		pr_err("unable to initialize mdss mdp resources\n");
		goto probe_done;
	}
	rc = mdss_mdp_pp_init(&pdev->dev);
	if (rc) {
		pr_err("unable to initialize mdss pp resources\n");
		goto probe_done;
	}
	rc = mdss_mdp_bus_scale_register(mdata);
	if (rc) {
		pr_err("unable to register bus scaling\n");
		goto probe_done;
	}

	rc = mdss_mdp_debug_init(mdata);
	if (rc) {
		pr_err("unable to initialize mdp debugging\n");
		goto probe_done;
	}

	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		mdss_mdp_footswitch_ctrl(mdata, true);

probe_done:
	if (IS_ERR_VALUE(rc)) {
		mdss_res = NULL;
		mdss_mdp_pp_term(&pdev->dev);
	}

	return rc;
}

static void mdss_mdp_parse_dt_regs_array(const u32 *arr, char __iomem *hw_base,
	struct mdss_hw_settings *hws, int count)
{
	u32 len, reg;
	int i;

	if (!arr)
		return;

	for (i = 0, len = count * 2; i < len; i += 2) {
		reg = be32_to_cpu(arr[i]);
		hws->reg = hw_base + reg;
		hws->val = be32_to_cpu(arr[i + 1]);
		pr_debug("reg: 0x%04x=0x%08x\n", reg, hws->val);
		hws++;
	}
}

int mdss_mdp_parse_dt_hw_settings(struct platform_device *pdev)
{
	struct mdss_data_type *mdata = platform_get_drvdata(pdev);
	struct mdss_hw_settings *hws;
	const u32 *vbif_arr, *mdp_arr;
	int vbif_len, mdp_len;

	vbif_arr = of_get_property(pdev->dev.of_node, "qcom,vbif-settings",
			&vbif_len);
	if (!vbif_arr || (mdp_len & 1)) {
		pr_warn("MDSS VBIF settings not found\n");
		vbif_len = 0;
	}
	vbif_len /= 2 * sizeof(u32);

	mdp_arr = of_get_property(pdev->dev.of_node, "qcom,mdp-settings",
			&mdp_len);
	if (!mdp_arr || (mdp_len & 1)) {
		pr_warn("MDSS MDP settings not found\n");
		mdp_len = 0;
	}
	mdp_len /= 2 * sizeof(u32);

	if ((mdp_len + vbif_len) == 0)
		return 0;

	hws = devm_kzalloc(&pdev->dev, sizeof(*hws) * (vbif_len + mdp_len + 1),
			GFP_KERNEL);
	if (!hws)
		return -ENOMEM;

	mdss_mdp_parse_dt_regs_array(vbif_arr, mdata->vbif_base, hws, vbif_len);
	mdss_mdp_parse_dt_regs_array(mdp_arr, mdata->mdp_base,
		hws + vbif_len, mdp_len);

	mdata->hw_settings = hws;

	return 0;
}

static int mdss_mdp_parse_dt(struct platform_device *pdev)
{
	int rc;

	rc = mdss_mdp_parse_dt_hw_settings(pdev);
	if (rc) {
		pr_err("Error in device tree : hw settings\n");
		return rc;
	}

	rc = mdss_mdp_parse_dt_pipe(pdev);
	if (rc) {
		pr_err("Error in device tree : pipes\n");
		return rc;
	}

	rc = mdss_mdp_parse_dt_mixer(pdev);
	if (rc) {
		pr_err("Error in device tree : mixers\n");
		return rc;
	}

	rc = mdss_mdp_parse_dt_ctl(pdev);
	if (rc) {
		pr_err("Error in device tree : ctl\n");
		return rc;
	}

	rc = mdss_mdp_parse_dt_video_intf(pdev);
	if (rc) {
		pr_err("Error in device tree : ctl\n");
		return rc;
	}

	return 0;
}


static int mdss_mdp_parse_dt_pipe(struct platform_device *pdev)
{
	u32 npipes, off;
	int rc = 0;
	u32 nids = 0;
	u32 *offsets = NULL, *ftch_id = NULL;

	struct mdss_data_type *mdata = platform_get_drvdata(pdev);

	mdata->nvig_pipes = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-pipe-vig-off");
	mdata->nrgb_pipes = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-pipe-rgb-off");
	mdata->ndma_pipes = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-pipe-dma-off");

	nids  += mdss_mdp_parse_dt_prop_len(pdev,
			"qcom,mdss-pipe-vig-fetch-id");
	nids  += mdss_mdp_parse_dt_prop_len(pdev,
			"qcom,mdss-pipe-rgb-fetch-id");
	nids  += mdss_mdp_parse_dt_prop_len(pdev,
			"qcom,mdss-pipe-dma-fetch-id");

	npipes = mdata->nvig_pipes + mdata->nrgb_pipes + mdata->ndma_pipes;

	if (npipes != nids) {
		pr_err("device tree err: unequal number of pipes and smp ids");
		return -EINVAL;
	}

	offsets = kzalloc(sizeof(u32) * npipes, GFP_KERNEL);
	if (!offsets) {
		pr_err("no mem assigned: kzalloc fail\n");
		return -ENOMEM;
	}

	ftch_id = kzalloc(sizeof(u32) * nids, GFP_KERNEL);
	if (!ftch_id) {
		pr_err("no mem assigned: kzalloc fail\n");
		rc = -ENOMEM;
		goto ftch_alloc_fail;
	}

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-vig-fetch-id",
		ftch_id, mdata->nvig_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-vig-off",
		offsets, mdata->nvig_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_pipe_addr_setup(mdata, offsets, ftch_id,
		MDSS_MDP_PIPE_TYPE_VIG, MDSS_MDP_SSPP_VIG0, mdata->nvig_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-rgb-fetch-id",
		ftch_id + mdata->nvig_pipes, mdata->nrgb_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-rgb-off",
		offsets + mdata->nvig_pipes, mdata->nrgb_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_pipe_addr_setup(mdata, offsets + mdata->nvig_pipes,
		ftch_id + mdata->nvig_pipes, MDSS_MDP_PIPE_TYPE_RGB,
		MDSS_MDP_SSPP_RGB0, mdata->nrgb_pipes);
	if (rc)
		goto parse_done;

	off = mdata->nvig_pipes + mdata->nrgb_pipes;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-dma-fetch-id",
		ftch_id + off, mdata->ndma_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-pipe-dma-off",
		offsets + off, mdata->ndma_pipes);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_pipe_addr_setup(mdata, offsets + off, ftch_id + off,
		MDSS_MDP_PIPE_TYPE_DMA, MDSS_MDP_SSPP_DMA0, mdata->ndma_pipes);
	if (rc)
		goto parse_done;

parse_done:
	kfree(ftch_id);
ftch_alloc_fail:
	kfree(offsets);
	return rc;
}

static int mdss_mdp_parse_dt_mixer(struct platform_device *pdev)
{

	u32 nmixers, ndspp;
	int rc = 0;
	u32 *mixer_offsets = NULL, *dspp_offsets = NULL;

	struct mdss_data_type *mdata = platform_get_drvdata(pdev);

	mdata->nmixers_intf = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-mixer-intf-off");
	mdata->nmixers_wb = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-mixer-wb-off");
	ndspp = mdss_mdp_parse_dt_prop_len(pdev,
				"qcom,mdss-dspp-off");
	nmixers = mdata->nmixers_intf + mdata->nmixers_wb;

	if (mdata->nmixers_intf != ndspp) {
		pr_err("device tree err: unequal no of dspp and intf mixers\n");
		return -EINVAL;
	}

	mixer_offsets = kzalloc(sizeof(u32) * nmixers, GFP_KERNEL);
	if (!mixer_offsets) {
		pr_err("no mem assigned: kzalloc fail\n");
		return -ENOMEM;
	}

	dspp_offsets = kzalloc(sizeof(u32) * ndspp, GFP_KERNEL);
	if (!dspp_offsets) {
		pr_err("no mem assigned: kzalloc fail\n");
		rc = -ENOMEM;
		goto dspp_alloc_fail;
	}

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-mixer-intf-off",
		mixer_offsets, mdata->nmixers_intf);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-mixer-wb-off",
		mixer_offsets + mdata->nmixers_intf, mdata->nmixers_wb);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-dspp-off",
		dspp_offsets, ndspp);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_mixer_addr_setup(mdata, mixer_offsets,
			dspp_offsets, MDSS_MDP_MIXER_TYPE_INTF,
			mdata->nmixers_intf);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_mixer_addr_setup(mdata, mixer_offsets +
			mdata->nmixers_intf, NULL,
			MDSS_MDP_MIXER_TYPE_WRITEBACK, mdata->nmixers_wb);
	if (rc)
		goto parse_done;

parse_done:
	kfree(dspp_offsets);
dspp_alloc_fail:
	kfree(mixer_offsets);

	return rc;
}

static int mdss_mdp_parse_dt_ctl(struct platform_device *pdev)
{
	u32 nwb;
	int rc = 0;
	u32 *ctl_offsets = NULL, *wb_offsets = NULL;

	struct mdss_data_type *mdata = platform_get_drvdata(pdev);

	mdata->nctl = mdss_mdp_parse_dt_prop_len(pdev,
			"qcom,mdss-ctl-off");
	nwb =  mdss_mdp_parse_dt_prop_len(pdev,
			"qcom,mdss-wb-off");

	if (mdata->nctl != nwb) {
		pr_err("device tree err: unequal number of ctl and wb\n");
		rc = -EINVAL;
		goto parse_done;
	}

	ctl_offsets = kzalloc(sizeof(u32) * mdata->nctl, GFP_KERNEL);
	if (!ctl_offsets) {
		pr_err("no more mem for ctl offsets\n");
		return -ENOMEM;
	}

	wb_offsets = kzalloc(sizeof(u32) * nwb, GFP_KERNEL);
	if (!wb_offsets) {
		pr_err("no more mem for writeback offsets\n");
		rc = -ENOMEM;
		goto wb_alloc_fail;
	}

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-ctl-off",
		ctl_offsets, mdata->nctl);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-wb-off",
		wb_offsets, nwb);
	if (rc)
		goto parse_done;

	rc = mdss_mdp_ctl_addr_setup(mdata, ctl_offsets, wb_offsets,
						 mdata->nctl);
	if (rc)
		goto parse_done;

parse_done:
	kfree(wb_offsets);
wb_alloc_fail:
	kfree(ctl_offsets);

	return rc;
}

static int mdss_mdp_parse_dt_video_intf(struct platform_device *pdev)
{
	struct mdss_data_type *mdata = platform_get_drvdata(pdev);
	u32 count;
	u32 *offsets;
	int rc;


	count = mdss_mdp_parse_dt_prop_len(pdev, "qcom,mdss-intf-off");
	if (count == 0)
		return -EINVAL;

	offsets = kzalloc(sizeof(u32) * count, GFP_KERNEL);
	if (!offsets) {
		pr_err("no mem assigned for video intf\n");
		return -ENOMEM;
	}

	rc = mdss_mdp_parse_dt_handler(pdev, "qcom,mdss-intf-off",
			offsets, count);
	if (rc)
		goto parse_fail;

	rc = mdss_mdp_video_addr_setup(mdata, offsets, count);
	if (rc)
		pr_err("unable to setup video interfaces\n");

parse_fail:
	kfree(offsets);

	return rc;
}

static int mdss_mdp_parse_dt_handler(struct platform_device *pdev,
		char *prop_name, u32 *offsets, int len)
{
	int rc;
	rc = of_property_read_u32_array(pdev->dev.of_node, prop_name,
					offsets, len);
	if (rc) {
		pr_err("Error from prop %s : u32 array read\n", prop_name);
		return -EINVAL;
	}

	return 0;
}

static int mdss_mdp_parse_dt_prop_len(struct platform_device *pdev,
				      char *prop_name)
{
	int len = 0;

	of_find_property(pdev->dev.of_node, prop_name, &len);

	if (len < 1) {
		pr_err("Error from prop %s : spec error in device tree\n",
		       prop_name);
		return 0;
	}

	len = len/sizeof(u32);

	return len;
}

struct mdss_data_type *mdss_mdp_get_mdata()
{
	return mdss_res;
}

static void mdss_mdp_footswitch_ctrl(struct mdss_data_type *mdata, int on)
{
	int ret;

	if (!mdata->fs)
		return;

	if (on && !mdata->fs_ena) {
		pr_debug("Enable MDP FS\n");
		ret = regulator_enable(mdata->fs);
		if (ret)
			pr_err("Footswitch failed to enable\n");
		mdss_iommu_attach(mdata);
		mdss_hw_init(mdata);
		mdata->fs_ena = true;
	} else if (!on && mdata->fs_ena) {
		pr_debug("Disable MDP FS\n");
		mdss_iommu_dettach(mdata);
		regulator_disable(mdata->fs);
		mdata->fs_ena = false;
	}
}

static inline int mdss_mdp_suspend_sub(struct mdss_data_type *mdata)
{
	int ret;

	ret = mdss_fb_suspend_all();
	if (IS_ERR_VALUE(ret)) {
		pr_err("Unable to suspend all fb panels (%d)\n", ret);
		return ret;
	}

	pr_debug("suspend done\n");

	return 0;
}

static inline int mdss_mdp_resume_sub(struct mdss_data_type *mdata)
{
	int ret = 0;

	ret = mdss_fb_resume_all();
	if (IS_ERR_VALUE(ret))
		pr_err("Unable to resume all fb panels (%d)\n", ret);

	pr_debug("resume done\n");

	return ret;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
static int mdss_mdp_pm_suspend(struct device *dev)
{
	struct mdss_data_type *mdata;

	mdata = dev_get_drvdata(dev);
	if (!mdata)
		return -ENODEV;

	dev_dbg(dev, "display pm suspend\n");

	return mdss_mdp_suspend_sub(mdata);
}

static int mdss_mdp_pm_resume(struct device *dev)
{
	struct mdss_data_type *mdata;

	mdata = dev_get_drvdata(dev);
	if (!mdata)
		return -ENODEV;

	dev_dbg(dev, "display pm resume\n");

	return mdss_mdp_resume_sub(mdata);
}

#define mdss_mdp_suspend NULL
#define mdss_mdp_resume NULL
#else
static int mdss_mdp_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mdss_data_type *mdata = platform_get_drvdata(pdev);

	if (!mdata)
		return -ENODEV;

	dev_dbg(&pdev->dev, "display suspend\n");

	return mdss_mdp_suspend_sub(mdata);
}

static int mdss_mdp_resume(struct platform_device *pdev)
{
	struct mdss_data_type *mdata = platform_get_drvdata(pdev);

	if (!mdata)
		return -ENODEV;

	dev_dbg(&pdev->dev, "display resume\n");

	return mdss_mdp_resume_sub(mdata);
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int mdss_mdp_runtime_resume(struct device *dev)
{
	struct mdss_data_type *mdata = dev_get_drvdata(dev);
	if (!mdata)
		return -ENODEV;

	dev_dbg(dev, "pm_runtime: resuming...\n");

	mdss_mdp_footswitch_ctrl(mdata, true);

	return 0;
}

static int mdss_mdp_runtime_idle(struct device *dev)
{
	struct mdss_data_type *mdata = dev_get_drvdata(dev);
	if (!mdata)
		return -ENODEV;

	dev_dbg(dev, "pm_runtime: idling...\n");

	flush_workqueue(mdata->clk_ctrl_wq);

	return 0;
}

static int mdss_mdp_runtime_suspend(struct device *dev)
{
	struct mdss_data_type *mdata = dev_get_drvdata(dev);
	if (!mdata)
		return -ENODEV;
	dev_dbg(dev, "pm_runtime: suspending...\n");

	if (mdata->clk_ena) {
		pr_err("MDP suspend failed\n");
		return -EBUSY;
	}
	mdss_mdp_footswitch_ctrl(mdata, false);

	return 0;
}
#endif
#endif

static const struct dev_pm_ops mdss_mdp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mdss_mdp_pm_suspend, mdss_mdp_pm_resume)
	SET_RUNTIME_PM_OPS(mdss_mdp_runtime_suspend,
			mdss_mdp_runtime_resume,
			mdss_mdp_runtime_idle)
};

static int mdss_mdp_remove(struct platform_device *pdev)
{
	struct mdss_data_type *mdata = platform_get_drvdata(pdev);
	if (!mdata)
		return -ENODEV;
	pm_runtime_disable(&pdev->dev);
	mdss_mdp_pp_term(&pdev->dev);
	mdss_mdp_bus_scale_unregister(mdata);
	mdss_debugfs_remove(mdata);
	return 0;
}

static const struct of_device_id mdss_mdp_dt_match[] = {
	{ .compatible = "qcom,mdss_mdp",},
	{}
};
MODULE_DEVICE_TABLE(of, mdss_mdp_dt_match);

static struct platform_driver mdss_mdp_driver = {
	.probe = mdss_mdp_probe,
	.remove = mdss_mdp_remove,
	.suspend = mdss_mdp_suspend,
	.resume = mdss_mdp_resume,
	.shutdown = NULL,
	.driver = {
		/*
		 * Driver name must match the device name added in
		 * platform.c.
		 */
		.name = "mdp",
		.of_match_table = mdss_mdp_dt_match,
		.pm = &mdss_mdp_pm_ops,
	},
};

static int mdss_mdp_register_driver(void)
{
	return platform_driver_register(&mdss_mdp_driver);
}

static int __init mdss_mdp_driver_init(void)
{
	int ret;

	ret = mdss_mdp_register_driver();
	if (ret) {
		pr_err("mdp_register_driver() failed!\n");
		return ret;
	}

	return 0;

}

module_init(mdss_mdp_driver_init);
