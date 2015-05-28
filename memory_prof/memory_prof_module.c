/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/iommu.h>
#include <asm/cacheflush.h>
#include <asm-generic/sizes.h>
#include <linux/msm_ion.h>
#include <asm/uaccess.h>

#include "memory_prof_module.h"
#include "timing_debug.h"

#define MEMORY_PROF_DEV_NAME "memory_prof"

struct memory_prof_module_data {
	struct ion_client *client;
	struct list_head danglers;
};

struct dangling_allocation {
	struct list_head list;
	struct page *page;
	int order;
};

static struct class *memory_prof_class;
static int memory_prof_major;
static struct device *memory_prof_dev;

static void zero_the_pages_plz(struct page **pages, int npages,
			unsigned int page_size)
{
	void *ptr;
	int len = npages * page_size;
	ptr = vmap(pages, npages, VM_IOREMAP, pgprot_writecombine(PAGE_KERNEL));
	if (ptr == NULL) {
		WARN(1, "BOGUS vmap ERROR!!!\n");
	} else {
		memset(ptr, 0, len);
		dmac_flush_range(ptr, ptr + len);
		vunmap(ptr);
	}
}

static void test_kernel_alloc(const char *tag, gfp_t thegfp,
			unsigned int page_size,
			void (*transform_page)(struct page *),
			void (*transform_pages)(struct page **, int, unsigned int))
{
	char st1[200];
	char st2[200];
	int i, j;
	int order = get_order(page_size);
	int size = (SZ_1M * 20);
	int npages = PAGE_ALIGN(size) / page_size;
	struct page **pages;

	pages = kmalloc(npages * sizeof(struct page *),
				GFP_KERNEL);
	if(!pages)
		return;

	snprintf(st1, 200, "before %s%s", tag,
		transform_page ? " (flush each)" : "");
	snprintf(st2, 200, "after  %s%s", tag,
		transform_page ? " (flush each)" : "");

	timing_debug_tick(st1);
	for (i = 0; i < npages; ++i) {
		pages[i] = alloc_pages(thegfp, order);
		if (!pages[i]) {
			WARN(1, "BOGUS alloc_pages ERROR!!!\n");
			npages = i;
			goto out;
		}
		if (transform_page)
			for (j = 0; j < order; j++)
				transform_page(nth_page(pages[i], j));
	}
	timing_debug_tock(st2);

	if (transform_pages) {
		timing_debug_tick("before transform_pages");
		transform_pages(pages, npages, page_size);
		timing_debug_tock("after  transform_pages");
	}

out:
	for (i = 0; i < npages; ++i)
		__free_pages(pages[i], order);

	kfree(pages);
}

#define DO_TEST_KERNEL_ALLOC(thegfp, sz, transform_page, transform_pages) \
	test_kernel_alloc(#thegfp " " #sz, thegfp, sz,	\
			transform_page, transform_pages)

static void test_kernel_allocs(void)
{
	int i;
	timing_debug_init();
	timing_debug_start_new_timings();

	pr_err("Testing small pages without flushing individual pages\n");
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO,
				PAGE_SIZE, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM,
				PAGE_SIZE, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM,
				PAGE_SIZE, NULL, zero_the_pages_plz);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_ZERO,
				PAGE_SIZE, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL,
				PAGE_SIZE, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL,
				PAGE_SIZE, NULL, zero_the_pages_plz);

	pr_err("Testing small pages with flushing individual pages\n");
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO,
				PAGE_SIZE, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM,
				PAGE_SIZE, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_HIGHMEM,
				PAGE_SIZE, flush_dcache_page,
				zero_the_pages_plz);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL | __GFP_ZERO,
				PAGE_SIZE, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL,
				PAGE_SIZE, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(GFP_KERNEL,
				PAGE_SIZE, flush_dcache_page,
				zero_the_pages_plz);

	pr_err("Testing with large page sizes without flushing individual pages\n");
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_HIGHMEM | __GFP_COMP | __GFP_ZERO,
			SZ_64K, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_HIGHMEM | __GFP_COMP,
			SZ_64K, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_COMP | __GFP_ZERO,
			SZ_64K, NULL, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_COMP,
			SZ_64K, NULL, NULL);

	pr_err("Testing with large page sizes with flushing individual pages\n");
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_HIGHMEM | __GFP_COMP | __GFP_ZERO,
			SZ_64K, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_HIGHMEM | __GFP_COMP,
			SZ_64K, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_COMP | __GFP_ZERO,
			SZ_64K, flush_dcache_page, NULL);
	for (i = 0; i < 7; ++i)
		DO_TEST_KERNEL_ALLOC(
			GFP_KERNEL | __GFP_COMP,
			SZ_64K, flush_dcache_page, NULL);

	timing_debug_dump_results();
}

static void free_all_danglers(struct list_head *danglers)
{
	struct list_head *iter, *tmp;
	list_for_each_safe(iter, tmp, danglers) {
		struct dangling_allocation *dangler =
			container_of(iter, struct dangling_allocation, list);
		__free_pages(dangler->page, dangler->order);
		list_del(iter);
		kfree(dangler);
	}
}

static gfp_t get_gfp_from_memory_prof_gfp(u64 mp_gfp)
{
	gfp_t gfp = 0;

	if (mp_gfp & MP_GFP_KERNEL)
		gfp |= GFP_KERNEL;
	if (mp_gfp & MP_GFP_HIGHMEM)
		gfp |= __GFP_HIGHMEM;
	if (mp_gfp & MP_GFP_ZERO)
		gfp |= __GFP_ZERO;
	if (mp_gfp & MP_GFP_HIGHUSER)
		gfp |= GFP_HIGHUSER;
	if (mp_gfp & MP_GFP_NOWARN)
		gfp |= __GFP_NOWARN;
	if (mp_gfp & MP_GFP_NORETRY)
		gfp |= __GFP_NORETRY;
	if (mp_gfp & MP_GFP_NO_KSWAPD)
		gfp |= __GFP_NO_KSWAPD;
	if (mp_gfp & MP_GFP_WAIT)
		gfp |= __GFP_WAIT;

	if (mp_gfp & MP_GFPNOT_WAIT)
		gfp &= ~__GFP_WAIT;

	return gfp;
}

static long memory_prof_test_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret = 0;
	struct memory_prof_module_data *module_data = file->private_data;
	switch (cmd) {
	case MEMORY_PROF_IOC_CLIENT_CREATE:
	{
		module_data->client = msm_ion_client_create("memory_prof_module");
		if (IS_ERR_OR_NULL(module_data->client))
			return -EIO;
		break;
	}
	case MEMORY_PROF_IOC_CLIENT_DESTROY:
	{
		ion_client_destroy(module_data->client);
		break;
	}
	case MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS:
	{
		test_kernel_allocs();
		break;
	}
	case MEMORY_PROF_IOC_TEST_ALLOC_PAGES:
	{
		struct dangling_allocation *new;
		struct mp_alloc_pages_args args;
		gfp_t gfp;
		struct timespec before, after, diff;

		if (copy_from_user(&args, (void __user *)arg,
					sizeof(struct mp_alloc_pages_args)))
			return -EFAULT;

		new = (struct dangling_allocation *) kmalloc(sizeof(*new),
							GFP_KERNEL);
		if (!new)
			return -ENOMEM;

		gfp = get_gfp_from_memory_prof_gfp(args.gfp);
		new->order = args.order;
		getnstimeofday(&before);
		new->page = alloc_pages(gfp, new->order);
		getnstimeofday(&after);

		if (!new->page) {
			kfree(new);
			return -ENOMEM;
		}

		diff = timespec_sub(after, before);
		args.time_elapsed_us = div_s64(timespec_to_ns(&diff), 1000);

		if (copy_to_user((void __user *)arg, &args,
					sizeof(struct mp_alloc_pages_args))) {
			kfree(new);
			__free_pages(new->page, args.order);
			return -EFAULT;
		}

		list_add(&new->list, &module_data->danglers);
		break;
	}
	case MEMORY_PROF_IOC_CLEANUP_ALLOC_PAGES:
	{
		free_all_danglers(&module_data->danglers);
		break;
	}
	case MEMORY_PROF_IOC_IOMMU_MAP_RANGE_TEST:
	case MEMORY_PROF_IOC_IOMMU_UNMAP_RANGE_TEST:
	case MEMORY_PROF_IOC_IOMMU_MAP_TEST:
	case MEMORY_PROF_IOC_IOMMU_UNMAP_TEST:
	{
		struct mp_iommu_map_test_args args;

		if (copy_from_user(
				&args, (void __user *)arg,
				sizeof(struct mp_iommu_map_test_args)))
			return -EFAULT;
		break;
	}
	case MEMORY_PROF_IOC_IOMMU_ATTACH_TEST:
	case MEMORY_PROF_IOC_IOMMU_DETACH_TEST:
	{
		struct mp_iommu_attach_test_args args;

		if (copy_from_user(
				&args, (void __user *)arg,
				sizeof(struct mp_iommu_attach_test_args)))
			return -EFAULT;

		break;
	}
	default:
		pr_info("command not supproted\n");
		ret = -EINVAL;
	}
	return ret;
}

static int memory_prof_test_open(struct inode *inode, struct file *file)
{
	struct memory_prof_module_data *module_data = kzalloc(
		sizeof(struct memory_prof_module_data), GFP_KERNEL);

	if (!module_data)
		return -ENOMEM;

	INIT_LIST_HEAD(&module_data->danglers);
	file->private_data = module_data;

	pr_debug("memory_prof test device opened\n");
	return 0;
}

static int memory_prof_test_release(struct inode *inode, struct file *file)
{
	struct memory_prof_module_data *module_data = file->private_data;

	free_all_danglers(&module_data->danglers);

	kfree(module_data);
	pr_debug("memory_prof test device closed\n");
	return 0;
}

static const struct file_operations memory_prof_test_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = memory_prof_test_ioctl,
	.open = memory_prof_test_open,
	.release = memory_prof_test_release,
};

static int memory_prof_device_create(void)
{
	int rc = 0;

	memory_prof_major = register_chrdev(0, MEMORY_PROF_DEV_NAME,
					&memory_prof_test_fops);
	if (memory_prof_major < 0) {
		rc = memory_prof_major;
		pr_err("Unable to register chrdev: %d\n", memory_prof_major);
		goto out;
	}

	memory_prof_class = class_create(THIS_MODULE, MEMORY_PROF_DEV_NAME);
	if (IS_ERR(memory_prof_class)) {
		rc = PTR_ERR(memory_prof_class);
		pr_err("Unable to create class: %d\n", rc);
		goto err_create_class;
	}

	memory_prof_dev = device_create(memory_prof_class, NULL,
					MKDEV(memory_prof_major, 0),
					NULL, MEMORY_PROF_DEV_NAME);
	if (IS_ERR(memory_prof_dev)) {
		rc = PTR_ERR(memory_prof_dev);
		pr_err("Unable to create device: %d\n", rc);
		goto err_create_device;
	}
	return rc;

err_create_device:
	class_destroy(memory_prof_class);
err_create_class:
	unregister_chrdev(memory_prof_major, MEMORY_PROF_DEV_NAME);
out:
	return rc;
}

static void memory_prof_device_destroy(void)
{
	device_destroy(memory_prof_class, MKDEV(memory_prof_major, 0));
	class_destroy(memory_prof_class);
	unregister_chrdev(memory_prof_major, MEMORY_PROF_DEV_NAME);
}

static int memory_prof_test_init(void)
{
	return memory_prof_device_create();
}

static void memory_prof_test_exit(void)
{
	return memory_prof_device_destroy();
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Profile memory stuff");
module_init(memory_prof_test_init);
module_exit(memory_prof_test_exit);
