/* Copyright (c) 2012-2014, Linux Foundation. All rights reserved.
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

/*
 * page-nom is a memory eater.  It can be used to finely allocate large amounts
 * of system or vmalloc memory in order to better test applications in low
 * memory situations.
 *
 * Memory is allocated in individual blocks of a given size which can be tuned.
 * By allocating in chunks it makes it more likely that we can allocate as close
 * to the full request as possible and also makes it easy to dynamically add and
 * remove blocks without having to free and re-initalize (with all the panic
 * that might entail on a system wide basis).  New memory is allocated by
 * specifying how many chunks one wants allocated.  This all happens via
 * debugfs:
 *
 * /sys/kernel/debug/page-nom/vmalloc_block_size - size in bytes of the
 * individual blocks of memory allocated from vmalloc.  The input size is
 * page aligned.  The block size can only be changed when page-nom hasn't eaten
 * any vmalloc pages yet.
 *
 * /sys/kernel/debug/page-nom/vmalloc_blocks - number of 'vmalloc_block_size'
 * blocks to eat from vmalloc.  Inputing a number higher than the current value
 * will allocate new memory and inputing a number lower than the current value
 * will free memory.  0 will free all eaten vmalloc memory.  The returned value
 * will be the number of allocated blocks regardless if this method or the
 * following method was used to allocate the blocks.
 *
 * /sys/kernel/ debug/page-nom/vmalloc_percent - percentage of the vmalloc
 * memory to eat.  This is an alternate method to 'vmalloc_blocks'.  Where
 * vmalloc_blocks is more precise, this is easier for automation. Just echo a
 * number between 0 and 100 for page-nom to try to consume that percentage of
 * total vmalloc memory.  The returned value from this file is the current
 * percentage of allocated vmalloc memory from either method.  This number will
 * be appropriately massaged so it ends up being a multiple of
 * vmalloc_block_size, so the number written won't always be the precise
 * percentage eaten.
 *
 * /sys/kernel/debug/page-nom/mem_block_size - size in bytes of the
 * individual blocks of memory allocated from memory.  The input size is
 * order aligned.  The block size can only be changed when page-nom hasn't eaten
 * any pages yet.
 *
 * /sys/kernel/debug/page-nom/mem_blocks - number of 'mem_block_size'
 * blocks to eat from memory.  Inputing a number higher than the current value
 * will allocate new memory and inputing a number lower than the current value
 * will free memory.  0 will free all eaten vmalloc memory.  The returned value
 * will be the number of allocated blocks regardless if this method or the
 * following method was used to allocate the blocks.
 *
 * /sys/kernel/ debug/page-nom/mem_percent - percentage of the vmalloc
 * memory to eat.  This is an alternate method to 'mem_blocks'.  Where
 * mem_blocks is more precise, this is easier for automation. Just echo a
 * number between 0 and 100 for page-nom to try to consume that percentage of
 * system memory.  The returned value from this file is the current
 * percentage of allocated system memory from either method.  This number will
 * be appropriately massaged so it ends up being a multiple of
 * mem_block_size, so the number written won't always be the precise
 * percentage eaten.
 *(
 */


#define pr_fmt(fmt) "page-nom: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/sysinfo.h>
#include <linux/mm.h>
#include <asm/pgtable.h>

static struct dentry *page_nom_dir;

/*
 * Default to 256K sized blocks from vmalloc.  This should be good enough to
 * allocate enough memory fast enough and not hit bad fragmenting issues
 */

static int vmalloc_block_size = (256 * 1024);

/* Default to page sized blocks from system memory */
static int mem_block_size = PAGE_SIZE;

/* Number of blocks allocated to each entity */
static int vmalloc_block_count;
static int mem_block_count;

/* Percentage of vmalloc or system memory eaten */
static int vmalloc_percent;
static int mem_percent;

static LIST_HEAD(vmalloc_blocks);
static LIST_HEAD(mem_blocks);

/* Sysinfo gathered at load time */

static struct sysinfo sysinfo;

#ifdef VMALLOC_TOTAL
  #undef VMALLOC_TOTAL
  #define VMALLOC_TOTAL (VMALLOC_END - VMALLOC_START)
#endif

struct mem_block {
	struct page *page;
	int size;
	struct list_head node;
};

struct vmalloc_block {
	void *ptr;
	int size;
	struct list_head node;
};

#define TOKB(val) ((val) >> 10)

#define UNEATEN_MSG(_type, _blocks, _kb) \
	pr_err("I just gave back (%d/%d Kb) of " _type "\n", \
		(_blocks), (_kb))

#define REMAINING_MSG(_type, _blocks, _kb, _percent) \
	pr_err("I still have (%d blocks/%d Kb/%d%%) of " _type "\n", \
		(_blocks), (_kb), (_percent))

#define EATEN_MSG(_type, _blocks, _kb) \
	pr_err("I just ate (%d blocks/%d Kb) of " _type ". Nom!\n", \
		(_blocks), (_kb))

#define SOFAR_MSG(_type, _blocks, _kb, _percent) \
	pr_err("So far I have eaten (%d blocks/%d Kb/%d%%) of " _type "\n", \
		(_blocks), (_kb), (_percent))

/* Calculate the percentage in pages to avoid messy overflow workarounds */

static void update_mem_percent(void)
{
	unsigned int val;

	val = 100 * mem_block_count * (mem_block_size >> PAGE_SHIFT);
	val /= sysinfo.totalram;

	mem_percent = val;
}

static int free_mem_blocks(int count)
{
	struct mem_block *block, *tmp;
	int i = 0;

	list_for_each_entry_safe(block, tmp, &mem_blocks, node) {
		if (i == count || mem_block_count == 0)
			break;

		__free_page(block->page);
		list_del(&block->node);
		kfree(block);

		mem_block_count--;
		i++;
	}

	return i;
}

static void del_mem_blocks(int count)
{
	int ret;

	ret = free_mem_blocks(count);

	update_mem_percent();

	UNEATEN_MSG("sysmem", ret, TOKB(ret * mem_block_size));

	if (mem_block_count)
		REMAINING_MSG("sysmem", mem_block_count,
			TOKB(mem_block_count * mem_block_size),
			mem_percent);
	else
		pr_err("I no longer have any eaten memory. Feed me!\n");
}

static void add_mem_blocks(int count)
{
	struct mem_block *block;
	int i;

	for (i = 0; i < count; i++) {
		unsigned int gfp = GFP_KERNEL | __GFP_HIGHMEM | __GFP_NOWARN;

		/*
		 * Don't let the allocator loop around and cause death
		 * TODO: Add an option to let the use shoot themselves in the
		 * foot
		 */

		gfp |= __GFP_NORETRY;

		block = kzalloc(sizeof(*block), GFP_KERNEL);
		if (block == NULL) {
			pr_err("Unable to kmalloc a mem block\n");
			break;
		}

		if (mem_block_size != PAGE_SIZE)
			gfp |= __GFP_COMP;

		block->size = mem_block_size;
		block->page = alloc_pages(gfp, get_order(mem_block_size));

		if (block->page == NULL) {
			pr_err("Unable to get a %d Kb block of pages\n",
				TOKB(mem_block_size));
			break;
		}

		list_add(&block->node, &mem_blocks);
		mem_block_count++;
	}

	update_mem_percent();

	if (i != count)
		pr_err("I could only eat %d of %d blocks requested\n",
			i, count);

	EATEN_MSG("sysmem", i, TOKB(i * mem_block_size));
	SOFAR_MSG("sysmem", mem_block_count,
		TOKB(mem_block_count * mem_block_size), mem_percent);
}

static int mem_blocks_set(void *data, u64 val)
{
	unsigned int count = (unsigned int) val;

	if (count > mem_block_count)
		add_mem_blocks(count - mem_block_count);
	else if (count < mem_block_count)
		del_mem_blocks(mem_block_count - count);

	return 0;
}

static int mem_blocks_get(void *data, u64 *val)
{
	*val = mem_block_count;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mem_blocks_fops,
			mem_blocks_get,
			mem_blocks_set, "%lld\n");

static int mem_percent_set(void *data, u64 val)
{
	unsigned int percent = (unsigned int) val;
	unsigned int blocks = mem_block_count;

	if (percent > 100)
		return 0;

	if (percent > 0) {
		blocks = abs(percent - mem_percent);

		/* Figure out how many pages of +/- memory we want */
		blocks = (blocks * sysinfo.totalram) / 100;

		/* Turn that into mem_block_size(d) blocks */
		blocks = blocks / (mem_block_size >> PAGE_SHIFT);
	}

	if (percent > mem_percent)
		add_mem_blocks(blocks);
	else
		del_mem_blocks(blocks);

	return 0;
}

static int mem_percent_get(void *data, u64 *val)
{
	*val = mem_percent;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mem_percent_fops,
			mem_percent_get,
			mem_percent_set, "%lld\n");


/* Calculate the percentage in pages to avoid messy overflow workarounds */

static void update_vmalloc_percent(void)
{
	unsigned int val;

	val = 100 * vmalloc_block_count * (vmalloc_block_size >> PAGE_SHIFT);
	val /= VMALLOC_TOTAL >> PAGE_SHIFT;

	pr_err("%d = %ld / (%d * %d)\n", val,
			VMALLOC_TOTAL >> PAGE_SHIFT,
			vmalloc_block_count,
			vmalloc_block_size >> PAGE_SHIFT);

	vmalloc_percent = val;
}

static int free_vmalloc_blocks(int count)
{
	struct vmalloc_block *block, *tmp;
	int i = 0;

	list_for_each_entry_safe(block, tmp, &vmalloc_blocks, node) {
		if (i == count || vmalloc_block_count == 0)
			break;

		vfree(block->ptr);
		list_del(&block->node);
		kfree(block);

		vmalloc_block_count--;
		i++;
	}

	return i;
}

static void del_vmalloc_blocks(int count)
{
	int ret;

	ret = free_vmalloc_blocks(count);

	update_vmalloc_percent();

	UNEATEN_MSG("vmalloc", ret, TOKB(ret * vmalloc_block_size));

	if (vmalloc_block_count)
		REMAINING_MSG("vmalloc", vmalloc_block_count,
			TOKB(vmalloc_block_count * vmalloc_block_size),
			vmalloc_percent);
	else
		pr_err("I no longer have any eaten memory. Feed me!\n");
}

static void add_vmalloc_blocks(int count)
{
	struct vmalloc_block *block;
	int i;

	for (i = 0; i < count; i++) {
		block = kmalloc(sizeof(*block), GFP_KERNEL);
		if (block == NULL) {
			pr_err("Unable to kmalloc a vmalloc block\n");
			break;
		}

		block->size = vmalloc_block_size;
		block->ptr = vmalloc(vmalloc_block_size);

		if (block->ptr == NULL) {
			pr_err("Unable to get a %d Kb vmalloc block\n",
				TOKB(vmalloc_block_size));
			break;
		}

		list_add(&block->node, &vmalloc_blocks);
		vmalloc_block_count++;
	}

	update_vmalloc_percent();

	if (i != count)
		pr_err("I could only eat %d of %d blocks requested\n",
			i, count);

	EATEN_MSG("vmalloc", i, TOKB(i * vmalloc_block_size));
	SOFAR_MSG("vmalloc",
		vmalloc_block_count,
		TOKB(vmalloc_block_count * vmalloc_block_size),
		vmalloc_percent);
}

static int vmalloc_blocks_set(void *data, u64 val)
{
	unsigned int count = (unsigned int) val;

	if (count > vmalloc_block_count)
		add_vmalloc_blocks(count - vmalloc_block_count);
	else if (count < vmalloc_block_count)
		del_vmalloc_blocks(vmalloc_block_count - count);

	return 0;
}

static int vmalloc_blocks_get(void *data, u64 *val)
{
	*val = vmalloc_block_count;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vmalloc_blocks_fops,
			vmalloc_blocks_get,
			vmalloc_blocks_set, "%lld\n");

static int vmalloc_percent_set(void *data, u64 val)
{
	unsigned int percent = (unsigned int) val;
	unsigned int blocks = vmalloc_block_count;

	if (percent > 100)
		return 0;

	if (percent != 0) {
		blocks = abs(percent - vmalloc_percent);

		/* Figure out how many pages of +/- memory we want */
		blocks = (blocks * (VMALLOC_TOTAL >> PAGE_SHIFT)) / 100;

		/* Turn that into vmalloc_block_size(d) blocks */
		blocks = blocks / (vmalloc_block_size >> PAGE_SHIFT);
	}

	if (percent > vmalloc_percent)
		add_vmalloc_blocks(blocks);
	else
		del_vmalloc_blocks(blocks);

	return 0;
}

static int vmalloc_percent_get(void *data, u64 *val)
{
	*val = vmalloc_percent;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vmalloc_percent_fops,
			vmalloc_percent_get,
			vmalloc_percent_set, "%lld\n");

static int mem_block_size_set(void *data, u64 val)
{
	int order;

	if (vmalloc_block_count) {
		pr_err("Cannot change the block size if any memory is eaten\n");
		return 0;
	}

	/* Align the request to the next nearest order to make things easier */
	order = get_order((int) val);
	mem_block_size = 1 << (order + PAGE_SHIFT);

	return 0;
}

static int mem_block_size_get(void *data, u64 *val)
{
	*val = mem_block_size;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mem_block_size_fops,
			mem_block_size_get,
			mem_block_size_set, "%lld\n");

static int vmalloc_block_size_set(void *data, u64 val)
{
	if (vmalloc_block_count) {
		pr_err("Cannot change the block size if any memory is eaten\n");
		return 0;
	}

	/* Align the requested size to a page */
	vmalloc_block_size = PAGE_ALIGN((int) val);
	return 0;
}

static int vmalloc_block_size_get(void *data, u64 *val)
{
	*val = vmalloc_block_size;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vmalloc_block_size_fops,
			vmalloc_block_size_get,
			vmalloc_block_size_set, "%lld\n");

static int __init page_nom_init(void)
{
	si_meminfo(&sysinfo);

	page_nom_dir = debugfs_create_dir("page-nom", 0);

	if (IS_ERR(page_nom_dir))
		return PTR_ERR(page_nom_dir);

	debugfs_create_file("vmalloc_block_size", 0644, page_nom_dir, NULL,
		&vmalloc_block_size_fops);
	debugfs_create_file("vmalloc_percent", 0644, page_nom_dir, NULL,
		&vmalloc_percent_fops);
	debugfs_create_file("vmalloc_blocks", 0644, page_nom_dir, NULL,
		&vmalloc_blocks_fops);
	debugfs_create_file("mem_block_size", 0644, page_nom_dir, NULL,
		&mem_block_size_fops);
	debugfs_create_file("mem_percent", 0644, page_nom_dir, NULL,
		&mem_percent_fops);
	debugfs_create_file("mem_blocks", 0644, page_nom_dir, NULL,
		&mem_blocks_fops);

	pr_err("I am hungry! Give me pages\n");
	return 0;
}

static void __exit page_nom_exit(void)
{
	if (!IS_ERR(page_nom_dir))
		debugfs_remove_recursive(page_nom_dir);

	free_mem_blocks(mem_block_count);
	free_vmalloc_blocks(vmalloc_block_count);

	pr_err("Thanks for the noms\n");
}

module_init(page_nom_init);
module_exit(page_nom_exit);

MODULE_DESCRIPTION("page-nom");
MODULE_LICENSE("GPL v2");
