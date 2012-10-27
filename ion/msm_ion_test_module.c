/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/msm_ion.h>
#include "iontest.h"

#define CLIENT_NAME "ion_test_client"
struct msm_ion_test {
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
	struct ion_test_data test_data;
	struct miscdevice *dev;
};

/*Utility apis*/
static inline int create_ion_client(struct msm_ion_test *ion_test)
{
	ion_test->ion_client = msm_ion_client_create(UINT_MAX, CLIENT_NAME);
	if (IS_ERR_OR_NULL(ion_test->ion_client))
		return -EIO;
	return 0;
}

static inline void free_ion_client(struct msm_ion_test *ion_test)
{
	ion_client_destroy(ion_test->ion_client);
}

static int alloc_ion_buf(struct msm_ion_test *ion_test,
					struct ion_test_data *test_data)
{
	ion_test->ion_handle = ion_alloc(ion_test->ion_client, test_data->size,
					test_data->align, test_data->heap_mask,
					test_data->flags);
	if (IS_ERR_OR_NULL(ion_test->ion_handle))
		return -EIO;
	return 0;
}

static inline void free_ion_buf(struct msm_ion_test *ion_test)
{
	ion_free(ion_test->ion_client, ion_test->ion_handle);
}

static int ion_test_open(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct msm_ion_test *ion_test = kzalloc(sizeof(struct msm_ion_test),
								GFP_KERNEL);
	if (!ion_test)
		return -ENOMEM;
	pr_info("ion test device opened\n");
	ion_test->dev = dev;
	file->private_data = ion_test;
	return 0;
}

static long ion_test_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret;
	ion_phys_addr_t phys_addr;
	void *addr;
	size_t len;
	unsigned long flags, size;
	struct msm_ion_test *ion_test = file->private_data;
	struct ion_test_data *test_data = &ion_test->test_data;

	switch (cmd) {
	case IOC_ION_KCLIENT_CREATE:
	{
		ret = create_ion_client(ion_test);
		break;
	}
	case IOC_ION_KCLIENT_DESTROY:
	{
		free_ion_client(ion_test);
		ret = 0;
		break;
	}
	case IOC_ION_KALLOC:
	{
		if (copy_from_user(test_data, (void __user *)arg,
						sizeof(struct ion_test_data)))
			return -EFAULT;
		ret = alloc_ion_buf(ion_test, test_data);
		if (ret)
			pr_info("allocating ion buffer failed\n");
		break;
	}
	case IOC_ION_KFREE:
	{
		free_ion_buf(ion_test);
		ret = 0;
		break;
	}
	case IOC_ION_KPHYS:
	{
		ret = ion_phys(ion_test->ion_client, ion_test->ion_handle,
							&phys_addr, &len);
		if (!ret)
			pr_info("size is 0x%x\n phys addr 0x%x", len,
						(unsigned int)phys_addr);
		break;
	}
	case IOC_ION_KMAP:
	{
		addr = ion_map_kernel(ion_test->ion_client,
					ion_test->ion_handle);
		if (IS_ERR_OR_NULL(addr)) {
			ret = -EIO;
			pr_info("mapping kernel buffer failed\n");
		} else {
			ret = 0;
			test_data->vaddr = (unsigned long)addr;
		}
		break;
	}
	case IOC_ION_KUMAP:
	{
		ion_unmap_kernel(ion_test->ion_client, ion_test->ion_handle);
		ret = 0;
		break;
	}
	case IOC_ION_UIMPORT:
	{
		if (copy_from_user(test_data, (void __user *)arg,
						sizeof(struct ion_test_data)))
			return -EFAULT;
		ion_test->ion_handle = ion_import_dma_buf(ion_test->ion_client,
							test_data->shared_fd);
		if (IS_ERR_OR_NULL(ion_test->ion_handle)) {
			ret = -EIO;
			pr_info("import of user buf failed\n");
		} else
			ret = 0;
		break;
	}
	case IOC_ION_UBUF_FLAGS:
	{
		ret = ion_handle_get_flags(ion_test->ion_client,
						ion_test->ion_handle, &flags);
		if (ret)
			pr_info("user flags cannot be retrieved\n");
		else
			if (copy_to_user((void __user *)arg, &flags,
						sizeof(unsigned long)))
				ret = -EFAULT;
		break;
	}
	case IOC_ION_UBUF_SIZE:
	{
		ret = ion_handle_get_size(ion_test->ion_client,
						ion_test->ion_handle, &size);
		if (ret)
			pr_info("buffer size cannot be retrieved\n");
		else
			if (copy_to_user((void __user *)arg, &size,
							sizeof(unsigned long)))
				ret = -EFAULT;
		break;
	}
	case IOC_ION_WRITE_VERIFY:
	{
		write_pattern(test_data->vaddr, test_data->size);
		if (verify_pattern(test_data->vaddr, test_data->size)) {
			pr_info("verify of mapped buf failed\n");
			ret = -EIO;
		} else
			ret = 0;
		break;
	}
	case IOC_ION_VERIFY:
	{
		if (verify_pattern(test_data->vaddr, test_data->size)) {
			pr_info("fail in verifying imported buffer\n");
			ret = -EIO;
		} else
			ret = 0;
		break;
	}
	case IOC_ION_SEC:
	{
		ret = msm_ion_secure_heap(ION_CP_MM_HEAP_ID);
		if (ret)
			pr_info("unable to secure heap\n");
		else
			pr_info("able to secure heap\n");
		break;
	}
	case IOC_ION_UNSEC:
	{
		ret = msm_ion_unsecure_heap(ION_CP_MM_HEAP_ID);
		if (ret)
			pr_info("unable to unsecure heap\n");
		else
			pr_info("able to unsecure heap\n");
		break;
	}
	default:
	{
		pr_info("command not supproted\n");
		ret = -EINVAL;
	}
	};
	return ret;
}

static int ion_test_release(struct inode *inode, struct file *file)
{
	struct msm_ion_test *ion_test = file->private_data;
	pr_info("ion test device closed\n");
	kfree(ion_test);
	return 0;
}

/*
 * Register ourselves as a misc device to be able to test the ion code
 * from userspace.
 */

static const struct file_operations ion_test_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ion_test_ioctl,
	.open = ion_test_open,
	.release = ion_test_release,
};

static struct miscdevice ion_test_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msm_ion_test",
	.fops = &ion_test_fops,
};

static int ion_test_init(void)
{
	int ret;
	ret = misc_register(&ion_test_dev);
	if (ret < 0)
		return ret;
	pr_alert("%s, minor number %d\n", __func__,
						ion_test_dev.minor);
	return 0;
}

static void ion_test_exit(void)
{
	misc_deregister(&ion_test_dev);
	pr_alert("%s\n", __func__);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Test for MSM ION implementation");
module_init(ion_test_init);
module_exit(ion_test_exit);
