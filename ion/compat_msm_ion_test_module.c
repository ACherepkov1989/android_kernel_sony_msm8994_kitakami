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

#include <linux/compat.h>
#include <linux/uaccess.h>
#include "iontest.h"

struct compat_ion_test_data {
	compat_uint_t	align;
	compat_uint_t	size;
	compat_uint_t	heap_type_req;
	compat_ulong_t	heap_id_mask;
	compat_ulong_t	flags;
	compat_ulong_t	vaddr;
	compat_ulong_t	cache;
	compat_int_t	shared_fd;
	compat_uint_t	valid;
};

struct compat_ion_heap_data {
	compat_int_t	type;
	compat_uint_t	size;
	compat_ulong_t	heap_id_mask;
	compat_uint_t	valid;
};

#define COMPAT_IOC_ION_KALLOC   _IOW(MSM_ION_MAGIC, 2, \
					struct compat_ion_test_data)
#define COMPAT_IOC_ION_UIMPORT _IOW(MSM_ION_MAGIC, 7, \
					struct compat_ion_test_data)

#define COMPAT_IOC_ION_UBUF_FLAGS      _IOR(MSM_ION_MAGIC, 10, \
					compat_ulong_t)
#define COMPAT_IOC_ION_UBUF_SIZE       _IOR(MSM_ION_MAGIC, 11, \
					compat_ulong_t)

#define COMPAT_IOC_ION_FIND_PROPER_HEAP _IOWR(MSM_ION_MAGIC, 14, \
					struct compat_ion_heap_data)
static int compat_get_ion_test_data(
			struct compat_ion_test_data __user *data32,
			struct ion_test_data __user *data)

{
	compat_int_t i;
	compat_uint_t  u;
	compat_ulong_t l;
	int err;

	err = get_user(u, &data32->align);
	err |= put_user(u, &data->align);
	err |= get_user(u, &data32->size);
	err |= put_user(u, &data->size);
	err |= get_user(u, &data32->heap_type_req);
	err |= put_user(u, &data->heap_type_req);
	err |= get_user(l, &data32->heap_id_mask);
	err |= put_user(l, &data->heap_id_mask);
	err |= get_user(l, &data32->flags);
	err |= put_user(l, &data->flags);
	err |= get_user(l, &data32->vaddr);
	err |= put_user(l, &data->vaddr);
	err |= get_user(l, &data32->cache);
	err |= put_user(l, &data->cache);
	err |= get_user(i, &data32->shared_fd);
	err |= put_user(i, &data->shared_fd);
	err |= get_user(u, &data32->valid);
	err |= put_user(u, &data->valid);

	return err;
}

static int compat_get_ion_heap_data(
			struct compat_ion_heap_data __user *data32,
			struct ion_heap_data __user *data)

{
	compat_int_t i;
	compat_uint_t  u;
	compat_ulong_t l;
	int err;

	err = get_user(i, &data32->type);
	err |= put_user(i, &data->type);
	err |= get_user(u, &data32->size);
	err |= put_user(u, &data->size);
	err |= get_user(l, &data32->heap_id_mask);
	err |= put_user(l, &data->heap_id_mask);
	err |= get_user(u, &data32->valid);
	err |= put_user(u, &data->valid);

	return err;
}

static int compat_put_ion_heap_data(
			struct compat_ion_heap_data __user *data32,
			struct ion_heap_data __user *data)

{
	compat_int_t i;
	compat_uint_t  u;
	compat_ulong_t l;
	int err;

	err = get_user(i, &data->type);
	err |= put_user(i, &data32->type);
	err |= get_user(u, &data->size);
	err |= put_user(u, &data32->size);
	err |= get_user(l, &data->heap_id_mask);
	err |= put_user(l, &data32->heap_id_mask);
	err |= get_user(u, &data->valid);
	err |= put_user(u, &data32->valid);

	return err;
}


static int compat_put_unsigned_long(compat_ulong_t __user *data32,
				unsigned long __user *data)
{
	compat_ulong_t l;
	int err;

	err = get_user(l, data);
	err |= put_user(l, data32);
	return err;
}

static unsigned convert_cmd(unsigned cmd)
{
	switch(cmd) {
	case COMPAT_IOC_ION_KALLOC:
		return IOC_ION_KALLOC;
	case COMPAT_IOC_ION_UIMPORT:
		return IOC_ION_KALLOC;
	case COMPAT_IOC_ION_UBUF_FLAGS:
		return IOC_ION_UBUF_FLAGS;
	case COMPAT_IOC_ION_UBUF_SIZE:
		return IOC_ION_UBUF_SIZE;
	case COMPAT_IOC_ION_FIND_PROPER_HEAP:
		return IOC_ION_FIND_PROPER_HEAP;
	default:
		return cmd;
	}
}

long compat_ion_test_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	long ret;

	switch(cmd) {
	case COMPAT_IOC_ION_UIMPORT:
	case COMPAT_IOC_ION_KALLOC: {
		struct compat_ion_test_data __user *data32;
		struct ion_test_data __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_ion_test_data(data32, data);
		if (err)
			return err;

		return file->f_op->unlocked_ioctl(file, convert_cmd(cmd),
							(unsigned long)data);
	}
	case COMPAT_IOC_ION_UBUF_SIZE:
	case COMPAT_IOC_ION_UBUF_FLAGS: {
		compat_ulong_t __user *data32;
		unsigned long __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		ret = file->f_op->unlocked_ioctl(file, convert_cmd(cmd),
							(unsigned long)data);
		err = compat_put_unsigned_long(data32, data);
		return ret ? ret : err;
	}

	case COMPAT_IOC_ION_FIND_PROPER_HEAP: {
		struct compat_ion_heap_data __user *data32;
		struct ion_heap_data __user *data;
		int err;

		data32 = compat_ptr(arg);
		data = compat_alloc_user_space(sizeof(*data));
		if (data == NULL)
			return -EFAULT;

		err = compat_get_ion_heap_data(data32, data);
		if (err)
			return err;

		ret = file->f_op->unlocked_ioctl(file, convert_cmd(cmd),
							(unsigned long)data);

		err = compat_put_ion_heap_data(data32, data);
		return ret ? ret : err;
	}
	default:
		return file->f_op->unlocked_ioctl(file, cmd,
			(unsigned long)compat_ptr(arg));
	}
}
