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

#ifndef _MEMORY_PROF_MODULE_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * Since userspace doesn't have the definitions for the various gfp
 * flags, we define some common ones here. You can't compose gfp flags
 * the same way you can in the kernel like: GFP_KERNEL & ~GFP_WAIT
 * since these are simple, independent bitfields. To accomplish
 * not'ing something out, a dedicated flag must be used, like
 * MP_GFPNOT_WAIT.
 *
 * If you add any flags here make sure you also add them to
 * struct flag_info flag_info[].
 */
#define MP_GFP_KERNEL		(1<<0)
#define MP_GFP_HIGHMEM		(1<<1)
#define MP_GFP_ZERO		(1<<2)
#define MP_GFP_HIGHUSER		(1<<3)
#define MP_GFP_NOWARN		(1<<4)
#define MP_GFP_NORETRY		(1<<5)
#define MP_GFP_NO_KSWAPD	(1<<6)
#define MP_GFP_WAIT		(1<<7)
#define MP_GFPNOT_WAIT		(1<<8)

/**
 * Similar "mirror" definitions for IOMMU prot flags...
 *
 * If you add any flags here make sure you also add them to
 * struct flag_info flag_info[].
 */
#define MP_IOMMU_WRITE	(1<<0)
#define MP_IOMMU_READ	(1<<1)
#define MP_IOMMU_CACHE	(1<<2)
#define MP_IOMMU_SECURE (1<<3)
#define MP_IOMMU_ATTACH (1<<4)

struct mp_alloc_pages_args {
	unsigned int order;
	unsigned long long gfp;
	unsigned long long time_elapsed_us;
};

#define MAX_IOMMU_CTX_NAME 64

struct mp_iommu_map_test_args {
	char ctx_name[MAX_IOMMU_CTX_NAME];
	size_t chunk_order;
	int nchunks;
	unsigned int iterations;
	int prot;
	unsigned int flags;
	unsigned long long time_elapsed_us;
	unsigned long long time_elapsed_min_us;
	unsigned long long time_elapsed_max_us;
};

struct mp_iommu_attach_test_args {
	char ctx_name[MAX_IOMMU_CTX_NAME];
	unsigned int iterations;
	unsigned int flags;
	unsigned long long time_elapsed_us;
	unsigned long long time_elapsed_min_us;
	unsigned long long time_elapsed_max_us;
};


#define MEMORY_PROF_MAGIC     'H'

#define MEMORY_PROF_IOC_CLIENT_CREATE _IO(MEMORY_PROF_MAGIC, 0)
#define MEMORY_PROF_IOC_CLIENT_DESTROY _IO(MEMORY_PROF_MAGIC, 1)

#define MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS _IO(MEMORY_PROF_MAGIC, 3)

#define MEMORY_PROF_IOC_TEST_ALLOC_PAGES \
	_IOWR(MEMORY_PROF_MAGIC, 4, struct mp_alloc_pages_args)
#define MEMORY_PROF_IOC_CLEANUP_ALLOC_PAGES _IO(MEMORY_PROF_MAGIC, 5)

#define MEMORY_PROF_IOC_IOMMU_MAP_RANGE_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 6, struct mp_iommu_map_test_args)

#define MEMORY_PROF_IOC_IOMMU_UNMAP_RANGE_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 7, struct mp_iommu_map_test_args)

#define MEMORY_PROF_IOC_IOMMU_MAP_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 8, struct mp_iommu_map_test_args)

#define MEMORY_PROF_IOC_IOMMU_UNMAP_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 9, struct mp_iommu_map_test_args)

#define MEMORY_PROF_IOC_IOMMU_ATTACH_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 10, struct mp_iommu_attach_test_args)

#define MEMORY_PROF_IOC_IOMMU_DETACH_TEST \
	_IOWR(MEMORY_PROF_MAGIC, 11, struct mp_iommu_attach_test_args)
#endif /* #ifndef _MEMORY_PROF_MODULE_H */
