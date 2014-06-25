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

#include <linux/ioctl.h>

#define OCMEM_KERNEL_TEST_MAGIC 0xc1

#define OCMEM_TEST_TYPE_NOMINAL \
	_IO(OCMEM_KERNEL_TEST_MAGIC, 1)
#define OCMEM_TEST_TYPE_ADVERSARIAL \
	_IO(OCMEM_KERNEL_TEST_MAGIC, 2)
#define OCMEM_TEST_TYPE_STRESS \
	_IO(OCMEM_KERNEL_TEST_MAGIC, 3)
#define OCMEM_TEST_VERBOSE_MODE \
	_IO(OCMEM_KERNEL_TEST_MAGIC, 4)
#define OCMEM_TEST_DEBUG_MODE \
	_IO(OCMEM_KERNEL_TEST_MAGIC, 5)

#define TEST_OCMEM_CLIENT OCMEM_GRAPHICS

/* "OCMM" */
#define OCMEM_READ_WRITE_MAGIC 0xA110CA7E
#define OCMEM_MAX_SIZE 0x180000
#define OCMEM_BASE_ADDR 0xFEC00000
long msm_ocmem_test_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
