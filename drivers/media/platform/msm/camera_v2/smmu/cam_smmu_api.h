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
#ifndef _CAM_SMMU_API_H_
#define _CAM_SMMU_API_H_

#include <linux/dma-direction.h>
#include <linux/module.h>
#include <linux/qcom_iommu.h>
#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/dma-direction.h>
#include <linux/dma-attrs.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/random.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>
#include "../msm.h"

/*
 * Enum for possible CAM SMMU operations
 */

enum cam_smmu_ops_param {
	CAM_SMMU_ATTACH,
	CAM_SMMU_DETACH,
	CAM_SMMU_VOTE,
	CAM_SMMU_DEVOTE,
	CAM_SMMU_OPS_INVALID
};

enum cam_smmu_map_dir {
	CAM_SMMU_MAP_READ,
	CAM_SMMU_MAP_WRITE,
	CAM_SMMU_MAP_RW,
	CAM_SMMU_MAP_INVALID
};

/**
 * @param identifier: Unique identifier to be used by clients which they
 *                    should get from device tree. CAM SMMU driver will
 *                    not enforce how this string is obtained and will
 *                    only validate this against the list of permitted
 *                    identifiers
 * @param handle_ptr: Based on the indentifier, CAM SMMU drivier will
 *		      fill the handle pointed by handle_ptr
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_handle(char *identifier, int *handle_ptr);

/**
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param op    : Operation to be performed. Can be either CAM_SMMU_ATTACH
 *                or CAM_SMMU_DETACH
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_ops(int handle, enum cam_smmu_ops_param op);

/**
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 * @param ion_fd: ION handle identifying the memory buffer.
 * @phys_addr   : Pointer to physical address where mapped address will be
 *                returned.
 * @dir         : Mapping direction: which will traslate toDMA_BIDIRECTIONAL,
 *                DMA_TO_DEVICE or DMA_FROM_DEVICE
 * @len         : Length of buffer mapped returned by CAM SMMU driver.
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_get_phy_addr(int handle,
				int ion_fd, enum cam_smmu_map_dir dir,
				dma_addr_t *dma_addr, size_t *len_ptr);

/**
 * @param handle: Handle to identify the CAMSMMU client (VFE, CPP, FD etc.)
 * @param ion_fd: ION handle identifying the memory buffer.
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_put_phy_addr(int handle, int ion_fd);

/**
 * @param handle: Handle to identify the CAM SMMU client (VFE, CPP, FD etc.)
 *
 * @return Status of operation. Negative in case of error. Zero otherwise.
 */
int cam_smmu_destroy_handle(int handle);

#endif /* _CAM_SMMU_API_H_ */
