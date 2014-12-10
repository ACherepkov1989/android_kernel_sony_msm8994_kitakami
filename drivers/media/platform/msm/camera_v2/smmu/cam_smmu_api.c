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
#define pr_fmt(fmt) "CAM-SMMU %s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/qcom_iommu.h>
#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/dma-direction.h>
#include <linux/dma-attrs.h>
#include <linux/of_platform.h>
#include <linux/iommu.h>
#include <linux/dma-mapping.h>
#include <linux/qcom_iommu.h>
#include <linux/msm_iommu_domains.h>
#include "cam_smmu_api.h"

#define BYTE_SIZE 8
#define COOKIE_BYTE 2
#define HANDLE_INIT (-1)

#ifdef CONFIG_CAM_SMMU_DBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

enum cam_protection_type {
	CAM_PROT_INVALID,
	CAM_NON_SECURE,
	CAM_SECURE,
	CAM_PROT_MAX,
};

enum cam_iommu_type {
	CAM_SMMU_INVALID,
	CAM_QSMMU,
	CAM_ARM_SMMU,
	CAM_SMMU_MAX,
};

enum cam_smmu_buf_state {
	CAM_SMMU_BUFF_EXIST,
	CAM_SMMU_BUFF_NOT_EXIST
};

struct cam_context_bank_info {
	struct device *dev;
	struct dma_iommu_mapping *mapping;
	dma_addr_t va_start;
	size_t va_len;
	const char *name;
	bool is_secure;
	struct list_head list_head;
	struct mutex lock;
	int handle;
	enum cam_smmu_ops_param state;
};

struct cam_iommu_cb_set {
	struct cam_context_bank_info *cb_info;
	u32 cb_num;
	u32 cb_init_count;
};

static struct of_device_id msm_cam_smmu_dt_match[] = {
	{ .compatible = "qcom,msm-cam-smmu", },
	{ .compatible = "qcom,msm-cam-smmu-cb", },
	{ .compatible = "qcom,qsmmu-cam-cb", },
	{}
};

struct cam_dma_buff_info {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *table;
	enum dma_data_direction dir;
	int ref_count;
	dma_addr_t paddr;
	spinlock_t sp_lock;
	struct list_head list;
	int ion_fd;
	size_t len;
};

static struct cam_iommu_cb_set iommu_cb_set;
static struct mutex iommu_table_lock;

static enum dma_data_direction cam_smmu_translate_dir(
	enum cam_smmu_map_dir dir);
static int cam_smmu_check_handle_unique(int hdl);
static int cam_smmu_create_iommu_handle(void);
static int cam_smmu_check_hardware_in_iommu_table(char *name);
static int cam_smmu_create_add_handle_in_table(char *name,
	int *hdl);
static int cam_smmu_find_index_by_handle(int hdl);
static struct cam_dma_buff_info *cam_smmu_find_mapping_by_ion_index(int idx,
	int ion_fd);
static int cam_smmu_map_buffer_and_add_to_list(int idx, int ion_fd,
	enum dma_data_direction dma_dir, dma_addr_t *paddr_ptr,
	size_t *len_ptr);
static int cam_smmu_unmap_buf_and_remove_from_list(
	struct cam_dma_buff_info *mapping_info, int idx);
static void cam_smmu_clean_buffer_list(int idx);
static void cam_smmu_init_iommu_table(void);
static void cam_smmu_print_list(int idx);
static void cam_smmu_print_list_and_table(void);
static int cam_smmu_probe(struct platform_device *pdev);

void cam_smmu_print_list(int idx)
{
	struct cam_dma_buff_info *mapping;
	CDBG("index = %d\n", idx);
	list_for_each_entry(mapping,
		&iommu_cb_set.cb_info[idx].list_head, list) {
		CDBG("ion_fd = %d, sg_table = %p\n",
			 mapping->ion_fd, (void *)mapping->table);
		CDBG("paddr= %p, len = %u\n",
			 (void *)mapping->paddr, (unsigned int)mapping->len);
	}
}

void cam_smmu_print_list_and_table(void)
{
	int i;

	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		CDBG("i= %d, handle= %d, name_addr=%p\n", i,
			   (int)iommu_cb_set.cb_info[i].handle,
			   (void *)iommu_cb_set.cb_info[i].name);
		CDBG("dev = %p\n", iommu_cb_set.cb_info[i].dev);
		if (iommu_cb_set.cb_info[i].name) {
			CDBG("name = %s\n print list:",
					iommu_cb_set.cb_info[i].name);
			cam_smmu_print_list(i);
		}
	}
}

static enum dma_data_direction cam_smmu_translate_dir(
				enum cam_smmu_map_dir dir)
{
	switch (dir) {
	case CAM_SMMU_MAP_READ:
		return DMA_FROM_DEVICE;
	case CAM_SMMU_MAP_WRITE:
		return DMA_TO_DEVICE;
	case CAM_SMMU_MAP_RW:
		return DMA_BIDIRECTIONAL;
	case CAM_SMMU_MAP_INVALID:
	default:
		pr_err(":%s: Error, Direction is invalid. dir = %d\n",
			__func__, (int)dir);
		break;
	}
	return DMA_NONE;
}

void cam_smmu_init_iommu_table(void)
{
	unsigned int i;
	mutex_init(&iommu_table_lock);
	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		iommu_cb_set.cb_info[i].handle = HANDLE_INIT;
		INIT_LIST_HEAD(&iommu_cb_set.cb_info[i].list_head);
		iommu_cb_set.cb_info[i].state = CAM_SMMU_DETACH;
		iommu_cb_set.cb_info[i].dev = NULL;
	}

	cam_smmu_print_list_and_table();
	return;
}

static int cam_smmu_check_handle_unique(int hdl)
{
	int i;

	if (hdl == HANDLE_INIT) {
		CDBG("%s:iommu handle is init number. Need to try again\n",
			__func__);
		return 1;
	}

	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		if (iommu_cb_set.cb_info[i].handle == HANDLE_INIT)
			continue;

		if (iommu_cb_set.cb_info[i].handle == hdl) {
			CDBG("%s: iommu handle %d conflicts\n",
					__func__, (int)hdl);
			return 1;
		}
	}
	return 0;
}

/**
 *  use low 2 bytes for handle cookie
 */
static int cam_smmu_create_iommu_handle(void)
{
	int hdl = 0;
	get_random_bytes(&hdl, (COOKIE_BYTE > sizeof(hdl) ?
					  sizeof(hdl):COOKIE_BYTE));
	CDBG("%s: create handle value = %d\n", __func__, (int)hdl);
	return hdl;
}

static int cam_smmu_check_hardware_in_iommu_table(char *name)
{
	int i;

	mutex_lock(&iommu_table_lock);
	/* check wether it is in the table */
	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		if (iommu_cb_set.cb_info[i].handle == HANDLE_INIT
		   && (!strcmp(iommu_cb_set.cb_info[i].name, name))) {
			CDBG("%s: Add cb in index %d in the table\n",
				 __func__, i);
			mutex_unlock(&iommu_table_lock);
			return 0;
		}
	}

	pr_err("%s: Error: No such cb or it is already in the table\n",
		__func__);
	cam_smmu_print_list_and_table();
	mutex_unlock(&iommu_table_lock);
	return -EINVAL;
}

static int cam_smmu_attach_device(int idx)
{
	int rc;
	struct cam_context_bank_info *cb = &iommu_cb_set.cb_info[idx];

	/* attach the mapping to device */
	rc = arm_iommu_attach_device(cb->dev, cb->mapping);
	if (rc < 0) {
		pr_err("%s:Failed : ret val: %d\n", __func__, rc);
		return -ENODEV;
	}
	return rc;
}

static void cam_smmu_detach_device(int idx)
{
	struct cam_context_bank_info *cb = &iommu_cb_set.cb_info[idx];
	arm_iommu_detach_device(cb->dev);
	return;
}

static int cam_smmu_create_add_handle_in_table(char *name,
					int *hdl)
{
	int i;
	int handle;

	mutex_lock(&iommu_table_lock);
	/* create handle and add in the iommu hardware table */
	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		if (iommu_cb_set.cb_info[i].handle == HANDLE_INIT &&
		   (!strcmp(iommu_cb_set.cb_info[i].name, name))) {
			/* make sure handle is unique */
			do {
				handle = cam_smmu_create_iommu_handle();
			} while (cam_smmu_check_handle_unique(handle));

			/* put handle in the table */
			iommu_cb_set.cb_info[i].handle = handle;
			mutex_init(&iommu_cb_set.cb_info[i].lock);
			*hdl = handle;
			mutex_unlock(&iommu_table_lock);
			return 0;
		}
	}

	/* if i == iommu_cb_set.cb_num */
	pr_err("%s: Error: hardware table is full with entries\n", __func__);
	mutex_unlock(&iommu_table_lock);
	return -EINVAL;
}

static int cam_smmu_find_index_by_handle(int hdl)
{
	int i;
	CDBG("%s: find handle %d\n", __func__, (int)hdl);

	mutex_lock(&iommu_table_lock);
	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		if (iommu_cb_set.cb_info[i].handle == hdl) {
			CDBG("%s: handle found, index=%d\n", __func__, i);
			mutex_unlock(&iommu_table_lock);
			return i;
		}
	}
	mutex_unlock(&iommu_table_lock);
	pr_err("%s: handle cannot be found\n", __func__);
	return -EINVAL;
}

static struct cam_dma_buff_info *cam_smmu_find_mapping_by_ion_index(int idx,
		int ion_fd)
{
	struct cam_dma_buff_info *mapping;

	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	list_for_each_entry(mapping, &iommu_cb_set.cb_info[idx].list_head,
			list) {
		if (mapping->ion_fd == ion_fd) {
			CDBG("%s: find ion_fd %d\n", __func__, ion_fd);
			mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
			return mapping;
		}
	}

	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	pr_err("%s: Error: Cannot find fd %d by index %d\n",
		__func__, idx, ion_fd);
	return NULL;
}

static void cam_smmu_clean_buffer_list(int idx)
{
	int ret;
	struct cam_dma_buff_info *mapping_info, *temp;

	list_for_each_entry_safe(mapping_info, temp,
				&iommu_cb_set.cb_info[idx].list_head, list) {
		CDBG("%s: Free mapping address %p, i = %d, fd = %d\n",
			__func__, (void *)mapping_info->paddr, idx,
			mapping_info->ion_fd);
		ret = cam_smmu_unmap_buf_and_remove_from_list(mapping_info,
				idx);
		if (ret < 0) {
			pr_err("%s: Error: Deleting one buffer failed\n",
				__func__);
			/*
			 * Ignore this error and continue to delete other
			 * buffers in the list
			 */
			continue;
		}
	}
}

static int cam_smmu_attach(int idx)
{
	int ret;

	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	if (iommu_cb_set.cb_info[idx].state == CAM_SMMU_ATTACH) {
		pr_err("%s: Error: It got attached before\n", __func__);
		ret = -EINVAL;
	} else if (iommu_cb_set.cb_info[idx].state == CAM_SMMU_DETACH) {
		ret = cam_smmu_attach_device(idx);
		if (ret < 0) {
			pr_err("%s: Error: ATTACH fail\n", __func__);
			mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
			return -ENODEV;
		}
		iommu_cb_set.cb_info[idx].state = CAM_SMMU_ATTACH;
		ret = 0;
	} else {
		pr_err("%s: Error: Not detach/attach\n", __func__);
		ret = -EINVAL;
	}
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	return ret;
}

static int cam_smmu_detach(int idx)
{
	int ret;

	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	if (iommu_cb_set.cb_info[idx].state == CAM_SMMU_DETACH) {
		pr_err("%s: Error: It got detached before\n", __func__);
		ret = -EINVAL;
	} else if (iommu_cb_set.cb_info[idx].state == CAM_SMMU_ATTACH) {
		iommu_cb_set.cb_info[idx].state = CAM_SMMU_DETACH;
		mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
		cam_smmu_clean_buffer_list(idx);
		cam_smmu_detach_device(idx);
		return 0;
	} else {
		pr_err("%s: Error: Not detach/attach\n", __func__);
		ret = -EINVAL;
	}
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	return ret;
}

static int cam_smmu_map_buffer_and_add_to_list(int idx, int ion_fd,
		 enum dma_data_direction dma_dir, dma_addr_t *paddr_ptr,
		 size_t *len_ptr)
{
	int rc = -1;
	struct cam_dma_buff_info *mapping_info;
	struct dma_buf *buf = NULL;
	struct dma_buf_attachment *attach = NULL;
	struct sg_table *table = NULL;

	/* allocate memory for each buffer information */
	buf = dma_buf_get(ion_fd);
	if (IS_ERR_OR_NULL(buf)) {
		rc = PTR_ERR(buf);
		pr_err("%s: Error: dma get buf failed\n", __func__);
		goto err_out;
	}

	attach = dma_buf_attach(buf, iommu_cb_set.cb_info[idx].dev);
	if (IS_ERR_OR_NULL(attach)) {
		rc = PTR_ERR(attach);
		pr_err("%s: Error: dma buf attach failed\n", __func__);
		goto err_put;
	}

	table = dma_buf_map_attachment(attach, dma_dir);
	if (IS_ERR_OR_NULL(table)) {
		rc = PTR_ERR(table);
		pr_err("%s: Error: dma buf map attachment failed\n", __func__);
		goto err_detach;
	}

	rc = dma_map_sg(iommu_cb_set.cb_info[idx].dev, table->sgl,
			table->nents, dma_dir);
	if (!rc) {
		pr_err("%s: Error: dma_map_sg failed\n", __func__);
		goto err_unmap_sg;
	}

	if (table->sgl) {
		CDBG("%s: DMA buf: %p, device: %p, attach: %p, table: %p\n",
				__func__,
				(void *)buf,
				(void *)iommu_cb_set.cb_info[idx].dev,
				(void *)attach, (void *)table);
		CDBG("table sgl: %p, rc: %d, dma_address: 0x%x\n",
				(void *)table->sgl, rc,
				(unsigned int)table->sgl->dma_address);
	} else {
		rc = -EINVAL;
		pr_err("%s: Error: table sgl is null\n", __func__);
		goto err_unmap_sg;
	}

	/* fill up mapping_info */
	mapping_info = kzalloc(sizeof(struct cam_dma_buff_info), GFP_KERNEL);
	if (!mapping_info) {
		pr_err("%s: Error: No enough space!\n", __func__);
		rc = -ENOSPC;
		goto err_unmap_sg;
	}
	mapping_info->ion_fd = ion_fd;
	mapping_info->buf = buf;
	mapping_info->attach = attach;
	mapping_info->table = table;
	mapping_info->paddr = sg_dma_address(table->sgl);
	mapping_info->len = (size_t)sg_dma_len(table->sgl);
	mapping_info->dir = dma_dir;
	mapping_info->ref_count = 0;

	/* return paddr and len to client */
	*paddr_ptr = sg_dma_address(table->sgl);
	*len_ptr = (size_t)sg_dma_len(table->sgl);

	if (!paddr_ptr) {
		pr_err("%s: Error: Space Allocation failed!\n", __func__);
		rc = -ENOSPC;
		goto err_unmap_sg;
	}
	CDBG("%s: ion_fd = %d, dev = %p, paddr= %p, len = %u\n",
			__func__, ion_fd,
			(void *)iommu_cb_set.cb_info[idx].dev,
			(void *)*paddr_ptr, (unsigned int)*len_ptr);

	/* add to the list */
	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	list_add(&mapping_info->list, &iommu_cb_set.cb_info[idx].list_head);
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	return 0;

err_unmap_sg:
	dma_buf_unmap_attachment(attach, table, dma_dir);
err_detach:
	dma_buf_detach(buf, attach);
err_put:
	dma_buf_put(buf);
err_out:
	return rc;
}

static int cam_smmu_unmap_buf_and_remove_from_list(
		struct cam_dma_buff_info *mapping_info,
		int idx)
{
	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	if ((!mapping_info->buf) || (!mapping_info->table) ||
		(!mapping_info->attach)) {
		pr_err("%s: Error: Invalid params dev = %p, table = %p",
			__func__, (void *)iommu_cb_set.cb_info[idx].dev,
			(void *)mapping_info->table);
		pr_err("%s: Error:dma_buf = %p, attach = %p\n",
			__func__, (void *)mapping_info->buf,
			(void *)mapping_info->attach);
		mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
		return -EINVAL;
	}

	/* iommu buffer clean up */
	dma_unmap_sg(iommu_cb_set.cb_info[idx].dev, mapping_info->table->sgl,
		mapping_info->table->nents, mapping_info->dir);
	dma_buf_unmap_attachment(mapping_info->attach,
		mapping_info->table, mapping_info->dir);
	dma_buf_detach(mapping_info->buf, mapping_info->attach);
	dma_buf_put(mapping_info->buf);
	mapping_info->buf = NULL;

	list_del_init(&mapping_info->list);
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);

	/* free one buffer */
	kfree(mapping_info);
	return 0;
}

static enum cam_smmu_buf_state cam_smmu_check_fd_in_list(int idx,
					int ion_fd, dma_addr_t *paddr_ptr,
					size_t *len_ptr)
{
	struct cam_dma_buff_info *mapping;
	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	list_for_each_entry(mapping,
			&iommu_cb_set.cb_info[idx].list_head,
			list) {
		if (mapping->ion_fd == ion_fd) {
			mapping->ref_count++;
			*paddr_ptr = mapping->paddr;
			*len_ptr = mapping->len;
			mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
			return CAM_SMMU_BUFF_EXIST;
		}
	}
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	return CAM_SMMU_BUFF_NOT_EXIST;
}

int cam_smmu_get_handle(char *identifier, int *handle_ptr)
{
	int ret = 0;

	if (!identifier) {
		pr_err("%s: iommu harware name is NULL\n", __func__);
		return -EFAULT;
	}

	if (!handle_ptr) {
		pr_err("%s: handle pointer is NULL\n", __func__);
		return -EFAULT;
	}

	ret = cam_smmu_check_hardware_in_iommu_table(identifier);
	if (ret < 0) {
		pr_err("%s: Error: got cb before or no such device, name = %p\n",
			__func__, identifier);
		return ret;
	}

	/* create and put handle in the table */
	ret = cam_smmu_create_add_handle_in_table(identifier, handle_ptr);
	if (ret < 0) {
		pr_err("%s: Error: Iommu hardware table is full\n", __func__);
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL(cam_smmu_get_handle);

int cam_smmu_ops(int handle, enum cam_smmu_ops_param ops)
{
	int ret = 0, idx;

	CDBG("E: ops = %d\n", ops);
	idx = cam_smmu_find_index_by_handle(handle);
	if (idx < 0 || idx >= iommu_cb_set.cb_num) {
		pr_err("%s: index is not valid, index = %d\n",
			 __func__, idx);
		return -EINVAL;
	}

	switch (ops) {
	case CAM_SMMU_ATTACH: {
		ret = cam_smmu_attach(idx);
		break;
	}
	case CAM_SMMU_DETACH: {
		ret = cam_smmu_detach(idx);
		break;
	}
	case CAM_SMMU_VOTE:
	case CAM_SMMU_DEVOTE:
	default:
		pr_err("%s: Error: idx = %d, ops = %d\n",
			__func__, idx, ops);
		return -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(cam_smmu_ops);

int cam_smmu_get_phy_addr(int handle, int ion_fd,
		enum cam_smmu_map_dir dir, dma_addr_t *paddr_ptr,
		size_t *len_ptr)
{
	int idx, rc;
	enum dma_data_direction dma_dir;
	enum cam_smmu_buf_state buf_state;

	if (!paddr_ptr || !len_ptr) {
		pr_err("%s: Error: Input pointers are invalid\n",
			__func__);
		return -EINVAL;
	}
	/* clean the content from clients */
	*paddr_ptr = (dma_addr_t)NULL;
	*len_ptr = (size_t)0;

	dma_dir = cam_smmu_translate_dir(dir);
	if (dma_dir == DMA_NONE) {
		pr_err("%s: translate direction failed. dir = %d\n",
			__func__, dir);
		return -EINVAL;
	}

	idx = cam_smmu_find_index_by_handle(handle);
	if (idx < 0 || idx >= iommu_cb_set.cb_num) {
		pr_err("%s: index is not valid, index = %d\n",
			__func__, idx);
		return -EINVAL;
	}

	buf_state = cam_smmu_check_fd_in_list(idx, ion_fd, paddr_ptr, len_ptr);
	if (buf_state == CAM_SMMU_BUFF_EXIST) {
		pr_debug("ion_fd:%d already in the list, give same addr and len back",
				 ion_fd);
		return 0;
	}
	rc = cam_smmu_map_buffer_and_add_to_list(idx, ion_fd, dma_dir,
			paddr_ptr, len_ptr);
	if (rc < 0) {
		pr_err("%s: Error: mapping or add list fail\n", __func__);
		return rc;
	}
	return 0;
}
EXPORT_SYMBOL(cam_smmu_get_phy_addr);

int cam_smmu_put_phy_addr(int handle, int ion_fd)
{
	int idx;
	int ret = -1;
	struct cam_dma_buff_info *mapping_info;

	/* find index in the iommu_cb_set.cb_info */
	idx = cam_smmu_find_index_by_handle(handle);
	if (idx < 0 || idx >= iommu_cb_set.cb_num) {
		pr_err("%s: index is not valid, index = %d.\n", __func__, idx);
		return -EINVAL;
	}


	/* based on ion fd and index, we can find mapping info of buffer */
	mapping_info = cam_smmu_find_mapping_by_ion_index(idx, ion_fd);
	if (!mapping_info) {
		pr_err("%s: Error: Invalid params\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	mapping_info->ref_count--;
	if (mapping_info->ref_count > 0) {
		pr_debug("There are still %u buffer(s) with same fd %d",
			mapping_info->ref_count, mapping_info->ion_fd);
		mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
		return 0;
	}
	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);

	/* unmapping one buffer from device */
	ret = cam_smmu_unmap_buf_and_remove_from_list(mapping_info, idx);
	if (ret < 0) {
		pr_err("%s: Error: unmap or remove list fail\n", __func__);
		cam_smmu_print_list_and_table();
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL(cam_smmu_put_phy_addr);

int cam_smmu_destroy_handle(int handle)
{
	int idx, ret;
	idx = cam_smmu_find_index_by_handle(handle);
	if (idx < 0 || idx >= iommu_cb_set.cb_num) {
		pr_err("%s: index is not valid, index = %d\n",
			__func__, idx);
		return -EINVAL;
	}

	mutex_lock(&iommu_table_lock);
	cam_smmu_clean_buffer_list(idx);
	/* delete handle in the iommu harware table */
	if (list_empty_careful(&iommu_cb_set.cb_info[idx].list_head)) {
		iommu_cb_set.cb_info[idx].handle = HANDLE_INIT;
	} else {
		pr_err("%s: Error: List is not clean\n", __func__);
		cam_smmu_print_list(idx);
	}

	mutex_lock(&iommu_cb_set.cb_info[idx].lock);
	if (iommu_cb_set.cb_info[idx].state == CAM_SMMU_ATTACH) {
		CDBG("%s: It should get detached before.\n", __func__);
		ret = cam_smmu_detach(idx);
		if (ret < 0) {
			pr_err("%s: Error: Detach idx %d fail\n",
				__func__, idx);
			mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
			mutex_unlock(&iommu_table_lock);
			return -EINVAL;
		}
	}

	mutex_unlock(&iommu_cb_set.cb_info[idx].lock);
	mutex_unlock(&iommu_table_lock);
	cam_smmu_print_list_and_table();
	return 0;
}
EXPORT_SYMBOL(cam_smmu_destroy_handle);

static void cam_smmu_release_cb(struct platform_device *pdev)
{
	int i = 0;

	for (i = 0; i < iommu_cb_set.cb_num; i++) {
		arm_iommu_detach_device(iommu_cb_set.cb_info[i].dev);
		arm_iommu_release_mapping(iommu_cb_set.cb_info[i].mapping);
	}

	devm_kfree(&pdev->dev, iommu_cb_set.cb_info);
	iommu_cb_set.cb_num = 0;
}

static int cam_smmu_setup_cb(struct cam_context_bank_info *cb,
	struct device *dev)
{
	int rc = 0;
	int order = 0;
	int disable_htw = 1;

	if (!cb || !dev) {
		pr_err("%s: Error: invalid input params\n", __func__);
		return -EINVAL;
	}

	cb->dev = dev;
	cb->va_start = SZ_128K;
	cb->va_len = SZ_2G - SZ_128K;

	/* create a virtual mapping */
	cb->mapping = arm_iommu_create_mapping(&platform_bus_type,
		cb->va_start, cb->va_len, order);
	if (IS_ERR(cb->mapping)) {
		pr_err("%s: Error: Failed\n", __func__);
		rc = -ENODEV;
		goto end;
	}

	/*
	 * Set the domain attributes
	 * disable L2 redirect since it decreases
	 * performance
	 */
	if (iommu_domain_set_attr(cb->mapping->domain,
		DOMAIN_ATTR_COHERENT_HTW_DISABLE,
		&disable_htw)) {
		pr_err("%s: Error: couldn't disable coherent HTW\n", __func__);
		rc = -ENODEV;
		goto err_set_attr;
	}
	return 0;
err_set_attr:
	arm_iommu_release_mapping(cb->mapping);
end:
	return rc;
}

static int cam_alloc_smmu_context_banks(struct device *dev)
{
	struct device_node *domains_child_node = NULL;
	if (!dev) {
		pr_err("%s: Error: Invalid device\n", __func__);
		return -ENODEV;
	}

	iommu_cb_set.cb_num = 0;

	/* traverse thru all the child nodes and increment the cb count */
	for_each_child_of_node(dev->of_node, domains_child_node) {
		if (of_device_is_compatible(domains_child_node,
			"qcom,msm-cam-smmu-cb"))
			iommu_cb_set.cb_num++;

		if (of_device_is_compatible(domains_child_node,
			"qcom,qsmmu-cam-cb"))
			iommu_cb_set.cb_num++;
	}

	if (iommu_cb_set.cb_num == 0) {
		pr_err("%s: Error: no context banks present\n", __func__);
		return -ENOENT;
	}

	/* allocate memory for the context banks */
	iommu_cb_set.cb_info = devm_kzalloc(dev,
		iommu_cb_set.cb_num * sizeof(struct cam_context_bank_info),
		GFP_KERNEL);

	if (!iommu_cb_set.cb_info) {
		pr_err("%s: Error: cannot allocate context banks\n", __func__);
		return -ENOMEM;
	}

	cam_smmu_init_iommu_table();
	iommu_cb_set.cb_init_count = 0;

	CDBG("%s: no of context banks :%d\n", __func__, iommu_cb_set.cb_num);
	return 0;
}

static int cam_populate_smmu_context_banks(struct device *dev,
	enum cam_iommu_type type)
{
	int rc = 0;
	struct cam_context_bank_info *cb;
	struct device *ctx;

	if (!dev) {
		pr_err("Error: Invalid device\n");
		return -ENODEV;
	}

	/* check the bounds */
	if (iommu_cb_set.cb_init_count >= iommu_cb_set.cb_num) {
		pr_err("Error: populate more than allocated cb\n");
		rc = -EBADHANDLE;
		goto cb_init_fail;
	}

	/* read the context bank from cb set */
	cb = &iommu_cb_set.cb_info[iommu_cb_set.cb_init_count];

	/* set the name of the context bank */
	rc = of_property_read_string(dev->of_node, "label", &cb->name);
	if (rc) {
		pr_err("Error: failed to read label from sub device\n");
		goto cb_init_fail;
	}

	/* set the secure/non secure domain type */
	if (of_property_read_bool(dev->of_node, "qcom,secure-context"))
		cb->is_secure = CAM_SECURE;
	else
		cb->is_secure = CAM_NON_SECURE;

	CDBG("cb->name : %s, cb->is_secure :%d\n", cb->name, cb->is_secure);

	/* set up the iommu mapping for the  context bank */

	if (type == CAM_QSMMU) {
		ctx = msm_iommu_get_ctx(cb->name);
		pr_info("%s: getting QSMMU ctx : %s\n", __func__, cb->name);
	} else {
		ctx = dev;
		pr_info("%s: getting Arm SMMU ctx : %s\n", __func__, cb->name);
	}

	rc = cam_smmu_setup_cb(cb, ctx);
	if (rc < 0)
		pr_err("%s: Error: failed to setup cb : %s\n",
			__func__, cb->name);

	/* increment count to next bank */
	iommu_cb_set.cb_init_count++;

	CDBG("X: cb init count :%d\n", iommu_cb_set.cb_init_count);
	return rc;

cb_init_fail:
	iommu_cb_set.cb_info = NULL;
	return rc;
}

static int cam_smmu_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device *dev = &pdev->dev;

	if (of_device_is_compatible(dev->of_node, "qcom,msm-cam-smmu")) {
		rc = cam_alloc_smmu_context_banks(dev);
		if (rc < 0)	{
			pr_err("%s: Error: allocating context banks\n",
				__func__);
			return -ENOMEM;
		}
	}
	if (of_device_is_compatible(dev->of_node, "qcom,msm-cam-smmu-cb")) {
		rc = cam_populate_smmu_context_banks(dev, CAM_ARM_SMMU);
		if (rc < 0) {
			pr_err("%s: Error: populating context banks\n",
				__func__);
			return -ENOMEM;
		}
		return rc;
	}
	if (of_device_is_compatible(dev->of_node, "qcom,qsmmu-cam-cb")) {
		rc = cam_populate_smmu_context_banks(dev, CAM_QSMMU);
		if (rc < 0) {
			pr_err("%s: Error: populating context banks\n",
				__func__);
			return -ENOMEM;
		}
		return rc;
	}

	/* probe thru all the subdevices */
	rc = of_platform_populate(pdev->dev.of_node, msm_cam_smmu_dt_match,
				NULL, &pdev->dev);
	if (rc < 0)
		pr_err("%s: Error: populating devices\n", __func__);
	cam_smmu_print_list_and_table();
	return rc;
}

static int cam_smmu_remove(struct platform_device *pdev)
{
	/* release all the context banks and memory allocated */
	if (of_device_is_compatible(pdev->dev.of_node, "qcom,msm-cam-smmu"))
		cam_smmu_release_cb(pdev);
	return 0;
}
static struct platform_driver cam_smmu_driver = {
	.probe = cam_smmu_probe,
	.remove = cam_smmu_remove,
	.driver = {
		.name = "msm_cam_smmu",
		.owner = THIS_MODULE,
		.of_match_table = msm_cam_smmu_dt_match,
	},
};

static int __init cam_smmu_init_module(void)
{
	return platform_driver_register(&cam_smmu_driver);
}

static void __exit cam_smmu_exit_module(void)
{
	platform_driver_unregister(&cam_smmu_driver);
}

module_init(cam_smmu_init_module);
module_exit(cam_smmu_exit_module);
MODULE_DESCRIPTION("MSM Camera SMMU driver");
MODULE_LICENSE("GPL v2");

