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

#ifndef _LOOPBACK_SPS_H_
#define _LOOPBACK_SPS_H_

#include <linux/msm-sps.h>

int mock_sps_connect(struct sps_pipe *h, struct sps_connect *connect);
int mock_sps_disconnect(struct sps_pipe *h);
int mock_sps_register_bam_device(const struct sps_bam_props *bam_props,
	unsigned long *dev_handle);
int mock_sps_deregister_bam_device(unsigned long dev_handle);
struct sps_pipe *mock_sps_alloc_endpoint(void);
int mock_sps_free_endpoint(struct sps_pipe *h);
int mock_sps_set_config(struct sps_pipe *h,
	struct sps_connect *config);
int mock_sps_get_config(struct sps_pipe *h,
	struct sps_connect *config);
int mock_sps_device_reset(unsigned long dev);
int mock_sps_register_event(struct sps_pipe *h,
	struct sps_register_event *reg);
int mock_sps_transfer_one(struct sps_pipe *h, phys_addr_t addr, u32 size,
	void *user, u32 flags);
int mock_sps_get_iovec(struct sps_pipe *h, struct sps_iovec *iovec);
int mock_sps_get_unused_desc_num(struct sps_pipe *h, u32 *desc_num);
int debug_mock_sps_log(char *buff, int max, loff_t *ppos);
int mock_sps_init(void);
void mock_sps_trigger_event(void *data,
	enum sps_option o_option, void *pkt);
void mock_sps_rewrite_notify(void);
#endif /* _LOOPBACK_SPS_H_ */
