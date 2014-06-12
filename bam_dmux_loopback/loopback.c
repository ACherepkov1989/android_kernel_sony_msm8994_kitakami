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

#include "loopback_a2.h"
#include "loopback_smsm.h"
#include "loopback_sps.h"

static struct bam_ops_if loopback_ops = {
	.smsm_change_state_ptr = &mock_smsm_change_state,
	.smsm_get_state_ptr = &mock_smsm_get_state,
	.smsm_state_cb_register_ptr = &mock_smsm_state_cb_register,
	.smsm_state_cb_deregister_ptr = &mock_smsm_state_cb_deregister,

	.sps_connect_ptr = &mock_sps_connect,
	.sps_disconnect_ptr = &mock_sps_disconnect,
	.sps_register_bam_device_ptr =
			&mock_sps_register_bam_device,
	.sps_deregister_bam_device_ptr =
			&mock_sps_deregister_bam_device,
	.sps_alloc_endpoint_ptr = &mock_sps_alloc_endpoint,
	.sps_free_endpoint_ptr = &mock_sps_free_endpoint,
	.sps_set_config_ptr = &mock_sps_set_config,
	.sps_get_config_ptr = &mock_sps_get_config,
	.sps_device_reset_ptr = &mock_sps_device_reset,
	.sps_register_event_ptr = &mock_sps_register_event,
	.sps_transfer_one_ptr = &mock_sps_transfer_one,
	.sps_get_iovec_ptr = &mock_sps_get_iovec,
	.sps_get_unused_desc_num_ptr = &mock_sps_get_unused_desc_num,

	.dma_to = DMA_BIDIRECTIONAL,
	.dma_from = DMA_BIDIRECTIONAL,
};

/**
 * bam_dmux_loopback_init() - initializes loopback functionality
 *
 * Places bam dmux into an idle state, initializes the mock modules, swaps
 * hooks and then reinitializes bam dmux.
 *
 * Return: 0 on success, error code otherwise
 */
static int __init bam_dmux_loopback_init(void)
{
	int ret = 0;

	msm_bam_dmux_deinit();

	ret = mock_smsm_init();
	if (ret) {
		pr_err("%s: mock_smsm_init fail:%d\n", __func__, ret);
		return ret;
	}
	ret = mock_sps_init();
	if (ret) {
		pr_err("%s: mock_sps_init fail:%d\n", __func__, ret);
		return ret;
	}

	ret = mock_a2_init();
	if (ret) {
		pr_err("%s: mock_a2_init fail:%d\n", __func__, ret);
		return ret;
	}

	msm_bam_dmux_set_bam_ops(&loopback_ops);
	msm_bam_dmux_reinit();

	return ret;
}

/**
 * bam_dmux_loopback_exit() - cleans up the code inserted by loading the
 * module
 *
 * Places bam dmux into an idle state, restores hooks to default state and
 * then reinitializes bam dmux.
 */
static void __exit bam_dmux_loopback_exit(void)
{
	msm_bam_dmux_deinit();

	msm_bam_dmux_set_bam_ops(NULL);
	msm_bam_dmux_reinit();
}

module_init(bam_dmux_loopback_init);
module_exit(bam_dmux_loopback_exit);
MODULE_DESCRIPTION("MSM BAM DMUX LOOPBACK");
MODULE_LICENSE("GPL v2");
