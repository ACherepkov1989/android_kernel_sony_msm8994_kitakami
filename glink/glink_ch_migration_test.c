/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/ipc_logging.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <soc/qcom/glink.h>
#include <soc/qcom/glink_rpm_xprt.h>
#include <soc/qcom/smsm.h>
#include <soc/qcom/subsystem_restart.h>
#include "glink_ch_migration_test.h"
#include "glink_core_if.h"
#include "glink_loopback_client.h"
#include "glink_mock_xprt.h"
#include "glink_private.h"
#include "glink_test_common.h"

/*
 * glink_ut0_mock_migration_1 - Basic mock migration test 1
 *
 * @s: Pointer to output file
 *
 * This tests that specified transport should exist. It performs the following:
 * - Reset the medium mock transport
 * - Attempt a local open with the medium mock transport specified, but no
 *   initial flag
 * - Verify the local open fails
 */
void glink_ut0_mock_migration_1(struct seq_file *s)
{
	int failed = 0;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);
	glink_loopback_xprt_exit();
	glink_loopback_client_exit();
	do {
		mock_ptr = mock_xprt_get(MOCK);
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(open_cfg));
		open_cfg.transport = "mock";
		open_cfg.edge = "local";
		open_cfg.name = "loopback";

		open_cfg.notify_rx =  glink_test_notify_rx;
		open_cfg.notify_tx_done = glink_test_notify_tx_done;
		open_cfg.notify_state = glink_test_notify_state;
		open_cfg.notify_rx_intent_req = glink_test_rmt_rx_intent_req_cb;
		open_cfg.priv = &cb_data;

		handle = glink_open(&open_cfg);
		UT_ASSERT_PTR(handle, ==, ERR_PTR(-ENODEV));

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event, mock_ptr);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (!IS_ERR_OR_NULL(handle)) {
			glink_close(handle);
			handle = NULL;
		}
	}
	glink_loopback_xprt_init();
	glink_loopback_client_init();
}

/*
 * glink_ut0_mock_migration_2 - Basic mock migration test 2, no migration
 *
 * @s: Pointer to output file
 *
 * This tests a basic channel connect where there is no migration. It performs
 * the following:
 * - Reset all mock transports
 * - Initialize all mock transports
 * - Attempt a local open with the medium mock transport specified, but no
 *   initial flag
 * - Perform a remote open on the medium mock transport
 * - Verify that the open succeeds (the CONNECTED event is received)
 */
void glink_ut0_mock_migration_2(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_high_ptr;
	struct glink_mock_xprt *mock_med_ptr;
	struct glink_mock_xprt *mock_low_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct ut_notify_data cb_data;
	struct completion event_high;
	struct completion event_med;
	struct completion event_low;

	GLINK_STATUS(s, "Running %s\n", __func__);
	glink_loopback_xprt_exit();
	glink_loopback_client_exit();
	do {
		mock_high_ptr = mock_xprt_get(MOCK_HIGH);
		mock_med_ptr = mock_xprt_get(MOCK);
		mock_low_ptr = mock_xprt_get(MOCK_LOW);

		mock_xprt_reset(mock_high_ptr);
		mock_xprt_reset(mock_med_ptr);
		mock_xprt_reset(mock_low_ptr);

		init_completion(&event_high);
		register_completion(&mock_high_ptr->if_ptr, &event_high);

		init_completion(&event_med);
		register_completion(&mock_med_ptr->if_ptr, &event_med);

		init_completion(&event_low);
		register_completion(&mock_low_ptr->if_ptr, &event_low);

		cb_data_init(&cb_data);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0,
					MOCK_HIGH));
		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0,
					MOCK));
		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0,
					MOCK_LOW));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(open_cfg));
		open_cfg.transport = "mock";
		open_cfg.edge = "local";
		open_cfg.name = "loopback";

		open_cfg.notify_rx =  glink_test_notify_rx;
		open_cfg.notify_tx_done = glink_test_notify_tx_done;
		open_cfg.notify_state = glink_test_notify_state;
		open_cfg.notify_rx_intent_req = glink_test_rmt_rx_intent_req_cb;
		open_cfg.priv = &cb_data;

		handle = glink_open(&open_cfg);
		UT_ASSERT_ERR_PTR(handle);

		tx_cmd = mock_xprt_get_next_cmd(mock_med_ptr);
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);
		mock_med_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_med_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_med_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_med_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd(mock_med_ptr);
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		kfree(tx_cmd);
		/* open channel completed */

		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd(mock_med_ptr);
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_med_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_med_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);

		mock_med_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_med_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd(mock_med_ptr);
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);


		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event_high, mock_high_ptr);
	unregister_completion(&event_med, mock_med_ptr);
	unregister_completion(&event_low, mock_low_ptr);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (!IS_ERR_OR_NULL(handle)) {
			glink_close(handle);
			handle = NULL;
		}
	}
	glink_loopback_xprt_init();
	glink_loopback_client_init();
}

/**
 * glink_ut0_smd_trans_migration_link_state_cb() - Link state callback
 *
 * @cb_info:	callback information
 * @priv:	pointer to storage of best transport ID (uint16_t)
 */
static void glink_ut0_smd_trans_migration_link_state_cb(
		struct glink_link_state_cb_info *cb_info, void *priv)
{
	uint16_t *best_xprt_ptr = (uint16_t *)priv;
	uint16_t xprt_id;

	if (!best_xprt_ptr)
		return;

	if (cb_info->link_state != GLINK_LINK_STATE_UP)
		return;

	if (glink_xprt_name_to_id(cb_info->transport, &xprt_id))
		return;

	*best_xprt_ptr = min_t(uint16_t, xprt_id, *best_xprt_ptr);
}

/**
 * glink_ut0_smd_trans_migration() - SMD Transition channel migration test
 *
 * @s: pointer to output file
 *
 * This test opens a channel with the GLINK_OPT_INITIAL_XPORT flag on the
 * "smd_trans" transport.  The channel should then migrate to a better
 * transport (if available).
 */
void glink_ut0_smd_trans_migration_1(struct seq_file *s)
{
	struct glink_open_config cfg;
	void *handle = NULL;
	struct smd_trans_notify notify;
	bool failed = false;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	struct glink_link_info xprt_notif;
	char *xprt_name;
	void *xprt_notif_handle;
	uint16_t initial_xprt_id;
	uint16_t best_xprt_id;
	uint16_t final_xprt_id;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		GLINK_STATUS(s, "FAILED: Modem not SMSM and SMD initialized\n");
		return;
	}

	init_smd_trans_notify(&notify);

	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);

	do {
		/* get best edge on transport */
		UT_ASSERT_INT(0, ==, glink_xprt_name_to_id(ut_dfs_d->xprt_name,
					&initial_xprt_id));
		best_xprt_id = initial_xprt_id;
		xprt_notif.transport = NULL;
		xprt_notif.edge = ut_dfs_d->edge_name;
		xprt_notif.glink_link_state_notif_cb =
			glink_ut0_smd_trans_migration_link_state_cb;
		xprt_notif_handle = glink_register_link_state_cb(&xprt_notif,
				&best_xprt_id);
		UT_ASSERT_ERR_PTR(xprt_notif_handle);
		UT_ASSERT_INT(best_xprt_id, >=, 0);
		UT_ASSERT_INT(initial_xprt_id, >=, best_xprt_id);
		glink_unregister_link_state_cb(xprt_notif_handle);

		/* open channel */
		memset(&cfg, 0, sizeof(struct glink_open_config));
		cfg.priv = &notify;
		cfg.options = GLINK_OPT_INITIAL_XPORT;
		cfg.transport = ut_dfs_d->xprt_name;
		cfg.edge = ut_dfs_d->edge_name;
		cfg.name = "LOOPBACK";
		cfg.notify_rx = smd_trans_notify_rx;
		cfg.notify_tx_done = smd_trans_notify_tx_done;
		cfg.notify_state = smd_trans_notify_state;
		cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

		reset_smd_trans_notify(&notify);

		handle = glink_open(&cfg);
		UT_ASSERT_ERR_PTR(handle);
		UT_ASSERT_INT((int)wait_for_completion_timeout(
					&notify.connected_completion, 5 * HZ),
				>, 0);

		/* confirm final negotiated transport is the best transport */
		xprt_name = glink_get_ch_xprt_name(handle);
		UT_ASSERT_ERR_PTR(xprt_name);
		UT_ASSERT_INT(0, ==, glink_xprt_name_to_id(xprt_name,
					&final_xprt_id));

		UT_ASSERT_UINT((unsigned)final_xprt_id, ==,
				(unsigned)best_xprt_id);
		if (initial_xprt_id == best_xprt_id)
			seq_puts(s, "\tNote:  Migration did not occur because initial transport was the best transport\n");

		glink_close(handle);
		UT_ASSERT_INT((int)wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ),
				>, 0);
		handle = NULL;
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed) {
		if (handle) {
			glink_close(handle);
			if (wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ)
					== 0)
				GLINK_STATUS(s, "\tFailed to close port\n");

		}
		GLINK_STATUS(s, "FAILED\n");
	}
}
