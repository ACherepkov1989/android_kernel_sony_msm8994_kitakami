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
void glink_ut0_smd_trans_migration(struct seq_file *s)
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
