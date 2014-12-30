/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/ipc_logging.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/ctype.h>
#include <soc/qcom/glink.h>
#include <soc/qcom/glink_rpm_xprt.h>
#include <soc/qcom/smsm.h>
#include <soc/qcom/subsystem_restart.h>
#include "glink_loopback_client.h"
#include "glink_mock_xprt.h"
#include "glink_test_common.h"
#include "glink_core_if.h"
#include "glink_private.h"

static atomic_t local_mt_test_num_workers = ATOMIC_INIT(0);
static atomic_t mpss_mt_test_num_workers = ATOMIC_INIT(0);
static int ut_iterations = 5;
module_param_named(iterations, ut_iterations,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
static int ut_vector_buf_count = 1;
module_param_named(vector_buf_count, ut_vector_buf_count,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

/* Begin - Module parameters to configure multi-threaded test parameters */
/* Interval between two packet transmission by a latency-specific client */
static int ut_mt_lat_pkt_ntrvl_ms = 20;
module_param_named(pkt_interval_ms, ut_mt_lat_pkt_ntrvl_ms,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Number of latency-sensitive clients */
static int ut_mt_num_lat_clnts = 1;
module_param_named(num_lat_clnts, ut_mt_num_lat_clnts,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Number of throughput-sensitive clients */
static int ut_mt_num_tput_clnts = 1;
module_param_named(num_tput_clnts, ut_mt_num_tput_clnts,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Number of packets to be transmitted by a latency-sensitive client */
static int ut_mt_num_pkts_lat_ch = 100;
module_param_named(num_pkts_lat_ch, ut_mt_num_pkts_lat_ch,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Number of packets to be transmitted by a throughput-sensitive client */
static int ut_mt_num_pkts_tput_ch = 100;
module_param_named(num_pkts_tput_ch, ut_mt_num_pkts_tput_ch,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Size of a packet transmitted by a latency-sensitive client */
static int ut_mt_pkt_size_lat_ch = 128;
module_param_named(pkt_size_lat_ch, ut_mt_pkt_size_lat_ch,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* Size of a packet transmitted by a throughput-sensitive client */
static int ut_mt_pkt_size_tput_ch = 512;
module_param_named(pkt_size_tput_ch, ut_mt_pkt_size_tput_ch,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
/* End - Module parameters to configure multi-threaded test parameters */

const char *testdata = "Hello GLINK";
static struct workqueue_struct *ut_intent_req_workqueue;
struct mutex multithread_test_mutex_lha0;
typedef int (*testfunc)(struct seq_file *s, void *cntl_handle,
							void **data_handle);
static void *glink_ut_link_state_notif_handle;

struct glink_ut_dbgfs {
	const char *ut_name;
	char xprt_name[GLINK_NAME_SIZE];
	char edge_name[GLINK_NAME_SIZE];
	testfunc tfunc;
};

static struct workqueue_struct *ut_dbgfs_create_workqueue;
struct glink_ut_dbgfs_work {
	const char *edge;
	const char *xprt;
	struct work_struct dbgfs_work;
};

struct ut_ch_close_work {
	void *ch_handle;
	struct ut_notify_data *cb_data;
	struct work_struct work;
};

/**
 * Structure to hold status of TX work function
 * @glink_tx_entered: Holds status about that the work function is called.
 * @glink_tx_exited: Status of work function executed or not.
 */
struct ut_tx_stat {
	bool glink_tx_entered;
	bool glink_tx_exited;
};

/**
 * struct ut_intent_req_tx_work - TX work structure
 * @handle: Pointer to channel context
 * @pkt_priv: Opague data value
 * @data: Pointer to data
 * @size: Size of data
 * req_intent: Request for rx_intent from remote side
 * @work: Work to be excuted
 * @tx_stat: Instance of struct ut_tx_stat
 */
struct ut_intent_req_tx_work {
	void *handle;
	void *pkt_priv;
	void *data;
	size_t size;
	bool req_intent;
	struct work_struct work;
	struct ut_tx_stat tx_stat;
	struct completion tx_worker_complete;
};

/**
 * struct ch_config - Channel Configuration in a multi-client test
 * @ch_name: Name of the channel to be used by the client
 * @num_pkts: Number of packets to be transmitted by the client
 * @data_len: Packet size to be transmitted by the client
 * @tx_delay_ms: Time delay before the server echoes back the data
 * @pkt_ntrvl: Interval between transmission by the client
 */
struct ch_config {
	char ch_name[GLINK_NAME_SIZE];
	uint32_t num_pkts;
	size_t data_len;
	uint32_t tx_delay_ms;
	uint32_t pkt_ntrvl;
};

/**
 * struct mt_test_info_struct - Multithreaded test information structure
 * @s: Pointer to output file
 * @pkt: Loopback server request structure
 * @cntl_handle: Pointer to control channel handle
 * @data_handle: Pointer to data channel handle
 * @name: Data channel name
 * @work: Work to be excuted
 * @waitqueue: Waitqueue structure for thread tracking
 * @result: Used to indicate test failure from within the worker function
 * @num_workers: The number of worker threads/clients still running
 * @ch_cfg: Channel Configuration to be used by a client/worker thread
 * @index: Index/Identity of the client/worker thread
 */
struct mt_test_info_struct {
	struct seq_file *s;
	struct req pkt;
	void *cntl_handle;
	void *data_handle;
	char name[GLINK_NAME_SIZE];
	struct work_struct work;
	wait_queue_head_t *waitqueue;
	int *result;
	atomic_t *num_workers;
	struct ch_config ch_cfg;
	int index;
};

static int do_mock_negotiation(struct seq_file *s, uint32_t version,
		uint32_t features);

static int glink_ut1_local_mt_send_config_reqs(struct seq_file *s, void
		*cntl_handle, const char *name);

static int glink_ut1_mpss_mt_send_config_reqs(struct seq_file *s, void
		*cntl_handle, const char *name,
		struct mt_test_info_struct *mpss_mt_thread_info);

static void ut_local_mt_worker_func(struct work_struct *work);
static void ut_mpss_mt_worker_func(struct work_struct *work);

static void glink_ut_to_upper(char *out, char *input)
{
	int i;
	for (i = 0; i < strlen(input); i++)
		out[i] = toupper(input[i]);
}
void glink_test_notify_rx(void *handle, const void *priv, const void *pkt_priv,
		const void *ptr, size_t size)
{
	struct ut_notify_data *cb_data_ptr = (struct ut_notify_data *)priv;
	if (!cb_data_ptr) {
		GLINK_UT_ERR("%s: Could not allocate data for cb_data_ptr\n",
				__func__);
		return;
	}

	GLINK_UT_INFO("%s: priv[%p] data[%p] size[%zu], thread: %d\n", __func__,
			pkt_priv, (char *)ptr, size, current->pid);
	cb_data_ptr->size = size;
	cb_data_ptr->rx_notify = true;
	complete(&cb_data_ptr->cb_completion);
}

void glink_test_notify_tx_done(void *handle, const void *priv, const void
		*pkt_priv, const void *ptr)
{
	struct ut_notify_data *cb_data_ptr = (struct ut_notify_data *)priv;
	if (!cb_data_ptr) {
		GLINK_UT_ERR("%s: Could not allocate data for cb_data_ptr\n",
				__func__);
		return;
	}
	GLINK_UT_INFO("%s: priv[%p] pkt_priv[%p] ptr[%p], thread: %d\n",
			__func__,
			priv, pkt_priv, ptr, current->pid);
	cb_data_ptr->tx_done = true;
}

void glink_test_notify_state(void *handle, const void *priv, unsigned event)
{
	struct ut_notify_data *cb_data_ptr = (struct ut_notify_data *)priv;
	if (!cb_data_ptr) {
		GLINK_UT_ERR("%s: Could not allocate data for cb_data_ptr\n",
				__func__);
		return;
	}

	GLINK_UT_INFO("%s: event[%d], thread: %d\n", __func__, event,
			current->pid);
	cb_data_ptr->event = event;
	complete(&cb_data_ptr->cb_completion);
}

#define RMT_RX_INTENT_REQ_SIZE 20
bool glink_test_rmt_rx_intent_req_cb(void *handle, const void *priv, size_t sz)
{
	bool ret = false;
	struct ut_notify_data *cb_data_ptr = (struct ut_notify_data *)priv;
	if (!cb_data_ptr) {
		GLINK_UT_ERR("%s: Could not allocate data for cb_data_ptr\n",
				__func__);
		return -ENOMEM;
	}

	GLINK_UT_INFO("%s: Triggered with size %zu", __func__, sz);
	cb_data_ptr->send_intent = false;
	cb_data_ptr->intent_cb_ntfy = true;
	if (sz == RMT_RX_INTENT_REQ_SIZE) {
		ret = true;
		cb_data_ptr->send_intent = ret;
	}
	return ret;
}

/**
 * ut_tx_worker() - Work function to TX data
 * @work: Pointer to the work structure
 * @returns: void
 *
 * This function transmits a data packet to the remote side
 */
static void ut_tx_worker(struct work_struct *work)
{
	struct ut_intent_req_tx_work *tx_work =
		container_of(work, struct ut_intent_req_tx_work, work);
	tx_work->tx_stat.glink_tx_entered = true;
	glink_tx(tx_work->handle, tx_work->pkt_priv, tx_work->data,
			tx_work->size,
			(tx_work->req_intent ? GLINK_TX_REQ_INTENT : 0));
	tx_work->tx_stat.glink_tx_exited = true;
	complete(&tx_work->tx_worker_complete);
}

/**
 * glink_ut1_mock_rmt_rx_intent_req - Use mock transport to test
 *				remote rx intent request handling
 *				by sending ack and rx intent.
 *
 * @s: pointer to output file
 *
 * This test simulates linkup a transport, accept a remote rx intent
 * and send acknowledgment to the request. Then the test sends a rx
 * intent.
 */
static void glink_ut1_mock_rmt_rx_intent_req(struct seq_file *s)
{
	void *handle = NULL;
	int failed = 0;
	int local_rcid;
	int ret;
	struct glink_mock_xprt *mock_ptr;
	struct glink_mock_cmd *tx_cmd;
	struct glink_open_config open_cfg;
	struct glink_mock_rx_intent *intent;
	struct ut_notify_data cb_data;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open Glink channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
		open_cfg.transport = "mock";
		open_cfg.edge = "local";
		open_cfg.name = "loopback";
		open_cfg.notify_rx =  glink_test_notify_rx;
		open_cfg.notify_rx_intent_req = glink_test_rmt_rx_intent_req_cb;
		open_cfg.notify_tx_done = glink_test_notify_tx_done;
		open_cfg.notify_state = glink_test_notify_state;
		open_cfg.priv = &cb_data;

		cb_data.intent_cb_ntfy = false;
		cb_data.send_intent = false;

		handle = glink_open(&open_cfg);
		UT_ASSERT_ERR_PTR(handle);

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		local_rcid = tx_cmd->remote_open_ack.rcid;
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);

		/* Simulate Remote Intent Request with non-zero size */
		mock_ptr->if_ptr.glink_core_if_ptr->
		rx_cmd_remote_rx_intent_req(&mock_ptr->if_ptr, local_rcid,
					RMT_RX_INTENT_REQ_SIZE);

		UT_ASSERT_INT(1, ==, cb_data.intent_cb_ntfy);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_RX_INTENT_ACK, ==, tx_cmd->type);
		kfree(tx_cmd);

		if (cb_data.send_intent) {
			glink_queue_rx_intent(handle, (void *)&cb_data,
				RMT_RX_INTENT_REQ_SIZE);
			intent = mock_xprt_get_intent();
			UT_ASSERT_PTR(NULL, !=, intent);
			UT_ASSERT_INT(intent->size, ==,
						RMT_RX_INTENT_REQ_SIZE);
			kfree(intent);
		}

		/* Simulate Remote Intent Request with zero size */
		mock_ptr->if_ptr.glink_core_if_ptr->
			rx_cmd_remote_rx_intent_req(&mock_ptr->if_ptr,
							local_rcid, 0);

		UT_ASSERT_INT(1, ==, cb_data.intent_cb_ntfy);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_RX_INTENT_ACK, ==, tx_cmd->type);
		kfree(tx_cmd);
		UT_ASSERT_INT(0, ==, cb_data.send_intent);

		/* Close the Glink Channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);


		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_rx_intent_req - Use mock transport to test
 *				local rx intent request handling
 *				by sending ack and rx intent.
 *
 * @s: pointer to output file
 *
 * This test simulates linkup a transport, accept a remote rx intent
 * and send acknowledgement to the request. Then the test sends a rx
 * intent.
 */
static void glink_ut1_mock_rx_intent_req(struct seq_file *s)
{
	void *handle = NULL;
	int failed = 0;
	int local_rcid;
	int ret;
	struct glink_mock_xprt *mock_ptr;
	struct glink_mock_cmd *tx_cmd;
	struct glink_mock_tx_data *tx_data = NULL;
	struct glink_open_config open_cfg;
	struct ut_intent_req_tx_work *tx_work_ptr;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);

	ut_intent_req_workqueue =
		create_singlethread_workqueue("ut_intent_req");

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open Glink channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
		open_cfg.transport = "mock";
		open_cfg.edge = "local";
		open_cfg.name = "loopback";
		open_cfg.notify_rx =  glink_test_notify_rx;
		open_cfg.notify_rx_intent_req = glink_test_rmt_rx_intent_req_cb;
		open_cfg.notify_tx_done = glink_test_notify_tx_done;
		open_cfg.notify_state = glink_test_notify_state;
		open_cfg.priv = &cb_data;

		cb_data.intent_cb_ntfy = false;
		cb_data.send_intent = false;

		handle = glink_open(&open_cfg);
		UT_ASSERT_ERR_PTR(handle);

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		local_rcid = tx_cmd->remote_open_ack.rcid;
		kfree(tx_cmd);

		/* Populate the G-Link TX context*/
		tx_work_ptr = kmalloc(sizeof(struct ut_intent_req_tx_work),
								GFP_KERNEL);
		UT_ASSERT_PTR(NULL, !=, tx_work_ptr);
		tx_work_ptr->handle = (void *)handle;
		tx_work_ptr->pkt_priv = (void *)&cb_data;
		tx_work_ptr->data = (void *)testdata;
		tx_work_ptr->size = (size_t)strlen(testdata);
		tx_work_ptr->req_intent = true;
		tx_work_ptr->tx_stat.glink_tx_entered = false;
		tx_work_ptr->tx_stat.glink_tx_exited = false;
		init_completion(&tx_work_ptr->tx_worker_complete);
		/* The riid passed below is hardcoded */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
		&mock_ptr->if_ptr, local_rcid, 0x10, (strlen(testdata) - 1));

		/* Queue the glink_tx */
		INIT_WORK(&tx_work_ptr->work, ut_tx_worker);
		queue_work(ut_intent_req_workqueue, &tx_work_ptr->work);

		UT_ASSERT_INT(
			(int)wait_for_completion_timeout(&event, HZ / 2), >, 0);

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(RX_INTENT_REQ, ==, tx_cmd->type);
		kfree(tx_cmd);
		UT_ASSERT_BOOL(true , ==,
				tx_work_ptr->tx_stat.glink_tx_entered);
		UT_ASSERT_BOOL(false , ==,
				tx_work_ptr->tx_stat.glink_tx_exited);
		/* Generate Ack for rx_intent request from local channel */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_rx_intent_req_ack
					(&mock_ptr->if_ptr, local_rcid,	true);

		/* Pass an rx_intent of sufficient size */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
		&mock_ptr->if_ptr, local_rcid, 0x11, strlen(testdata));

		UT_ASSERT_INT(
			(int)wait_for_completion_timeout(&event, HZ / 2), >, 0);

		tx_data = mock_xprt_get_tx_data();
		UT_ASSERT_PTR(NULL, !=, tx_data);
		UT_ASSERT_STRING_COMPARE(testdata, tx_data->data);
		kfree(tx_data);

		wait_for_completion(&tx_work_ptr->tx_worker_complete);
		UT_ASSERT_BOOL(true , ==,
				tx_work_ptr->tx_stat.glink_tx_entered);
		UT_ASSERT_BOOL(true , ==,
				tx_work_ptr->tx_stat.glink_tx_exited);
		kfree(tx_work_ptr);
		/* Close the G-Link Channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);
		flush_workqueue(ut_intent_req_workqueue);
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
	destroy_workqueue(ut_intent_req_workqueue);
}

/**
 * do_mock_negotiation - helper function that does version negotiation
 *
 * @s: Pointer to output file
 * @version: Version to negotiate
 * @features: Feature to negotiate
 *
 * Return: 0 is negotiation successful, != 0 otherwise
 */
static int do_mock_negotiation(struct seq_file *s, uint32_t version,
		uint32_t features)
{
	int failed = 0;
	struct glink_mock_xprt *mock_ptr;
	struct glink_mock_cmd *tx_cmd;

	do {
		mock_ptr = mock_xprt_get();

		/* Bring-up transport and do local negotiation */
		mock_ptr->if_ptr.glink_core_if_ptr->link_up(&mock_ptr->if_ptr);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(0x5, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x55, ==, tx_cmd->version.features);
		kfree(tx_cmd);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version_ack(
				&mock_ptr->if_ptr, version, features);

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(version, ==, tx_cmd->version.version);
		UT_ASSERT_INT(features, ==, tx_cmd->version.features);
		kfree(tx_cmd);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version_ack(
				&mock_ptr->if_ptr, version, features);

		/* do remote negotiation */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version(
				&mock_ptr->if_ptr, version, features);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION_ACK, ==, tx_cmd->type);
		kfree(tx_cmd);

		/* confirm negotiation is now complete */
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(SET_VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(version, ==, tx_cmd->version.version);
		UT_ASSERT_INT(features, ==, tx_cmd->version.features);
		kfree(tx_cmd);
	} while (0);

	return failed;
}

/**
 * glink_ut0_mock_basic - Basic sanity test using local loopback.
 *
 * @s: pointer to output file
 *
 * This test simulates open, read, write and close of glink_channel
 * when remote processor does not exits.
 */
static void glink_ut0_mock_basic(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct glink_mock_rx_intent *intent;
	struct glink_mock_tx_data *tx_data;
	struct glink_core_rx_intent *rx_intent_ptr;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/*
		 * Read the data [hello GLINK]
		 */
		/* 1. first sent intent for testdata length */
		glink_queue_rx_intent(handle, (void *)&cb_data,
				strlen(testdata));
		intent = mock_xprt_get_intent();
		UT_ASSERT_PTR(NULL, !=, intent);
		UT_ASSERT_INT(strlen(testdata), ==, intent->size);

		/* 2. simulate remote write*/
		rx_intent_ptr =
		    mock_ptr->if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
			    &mock_ptr->if_ptr, 1, intent->liid);
		UT_ASSERT_PTR(NULL, ==, (void *)rx_intent_ptr->data);
		rx_intent_ptr->data = (void *)testdata;
		rx_intent_ptr->pkt_size = strlen(testdata);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_put_pkt_ctx(
				&mock_ptr->if_ptr, 1, rx_intent_ptr, true);

		/* 3. check glink_test_notify_rx() callback data*/
		UT_ASSERT_INT(true, ==, cb_data.rx_notify);
		UT_ASSERT_INT(strlen(testdata), ==, cb_data.size);
		kfree(intent);

		/*
		 * Write the data [Hello GLINK]
		 */
		/* 1. Simulate remote intent*/
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
				&mock_ptr->if_ptr, 1, 1, strlen(testdata));
		/* 2. write data */
		ret = glink_tx(handle, (void *)&cb_data, (void *)testdata,
						strlen(testdata), 0);
		UT_ASSERT_INT(ret, ==, 0);

		UT_ASSERT_INT(
			(int)wait_for_completion_timeout(&event, HZ / 2), >, 0);

		tx_data = mock_xprt_get_tx_data();
		UT_ASSERT_PTR(NULL, !=, tx_data);
		UT_ASSERT_STRING_COMPARE(testdata, tx_data->data);
		/*
		 * 3. simulate and check tx_done and
		 * glink_test_notify_tx_done() callback data
		 */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_tx_done(
						&mock_ptr->if_ptr, 1, 1, false);
		UT_ASSERT_INT(true, == , cb_data.tx_done);

		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);


		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

void ut_ch_close_work_func(struct work_struct *work)
{
	struct ut_ch_close_work  *close_work =
		container_of(work, struct ut_ch_close_work, work);
	do {
		wait_for_completion(&close_work->cb_data->cb_completion);

	} while (close_work->cb_data->event != GLINK_REMOTE_DISCONNECTED);

	glink_close(close_work->ch_handle);
}

/**
 * glink_ut0_mock_ssr - Basic SSR test case using mock transport.
 *
 * @s: pointer to output file
 *
 * This test simulates a basic SSR case. A channel is opened, data
 * is transmitted on it, and then SSR occurs. This test specifically tests
 * the functionality of glink_ssr() in the G-Link core. It verifies that
 * after an SSR, channels to the affected subsystem receive a
 * REMOTE_DISCONNECTED notification, and verifies that the expected client
 * response (calling glink_close()) is handled correctly.
 */
static void glink_ut0_mock_ssr(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct glink_mock_rx_intent *intent;
	struct glink_mock_tx_data *tx_data;
	struct glink_core_rx_intent *rx_intent_ptr;
	struct ut_notify_data cb_data;
	struct completion event;
	struct workqueue_struct *ch_close_wq;
	struct ut_ch_close_work *ch_close_work;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);
		ch_close_wq = create_singlethread_workqueue("ssr_ch_close_q");

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);

		glink_queue_rx_intent(handle, (void *)&cb_data,
				strlen(testdata));
		intent = mock_xprt_get_intent();
		UT_ASSERT_PTR(NULL, !=, intent);
		UT_ASSERT_INT(strlen(testdata), ==, intent->size);

		rx_intent_ptr =
		    mock_ptr->if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
			    &mock_ptr->if_ptr, 1, intent->liid);
		UT_ASSERT_PTR(NULL, ==, (void *)rx_intent_ptr->data);
		rx_intent_ptr->data = (void *)testdata;
		rx_intent_ptr->pkt_size = strlen(testdata);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_put_pkt_ctx(
				&mock_ptr->if_ptr, 1, rx_intent_ptr, true);

		UT_ASSERT_INT(true, ==, cb_data.rx_notify);
		UT_ASSERT_INT(strlen(testdata), ==, cb_data.size);
		kfree(intent);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
				&mock_ptr->if_ptr, 1, 1, strlen(testdata));

		ret = glink_tx(handle, (void *)&cb_data, (void *)testdata,
						strlen(testdata), 0);
		UT_ASSERT_INT(ret, ==, 0);

		UT_ASSERT_INT(
			(int)wait_for_completion_timeout(&event, HZ / 2), >, 0);

		tx_data = mock_xprt_get_tx_data();
		UT_ASSERT_PTR(NULL, !=, tx_data);
		UT_ASSERT_STRING_COMPARE(testdata, tx_data->data);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_tx_done(
						&mock_ptr->if_ptr, 1, 1, false);
		UT_ASSERT_INT(true, == , cb_data.tx_done);

		ch_close_work = kzalloc(sizeof(struct ut_ch_close_work),
							GFP_KERNEL);
		ch_close_work->cb_data = &cb_data;
		ch_close_work->ch_handle = handle;
		INIT_WORK(&ch_close_work->work, ut_ch_close_work_func);
		queue_work(ch_close_wq, &ch_close_work->work);
		glink_ssr("local");

		kfree(tx_cmd);
		destroy_workqueue(ch_close_wq);
		kfree(ch_close_work);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		glink_loopback_xprt_link_up();
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_open_close - Basic sanity test using local loopback.
 *
 * @s: pointer to output file
 *
 * This test simulates opening and closing a G-Link channel. When the close is
 * completed, the local close ack is received before the remote close.
 */
static void glink_ut1_mock_open_close(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/* close channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_open_close_remote_first - Basic sanity test using local
 * loopback.
 *
 * @s: pointer to output file
 *
 * This test simulates opening and closing a G-Link channel. In this test,
 * the remote close is received before the local close ack.
 */
static void glink_ut1_mock_open_close_remote_first(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/* close channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_REMOTE_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, 1);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_transmit_no_rx_intent - Transmit a packet on local loopback
 *			without first sending an rx intent.
 *
 * @s: pointer to output file
 *
 * This test simulates sending a packet when no rx intent has been queued.
 */
static void glink_ut1_mock_transmit_no_rx_intent(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct glink_mock_tx_data *tx_data;
	struct ut_notify_data cb_data;
	struct completion event;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);
		init_completion(&event);
		register_completion(&mock_ptr->if_ptr, &event);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/*
		 * Write the data [Hello GLINK]
		 * 1. Do the send, make sure it fails
		 * 2. Then do the intent, and try the send again; it should
		 * succeed
		 */
		/* write data without simulating receipt of an rx intent */
		ret = glink_tx(handle, (void *)&cb_data, (void *)testdata,
						strlen(testdata), 0);
		/* glink_tx returns -EAGAIN if no rx_intent was present */
		UT_ASSERT_INT(ret, ==, -EAGAIN);

		/* 1. Simulate remote intent */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
				&mock_ptr->if_ptr, 1, 1, strlen(testdata));
		/* 2. write data */
		ret = glink_tx(handle, (void *)&cb_data, (void *)testdata,
						strlen(testdata), 0);
		/*
		 * 3. This time the write will succeed since an rx_intent was
		 * received.
		 */
		UT_ASSERT_INT(ret, ==, 0);
		UT_ASSERT_INT(
			(int)wait_for_completion_timeout(&event, HZ / 2), >, 0);

		tx_data = mock_xprt_get_tx_data();
		UT_ASSERT_PTR(NULL, !=, tx_data);
		UT_ASSERT_STRING_COMPARE(testdata, tx_data->data);
		/*
		 * 3. simulate and check tx_done and
		 * glink_test_notify_tx_done() callback data
		 */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_tx_done(
						&mock_ptr->if_ptr, 1, 1, false);
		UT_ASSERT_INT(true, == , cb_data.tx_done);

		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);


		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	unregister_completion(&event);
	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_size_greater_than - Transmit a packet of size one byte
 * greater than expected.
 *
 * @s: pointer to output file
 *
 * This test simulates open, write and close of glink_channel
 * on the mock loopback transport. In particular, this test is for
 * the case in which a packet is sent that has one more byte than the
 * receiver expects.
 */
static void glink_ut1_mock_size_greater_than(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct ut_notify_data cb_data;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/*
		 * Write the data [Hello GLINK]
		 */
		/* 1. Simulate remote intent for size one less than needed */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
				&mock_ptr->if_ptr, 1, 1, strlen(testdata - 1));
		/*
		 *  2. write data - send one more byte than expected - glink_tx
		 *  returns -EAGAIN if remote side has not provided a receive
		 *  intent that is big enough
		 */
		ret = glink_tx(handle, (void *)&cb_data, (void *)testdata,
						strlen(testdata), 0);
		UT_ASSERT_INT(ret, ==, -EAGAIN);

		/* Close channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);


		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut1_mock_read_no_rx_intent - Use mock transport to test
 *				failure case of attempting to read data
 *				without first queueing an rx intent.
 *
 * @s: pointer to output file
 *
 * This test simulates open, read, and close. First, a read is attempted
 * without first queueing an rx_intent. This fails. Then a second read is
 * attempted after queueing an rx_intent. This succeeds.
 */
static void glink_ut1_mock_read_no_rx_intent(struct seq_file *s)
{
	int failed = 0;
	int ret;
	void *handle = NULL;
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct glink_mock_rx_intent *intent;
	struct glink_core_rx_intent *rx_intent_ptr;
	struct ut_notify_data cb_data;

	GLINK_STATUS(s, "Running %s\n", __func__);

	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		cb_data_init(&cb_data);

		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
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

		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE("loopback", tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, 1, "loopback",
					MOCK_XPRT_ID);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		/*
		 * Read the data. Simulate remote attempting to write without
		 * queuing an rx intent and confirm that we cannot retrieve the
		 * RX intent.
		 */
		rx_intent_ptr =
		    mock_ptr->if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
			    &mock_ptr->if_ptr, 1, 0x1234);
		UT_ASSERT_PTR(NULL, ==, rx_intent_ptr);

		/* 3. check glink_test_notify_rx() callback data */
		UT_ASSERT_INT(false, ==, cb_data.rx_notify);

		/*
		 * Try again with rx intent queued and verify success
		 */
		/* 1. first, send intent for testdata length */
		glink_queue_rx_intent(handle, (void *)&cb_data,
				strlen(testdata));
		intent = mock_xprt_get_intent();
		UT_ASSERT_PTR(NULL, !=, intent);
		UT_ASSERT_INT(strlen(testdata), ==, intent->size);
		/* 2. simulate remote write of more data than we asked for */
		rx_intent_ptr =
		    mock_ptr->if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
			    &mock_ptr->if_ptr, 1, intent->liid);
		UT_ASSERT_PTR(NULL, ==, (void *)rx_intent_ptr->data);
		rx_intent_ptr->data = (void *)testdata;
		rx_intent_ptr->pkt_size = strlen(testdata);
		mock_ptr->if_ptr.glink_core_if_ptr->rx_put_pkt_ctx(
				&mock_ptr->if_ptr, 1, rx_intent_ptr, true);

		/* 3. check glink_test_notify_rx() callback data*/
		UT_ASSERT_INT(true, ==, cb_data.rx_notify);
		UT_ASSERT_INT(strlen(testdata), ==, cb_data.size);
		kfree(intent);

		/* Close channel */
		ret = glink_close(handle);
		UT_ASSERT_INT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);


		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, 1);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(1, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

/**
 * glink_ut0_mock_remote_negotiation - Use mock transport to test
 *				remote negotiation by sending
 *				different versio and features.
 *
 * @s: pointer to output file
 *
 * This test simulates linkup a transport and read the local supported
 * version and feature and send different remote versions and fetures
 * to validate the remote negotiation.
 */
static void glink_ut0_mock_remote_negotiation(struct seq_file *s)
{
	int failed = 0;
	struct glink_mock_xprt *mock_ptr;
	struct glink_mock_cmd *tx_cmd;

	GLINK_STATUS(s, "Running %s\n", __func__);
	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);

		/* Bring-up transport and do local negotiation */
		mock_ptr->if_ptr.glink_core_if_ptr->link_up(&mock_ptr->if_ptr);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(0x5, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x55, ==, tx_cmd->version.features);
		kfree(tx_cmd);

		/*
		 * check with version which is not supported by local Glink
		 * core
		 */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version(
				&mock_ptr->if_ptr,
				0x6, 0x66);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(0x6, >=, tx_cmd->version.version);
		UT_ASSERT_INT(0x66, >=, tx_cmd->version.features);
		kfree(tx_cmd);

		mock_xprt_reset(mock_ptr);
		/* Check with supported versioni but different features*/
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version(
				&mock_ptr->if_ptr,
				0x2, 0x33);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(0x2, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x22, ==, tx_cmd->version.features);
		kfree(tx_cmd);

		mock_xprt_reset(mock_ptr);
		/* Check with supported version and features*/
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version(
				&mock_ptr->if_ptr,
				0x2, 0x22);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT(0x2, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x22, ==, tx_cmd->version.features);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version_ack(
				&mock_ptr->if_ptr,
				0x2, 0x22);
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(0x2, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x22, ==, tx_cmd->version.features);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_version_ack(
				&mock_ptr->if_ptr,
				0x2, 0x22);
		/* confirm negotiation is now complete */
		tx_cmd = mock_xprt_get_next_cmd();
		UT_ASSERT_PTR(NULL, !=, tx_cmd);
		UT_ASSERT_INT(SET_VERSION, ==, tx_cmd->type);
		UT_ASSERT_INT(0x2, ==, tx_cmd->version.version);
		UT_ASSERT_INT(0x22, ==, tx_cmd->version.features);
		kfree(tx_cmd);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "\tFailed\n");
}

/**
 * glink_ut_smem_ssr - WCNSS SSR test
 * @s: pointer to output file
 *
 * Return: Nothing
 *
 * This function triggers SSR for WCNSS, which notifies the modem and triggers
 * local cleanup. The function then verifies that all responses to SSR
 * notifications have been received successfully.
 */
static void glink_ut_smem_ssr(struct seq_file *s)
{
	int failed = 0;
	unsigned timeout_mult = 5;

	GLINK_STATUS(s, "Running %s\n", __func__);
	do {
		subsystem_restart("wcnss");
		UT_ASSERT_BOOL(true, ==,
				glink_ssr_wait_cleanup_done(timeout_mult));

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "\tFailed\n");
}

/**
 * ut_loopback_stress_test - Stress test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to stress test on the transport by doing
 * multiple reads and writes with same data.
 */
static int ut_loopback_stress_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	struct req pkt;
	int ret, i;
	int len = strlen(testdata);
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)*data_handle;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);

	pkt.payload.q_rx_int_conf.random_delay = 0;
	pkt.payload.q_rx_int_conf.delay_ms = 0;
	pkt.payload.q_rx_int_conf.num_intents = ut_iterations;
	pkt.payload.q_rx_int_conf.intent_size = len;
	strlcpy(pkt.payload.q_rx_int_conf.ch_name,
			lpb_ch->open_cfg.name,
			strlen(lpb_ch->open_cfg.name) + 1);
	pkt.payload.q_rx_int_conf.name_len =
			strlen(lpb_ch->open_cfg.name);
	ret = glink_loopback_send_request(cntl_handle, &pkt,
					  QUEUE_RX_INTENT_CONFIG, true);
	if (ret) {
		GLINK_UT_ERR("%s: glink_loopback_send_request failed\n",
				__func__);
		return ret;
	}

	for (i = 0; i < ut_iterations; i++) {
		if (i == 0)
			ret = glink_loopback_tx(*data_handle, (void *)testdata,
					len, true, true, lpb_ch->rx_reuse);
		else
			ret = glink_loopback_tx(*data_handle, (void *)testdata,
					len, false, false, lpb_ch->rx_reuse);
		if (ret) {
			GLINK_UT_ERR(
				"%s:%s:%s %s: glink_loopback_tx failed\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name, __func__);
			return ret;
		}
	}

	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;

}

/**
 * ut_loopback_performance_test - Perfromance test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to do performance test on the transport by
 * doing multiple reads and writes with different sizes of data.
 */
static int ut_loopback_performance_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	uint32_t size;
	char *data = NULL;
	struct req pkt;
	int ret, i;
	uint64_t t_start, t_end;
	uint64_t rtt_us, throughput_kBps;
	unsigned long rem;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)*data_handle;
	int len = strlen(lpb_ch->open_cfg.name);
	int ut_iterations_temp = ut_iterations * 10;
	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations_temp);

	GLINK_STATUS(s, "Size\tIterations\tRTT(us)\tThroughput(KB/s)\n");
	for (size = 1; size <= SZ_8K; size = size * 2) {
		pkt.payload.q_rx_int_conf.random_delay = 0;
		pkt.payload.q_rx_int_conf.delay_ms = 0;
		pkt.payload.q_rx_int_conf.num_intents = ut_iterations_temp;
		pkt.payload.q_rx_int_conf.intent_size = size;
		strlcpy(pkt.payload.q_rx_int_conf.ch_name,
				lpb_ch->open_cfg.name, len + 1);
		pkt.payload.q_rx_int_conf.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt,
					QUEUE_RX_INTENT_CONFIG, true);
		if (ret) {
			GLINK_UT_ERR("%s: glink_loopback_send_request failed\n",
					__func__);
			return ret;
		}

		data = kzalloc(size, GFP_KERNEL);
		if (!data) {
			GLINK_UT_ERR("%s: No memory for allocation\n",
					__func__);
			return -ENOMEM;
		}
		memset(data, size, size);
		t_start = sched_clock();
		for (i = 0; i < ut_iterations_temp; i++) {
			if (i == 0)
				ret = glink_loopback_tx(*data_handle, data,
						size, true, true , false);
			else
				ret = glink_loopback_tx(*data_handle, data,
						size, false, true, false);

			if (ret) {
				GLINK_UT_ERR(
				"%s:%s:%s %s: glink_loopback_tx failed\n",
					lpb_ch->open_cfg.transport,
					lpb_ch->open_cfg.edge,
					lpb_ch->open_cfg.name, __func__);
				return ret;
			}
		}
		t_end = sched_clock();
		rtt_us = t_end - t_start;

		rem = do_div(rtt_us, (1000 * ut_iterations_temp));

		/*
		 * Convert B / us to KB/s
		 *
		 * B    10^6 us    1 KB     10^6 KB    15625 KB
		 * -- * ------- * ------- = ------- =  --------
		 * us     1 s      1024 B    1024 s       16 S
		 */
		throughput_kBps = (15625 * size * 2);
		rem = do_div(throughput_kBps, (16 * rtt_us));

		GLINK_STATUS(s, "%u\t%u\t\t%u\t%u\n", size, ut_iterations_temp,
			     (unsigned int)rtt_us,
			     (unsigned int)throughput_kBps);
		kfree(data);
	}
	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;

}

/**
 * ut_loopback_open_close_test - Re-open test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to do open and close test on the transport by
 * doing multiple times open and close of same channel.
 */
static int ut_loopback_open_close_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	int i;
	int ret;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)*data_handle;
	const char *transport = lpb_ch->open_cfg.transport;
	const char *edge = lpb_ch->open_cfg.edge;
	const char *name = lpb_ch->open_cfg.name;
	int len = strlen(name);
	struct req pkt;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);

	for (i = 0; i < ut_iterations; i++) {

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name, len + 1);
		pkt.payload.close.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		if (ret) {
			GLINK_UT_INFO("%s: remote close fail ret[%d]\n",
					__func__, ret);
			return ret;
		}

		ret = glink_loopback_close(*data_handle);
		*data_handle = NULL;
		if (ret) {
			GLINK_UT_INFO("%s:%s:%s %s: close fail ret[%d]\n",
				transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				__func__, ret);
			return ret;
		}

		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, name, len + 1);
		pkt.payload.open.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
				true);
		if (ret) {
			GLINK_UT_INFO("%s: remote close fail ret[%d]\n",
					__func__, ret);
			return ret;
		}

		*data_handle = glink_loopback_open(transport, edge, name,
				CH_DATA_TYPE, LINEAR_RX, false);
		if (!(*data_handle)) {
			GLINK_UT_INFO("%s: open fail ret[%d]\n", __func__, ret);
			return -ENODEV;
		}
	}
	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;
}

/**
 * ut_loopback_queue_rx_intent_test - Queue rx intent test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to queue muliple rx intent on data channel and
 * repeat same on every re-open.
 */
static int ut_loopback_queue_rx_intent_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	int i;
	uint32_t size;
	int ret;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)*data_handle;
	const char *transport = lpb_ch->open_cfg.transport;
	const char *edge = lpb_ch->open_cfg.edge;
	const char *name = lpb_ch->open_cfg.name;
	int len = strlen(name);
	struct req pkt;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);

	for (i = 0; i < ut_iterations; i++) {
		lpb_ch = (struct loopback_channel *)*data_handle;
		for (size = 1; size <= SZ_8K; size = size * 2) {
			ret = glink_queue_rx_intent(lpb_ch->handle,
					(void *)lpb_ch, size);
			if (ret) {
				GLINK_UT_ERR("%s: %s ret[%d]\n", __func__,
						"glink_queue_rx_intent failed",
						ret);
				return ret;
			}
		}

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name, len + 1);
		pkt.payload.close.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		if (ret) {
			GLINK_UT_INFO("%s: remote close fail ret[%d]\n",
					__func__, ret);
			return ret;
		}

		ret = glink_loopback_close(*data_handle);
		*data_handle = NULL;
		if (ret) {
			GLINK_UT_INFO("%s:%s:%s %s: close fail ret[%d]\n",
					transport, edge, name, __func__, ret);
			return ret;
		}

		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, name, len + 1);
		pkt.payload.open.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
				true);
		if (ret) {
			GLINK_UT_INFO("%s: remote close fail ret[%d]\n",
					__func__, ret);
			return ret;
		}

		*data_handle = glink_loopback_open(transport, edge, name,
				CH_DATA_TYPE, LINEAR_RX, false);
		if (!(*data_handle)) {
			GLINK_UT_INFO("%s:%s:%s %s: open fail ret[%d]\n",
					transport, edge, name, __func__, ret);
			return -ENODEV;
		}
	}
	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;
}


/**
 * ut_loopback_tx_with_req_intent_test - Tx with req_intent test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to transmit data with with no intent queue by remote
 * client.
 */
static int ut_loopback_tx_with_req_intent_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	uint32_t size;
	char *data = NULL;
	int ret;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);
	for (size = 1; size <= SZ_8K; size = size * 2) {

		data = kzalloc(size, GFP_KERNEL);
		if (!data) {
			GLINK_UT_ERR("%s: No memory for allocation\n",
					__func__);
			return -ENOMEM;
		}
		memset(data, size, size);
		ret = glink_loopback_tx(*data_handle, data, size, true, true,
									false);
		if (ret) {
			GLINK_UT_ERR("%s: glink_loopback_tx failed\n",
					__func__);
			return ret;
		}
		kfree(data);
	}
	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;

}

static void *ut_skb_vbuf_provider(void *iovec, size_t offset, size_t *buf_size)
{
	struct sk_buff *skb;
	struct sk_buff_head *skb_head = (struct sk_buff_head *)iovec;
	size_t temp_size = 0;

	*buf_size = 0;
	skb_queue_walk(skb_head, skb) {
		temp_size += skb->len;
		if (offset >= temp_size)
			continue;

		*buf_size = temp_size - offset;
		return (void *)skb->data + skb->len - *buf_size;
	}
	return NULL;
}

static void free_skb_vector(struct sk_buff_head *skb_head)
{
	struct sk_buff *skb;

	while (!skb_queue_empty(skb_head)) {
		skb = skb_dequeue(skb_head);
		kfree_skb(skb);
	}
	kfree(skb_head);
}

static struct sk_buff_head *create_skb_vector(int buf_count,
					int buf_len, char *data)
{
	struct sk_buff_head *skb_head;
	struct sk_buff *skb;
	void *dest_data;
	int i;

	skb_head = kmalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	if (!skb_head) {
		GLINK_UT_ERR("%s: Failed to allocate skb_head\n", __func__);
		return NULL;
	}
	skb_queue_head_init(skb_head);

	for (i = 0; i < buf_count; i++) {
		skb = alloc_skb(buf_len, GFP_KERNEL);
		if (!skb) {
			GLINK_UT_ERR("%s: Error allocating SKB@%d iteration\n",
				     __func__, i);
			goto out_create_skb_vector;
		}
		dest_data = skb_put(skb, buf_len);
		memcpy(skb->data, data, buf_len);
		skb_queue_tail(skb_head, skb);
	}
	return skb_head;
out_create_skb_vector:
	free_skb_vector(skb_head);
	return NULL;
}

/**
 * ut_loopback_skb_stress_test - SKB Stress test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to perform vectored stress test on the transport by doing
 * multiple SKB reads and writes with same data.
 */
static int ut_loopback_skb_stress_test(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	struct req pkt;
	int ret, i;
	int len = strlen(testdata);
	int buf_count = ut_vector_buf_count;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)*data_handle;
	struct sk_buff_head *skb_head;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);
	skb_head = create_skb_vector(buf_count, len, (char *)testdata);
	if (!skb_head) {
		GLINK_UT_INFO("%s: Error creating skb vector\n", __func__);
		return -ENOMEM;
	}
	pkt.payload.q_rx_int_conf.random_delay = 0;
	pkt.payload.q_rx_int_conf.delay_ms = 0;
	pkt.payload.q_rx_int_conf.num_intents = ut_iterations;
	pkt.payload.q_rx_int_conf.intent_size = buf_count * len;
	strlcpy(pkt.payload.q_rx_int_conf.ch_name,
			lpb_ch->open_cfg.name,
			strlen(lpb_ch->open_cfg.name) + 1);
	pkt.payload.q_rx_int_conf.name_len =
			strlen(lpb_ch->open_cfg.name);
	ret = glink_loopback_send_request(cntl_handle, &pkt,
					  QUEUE_RX_INTENT_CONFIG, true);
	if (ret) {
		GLINK_UT_ERR("%s: glink_loopback_send_request failed\n",
				__func__);
		free_skb_vector(skb_head);
		return ret;
	}

	for (i = 0; i < ut_iterations; i++) {
		ret = glink_loopback_txv(*data_handle, (void *)skb_head,
					 buf_count * len, ut_skb_vbuf_provider,
					 NULL, false);
		if (ret) {
			GLINK_UT_ERR(
				"%s:%s:%s %s: glink_loopback_tx failed\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name, __func__);
			return ret;
		}
	}

	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;

}

/**
 * ut_loopback_sigs - Signal test
 *
 * @s: pointer to output file
 * @handle: handle returned by glink_open() for loopback control channel
 * @data_handle: handle returned by glink_open() for loopback data channel
 *
 * This function used to test signals of the channel by
 * set signal and remote get signal.
 */
static int ut_loopback_sigs(struct seq_file *s,
				void *cntl_handle, void **data_handle)
{
	int ret;
	uint32_t i;

	GLINK_UT_INFO("%s: start for iterations[%d]\n", __func__,
			ut_iterations);

	for (i = 0; i < ut_iterations; i++) {
		ret = glink_loopback_sigs_set(*data_handle, i);
		if (ret) {
			GLINK_UT_ERR("%s: glink_loopback_sigs_set failed\n",
					__func__);
			return ret;
		}
	}
	GLINK_UT_INFO("%s: END\n", __func__);
	return 0;

}

/**
* glink_ut_local_basic_core - The loopback core function.
*
* @s: pointer to output file
* @xprt_id:
* @name:  test name
* @test_type: Type of the test to run
*
* This test simulates a simple write and read
* when remote processor does not exist.
*/
static void glink_ut_local_basic_core(struct seq_file *s)
{
	struct glink_dbgfs_data *dfs_d;
	struct glink_ut_dbgfs *ut_dfs_d;
	int failed = 0;
	int ret;
	void *cntl_handle = NULL;
	void *data_handle = NULL;
	char *cntl_ch_name;
	char *data_ch_name;
	bool b_ch_mem_alloc = false;
	unsigned int cntl_ch_len;
	unsigned int data_ch_len;
	char edge[GLINK_NAME_SIZE] = {0};
	const char *cntl_ch_pfix = "LOOPBACK_CTL_";
	const char *data_ch_pfix = "LOOPBACK_DATA_";

	int len = 0;
	struct req pkt;
	int rx_type;
	testfunc do_test;
	bool rx_reuse = false;

	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	do_test = ut_dfs_d->tfunc;
	if (!strcmp(ut_dfs_d->ut_name, "ut0_skb_basic")
		|| !strcmp(ut_dfs_d->ut_name, "ut0_skb_stress"))
		rx_type = LINEAR_VECTOR_RX;
	else
		rx_type = LINEAR_RX;
	if (!strcmp(ut_dfs_d->ut_name, "ut1_rx_reuse")
		|| !strcmp(ut_dfs_d->ut_name, "ut1_smem_rx_reuse"))
		rx_reuse = true;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);

	do {
		if (!strcmp(ut_dfs_d->xprt_name, "lloop")) {
			data_ch_name = "LOCAL_DATA_LOOP_CLNT";
			cntl_handle = glink_loopback_open("lloop",
						"local",
						"LOCAL_LOOPBACK_CLNT",
						CH_CNTL_TYPE, LINEAR_RX, false);
		} else {
			glink_ut_to_upper(edge, ut_dfs_d->edge_name);
			cntl_ch_len = GLINK_NAME_SIZE + strlen(cntl_ch_pfix);
			cntl_ch_name = kzalloc(cntl_ch_len,
						GFP_KERNEL);
			snprintf(cntl_ch_name, cntl_ch_len,
					"%s%s", cntl_ch_pfix, edge);
			data_ch_len = GLINK_NAME_SIZE + strlen(data_ch_pfix);
			data_ch_name = kzalloc(data_ch_len,
						GFP_KERNEL);
			snprintf(data_ch_name, data_ch_len,
					"%s%s", data_ch_pfix, edge);
			b_ch_mem_alloc = true;
			cntl_handle = glink_loopback_open(ut_dfs_d->xprt_name,
							ut_dfs_d->edge_name,
							cntl_ch_name,
							CH_CNTL_TYPE,
							LINEAR_RX, false);
		}
		UT_ASSERT_ERR_PTR(cntl_handle);

		len = strlen(data_ch_name);
		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, data_ch_name, len + 1);
		pkt.payload.open.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
				true);
		UT_ASSERT_INT(ret, ==, 0);

		pkt.payload.tx_conf.random_delay = 0;
		pkt.payload.tx_conf.delay_ms = 0;
		pkt.payload.tx_conf.echo_count = 1;
		pkt.payload.tx_conf.transform_type = NO_TRANSFORM;
		strlcpy(pkt.payload.tx_conf.ch_name, data_ch_name, len + 1);
		pkt.payload.tx_conf.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, TX_CONFIG,
				true);
		UT_ASSERT_INT(ret, ==, 0);

		pkt.payload.rx_done_conf.random_delay = 0;
		pkt.payload.rx_done_conf.delay_ms = 0;
		strlcpy(pkt.payload.rx_done_conf.ch_name,
				data_ch_name, len + 1);
		pkt.payload.rx_done_conf.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt,
							RX_DONE_CONFIG, true);
		UT_ASSERT_INT(ret, ==, 0);

		/* data channel */
		if (!strcmp(ut_dfs_d->xprt_name, "lloop"))
			data_handle = glink_loopback_open("lloop",
							"local",
							"LOCAL_DATA_LOOP_CLNT",
							CH_DATA_TYPE, rx_type,
							rx_reuse);
		else
			data_handle = glink_loopback_open(ut_dfs_d->xprt_name,
							ut_dfs_d->edge_name,
							data_ch_name,
							CH_DATA_TYPE, rx_type,
							rx_reuse);
		UT_ASSERT_ERR_PTR(data_handle);

		ret = do_test(s, cntl_handle, &data_handle);
		UT_ASSERT_INT(ret, ==, 0);

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, data_ch_name, len + 1);
		pkt.payload.close.name_len = len;
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT(ret, ==, 0);

		ret = glink_loopback_close(data_handle);
		UT_ASSERT_INT(ret, ==, 0);
		data_handle = NULL;

		/* Close the control Channel */
		ret = glink_loopback_close(cntl_handle);
		UT_ASSERT_INT(ret, ==, 0);
		cntl_handle = NULL;

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed) {
		GLINK_STATUS(s, "\tFailed\n");
		glink_loopback_close(data_handle);
		glink_loopback_close(cntl_handle);
	}

	if (b_ch_mem_alloc) {
		kfree(cntl_ch_name);
		kfree(data_ch_name);
	}
}

/**
 * glink_ut_mt_open_ctl_ch() - Construct the control channel name & open it
 * @edge: Edge on which the control channel has to be opened.
 * @xprt: Transport on which the control channel has to be opened.
 *
 * @return: Return the value as returned by glink_open()
 *
 * This function is used by multi-threaded tests to construct a control
 * channel name depending on the edge and then open that channel.
 */
static void *glink_ut_mt_open_ctl_ch(char *edge, char *xprt)
{
	char ch_name[GLINK_NAME_SIZE];
	char ch_name_suffix[GLINK_NAME_SIZE];

	if (!strcmp(xprt, "lloop"))
		return glink_loopback_open(xprt, "local",
				"LOCAL_LOOPBACK_CLNT", CH_CNTL_TYPE, LINEAR_RX,
				false);

	strlcpy(ch_name, "LOOPBACK_CTL_", GLINK_NAME_SIZE);
	glink_ut_to_upper(ch_name_suffix, edge);
	ch_name_suffix[strlen(edge)] = '\0';
	strlcat(ch_name, ch_name_suffix, GLINK_NAME_SIZE);
	return glink_loopback_open(xprt, edge, ch_name, CH_CNTL_TYPE,
				   LINEAR_RX, false);
}

/**
 * glink_ut_mt_send_config_reqs() - Send Configuration requests for data
 *                                  channels by a multi-threaded test
 * @s: Pointer to output file.
 * @cntl_handle: Handle to the control channel.
 * @mt_thread_info: Configuration info specific to the thread/client.
 *
 * @return: 0 on success, 1 on failure.
 *
 * This function is used by the multi-threaded test to send configuration
 * requests for data channel to be used by a client/thread of the multi-
 * threaded test.
 */
static int glink_ut_mt_send_config_reqs(struct seq_file *s,
	void *cntl_handle, struct mt_test_info_struct *mt_thread_info)
{
	struct req pkt;
	int ret;
	int failed = 0;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)cntl_handle;
	struct ch_config *config = &mt_thread_info->ch_cfg;
	const char *cntl_transport = lpb_ch->open_cfg.transport;
	const char *cntl_edge = lpb_ch->open_cfg.edge;
	const char *cntl_name = lpb_ch->open_cfg.name;
	void *data_handle = NULL;

	do {
		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, config->ch_name,
			GLINK_NAME_SIZE);
		pkt.payload.open.name_len = strlen(config->ch_name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
						  true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.tx_conf.random_delay = 0;
		pkt.payload.tx_conf.delay_ms = config->tx_delay_ms;
		pkt.payload.tx_conf.echo_count = 1;
		pkt.payload.tx_conf.transform_type = NO_TRANSFORM;
		strlcpy(pkt.payload.tx_conf.ch_name, config->ch_name,
			GLINK_NAME_SIZE);
		pkt.payload.tx_conf.name_len = strlen(config->ch_name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, TX_CONFIG,
						  true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG("%s:%s:%s %s: %s %s, thread: %d\n",
			     cntl_transport, cntl_edge, cntl_name, __func__,
			     "Sent TX_CONFIG request for data channel",
			     config->ch_name, current->pid);

		pkt.payload.rx_done_conf.random_delay = 0;
		pkt.payload.rx_done_conf.delay_ms = 0;
		strlcpy(pkt.payload.rx_done_conf.ch_name, config->ch_name,
			GLINK_NAME_SIZE);
		pkt.payload.rx_done_conf.name_len = strlen(config->ch_name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
						  RX_DONE_CONFIG, true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG("%s:%s:%s %s: %s %s, thread: %d\n",
			     cntl_transport, cntl_edge, cntl_name, __func__,
			     "Sent RX_DONE_CONFIG request for data channel",
			     config->ch_name, current->pid);

		/*
		 * Open data channel here, before sending the
		 * QUEUE_RX_INTENT_CONFIG request.
		 */
		data_handle = glink_loopback_open(cntl_transport, cntl_edge,
					config->ch_name, CH_DATA_TYPE,
					LINEAR_RX_NOWAIT, false);
		UT_ASSERT_ERR_PTR_MT(data_handle);
		mt_thread_info->data_handle = data_handle;

		GLINK_UT_DBG("%s: start for iterations[%d]\n", __func__,
			     config->num_pkts);
		pkt.payload.q_rx_int_conf.random_delay = 0;
		pkt.payload.q_rx_int_conf.delay_ms = 0;
		pkt.payload.q_rx_int_conf.num_intents = config->num_pkts;
		pkt.payload.q_rx_int_conf.intent_size = config->data_len;
		strlcpy(pkt.payload.q_rx_int_conf.ch_name, config->ch_name,
			GLINK_NAME_SIZE);
		pkt.payload.q_rx_int_conf.name_len = strlen(config->ch_name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
					QUEUE_RX_INTENT_CONFIG, true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG("%s:%s:%s %s: %s %s, thread: %d\n",
			     cntl_transport, cntl_edge, cntl_name, __func__,
			     "Sent QUEUE_RX_INTENT_CONFIG request for data channel",
			     config->ch_name, current->pid);
	} while (0);

	if (failed) {
		glink_loopback_close(data_handle);
		ret = 1;
	}

	return ret;
}

/**
 * init_mt_cb_data() - Initialize callback data for multi-threaded tests
 * @cb_data: Pointer to the multi-thread callback data.
 */
static void init_mt_cb_data(struct mt_cb_data *cb_data)
{
	init_waitqueue_head(&cb_data->test_comp);
	atomic_set(&cb_data->num_tx_pkts, 0);
	atomic_set(&cb_data->num_rx_pkts, 0);
}

#define glink_ut_mt_wait_cb_data(cb_data) \
	wait_event_interruptible((cb_data)->test_comp, \
		atomic_read(&((cb_data)->num_tx_pkts)) == \
		atomic_read(&((cb_data)->num_rx_pkts)))

/**
 * construct_ch_name() - Construct data channel name for multi-threaded tests
 * @dest: Destination Buffer where the channel name will be put.
 * @prefix: Prefix of the channel name.
 * @index: Index of the channel name.
 */
static void construct_ch_name(char *dest, char *prefix, int index)
{
	char *suffix = "_CLNT";

	scnprintf(dest, GLINK_NAME_SIZE, "%s%d%s", prefix, index, suffix);
}

/**
 * ut_mt_worker_func() - Worker function for multi-threaded test
 * @work: Work structure containing information about the work to be done.
 */
static void ut_mt_worker_func(struct work_struct *work)
{
	int failed = 0;
	int ret;
	int i;
	void *data_handle = NULL;
	struct seq_file *s;
	struct mt_test_info_struct *mt_thread_info;
	struct loopback_channel *lpb_ch;
	const char *transport;
	const char *edge;
	const char *name;
	struct ch_config *config;
	bool req_intent = true;
	void *data = NULL;
	struct mt_cb_data cb_data;
	struct req pkt;

	mt_thread_info = container_of(work,
			struct mt_test_info_struct, work);
	config = &mt_thread_info->ch_cfg;
	data_handle = mt_thread_info->data_handle;
	s = mt_thread_info->s;
	name = mt_thread_info->name;
	lpb_ch = (struct loopback_channel *)mt_thread_info->cntl_handle;
	transport = lpb_ch->open_cfg.transport;
	edge = lpb_ch->open_cfg.edge;

	init_mt_cb_data(&cb_data);
	do {
		data = kmalloc(config->data_len, GFP_KERNEL);
		UT_ASSERT_PTR_MT(data, !=, NULL);
		memset(data, mt_thread_info->index, config->data_len);

		for (i = 0; i < config->num_pkts; i++) {
			ret = glink_loopback_tx_nowait(data_handle,
					data, config->data_len, req_intent,
					&cb_data);
			UT_ASSERT_INT_MT(ret, ==, 0);
			if (failed)
				break;
			GLINK_UT_DBG("%s:%s:%s %s: %s iteration: %d, tid:%d\n",
				     transport, edge, name, __func__,
				     "Called glink_loopback_tx(), ",
				     i, current->pid);
			if (config->pkt_ntrvl)
				msleep(config->pkt_ntrvl);
			req_intent = false;
		}
		if (failed)
			break;
		glink_ut_mt_wait_cb_data(&cb_data);
	} while (0);

	kfree(data);
	if (failed)
		*mt_thread_info->result = 1;

	/* Close the data handle, but not the control handle */
	pkt.payload.close.delay_ms = 0;
	strlcpy(pkt.payload.close.ch_name, name, GLINK_NAME_SIZE);
	pkt.payload.close.name_len = strlen(name);
	ret = glink_loopback_send_request((void *)lpb_ch, &pkt, CLOSE, true);
	ret = glink_loopback_close(data_handle);

	/* Wakeup the main thread */
	GLINK_UT_DBG("%s: Before dec, mt_test_num_workers = %d. Tid: %d\n",
		     __func__, atomic_read(mt_thread_info->num_workers),
		     current->pid);
	atomic_dec(mt_thread_info->num_workers);
	GLINK_UT_DBG("%s: After dec, mt_test_num_workers = %d. Tid: %d\n",
		     __func__, atomic_read(mt_thread_info->num_workers),
		     current->pid);
	wake_up(mt_thread_info->waitqueue);
}

/**
 * glink_ut_mt_test_core() - Multi-threaded test core
 * @s: Output file to write to.
 */
static void glink_ut_mt_test_core(struct seq_file *s)
{
	struct glink_dbgfs_data *dfs_d;
	struct glink_ut_dbgfs *ut_dfs_d;
	testfunc do_test;
	void *cntl_handle;
	atomic_t mt_test_num_workers = ATOMIC_INIT(0);
	wait_queue_head_t waitqueue;
	int i = 0, j;
	int ret;
	int failed = 0;
	int result = 0;
	struct req pkt;

	uint32_t pkt_ntrvl = (uint32_t)ut_mt_lat_pkt_ntrvl_ms;
	uint32_t num_lat_clnts = (uint32_t)ut_mt_num_lat_clnts;
	uint32_t num_tput_clnts = (uint32_t)ut_mt_num_tput_clnts;
	uint32_t num_pkts_lat_ch = (uint32_t)ut_mt_num_pkts_lat_ch;
	uint32_t num_pkts_tput_ch = (uint32_t)ut_mt_num_pkts_tput_ch;
	size_t pkt_size_lat_ch = (uint32_t)ut_mt_pkt_size_lat_ch;
	size_t pkt_size_tput_ch = (uint32_t)ut_mt_pkt_size_tput_ch;

	struct workqueue_struct **mt_test_wq = NULL;
	struct mt_test_info_struct *mt_thread_info = NULL;

	GLINK_STATUS_MT(s, "Running %s\n", __func__);
	GLINK_INFO_PERF("%s config: %u, %u, %u, %u, %u, %zu, %zu\n", __func__,
		pkt_ntrvl, num_lat_clnts, num_tput_clnts, num_pkts_lat_ch,
		num_pkts_tput_ch, pkt_size_lat_ch, pkt_size_tput_ch);

	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	do_test = ut_dfs_d->tfunc;
	init_waitqueue_head(&waitqueue);
	do {
		cntl_handle = glink_ut_mt_open_ctl_ch(ut_dfs_d->edge_name,
						      ut_dfs_d->xprt_name);
		UT_ASSERT_ERR_PTR_MT(cntl_handle);

		atomic_set(&mt_test_num_workers,
			   (num_lat_clnts + num_tput_clnts));

		mt_thread_info = kzalloc(sizeof(*mt_thread_info) *
					(num_lat_clnts + num_tput_clnts),
					GFP_KERNEL);
		for (i = 0; i < num_lat_clnts; i++) {
			mt_thread_info[i].s = s;
			mt_thread_info[i].cntl_handle = cntl_handle;
			mt_thread_info[i].num_workers = &mt_test_num_workers;
			mt_thread_info[i].waitqueue = &waitqueue;
			mt_thread_info[i].result = &result;
			mt_thread_info[i].index = i;
			mt_thread_info[i].ch_cfg.pkt_ntrvl = pkt_ntrvl;
			mt_thread_info[i].ch_cfg.num_pkts = num_pkts_lat_ch;
			mt_thread_info[i].ch_cfg.tx_delay_ms = 500;
			mt_thread_info[i].ch_cfg.data_len = pkt_size_lat_ch;

			construct_ch_name(mt_thread_info[i].name, "LL_LAT_", i);
			INIT_WORK(&mt_thread_info[i].work,
				  ut_mt_worker_func);
			strlcpy(mt_thread_info[i].ch_cfg.ch_name,
				mt_thread_info[i].name,
				GLINK_NAME_SIZE);
			ret = glink_ut_mt_send_config_reqs(s, cntl_handle,
						&mt_thread_info[i]);
			UT_ASSERT_INT_MT(ret, ==, 0);
		}
		if (failed)
			break;

		for (j = 0; j < num_tput_clnts; i++, j++) {
			mt_thread_info[i].s = s;
			mt_thread_info[i].cntl_handle = cntl_handle;
			mt_thread_info[i].num_workers = &mt_test_num_workers;
			mt_thread_info[i].waitqueue = &waitqueue;
			mt_thread_info[i].result = &result;
			mt_thread_info[i].index = i;
			mt_thread_info[i].ch_cfg.num_pkts = num_pkts_tput_ch;
			mt_thread_info[i].ch_cfg.tx_delay_ms = 0;
			mt_thread_info[i].ch_cfg.data_len = pkt_size_tput_ch;

			construct_ch_name(mt_thread_info[i].name, "LL_TPUT_",
					  i);
			INIT_WORK(&mt_thread_info[i].work,
				  ut_mt_worker_func);
			strlcpy(mt_thread_info[i].ch_cfg.ch_name,
				mt_thread_info[i].name,
				GLINK_NAME_SIZE);
			ret = glink_ut_mt_send_config_reqs(s, cntl_handle,
						&mt_thread_info[i]);
			UT_ASSERT_INT_MT(ret, ==, 0);
		}
		if (failed)
			break;

		mt_test_wq = kzalloc(sizeof(struct workqueue_struct *) *
				     (num_lat_clnts + num_tput_clnts),
				     GFP_KERNEL);
		for (i = 0; i < (num_lat_clnts + num_tput_clnts); i++) {
			*(mt_test_wq + i) = create_singlethread_workqueue(
							mt_thread_info[i].name);
			UT_ASSERT_PTR_MT(NULL, !=, &mt_thread_info[i].work);
		}
		if (failed) {
			for (; i > 0; i--)
				destroy_workqueue(*(mt_test_wq + i));
			kfree(mt_test_wq);
			i = num_lat_clnts + num_tput_clnts;
			break;
		}

		for (i = 0; i < (num_lat_clnts + num_tput_clnts); i++)
			queue_work(*(mt_test_wq + i), &mt_thread_info[i].work);

		GLINK_UT_DBG("%s, Queued the work. Thread: %d\n", __func__,
				current->pid);

		wait_event_interruptible(waitqueue,
			atomic_read(&mt_test_num_workers) == 0);

		/* Flush and destroy the work */
		for (i = 0; i < (num_lat_clnts + num_tput_clnts); i++) {
			flush_workqueue(*(mt_test_wq + i));
			destroy_workqueue(*(mt_test_wq + i));
			*(mt_test_wq + i) = NULL;
		}
		kfree(mt_test_wq);
		kfree(mt_thread_info);
		ret = glink_loopback_close(cntl_handle);
		GLINK_UT_DBG("%s, CLOSED control channel. Thread: %d\n",
				__func__, current->pid);
		if (result)
			GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
	} while (0);

	if (failed) {
		for (; i > 0; i--) {
			pkt.payload.close.delay_ms = 0;
			strlcpy(pkt.payload.close.ch_name,
				mt_thread_info[i].name, GLINK_NAME_SIZE);
			pkt.payload.close.name_len =
				strlen(mt_thread_info[i].name);
			ret = glink_loopback_send_request(
				mt_thread_info[i].data_handle, &pkt, CLOSE,
				true);
			glink_loopback_close(mt_thread_info[i].data_handle);
		}
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		kfree(mt_thread_info);
		glink_loopback_close(cntl_handle);
	}
}

struct smd_trans_notify {
	struct completion connected_completion;
	struct completion local_disconnect_completion;
	struct completion remote_disconnect_completion;
	struct completion tx_done_completion;
	struct completion rx_intent_req_completion;
	struct completion rx_completion;
	bool rx_reuse;
};

static void init_smd_trans_notify(struct smd_trans_notify *n)
{
	init_completion(&n->connected_completion);
	init_completion(&n->local_disconnect_completion);
	init_completion(&n->remote_disconnect_completion);
	init_completion(&n->tx_done_completion);
	init_completion(&n->rx_intent_req_completion);
	init_completion(&n->rx_completion);
	n->rx_reuse = false;
}

static void reset_smd_trans_notify(struct smd_trans_notify *n)
{
	reinit_completion(&n->connected_completion);
	reinit_completion(&n->local_disconnect_completion);
	reinit_completion(&n->remote_disconnect_completion);
	reinit_completion(&n->tx_done_completion);
	reinit_completion(&n->rx_intent_req_completion);
	reinit_completion(&n->rx_completion);
	n->rx_reuse = false;
}

static void smd_trans_notify_state(void *handle, const void *priv,
				   unsigned event)
{
	struct smd_trans_notify *n = (struct smd_trans_notify *)priv;
	if (!n) {
		GLINK_UT_ERR("%s: smd_trans_notify not found\n", __func__);
		return;
	}

	switch (event) {
	case GLINK_CONNECTED:
		complete(&n->connected_completion);
		break;
	case GLINK_LOCAL_DISCONNECTED:
		complete(&n->local_disconnect_completion);
		break;
	case GLINK_REMOTE_DISCONNECTED:
		complete(&n->remote_disconnect_completion);
		break;
	default:
		GLINK_UT_ERR("%s: unrecognized event %d\n", __func__, event);
		break;
	};
}

static void smd_trans_notify_tx_done(void *handle, const void *priv,
				     const void *pkt_priv, const void *ptr)
{
	struct smd_trans_notify *n = (struct smd_trans_notify *)priv;
	if (!n) {
		GLINK_UT_ERR("%s: smd_trans_notify not found\n", __func__);
		return;
	}

	complete(&n->tx_done_completion);
}

bool smd_trans_rx_intent_req(void *handle, const void *priv, size_t sz)
{
	struct smd_trans_notify *n = (struct smd_trans_notify *)priv;
	if (!n) {
		GLINK_UT_ERR("%s: smd_trans_notify not found\n", __func__);
		return false;
	}
	complete(&n->rx_intent_req_completion);
	return true;
}

void smd_trans_notify_rx(void *handle, const void *priv, const void *pkt_priv,
			 const void *ptr, size_t size)
{
	struct smd_trans_notify *n = (struct smd_trans_notify *)priv;
	bool do_completion = false;

	if (!n) {
		GLINK_UT_ERR("%s: smd_trans_notify not found\n", __func__);
		return;
	}

	if ((!pkt_priv && size) || !memcmp(pkt_priv, ptr, size))
		do_completion = true;
	else
		GLINK_UT_ERR(
			"%s: received data did not match sent\n", __func__);

	glink_rx_done(handle, ptr, n->rx_reuse);

	if (do_completion)
		complete(&n->rx_completion);
}

/**
 * glink_ut0_emd_trans_basic_va() - Basic sanity test using SMD transitional
 *				    transport to MPSS
 *
 * @s: pointer to output file
 *
 * This test simulates a simple channel open and close to MPSS over SMD
 * transitional transport.  Variant A opens the channel then signals the modem
 * to create the SMD channel to test the local open first scenario.
 */
static void glink_ut0_smd_trans_basic_va(struct seq_file *s)
{
	struct glink_open_config cfg;
	void *handle;
	struct smd_trans_notify notify;
	unsigned long wait_ret;
	bool failed = false;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	if (smsm_get_state(SMSM_APPS_STATE) & SMSM_SMD_LOOPBACK)
		seq_puts(s, "Warning: SMSM signal for loopback already set\n");

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
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
static void glink_ut0_smd_trans_migration(struct seq_file *s)
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

/**
 * glink_ut0_emd_trans_basic_vb() - Basic sanity test using SMD transitional
 *				    transport to MPSS
 *
 * @s: pointer to output file
 *
 * This test simulates a simple channel open and close to MPSS over SMD
 * transitional transport.  Variant B signals the modem to create the SMD
 * channel then opens the channel to test the local open last scenario.
 */
static void glink_ut0_smd_trans_basic_vb(struct seq_file *s)
{
	struct glink_open_config cfg;
	void *handle;
	struct smd_trans_notify notify;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	unsigned long wait_ret;
	bool failed = false;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	if (smsm_get_state(SMSM_APPS_STATE) & SMSM_SMD_LOOPBACK)
		seq_puts(s, "Warning: SMSM signal for loopback already set\n");

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);
	msleep(500);

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
}

/**
 * glink_ut1_smd_trans_tx() - Basic sanity test using SMD transitional
 *			      transport to MPSS
 *
 * @s: pointer to output file
 *
 * This test simulates simple packet transmit and receive cases to MPSS over
 * SMD transitional transport.
 */
static void glink_ut1_smd_trans_tx(struct seq_file *s)
{
	struct glink_open_config cfg;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	void *handle;
	struct smd_trans_notify notify;
	unsigned long wait_ret;
	bool failed = false;
	void *buf;
	int ret;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	/* basic tx case - queue rx, then tx, then rx the loopbacked packet */
	buf = kmalloc(SZ_4K, GFP_KERNEL);
	if (!buf) {
		seq_puts(s, "Error: no memory for buffer\n");
		goto close;
	}
	memset(buf, SZ_4K, SZ_4K);

	ret = glink_queue_rx_intent(handle, buf, SZ_4K);
	if (ret) {
		seq_puts(s, "Failed: queue rx intent returned error\n");
		failed = true;
		goto close;
	}
	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_REQ_INTENT);
	if (ret) {
		seq_puts(s, "Failed: tx returned error\n");
		failed = true;
		goto close;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close;
	}
	wait_ret = wait_for_completion_timeout(&notify.rx_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx callback\n");
		failed = true;
		goto close;
	}

	seq_puts(s, "Case 1 complete\n");

	/* adv tx case - tx, get rx req, queue rx, then rx */
	reset_smd_trans_notify(&notify);

	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_REQ_INTENT);
	if (ret) {
		seq_puts(s, "Failed: tx returned error\n");
		failed = true;
		goto close;
	}

	wait_ret = wait_for_completion_timeout(&notify.rx_intent_req_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx intent req callback\n");
		failed = true;
		goto close;
	}

	ret = glink_queue_rx_intent(handle, buf, SZ_4K);
	if (ret) {
		seq_puts(s, "Failed: queue rx intent returned error\n");
		failed = true;
		goto close;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close;
	}
	wait_ret = wait_for_completion_timeout(&notify.rx_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx callback\n");
		failed = true;
		goto close;
	}

	seq_puts(s, "Case 2 complete\n");

	kfree(buf);

close:
	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
}

/**
 * glink_ut1_smd_trans_poll_and_mask() - Basic sanity test of poll() and mask()
 *
 * @s: pointer to output file
 *
 * This test simulates simple rpm poll() and irq_mask() to MPSS over
 * SMD transitional transport.
 */
static void glink_ut1_smd_trans_poll_and_mask(struct seq_file *s)
{
	struct glink_open_config cfg;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	void *handle;
	struct smd_trans_notify notify;
	unsigned long wait_ret;
	bool failed = false;
	void *buf;
	int ret;

	/*
	 * Need to change the smd trans driver to configure the modem edge as
	 * intentless, and then comment out these lines
	 */
	seq_puts(s, "Warning: Modem edge needs to be in intentless config\n");
	seq_puts(s, "Passed\n");
	return;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	/* basic tx case - queue rx, then tx, then rx the loopbacked packet */
	buf = kmalloc(SZ_4K, GFP_KERNEL);
	if (!buf) {
		seq_puts(s, "Error: no memory for buffer\n");
		goto close;
	}
	memset(buf, SZ_4K, SZ_4K);

	ret = glink_queue_rx_intent(handle, buf, SZ_4K);
	if (ret) {
		seq_puts(s, "Failed: queue rx intent returned error\n");
		failed = true;
		goto close1;
	}
	ret = glink_rpm_mask_rx_interrupt(handle, true, NULL);
	if (ret) {
		seq_printf(s, "Failed: rpm_mask_rx error:%d\n", ret);
		failed = true;
		goto close1;
	}
	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_REQ_INTENT);
	if (ret) {
		seq_puts(s, "Failed: tx returned error\n");
		failed = true;
		goto close2;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close2;
	}
	while (1) {
		ret = glink_rpm_rx_poll(handle);
		if (ret < 0) {
			seq_printf(s, "Failed: poll error:%d\n", ret);
			failed = true;
			goto close2;
		}
		if (completion_done(&notify.rx_completion))
			break;
	}

close2:
	glink_rpm_mask_rx_interrupt(handle, false, NULL);
close1:
	kfree(buf);
close:
	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
}

/**
 * glink_ut1_smd_trans_intentless() - Basic sanity test of intentless operation
 *
 * @s: pointer to output file
 *
 * This test simulates simple intentless operation to MPSS over
 * SMD transitional transport.
 */
static void glink_ut1_smd_trans_intentless(struct seq_file *s)
{
	struct glink_open_config cfg;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	void *handle;
	struct smd_trans_notify notify;
	unsigned long wait_ret;
	bool failed = false;
	void *buf;
	int ret;

	/*
	 * Need to change the smd trans driver to configure the modem edge as
	 * intentless, and then comment out these lines
	 */
	seq_puts(s, "Warning: Modem edge needs to be in intentless config\n");
	seq_puts(s, "Passed\n");
	return;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	buf = kmalloc(SZ_4K, GFP_KERNEL);
	if (!buf) {
		seq_puts(s, "Error: no memory for buffer\n");
		goto close;
	}
	memset(buf, SZ_4K, SZ_4K);

	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_SINGLE_THREADED);
	if (ret) {
		seq_printf(s, "Failed: tx returned error:%d\n", ret);
		failed = true;
		goto close1;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close1;
	}
	wait_ret = wait_for_completion_timeout(&notify.rx_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx callback\n");
		failed = true;
		goto close1;
	}

close1:
	kfree(buf);
close:
	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
}

/**
 * glink_ut1_smd_trans_rx_reuse() - RX intent reuse test using SMD transitional
 *				    transport to MPSS
 *
 * @s: pointer to output file
 *
 * This test simulates rx intent reuse to MPSS over SMD transitional transport.
 */
static void glink_ut1_smd_trans_rx_reuse(struct seq_file *s)
{
	struct glink_open_config cfg;
	struct glink_ut_dbgfs *ut_dfs_d;
	struct glink_dbgfs_data *dfs_d;
	void *handle;
	struct smd_trans_notify notify;
	unsigned long wait_ret;
	bool failed = false;
	void *buf;
	int ret;

	if (!(smsm_get_state(SMSM_MODEM_STATE) & (SMSM_INIT | SMSM_SMDINIT))) {
		seq_puts(s, "Error: Modem not SMSM and SMD initialized\n");
		return;
	}

	init_smd_trans_notify(&notify);
	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	GLINK_STATUS(s, "Running %s\n", ut_dfs_d->ut_name);
	memset(&cfg, 0, sizeof(struct glink_open_config));
	cfg.priv = &notify;
	cfg.options = 0;
	cfg.transport = ut_dfs_d->xprt_name;
	cfg.edge = ut_dfs_d->edge_name;
	cfg.name = "LOOPBACK";
	cfg.notify_rx = smd_trans_notify_rx;
	cfg.notify_tx_done = smd_trans_notify_tx_done;
	cfg.notify_state = smd_trans_notify_state;
	cfg.notify_rx_intent_req = smd_trans_rx_intent_req;

	reset_smd_trans_notify(&notify);
	notify.rx_reuse = true;

	handle = glink_open(&cfg);
	if (IS_ERR(handle)) {
		seq_printf(s, "Error: glink_open failed %d\n",
							(int)PTR_ERR(handle));
		return;
	}

	smsm_change_state(SMSM_APPS_STATE, 0, SMSM_SMD_LOOPBACK);

	wait_ret = wait_for_completion_timeout(&notify.connected_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out waiting for open callback\n");
		failed = true;
	}

	buf = kmalloc(SZ_4K, GFP_KERNEL);
	if (!buf) {
		seq_puts(s, "Error: no memory for buffer\n");
		goto close;
	}
	memset(buf, SZ_4K, SZ_4K);

	ret = glink_queue_rx_intent(handle, buf, SZ_4K);
	if (ret) {
		seq_puts(s, "Failed: queue rx intent returned error\n");
		failed = true;
		goto close;
	}
	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_REQ_INTENT);
	if (ret) {
		seq_puts(s, "Failed: tx returned error\n");
		failed = true;
		goto close;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close;
	}
	wait_ret = wait_for_completion_timeout(&notify.rx_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx callback\n");
		failed = true;
		goto close;
	}

	reset_smd_trans_notify(&notify);

	ret = glink_tx(handle, buf, buf, SZ_4K, GLINK_TX_REQ_INTENT);
	if (ret) {
		seq_puts(s, "Failed: tx returned error\n");
		failed = true;
		goto close;
	}

	wait_ret = wait_for_completion_timeout(&notify.tx_done_completion,
									5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for tx done callback\n");
		failed = true;
		goto close;
	}
	wait_ret = wait_for_completion_timeout(&notify.rx_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for rx callback\n");
		failed = true;
		goto close;
	}

close:
	kfree(buf);

	glink_close(handle);

	wait_ret = wait_for_completion_timeout(
				&notify.local_disconnect_completion, 5 * HZ);
	if (!wait_ret) {
		seq_puts(s, "Failed: timed out for local close callback\n");
		failed = true;
	}

	if (!failed)
		seq_puts(s, "Passed\n");
}

static void glink_local_mt_core(struct seq_file *s, int xprt_id,
		testfunc do_test)
{
	void *cntl_handle = NULL;
	void *data_handle = NULL;
	int failed = 0;
	int ret;

	do {
		ret = do_test(s, cntl_handle, data_handle);
		UT_ASSERT_INT(ret, ==, 0);
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "%s: Failed\n", __func__);
}

static int glink_ut1_local_mt_test_1_thread(struct seq_file *s,
		void *cntl_handle, void **data_handle)
{
	struct workqueue_struct *local_mt_test_thread_1_wq;
	struct mt_test_info_struct local_mt_thread_1_info;
	struct req pkt;
	int failed = 0;
	int ret = 0;
	const char *name = "LL_DATA_CLNT";
	char *cntl_xprt = "lloop";
	char *cntl_edge = "local";
	char *cntl_name = "LOCAL_LOOPBACK_CLNT";
	wait_queue_head_t waitqueue;

	int result = 0;
	init_waitqueue_head(&waitqueue);

	GLINK_STATUS_MT(s, "Running %s\n", __func__);
	do {
		cntl_handle = glink_loopback_open(cntl_xprt, cntl_edge,
				cntl_name, CH_CNTL_TYPE, LINEAR_RX, false);
		UT_ASSERT_ERR_PTR_MT(cntl_handle);

		local_mt_thread_1_info.s = s;
		local_mt_thread_1_info.pkt = pkt;
		local_mt_thread_1_info.cntl_handle = cntl_handle;
		local_mt_thread_1_info.waitqueue = &waitqueue;
		local_mt_thread_1_info.result = &result;
		strlcpy(local_mt_thread_1_info.name, name,
				sizeof(local_mt_thread_1_info.name) + 1);
		atomic_set(&local_mt_test_num_workers, 1);
		INIT_WORK_ONSTACK(&local_mt_thread_1_info.work,
				ut_local_mt_worker_func);
		ret = glink_ut1_local_mt_send_config_reqs(s, cntl_handle,
				name);

		local_mt_test_thread_1_wq =
			create_singlethread_workqueue(
					"local_mt_test_thread_1_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &local_mt_thread_1_info.work);

		queue_work(local_mt_test_thread_1_wq,
				&local_mt_thread_1_info.work);

		GLINK_UT_DBG("%s, Queued the work. Thread: %d\n", __func__,
				current->pid);

		wait_event_interruptible(waitqueue,
				atomic_read(&local_mt_test_num_workers) == 0);

		flush_workqueue(local_mt_test_thread_1_wq);
		destroy_workqueue(local_mt_test_thread_1_wq);
		local_mt_test_thread_1_wq = NULL;

		/* Send requests to the server to close each data channel */
		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name, strlen(name) + 1);
		pkt.payload.close.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		/* Close control channel */
		pkt.payload.close.delay_ms = 0;
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG("%s, Sent CLOSE request. Thread: %d\n", __func__,
				current->pid);

		ret = glink_loopback_close(cntl_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG("%s, Closed the control channel. Thread: %d\n",
				__func__, current->pid);
	} while (0);

	if (failed || result) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		glink_loopback_close(cntl_handle);
		ret = 1;
	}
	return ret;
}

static void glink_ut1_local_mt_1_thread(struct seq_file *s)
{
	testfunc do_test;
	int xprt_id = LOCAL_LOOPBACK_XPRT;
	do_test = glink_ut1_local_mt_test_1_thread;
	glink_local_mt_core(s, xprt_id, do_test);
}

static int glink_ut1_local_mt_test_3_threads(struct seq_file *s,
		void *cntl_handle, void **data_handle)
{
	struct workqueue_struct *local_mt_test_thread_1_wq;
	struct workqueue_struct *local_mt_test_thread_2_wq;
	struct workqueue_struct *local_mt_test_thread_3_wq;

	struct mt_test_info_struct local_mt_thread_1_info;
	struct mt_test_info_struct local_mt_thread_2_info;
	struct mt_test_info_struct local_mt_thread_3_info;

	struct req pkt;

	static wait_queue_head_t waitqueue;
	int failed = 0;
	int ret = 0;
	int result = 0;
	const char *name1 = "LL_DATA_1_CLNT";
	const char *name2 = "LL_DATA_2_CLNT";
	const char *name3 = "LL_DATA_3_CLNT";
	init_waitqueue_head(&waitqueue);

	GLINK_STATUS_MT(s, "Running %s\n", __func__);
	do {
		/* Open control channel */
		cntl_handle = glink_loopback_open("lloop",
						"local",
						"LOCAL_LOOPBACK_CLNT",
						CH_CNTL_TYPE, LINEAR_RX, false);
		UT_ASSERT_ERR_PTR_MT(cntl_handle);


		atomic_set(&local_mt_test_num_workers, 3);

		/* Pack info into workqueue structs */
		local_mt_thread_1_info.s = s;
		local_mt_thread_1_info.pkt = pkt;
		local_mt_thread_1_info.cntl_handle = cntl_handle;
		local_mt_thread_1_info.waitqueue = &waitqueue;
		local_mt_thread_1_info.result = &result;
		strlcpy(local_mt_thread_1_info.name, name1,
				sizeof(local_mt_thread_1_info.name) + 1);
		INIT_WORK_ONSTACK(&local_mt_thread_1_info.work,
				ut_local_mt_worker_func);
		ret = glink_ut1_local_mt_send_config_reqs(s, cntl_handle,
				name1);
		UT_ASSERT_INT_MT(ret, ==, 0);

		local_mt_thread_2_info.s = s;
		local_mt_thread_2_info.pkt = pkt;
		local_mt_thread_2_info.cntl_handle = cntl_handle;
		local_mt_thread_2_info.waitqueue = &waitqueue;
		local_mt_thread_2_info.result = &result;
		strlcpy(local_mt_thread_2_info.name, name2,
				sizeof(local_mt_thread_2_info.name) + 1);
		INIT_WORK_ONSTACK(&local_mt_thread_2_info.work,
				ut_local_mt_worker_func);
		ret = glink_ut1_local_mt_send_config_reqs(s, cntl_handle,
				name2);

		local_mt_thread_3_info.s = s;
		local_mt_thread_3_info.pkt = pkt;
		local_mt_thread_3_info.cntl_handle = cntl_handle;
		local_mt_thread_3_info.waitqueue = &waitqueue;
		local_mt_thread_3_info.result = &result;
		strlcpy(local_mt_thread_3_info.name, name3,
				sizeof(local_mt_thread_3_info.name) + 1);
		INIT_WORK_ONSTACK(&local_mt_thread_3_info.work,
				ut_local_mt_worker_func);
		ret = glink_ut1_local_mt_send_config_reqs(s, cntl_handle,
				name3);

		/* Create the workqueues */
		local_mt_test_thread_1_wq =
			create_singlethread_workqueue(
					"local_mt_test_thread_1_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &local_mt_thread_1_info.work);

		local_mt_test_thread_2_wq =
			create_singlethread_workqueue(
					"local_mt_test_thread_2_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &local_mt_thread_2_info.work);

		local_mt_test_thread_3_wq =
			create_singlethread_workqueue(
					"local_mt_test_thread_3_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &local_mt_thread_3_info.work);

		/* Queue the work */
		queue_work(local_mt_test_thread_1_wq,
				&local_mt_thread_1_info.work);
		queue_work(local_mt_test_thread_2_wq,
				&local_mt_thread_2_info.work);
		queue_work(local_mt_test_thread_3_wq,
				&local_mt_thread_3_info.work);

		GLINK_UT_DBG("%s, Queued the work. Thread: %d\n", __func__,
				current->pid);

		wait_event_interruptible(waitqueue,
				atomic_read(&local_mt_test_num_workers) == 0);

		/* Flush and destroy the work */
		flush_workqueue(local_mt_test_thread_1_wq);
		destroy_workqueue(local_mt_test_thread_1_wq);
		local_mt_test_thread_1_wq = NULL;

		flush_workqueue(local_mt_test_thread_2_wq);
		destroy_workqueue(local_mt_test_thread_2_wq);
		local_mt_test_thread_2_wq = NULL;

		flush_workqueue(local_mt_test_thread_3_wq);
		destroy_workqueue(local_mt_test_thread_3_wq);
		local_mt_test_thread_3_wq = NULL;

		/* Send requests to the server to close each data channel */
		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name1, strlen(name1) + 1);
		pkt.payload.close.name_len = strlen(name1);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name2, strlen(name2) + 1);
		pkt.payload.close.name_len = strlen(name2);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name3, strlen(name3) + 1);
		pkt.payload.close.name_len = strlen(name3);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		/* Close the control Channel */
		pkt.payload.close.delay_ms = 0;
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG("%s, Sent CLOSE request. Thread: %d\n", __func__,
				current->pid);

		ret = glink_loopback_close(cntl_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG("%s, CLOSED control channel. Thread: %d\n",
				__func__, current->pid);
	} while (0);

	if (failed || result) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		glink_loopback_close(cntl_handle);
		ret = 1;
	}
	return ret;
}

static int glink_ut1_local_mt_send_config_reqs(struct seq_file *s,
		void *cntl_handle, const char *name)
{
	struct req pkt;
	int ret;
	int failed = 0;
	int len = strlen(testdata);
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)cntl_handle;
	const char *cntl_transport = lpb_ch->open_cfg.transport;
	const char *cntl_edge = lpb_ch->open_cfg.edge;
	const char *cntl_name = lpb_ch->open_cfg.name;

	do {
		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, name, strlen(name) + 1);
		pkt.payload.open.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.tx_conf.random_delay = 0;
		pkt.payload.tx_conf.delay_ms = 0;
		pkt.payload.tx_conf.echo_count = 1;
		pkt.payload.tx_conf.transform_type = NO_TRANSFORM;
		strlcpy(pkt.payload.tx_conf.ch_name, name, strlen(name) + 1);
		pkt.payload.tx_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, TX_CONFIG,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG(
			"%s:%s:%s %s: %s %s, thread: %d\n",
				cntl_transport, cntl_edge, cntl_name, __func__,
				"Sent TX_CONFIG request for data channel",
				name, current->pid);

		pkt.payload.rx_done_conf.random_delay = 0;
		pkt.payload.rx_done_conf.delay_ms = 0;
		strlcpy(pkt.payload.rx_done_conf.ch_name, name,
				strlen(name) + 1);
		pkt.payload.rx_done_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
							RX_DONE_CONFIG, true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG(
			"%s:%s:%s %s: %s %s, thread: %d\n",
				cntl_transport, cntl_edge, cntl_name, __func__,
				name,
				"Sent RX_DONE_CONFIG request for data channel",
				current->pid);

		GLINK_UT_DBG("%s: start for iterations[%d]\n", __func__,
				ut_iterations);
		pkt.payload.q_rx_int_conf.random_delay = 0;
		pkt.payload.q_rx_int_conf.delay_ms = 0;
		pkt.payload.q_rx_int_conf.num_intents = ut_iterations;
		pkt.payload.q_rx_int_conf.intent_size = len;
		strlcpy(pkt.payload.q_rx_int_conf.ch_name, name,
				strlen(name) + 1);
		pkt.payload.q_rx_int_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
						  QUEUE_RX_INTENT_CONFIG,
						  true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG(
			"%s:%s:%s %s: %s %s, thread: %d\n",
			cntl_transport, cntl_edge, cntl_name, __func__,
			"Sent QUEUE_RX_INTENT_CONFIG request for data channel",
			name, current->pid);
	} while (0);

	if (failed)
		ret = 1;

	return ret;
}

static void glink_ut1_local_mt_3_threads(struct seq_file *s)
{
	testfunc do_test;
	int xprt_id = LOCAL_LOOPBACK_XPRT;

	do_test = glink_ut1_local_mt_test_3_threads;
	glink_local_mt_core(s, xprt_id, do_test);
}

static void ut_local_mt_worker_func(struct work_struct *work)
{
	int failed = 0;
	int ret, i;
	int len = strlen(testdata);
	void *cntl_handle;
	void *data_handle = NULL;
	struct req pkt;
	struct seq_file *s;
	struct mt_test_info_struct *local_mt_thread_info;
	const char *transport = "lloop";
	const char *edge = "local";
	const char *name;

	local_mt_thread_info = container_of(work,
			struct mt_test_info_struct, work);
	cntl_handle = local_mt_thread_info->cntl_handle;
	s = local_mt_thread_info->s;
	pkt = local_mt_thread_info->pkt;
	name = local_mt_thread_info->name;

	do {
		/* Open data channel on client */
		data_handle = glink_loopback_open(transport, edge, name,
				CH_DATA_TYPE, LINEAR_RX, false);
		UT_ASSERT_ERR_PTR_MT(data_handle);
		GLINK_UT_DBG(
			"%s:%s:%s %s: called glink_loopack_open() thread: %d\n",
			transport, edge, name, __func__, current->pid);


		for (i = 0; i < ut_iterations; i++) {
			if (i == 0)
				ret = glink_loopback_tx(data_handle,
						(void *)testdata, len, true,
						true, false);
			else
				ret = glink_loopback_tx(data_handle,
						(void *)testdata, len, false,
						false, false);
			UT_ASSERT_INT_MT(ret, ==, 0);
			if (failed)
				break;
			GLINK_UT_DBG(
				"%s:%s:%s %s: %s iteration: %d, thread: %d\n",
				transport, edge, name, __func__,
				"Called glink_loopback_tx(), ",
				i, current->pid);
		}
		if (failed)
			break;

		/* Close the data handle, but not the control handle */
		ret = glink_loopback_close(data_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG(
		"%s: Before dec, local_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&local_mt_test_num_workers),
			current->pid);

		atomic_dec(&local_mt_test_num_workers);
		GLINK_UT_DBG(
		"%s: After dec, local_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&local_mt_test_num_workers),
			current->pid);
		wake_up(local_mt_thread_info->waitqueue);

	} while (0);

	if (failed) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		ret = glink_loopback_close(data_handle);
		*local_mt_thread_info->result = 1;
		GLINK_UT_DBG(
		"%s: Before dec, local_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&local_mt_test_num_workers),
			current->pid);
		atomic_dec(&local_mt_test_num_workers);
		GLINK_UT_DBG(
		"%s: After dec, local_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&local_mt_test_num_workers),
			current->pid);
		GLINK_UT_DBG("%s:%s:%s %s: Closed channel, thread: %d\n",
				transport, edge, name,
				__func__, current->pid);
		wake_up(local_mt_thread_info->waitqueue);
	}
}

/**
 * glink_mpss_mt_core - The MPSS-SMEM core multithreaded test core function.
 *
 * @s: pointer to ouptut file
 * @do_test: The function containing the test to run
 *
 * Tests that use this core function simulate multithreaded data exchange.
 * There is a single-threaded and multithreaded variety.
 */
static void glink_mpss_mt_core(struct seq_file *s)
{
	struct glink_dbgfs_data *dfs_d;
	struct glink_ut_dbgfs *ut_dfs_d;
	int failed = 0;
	int ret;
	char *cntl_ch_name;
	char *data_ch_name;
	void *cntl_handle = NULL;
	const char *cntl_ch_pfix = "LOOPBACK_CTL_";
	const char *data_ch_pfix = "LOOPBACK_DATA_";
	unsigned int cntl_ch_len = GLINK_NAME_SIZE + strlen(cntl_ch_pfix);
	unsigned int data_ch_len = GLINK_NAME_SIZE + strlen(data_ch_pfix);
	char edge[GLINK_NAME_SIZE] = {0};

	dfs_d = s->private;
	ut_dfs_d = dfs_d->priv_data;
	glink_ut_to_upper(edge, ut_dfs_d->edge_name);
	cntl_ch_name = kzalloc(cntl_ch_len,
				GFP_KERNEL);
	snprintf(cntl_ch_name, cntl_ch_len,
			"%s%s", cntl_ch_pfix, edge);
	data_ch_name = kzalloc(data_ch_len,
				GFP_KERNEL);
	snprintf(data_ch_name, data_ch_len,
			"%s%s", data_ch_pfix, edge);

	do {
		cntl_handle = glink_loopback_open(ut_dfs_d->xprt_name,
				ut_dfs_d->edge_name,
				cntl_ch_name, CH_CNTL_TYPE, LINEAR_RX, false);
		UT_ASSERT_ERR_PTR_MT(cntl_handle);

		ret = ut_dfs_d->tfunc(s, cntl_handle, (void **)&data_ch_name);
		UT_ASSERT_INT(ret, ==, 0);
		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "%s: Failed\n", __func__);
	kfree(data_ch_name);
	kfree(cntl_ch_name);
}

/**
 * glink_ut1_mpss_mt_test_1_thread - The single-threaded MPSS-SMEM test.
 *
 * @s: pointer to output file
 * @cntl_handle: pointer to the control channel structure
 * @data_handle: pointer to the data channel structure
 *
 * This test configures the loopback server and sends ut_iterations messages on
 * to the modem on a separate workqueue.
 */
static int glink_ut1_mpss_mt_test_1_thread(struct seq_file *s,
		void *cntl_handle, void **data_handle)
{
	struct workqueue_struct *mpss_mt_test_thread_1_wq;
	struct mt_test_info_struct mpss_mt_thread_1_info;
	struct req pkt;
	int failed = 0;
	int ret = 0;
	bool wq_created = false;
	const char *name = (const char *)*data_handle;
	wait_queue_head_t waitqueue;

	int result = 0;
	init_waitqueue_head(&waitqueue);

	GLINK_STATUS_MT(s, "Running %s\n", __func__);
	do {

		mpss_mt_thread_1_info.s = s;
		mpss_mt_thread_1_info.pkt = pkt;
		mpss_mt_thread_1_info.cntl_handle = cntl_handle;
		mpss_mt_thread_1_info.waitqueue = &waitqueue;
		mpss_mt_thread_1_info.result = &result;
		strlcpy(mpss_mt_thread_1_info.name, name,
				sizeof(mpss_mt_thread_1_info.name) + 1);
		atomic_set(&mpss_mt_test_num_workers, 1);
		INIT_WORK_ONSTACK(&mpss_mt_thread_1_info.work,
				ut_mpss_mt_worker_func);
		ret = glink_ut1_mpss_mt_send_config_reqs(s, cntl_handle,
				name, &mpss_mt_thread_1_info);
		UT_ASSERT_INT_MT(ret, ==, 0);

		mpss_mt_test_thread_1_wq =
			create_singlethread_workqueue(
					"mpss_mt_test_thread_1_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &mpss_mt_thread_1_info.work);
		wq_created = true;

		queue_work(mpss_mt_test_thread_1_wq,
				&mpss_mt_thread_1_info.work);

		GLINK_UT_DBG("%s, Queued the work. Thread: %d\n", __func__,
				current->pid);

		wait_event_interruptible(waitqueue,
				atomic_read(&mpss_mt_test_num_workers) == 0);

		flush_workqueue(mpss_mt_test_thread_1_wq);
		destroy_workqueue(mpss_mt_test_thread_1_wq);
		mpss_mt_test_thread_1_wq = NULL;

		/* Send requests to the server to close each data channel */
		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name, strlen(name) + 1);
		pkt.payload.close.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		ret = glink_loopback_close(cntl_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG("%s, Closed the control channel. Thread: %d\n",
				__func__, current->pid);
	} while (0);

	if (failed || result) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		glink_loopback_close(cntl_handle);

		if (wq_created) {
			flush_workqueue(mpss_mt_test_thread_1_wq);
			destroy_workqueue(mpss_mt_test_thread_1_wq);
		}
		ret = 1;
	}
	return ret;
}

/**
 * glink_ut1_mpss_mt_test_3_threads - The multi-threaded MPSS-SMEM test.
 *
 * @s: pointer to output file
 * @cntl_handle: pointer to the control channel structure
 * @data_handle: pointer to the data channel structure
 *
 * This test configures the loopback server and sends ut_iterations messages on
 * to the modem on 3 separate single-threaded workqueues. ut_iterations messages
 * are sent on each workqueue.
 */
static int glink_ut1_mpss_mt_test_3_threads(struct seq_file *s,
		void *cntl_handle, void **data_handle)
{
	struct workqueue_struct *mpss_mt_test_thread_1_wq;
	struct workqueue_struct *mpss_mt_test_thread_2_wq;
	struct workqueue_struct *mpss_mt_test_thread_3_wq;

	struct mt_test_info_struct mpss_mt_thread_1_info;
	struct mt_test_info_struct mpss_mt_thread_2_info;
	struct mt_test_info_struct mpss_mt_thread_3_info;

	struct req pkt;

	static wait_queue_head_t waitqueue;
	int failed = 0;
	int ret = 0;
	int result = 0;
	char *name1;
	char *name2;
	char *name3;

	const char *data_ch_name = (const char *)*data_handle;
	unsigned int data_ch_len = strlen(data_ch_name) + 3;
	name1 = kzalloc(data_ch_len, GFP_KERNEL);
	name2 = kzalloc(data_ch_len, GFP_KERNEL);
	name3 = kzalloc(data_ch_len, GFP_KERNEL);
	snprintf(name1, data_ch_len, "%s%s", data_ch_name, "_1");
	snprintf(name2, data_ch_len, "%s%s", data_ch_name, "_2");
	snprintf(name3, data_ch_len, "%s%s", data_ch_name, "_3");

	init_waitqueue_head(&waitqueue);
	GLINK_STATUS_MT(s, "Running %s\n", __func__);
	do {
		/* Open control channel */
		atomic_set(&mpss_mt_test_num_workers, 3);

		/* Pack info into workqueue structs */
		mpss_mt_thread_1_info.s = s;
		mpss_mt_thread_1_info.pkt = pkt;
		mpss_mt_thread_1_info.cntl_handle = cntl_handle;
		mpss_mt_thread_1_info.waitqueue = &waitqueue;
		mpss_mt_thread_1_info.result = &result;
		strlcpy(mpss_mt_thread_1_info.name, name1,
				sizeof(mpss_mt_thread_1_info.name) + 1);
		INIT_WORK_ONSTACK(&mpss_mt_thread_1_info.work,
				ut_mpss_mt_worker_func);
		ret = glink_ut1_mpss_mt_send_config_reqs(s, cntl_handle,
				name1, &mpss_mt_thread_1_info);
		UT_ASSERT_INT_MT(ret, ==, 0);

		mpss_mt_thread_2_info.s = s;
		mpss_mt_thread_2_info.pkt = pkt;
		mpss_mt_thread_2_info.cntl_handle = cntl_handle;
		mpss_mt_thread_2_info.waitqueue = &waitqueue;
		mpss_mt_thread_2_info.result = &result;
		strlcpy(mpss_mt_thread_2_info.name, name2,
				sizeof(mpss_mt_thread_2_info.name) + 1);
		INIT_WORK_ONSTACK(&mpss_mt_thread_2_info.work,
				ut_mpss_mt_worker_func);
		ret = glink_ut1_mpss_mt_send_config_reqs(s, cntl_handle,
				name2, &mpss_mt_thread_2_info);
		UT_ASSERT_INT_MT(ret, ==, 0);

		mpss_mt_thread_3_info.s = s;
		mpss_mt_thread_3_info.pkt = pkt;
		mpss_mt_thread_3_info.cntl_handle = cntl_handle;
		mpss_mt_thread_3_info.waitqueue = &waitqueue;
		mpss_mt_thread_3_info.result = &result;
		strlcpy(mpss_mt_thread_3_info.name, name3,
				sizeof(mpss_mt_thread_3_info.name) + 1);
		INIT_WORK_ONSTACK(&mpss_mt_thread_3_info.work,
				ut_mpss_mt_worker_func);
		ret = glink_ut1_mpss_mt_send_config_reqs(s, cntl_handle,
				name3, &mpss_mt_thread_3_info);
		UT_ASSERT_INT_MT(ret, ==, 0);

		/* Create the workqueues */
		mpss_mt_test_thread_1_wq =
			create_singlethread_workqueue(
					"mpss_mt_test_thread_1_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &mpss_mt_thread_1_info.work);

		mpss_mt_test_thread_2_wq =
			create_singlethread_workqueue(
					"mpss_mt_test_thread_2_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &mpss_mt_thread_2_info.work);

		mpss_mt_test_thread_3_wq =
			create_singlethread_workqueue(
					"mpss_mt_test_thread_3_wq");
		UT_ASSERT_PTR_MT(NULL, !=, &mpss_mt_thread_3_info.work);

		/* Queue the work */
		queue_work(mpss_mt_test_thread_1_wq,
				&mpss_mt_thread_1_info.work);
		queue_work(mpss_mt_test_thread_2_wq,
				&mpss_mt_thread_2_info.work);
		queue_work(mpss_mt_test_thread_3_wq,
				&mpss_mt_thread_3_info.work);

		GLINK_UT_DBG("%s, Queued the work. Thread: %d\n", __func__,
				current->pid);

		wait_event_interruptible(waitqueue,
				atomic_read(&mpss_mt_test_num_workers) == 0);

		/* Flush and destroy the work */
		flush_workqueue(mpss_mt_test_thread_1_wq);
		destroy_workqueue(mpss_mt_test_thread_1_wq);
		mpss_mt_test_thread_1_wq = NULL;

		flush_workqueue(mpss_mt_test_thread_2_wq);
		destroy_workqueue(mpss_mt_test_thread_2_wq);
		mpss_mt_test_thread_2_wq = NULL;

		flush_workqueue(mpss_mt_test_thread_3_wq);
		destroy_workqueue(mpss_mt_test_thread_3_wq);
		mpss_mt_test_thread_3_wq = NULL;

		/* Send requests to the server to close each data channel */
		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name1, strlen(name1) + 1);
		pkt.payload.close.name_len = strlen(name1);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name2, strlen(name2) + 1);
		pkt.payload.close.name_len = strlen(name2);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.close.delay_ms = 0;
		strlcpy(pkt.payload.close.ch_name, name3, strlen(name3) + 1);
		pkt.payload.close.name_len = strlen(name3);
		ret = glink_loopback_send_request(cntl_handle, &pkt, CLOSE,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		ret = glink_loopback_close(cntl_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG("%s, CLOSED control channel. Thread: %d\n",
				__func__, current->pid);
	} while (0);

	if (failed || result) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		glink_loopback_close(cntl_handle);
		ret = 1;
	}
	kfree(name1);
	kfree(name2);
	kfree(name3);

	return ret;
}

/**
 * glink_ut1_mpss_mt_send_config_reqs - Send loopback server configuration
 *                                      requests.
 *
 * @s: pointer to output file
 * @cntl_handle: pointer to the control channel structure
 * @name: name of the data channel
 * @mpss_mt_thread_info: information particular to the channel being configured
 *
 * This function sends configuration requests to the loopback server on the
 * modem, and opens the data channel. It does the following actions in the
 * following order: send OPEN configuration request, send TX_CONFIG request,
 * send RX_DONE_CONFIG request, open the data channel, send
 * QUEUE_RX_INTENT_CONFIG request.
 */
static int glink_ut1_mpss_mt_send_config_reqs(struct seq_file *s,
		void *cntl_handle, const char *name,
		struct mt_test_info_struct *mpss_mt_thread_info)
{
	struct req pkt;
	int ret;
	int failed = 0;
	int len = strlen(testdata);
	void *data_handle = NULL;
	struct loopback_channel *lpb_ch =
		(struct loopback_channel *)cntl_handle;
	const char *cntl_transport = lpb_ch->open_cfg.transport;
	const char *cntl_edge = lpb_ch->open_cfg.edge;
	const char *cntl_name = lpb_ch->open_cfg.name;

	do {
		pkt.payload.open.delay_ms = 0;
		strlcpy(pkt.payload.open.ch_name, name, strlen(name) + 1);
		pkt.payload.open.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, OPEN,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);

		pkt.payload.tx_conf.random_delay = 0;
		pkt.payload.tx_conf.delay_ms = 0;
		pkt.payload.tx_conf.echo_count = 1;
		pkt.payload.tx_conf.transform_type = NO_TRANSFORM;
		strlcpy(pkt.payload.tx_conf.ch_name, name, strlen(name) + 1);
		pkt.payload.tx_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt, TX_CONFIG,
				true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG(
			"%s:%s:%s %s: %s %s, thread: %d\n",
				cntl_transport, cntl_edge, cntl_name, __func__,
				"Sent TX_CONFIG request for data channel",
				name, current->pid);

		pkt.payload.rx_done_conf.random_delay = 0;
		pkt.payload.rx_done_conf.delay_ms = 0;
		strlcpy(pkt.payload.rx_done_conf.ch_name, name,
				strlen(name) + 1);
		pkt.payload.rx_done_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
							RX_DONE_CONFIG, true);
		UT_ASSERT_INT_MT(ret, ==, 0);
		GLINK_UT_DBG(
			"%s:%s:%s %s: %s %s, thread: %d\n",
				cntl_transport, cntl_edge, cntl_name, __func__,
				name,
				"Sent RX_DONE_CONFIG request for data channel",
				current->pid);

		/*
		 * Open data channel here, before sending the
		 * QUEUE_RX_INTENT_CONFIG request. Pass the opened data channel
		 * to the worker function through the structure you created.
		 * This is necessary because the modem loopback server doesn not
		 * defer QUEUE_RX_INTENT_CONFIG requests until after the channel
		 * is fully opened. Therefore it is necessary to open the
		 * channel before sending this request.
		 */
		data_handle = glink_loopback_open(cntl_transport, cntl_edge,
				name, CH_DATA_TYPE, LINEAR_RX, false);
		UT_ASSERT_ERR_PTR_MT(data_handle);

		mpss_mt_thread_info->data_handle = data_handle;

		pkt.payload.q_rx_int_conf.random_delay = 0;
		pkt.payload.q_rx_int_conf.delay_ms = 0;
		pkt.payload.q_rx_int_conf.num_intents = ut_iterations;
		pkt.payload.q_rx_int_conf.intent_size = len;
		strlcpy(pkt.payload.q_rx_int_conf.ch_name, name,
				strlen(name) + 1);
		pkt.payload.q_rx_int_conf.name_len = strlen(name);
		ret = glink_loopback_send_request(cntl_handle, &pkt,
						  QUEUE_RX_INTENT_CONFIG,
						  true);
		UT_ASSERT_INT_MT(ret, ==, 0);
	} while (0);

	if (failed)
		ret = 1;

	return ret;
}

/**
 * ut1_mpss_mt_worker_func - Worker function for the MPSS-SMEM multithreaded
 *                           tests.
 *
 * @work: Pointer to work structure
 *
 * The worker function for the MPSS multi-threaded tests. Sends ut_iterations
 * messages to the remote side.
 */
static void ut_mpss_mt_worker_func(struct work_struct *work)
{
	int failed = 0;
	int ret, i;
	int len = strlen(testdata);
	void *cntl_handle;
	void *data_handle = NULL;
	struct req pkt;
	struct seq_file *s;
	struct mt_test_info_struct *mpss_mt_thread_info;
	struct loopback_channel *lpb_ch;
	const char *transport;
	const char *edge;
	const char *name;

	mpss_mt_thread_info = container_of(work,
			struct mt_test_info_struct, work);
	cntl_handle = mpss_mt_thread_info->cntl_handle;
	data_handle = mpss_mt_thread_info->data_handle;
	s = mpss_mt_thread_info->s;
	pkt = mpss_mt_thread_info->pkt;
	name = mpss_mt_thread_info->name;
	lpb_ch = (struct loopback_channel *)cntl_handle;
	transport = lpb_ch->open_cfg.transport;
	edge = lpb_ch->open_cfg.edge;

	do {
		for (i = 0; i < ut_iterations; i++) {
			if (i == 0)
				ret = glink_loopback_tx(data_handle,
						(void *)testdata, len, true,
						true, false);
			else
				ret = glink_loopback_tx(data_handle,
						(void *)testdata, len, false,
						true, false);
			UT_ASSERT_INT_MT(ret, ==, 0);
			if (failed)
				break;
			GLINK_UT_DBG(
				"%s:%s:%s %s: %s iteration: %d, thread: %d\n",
				transport, edge, name, __func__,
				"Called glink_loopback_tx(), ",
				i, current->pid);
		}
		if (failed)
			break;

		/* Close the data handle, but not the control handle */
		ret = glink_loopback_close(data_handle);
		UT_ASSERT_INT_MT(ret, ==, 0);

		GLINK_UT_DBG(
		"%s: Before dec, mpss_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&mpss_mt_test_num_workers),
			current->pid);

		atomic_dec(&mpss_mt_test_num_workers);
		GLINK_UT_DBG(
		"%s: After dec, mpss_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&mpss_mt_test_num_workers),
			current->pid);
		wake_up(mpss_mt_thread_info->waitqueue);

	} while (0);

	if (failed) {
		GLINK_STATUS_MT(s, "%s: Failed\n", __func__);
		ret = glink_loopback_close(data_handle);
		*mpss_mt_thread_info->result = 1;
		GLINK_UT_DBG(
		"%s: Before dec, mpss_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&mpss_mt_test_num_workers),
			current->pid);
		atomic_dec(&mpss_mt_test_num_workers);
		GLINK_UT_DBG(
		"%s: After dec, mpss_mt_test_num_workers == %d. Thread: %d\n",
			__func__,
			atomic_read(&mpss_mt_test_num_workers),
			current->pid);
		GLINK_UT_DBG("%s:%s:%s %s: Closed channel, thread: %d\n",
				transport, edge, name,
				__func__, current->pid);
		wake_up(mpss_mt_thread_info->waitqueue);
	}
}

/* Locking test */
static void channel_test_thread_worker(struct work_struct *work);

struct channel_test_info_struct {
	struct seq_file *s;
	int channel_id;
	char channel_name[GLINK_NAME_SIZE];
	struct work_struct work;
};

static void glink_ut1_mock_channel_locking_test_1_thread(struct seq_file *s)
{
	int failed = 0;
	struct glink_mock_xprt *mock_ptr;
	struct workqueue_struct *channel_test_thread_1_wq = NULL;
	struct channel_test_info_struct channel_test_thread_1_info;

	GLINK_STATUS(s, "Running %s\n", __func__);
	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		channel_test_thread_1_info.s = s;
		channel_test_thread_1_info.channel_id = 1;
		strlcpy(channel_test_thread_1_info.channel_name, "loopback1",
			sizeof(channel_test_thread_1_info.channel_name));
		INIT_WORK_ONSTACK(&channel_test_thread_1_info.work,
				channel_test_thread_worker);

		channel_test_thread_1_wq =
			create_singlethread_workqueue(
					"channel_test_thread_1_wq");
		UT_ASSERT_PTR(NULL, !=, channel_test_thread_1_wq);

		queue_work(channel_test_thread_1_wq,
				&channel_test_thread_1_info.work);
		flush_workqueue(channel_test_thread_1_wq);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "\tFailed\n");

	if (channel_test_thread_1_wq != NULL)
		destroy_workqueue(channel_test_thread_1_wq);
}

static void glink_ut1_mock_channel_locking_test_3_threads(struct seq_file *s)
{
	int failed = 0;

	struct glink_mock_xprt *mock_ptr;

	struct workqueue_struct *channel_test_thread_1_wq;
	struct workqueue_struct *channel_test_thread_2_wq;
	struct workqueue_struct *channel_test_thread_3_wq;

	struct channel_test_info_struct channel_test_thread_1_info;
	struct channel_test_info_struct channel_test_thread_2_info;
	struct channel_test_info_struct channel_test_thread_3_info;

	GLINK_STATUS(s, "Running %s\n", __func__);
	do {
		mock_ptr = mock_xprt_get();
		mock_xprt_reset(mock_ptr);
		UT_ASSERT_INT(0, ==, do_mock_negotiation(s, 0x1, 0x0));

		channel_test_thread_1_info.s = s;
		channel_test_thread_1_info.channel_id = 1;
		strlcpy(channel_test_thread_1_info.channel_name, "loopback1",
				sizeof("loopback1"));
		INIT_WORK_ONSTACK(&channel_test_thread_1_info.work,
				channel_test_thread_worker);

		channel_test_thread_2_info.s = s;
		channel_test_thread_2_info.channel_id = 2;
		strlcpy(channel_test_thread_2_info.channel_name, "loopback2",
				sizeof("loopback2"));
		INIT_WORK_ONSTACK(&channel_test_thread_2_info.work,
				channel_test_thread_worker);

		channel_test_thread_3_info.s = s;
		channel_test_thread_3_info.channel_id = 3;
		strlcpy(channel_test_thread_3_info.channel_name, "loopback3",
				sizeof("loopback3"));
		INIT_WORK_ONSTACK(&channel_test_thread_3_info.work,
				channel_test_thread_worker);

		channel_test_thread_1_wq =
			create_singlethread_workqueue(
					"channel_test_thread_1_wq");
		UT_ASSERT_PTR(NULL, !=, channel_test_thread_1_wq);

		channel_test_thread_2_wq =
			create_singlethread_workqueue(
					"channel_test_thread_2_wq");
		UT_ASSERT_PTR(NULL, !=, channel_test_thread_2_wq);

		channel_test_thread_3_wq =
			create_singlethread_workqueue(
					"channel_test_thread_3_wq");
		UT_ASSERT_PTR(NULL, !=, channel_test_thread_3_wq);

		queue_work(channel_test_thread_1_wq,
				&channel_test_thread_1_info.work);

		queue_work(channel_test_thread_2_wq,
				&channel_test_thread_2_info.work);

		queue_work(channel_test_thread_3_wq,
				&channel_test_thread_3_info.work);

		flush_workqueue(channel_test_thread_1_wq);
		destroy_workqueue(channel_test_thread_1_wq);
		channel_test_thread_1_wq = NULL;

		flush_workqueue(channel_test_thread_2_wq);
		destroy_workqueue(channel_test_thread_2_wq);
		channel_test_thread_2_wq = NULL;

		flush_workqueue(channel_test_thread_3_wq);
		destroy_workqueue(channel_test_thread_3_wq);
		channel_test_thread_3_wq = NULL;

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "\tFailed\n");
}

/**
 * struct rwref_lock_container - rwref_lock testing context
 * @rwref_lock:  lock being tested
 * @released:  lock release function called
 *
 * @write_work:  write worker structure
 * @write_worker_event:  write work has done something
 * @write_wq_started:  write work has started
 * @write_wq_lock_count:  num of times worker acquired write lock
 * @write_wq_ended:  write worker has terminated
 *
 * @read_work:  read worker structure
 * @read_worker_event:  read work has done something
 * @read_wq_started:  read work has started
 * @read_wq_lock_count:  num of times worker acquired read lock
 * @read_wq_ended:  read worker has terminated
 */
struct rwref_lock_container {
	struct rwref_lock rwref_lock;
	bool released;

	struct work_struct write_work;
	struct completion write_worker_event;
	bool write_wq_started;
	unsigned write_wq_lock_count;
	bool write_wq_ended;

	struct work_struct read_work;
	struct completion read_worker_event;
	bool read_wq_started;
	unsigned read_wq_lock_count;
	bool read_wq_ended;
};

/**
 * ut0_rwref_release() - rwref_lock free function
 * @rwref:  pointer to lock structure
 */
static void ut0_rwref_release(struct rwref_lock *rwref)
{
	struct rwref_lock_container *data_ptr;

	data_ptr = container_of(rwref, struct rwref_lock_container, rwref_lock);
	if (data_ptr)
		data_ptr->released = true;
}

/**
 * ut0_rwref_write_worker() - rwref_lock write worker
 *
 * Used to acquire the write lock in a separate execution context from main
 * thread.
 */
static void ut0_rwref_write_worker(struct work_struct *work)
{
	struct rwref_lock_container *rwref_ptr;

	rwref_ptr = container_of(work, struct rwref_lock_container, write_work);
	if (rwref_ptr == NULL)
		return;

	rwref_ptr->write_wq_started = true;
	complete(&rwref_ptr->write_worker_event);

	rwref_write_get(&rwref_ptr->rwref_lock);
	rwref_ptr->write_wq_lock_count++;
	complete(&rwref_ptr->write_worker_event);

	rwref_write_put(&rwref_ptr->rwref_lock);
	rwref_ptr->write_wq_ended = true;
	complete(&rwref_ptr->write_worker_event);
}

/**
 * glink_ut0_rwref_lock() - verify rwref lock
 *
 * @s: pointer to output file
 */
static void glink_ut0_rwref_lock(struct seq_file *s)
{
	int failed = 0;

	struct workqueue_struct *write_wq = NULL;
	struct workqueue_struct *read_wq = NULL;
	struct rwref_lock_container rwref_obj;

	GLINK_STATUS(s, "Running %s\n", __func__);
	do {
		memset(&rwref_obj, 0x00, sizeof(rwref_obj));
		rwref_lock_init(&rwref_obj.rwref_lock, ut0_rwref_release);

		write_wq = create_singlethread_workqueue("ut0_rwref_write_wq");
		UT_ASSERT_PTR(NULL, !=, write_wq);
		INIT_WORK_ONSTACK(&rwref_obj.write_work,
				ut0_rwref_write_worker);
		init_completion(&rwref_obj.write_worker_event);

		read_wq = create_singlethread_workqueue("ut0_rwref_read_wq");
		UT_ASSERT_PTR(NULL, !=, read_wq);
		INIT_WORK_ONSTACK(&rwref_obj.read_work, ut0_rwref_write_worker);
		init_completion(&rwref_obj.read_worker_event);

		/* verify read can be grabbed multiple times */
		UT_ASSERT_INT(0, ==, rwref_obj.rwref_lock.read_count);
		rwref_read_get(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(1, ==, rwref_obj.rwref_lock.read_count);
		rwref_read_get(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(2, ==, rwref_obj.rwref_lock.read_count);

		/* trigger workqueue to grab write lock */
		queue_work(write_wq, &rwref_obj.write_work);
		wait_for_completion_timeout(&rwref_obj.write_worker_event, HZ);
		UT_ASSERT_INT(true, ==, rwref_obj.write_wq_started);
		UT_ASSERT_INT(0, ==, rwref_obj.write_wq_lock_count);

		/* release read locks and confirm write lock is acquired */
		rwref_read_put(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(1, ==, rwref_obj.rwref_lock.read_count);
		UT_ASSERT_INT(0, ==, rwref_obj.write_wq_lock_count);

		rwref_read_put(&rwref_obj.rwref_lock);
		wait_for_completion_timeout(&rwref_obj.write_worker_event, HZ);
		UT_ASSERT_INT(0, ==, rwref_obj.rwref_lock.read_count);
		UT_ASSERT_INT(1, ==, rwref_obj.write_wq_lock_count);
		UT_ASSERT_INT(true, ==, rwref_obj.write_wq_ended);

		/* put ref with active read - release should not happen */
		rwref_read_get(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(1, ==, rwref_obj.rwref_lock.read_count);
		rwref_put(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(false, ==, rwref_obj.released);

		rwref_read_put(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(0, ==, rwref_obj.rwref_lock.read_count);
		UT_ASSERT_INT(true, ==, rwref_obj.released);

		/* put ref with active write - release should not happen */
		rwref_obj.released = false;
		rwref_get(&rwref_obj.rwref_lock);

		rwref_write_get(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(1, ==, rwref_obj.rwref_lock.write_count);
		rwref_put(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(false, ==, rwref_obj.released);

		rwref_write_put(&rwref_obj.rwref_lock);
		UT_ASSERT_INT(0, ==, rwref_obj.rwref_lock.write_count);
		UT_ASSERT_INT(true, ==, rwref_obj.released);

		GLINK_STATUS(s, "\tOK\n");
	} while (0);

	if (failed)
		GLINK_STATUS(s, "\tFailed\n");

	if (write_wq) {
		flush_workqueue(write_wq);
		destroy_workqueue(write_wq);

		flush_workqueue(read_wq);
		destroy_workqueue(read_wq);
	}
}

static void channel_test_thread_worker(struct work_struct *work)
{
	int failed = 0;
	int ret;
	int ITERATIONS = 50;
	int i = 0;
	int completion_result = 0;
	int completion_iter_count = 0;
	uint32_t channel_id;
	uint32_t received_intent_id;
	void *handle = NULL;
	char channel_name[GLINK_NAME_SIZE];
	struct glink_mock_xprt *mock_ptr;
	struct glink_open_config open_cfg;
	struct glink_mock_cmd *tx_cmd;
	struct glink_core_rx_intent *rx_intent_ptr;
	struct glink_mock_rx_intent *intent;
	struct glink_mock_tx_data *tx_data;
	struct seq_file *s;
	struct completion event;
	struct channel_test_info_struct *test_thread_work_info;
	struct ut_notify_data cb_data;

	test_thread_work_info = container_of(work,
			struct channel_test_info_struct, work);
	channel_id = test_thread_work_info->channel_id;
	mutex_lock(&multithread_test_mutex_lha0);
	s = test_thread_work_info->s;
	mutex_unlock(&multithread_test_mutex_lha0);
	strlcpy(channel_name, test_thread_work_info->channel_name,
			sizeof(test_thread_work_info->channel_name));
	cb_data_init(&cb_data);

	GLINK_STATUS_MT(s, "\n\tRunning %s, thread %d, channel_name: %s\n",
			__func__, current->pid, channel_name);
	do {
		/*
		 * Initialize mock transport and open channel.  No
		 * mock_xprt_reset() call is needed here as the transport was
		 * re-initialized in the glink_ut_mock_channel_locking_test()
		 * function before this work was scheduled to run.
		 */
		mock_ptr = mock_xprt_get();

		init_completion(&event);
		cb_data_reset(&cb_data);
		register_completion(&mock_ptr->if_ptr, &event);

		/* Open the channel */
		memset(&open_cfg, 0, sizeof(struct glink_open_config));
		open_cfg.transport = "mock";
		open_cfg.edge = "local";
		open_cfg.name = channel_name;

		open_cfg.notify_rx =  glink_test_notify_rx;
		open_cfg.notify_tx_done = glink_test_notify_tx_done;
		open_cfg.notify_state = glink_test_notify_state;
		open_cfg.notify_rx_intent_req = glink_test_rmt_rx_intent_req_cb;
		open_cfg.priv = &cb_data;


		handle = glink_open(&open_cfg);
		UT_ASSERT_ERR_PTR_MT(handle);

		channel_id = glink_get_channel_id_for_handle(handle);
		tx_cmd = mock_xprt_get_next_cmd_by_cid(channel_id, &event);

		UT_ASSERT_PTR_MT(NULL, !=, tx_cmd);
		UT_ASSERT_INT_MT(LOCAL_OPEN, ==, tx_cmd->type);
		UT_ASSERT_INT_MT(channel_id, ==, tx_cmd->local_open.lcid);
		UT_ASSERT_STRING_COMPARE_MT(channel_name,
				tx_cmd->local_open.name);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
			&mock_ptr->if_ptr, tx_cmd->local_open.lcid,
			MOCK_XPRT_ID);
		kfree(tx_cmd);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
					&mock_ptr->if_ptr, channel_id,
					channel_name, MOCK_XPRT_ID);

		tx_cmd = mock_xprt_get_next_cmd_by_cid(channel_id, &event);

		UT_ASSERT_PTR_MT(NULL, !=, tx_cmd);
		UT_ASSERT_INT_MT(REMOTE_OPEN_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT_MT(channel_id, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT_MT(GLINK_CONNECTED, ==, cb_data.event);
		/* open channel completed */

		while (i < ITERATIONS) {
			/*
			 * Read the data [hello GLINK]
			 */
			glink_queue_rx_intent(handle, (void *)&cb_data,
					strlen(testdata));
			intent = mock_xprt_get_intent_by_cid(channel_id,
					&event);
			received_intent_id = intent->liid;

			UT_ASSERT_PTR_MT(NULL, !=, intent);
			UT_ASSERT_INT_MT(strlen(testdata), ==, intent->size);

			rx_intent_ptr =
			    mock_ptr->if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
			    &mock_ptr->if_ptr, channel_id, intent->liid);
			UT_ASSERT_PTR(NULL, ==, (void *)rx_intent_ptr->data);
			rx_intent_ptr->data = (void *)testdata;
			rx_intent_ptr->pkt_size = strlen(testdata);
			mock_ptr->if_ptr.glink_core_if_ptr->rx_put_pkt_ctx(
				&mock_ptr->if_ptr, channel_id, rx_intent_ptr,
				true);

			UT_ASSERT_INT_MT(true, ==, cb_data.rx_notify);
			UT_ASSERT_INT_MT(strlen(testdata), ==, cb_data.size);
			kfree(intent);

			/*
			 * Write the data [Hello GLINK]
			 */
		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
					&mock_ptr->if_ptr, channel_id,
					received_intent_id, strlen(testdata));

			ret = glink_tx(handle, (void *)&cb_data, (void
						*)testdata, strlen(testdata),
					0);
			UT_ASSERT_INT_MT(ret, ==, 0);

			completion_result = (int)(HZ / 2);
			tx_data = NULL;

			while (tx_data == NULL) {
				if (completion_result > 0) {
					completion_result = (int)
						wait_for_completion_timeout(
							&event,
							completion_result);
				} else {
					failed = 1;
					break;
				}
				tx_data = mock_xprt_get_tx_data_by_cid(
						channel_id,
						&event);
				completion_iter_count++;
			}

			UT_ASSERT_PTR_MT(NULL, !=, tx_data);
			UT_ASSERT_STRING_COMPARE_MT(testdata, tx_data->data);
			mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_tx_done(
							&mock_ptr->if_ptr,
							channel_id,
							received_intent_id,
							false);
			UT_ASSERT_INT_MT(true, == , cb_data.tx_done);
			++i;
		}

		/* When the unit testing macros run "break;", they are intended
		 * to break out of the unit testing while loop. However, in the
		 * above case they only break out of the test iterations while
		 * loop. Thus we need to check if "failed" is set, and if so
		 * break out of the unit testing while loop.
		 */
		if (failed)
			break;

		ret = glink_close(handle);
		UT_ASSERT_INT_MT(ret, ==, 0);
		tx_cmd = mock_xprt_get_next_cmd_by_cid(channel_id, &event);
		UT_ASSERT_PTR_MT(NULL, !=, tx_cmd);
		UT_ASSERT_INT_MT(LOCAL_CLOSE, ==, tx_cmd->type);
		UT_ASSERT_INT_MT(channel_id, ==, tx_cmd->local_close.lcid);

		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
			&mock_ptr->if_ptr, tx_cmd->local_close.lcid);
		kfree(tx_cmd);
		UT_ASSERT_INT_MT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);


		mock_ptr->if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
					&mock_ptr->if_ptr, channel_id);
		tx_cmd = mock_xprt_get_next_cmd_by_cid(channel_id, &event);
		UT_ASSERT_PTR_MT(NULL, !=, tx_cmd);
		UT_ASSERT_INT_MT(REMOTE_CLOSE_ACK, ==, tx_cmd->type);
		UT_ASSERT_INT_MT(channel_id, ==, tx_cmd->remote_open_ack.rcid);
		UT_ASSERT_INT_MT(GLINK_LOCAL_DISCONNECTED, ==, cb_data.event);
		kfree(tx_cmd);

		GLINK_STATUS_MT(s,
		"\tChannel test thread %d, channel %d completed successfully\n",
			current->pid, channel_id);
	} while (0);

	unregister_completion(&event);
	if (failed) {
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}

		GLINK_STATUS_MT(s,
			"\tFailed channel test thread %d, channel: %d\n",
			current->pid, channel_id);
		if (handle != NULL) {
			glink_close(handle);
			handle = NULL;
		}
	}
}

void glink_ut_rss_debug_create(const char *ut_name, const char *xprt_name,
				const char *edge_name, testfunc tfunc,
				void (*show)(struct seq_file *)) {

	struct glink_dbgfs dfs;
	struct glink_ut_dbgfs *dbgfs_ut_d;
	char dir_name[2*GLINK_NAME_SIZE];

	dbgfs_ut_d = kzalloc(sizeof(struct glink_ut_dbgfs), GFP_KERNEL);
	if (dbgfs_ut_d == NULL)
		return;
	strlcpy(dbgfs_ut_d->edge_name, edge_name, GLINK_NAME_SIZE);
	strlcpy(dbgfs_ut_d->xprt_name, xprt_name, GLINK_NAME_SIZE);
	dbgfs_ut_d->ut_name = ut_name;
	dbgfs_ut_d->tfunc = tfunc;
	snprintf(dir_name, 2*GLINK_NAME_SIZE, "%s_%s", dbgfs_ut_d->edge_name,
				dbgfs_ut_d->xprt_name);
	dfs.curr_name = dir_name;
	dfs.par_name = "test";
	dfs.b_dir_create = false;
	glink_debugfs_create(ut_name, show, &dfs, (void *)dbgfs_ut_d, true);
}

void glink_ut_dbgfs_worker_func(struct work_struct *work)
{
	struct glink_dbgfs ut_dbgfs;
	const char *xprt_name;
	const char *edge_name;
	char dir_name[2*GLINK_NAME_SIZE];
	struct glink_ut_dbgfs_work  *dfs_work =
		container_of(work, struct glink_ut_dbgfs_work, dbgfs_work);

	ut_dbgfs.curr_name = "test";
	ut_dbgfs.par_name = "glink";
	ut_dbgfs.b_dir_create = true;
	xprt_name = dfs_work->xprt;
	edge_name = dfs_work->edge;
	snprintf(dir_name, 2*GLINK_NAME_SIZE, "%s_%s", edge_name, xprt_name);

	glink_debugfs_create(dir_name, NULL, &ut_dbgfs, NULL, false);

	/*
	 * Add Unit Test entries.
	 *
	 * The idea with unit tests is that you can run all of them
	 * from ADB shell by doing:
	 *  adb shell
	 *  cat ut*
	 *
	 * And if particular tests fail, you can then repeatedly run the
	 * failing tests as you debug and resolve the failing test.
	 *
	 * unit test names: ut[level no]_edge_test
	 * unit test levels: 0, 1, 2
	 *	level 0: Basic tests
	 *	level 1: functional tests
	 *	level 2: performance tests
	 */

	if (!strcmp(xprt_name, "smem")) {
		glink_ut_rss_debug_create("ut0_smem_basic",
					xprt_name, edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ssr0_wcnss_smem",
					xprt_name, edge_name, NULL,
					glink_ut_smem_ssr);
		glink_ut_rss_debug_create("ut1_smem_open_close",
					xprt_name, edge_name,
					ut_loopback_open_close_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_smem_queue_rx_intent",
				xprt_name, edge_name,
				ut_loopback_queue_rx_intent_test,
				glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_smem_stress", xprt_name,
					edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_smem_performance",
					xprt_name, edge_name,
					ut_loopback_performance_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_smem_mt_1_thread",
					xprt_name, edge_name,
					glink_ut1_mpss_mt_test_1_thread,
					glink_mpss_mt_core);
		glink_ut_rss_debug_create("ut1_smem_mt_3_threads",
				xprt_name, edge_name,
				glink_ut1_mpss_mt_test_3_threads,
				glink_mpss_mt_core);
		glink_ut_rss_debug_create("ut1_smem_rx_reuse",
					xprt_name, edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
	} else if (!strcmp(xprt_name, "smd_trans")
			&& !(strcmp(edge_name, "mpss"))) {
		/*below test cases are designed to work only with MPSS */
		glink_ut_rss_debug_create("ut0_smd_trans_migration",
					xprt_name, edge_name, NULL,
					glink_ut0_smd_trans_migration);
		glink_ut_rss_debug_create("ut0_smd_trans_basic_va",
					xprt_name, edge_name, NULL,
					glink_ut0_smd_trans_basic_va);
		glink_ut_rss_debug_create("ut0_smd_trans_basic_vb",
					xprt_name, edge_name, NULL,
					glink_ut0_smd_trans_basic_vb);
		glink_ut_rss_debug_create("ut1_smd_trans_tx", xprt_name,
					edge_name, NULL,
					glink_ut1_smd_trans_tx);
		glink_ut_rss_debug_create("ut1_smd_trans_poll_and_mask",
					xprt_name, edge_name, NULL,
					glink_ut1_smd_trans_poll_and_mask);
		glink_ut_rss_debug_create("ut1_smd_trans_intentless",
					xprt_name, edge_name, NULL,
					glink_ut1_smd_trans_intentless);
		glink_ut_rss_debug_create("ut1_smd_trans_rx_reuse",
					xprt_name, edge_name, NULL,
					glink_ut1_smd_trans_rx_reuse);
	} else if (!strcmp(xprt_name, "lloop")) {
		glink_ut_rss_debug_create("ut0_basic", xprt_name,
					edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut0_sigs", xprt_name,
					edge_name,
					ut_loopback_sigs,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_open_close", xprt_name,
					edge_name,
					ut_loopback_open_close_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_queue_rx_intent",
				xprt_name, edge_name,
				ut_loopback_queue_rx_intent_test,
				glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_tx_with_req_intent",
				xprt_name, edge_name,
				ut_loopback_tx_with_req_intent_test,
				glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_stress", xprt_name,
					edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_performance", xprt_name,
					edge_name,
					ut_loopback_performance_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut0_skb_basic", xprt_name,
					edge_name,
					ut_loopback_skb_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_skb_stress", xprt_name,
					edge_name,
					ut_loopback_skb_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut0_skb_tx_linear_rx_basic",
					xprt_name,
					edge_name,
					ut_loopback_skb_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_skb_tx_linear_rx_stress",
					xprt_name,
					edge_name,
					ut_loopback_skb_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut1_rx_reuse",
					xprt_name, edge_name,
					ut_loopback_stress_test,
					glink_ut_local_basic_core);
		glink_ut_rss_debug_create("ut2_mt_perf_test",
					xprt_name,
					edge_name,
					NULL,
					glink_ut_mt_test_core);
		glink_ut_rss_debug_create("ut1_mt_1_thread",
					xprt_name, edge_name,
					NULL,
					glink_ut1_local_mt_1_thread);
		glink_ut_rss_debug_create("ut1_mt_3_threads",
					xprt_name, edge_name,
					NULL,
					glink_ut1_local_mt_3_threads);
	} else if (!strcmp(xprt_name, "mock")) {
		glink_ut_rss_debug_create("ut0_basic",
					xprt_name, edge_name,
					NULL,
					glink_ut0_mock_basic);
		glink_ut_rss_debug_create("ut0_remote_negotiation",
					xprt_name, edge_name,
					NULL,
					glink_ut0_mock_remote_negotiation);
		glink_ut_rss_debug_create("ut1_size_greater_than",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_size_greater_than);
		glink_ut_rss_debug_create("ut1_transmit_no_rx_intent",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_transmit_no_rx_intent);
		glink_ut_rss_debug_create("ut1_read_no_rx_intent",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_read_no_rx_intent);
		glink_ut_rss_debug_create("ut1_rmt_rx_intent_req",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_rmt_rx_intent_req);
		glink_ut_rss_debug_create("ut1_rx_intent_req",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_rx_intent_req);
		glink_ut_rss_debug_create("ut1_channel_locking_test_1_thread",
				xprt_name, edge_name,
				NULL,
				glink_ut1_mock_channel_locking_test_1_thread);
		glink_ut_rss_debug_create("ut1_channel_locking_test_3_threads",
				xprt_name, edge_name,
				NULL,
				glink_ut1_mock_channel_locking_test_3_threads);
		glink_ut_rss_debug_create("ut1_open_close",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_open_close);
		glink_ut_rss_debug_create("ut1_open_close_remote_first",
					xprt_name, edge_name,
					NULL,
					glink_ut1_mock_open_close_remote_first);
		glink_ut_rss_debug_create("ut0_rwref_lock",
					xprt_name, edge_name,
					NULL,
					glink_ut0_rwref_lock);
		glink_ut_rss_debug_create("ut0_ssr",
					xprt_name, edge_name,
					NULL,
					glink_ut0_mock_ssr);
	}
	kfree(dfs_work);
}

void glink_ut_link_state_cb(struct glink_link_state_cb_info *cb_info,
				void *priv)
{
	struct glink_ut_dbgfs_work *ut_dfs_work;
	if (!cb_info)
		return;
	ut_dfs_work = kzalloc(sizeof(struct glink_ut_dbgfs_work), GFP_KERNEL);
	ut_dfs_work->xprt = cb_info->transport;
	ut_dfs_work->edge = cb_info->edge;

	if (cb_info->link_state == GLINK_LINK_STATE_UP) {
		INIT_WORK(&ut_dfs_work->dbgfs_work, glink_ut_dbgfs_worker_func);
		queue_work(ut_dbgfs_create_workqueue, &ut_dfs_work->dbgfs_work);
	}
}
void glink_ut_debug_remove(void)
{
	struct glink_dbgfs ut_rm_dbgfs;
	ut_rm_dbgfs.curr_name = "test";
	ut_rm_dbgfs.par_name = "glink";
	glink_debugfs_remove_recur(&ut_rm_dbgfs);
}

static int __init glink_init(void)
{

	struct glink_link_info xprt_link_notif = { NULL, NULL,
						glink_ut_link_state_cb};
	struct glink_dbgfs ut_dbgfs;
	ut_dbgfs.curr_name = "glink";
	ut_dbgfs.par_name = "root";
	ut_dbgfs.b_dir_create = true;
	glink_debugfs_create("test", NULL, &ut_dbgfs, NULL, false);

	ut_dbgfs_create_workqueue =
		create_singlethread_workqueue("dbgfs_queue");

	/* Initialize mutex for multithreaded locking test */
	mutex_init(&multithread_test_mutex_lha0);
	glink_mock_xprt_init();
	do_mock_negotiation(NULL, 0x1, 0x0);
	glink_loopback_xprt_init();
	glink_loopback_client_init();
	glink_ut_link_state_notif_handle = glink_register_link_state_cb(
						&xprt_link_notif, NULL);
	return 0;
}

static void glink_exit(void)
{
	glink_unregister_link_state_cb(glink_ut_link_state_notif_handle);
	flush_workqueue(ut_dbgfs_create_workqueue);
	glink_ut_debug_remove();
	destroy_workqueue(ut_dbgfs_create_workqueue);
	glink_mock_xprt_exit();
	glink_loopback_xprt_exit();
}

module_init(glink_init);
module_exit(glink_exit);
MODULE_DESCRIPTION("G-Link Unit Test");
MODULE_LICENSE("GPL v2");
