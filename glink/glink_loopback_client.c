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
#include <linux/ipc_logging.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include "glink_loopback_client.h"
#include "glink_private.h"

static atomic_t request_id = ATOMIC_INIT(0);
struct ut_notify_data data_cb_data;
#define WAIT_TIMEOUT HZ

struct workqueue_struct *glink_lbp_client_wq;

/**
 * struct clnt_rx_intent_req_work - Work structure for client RX intent requests
 * @req_intent_size:	Size of the intent being requested
 * @lpb_ch:		The loopback channel associated with this intent request
 * @work:		Work structure
 */
struct clnt_rx_intent_req_work {
	size_t req_intent_size;
	struct loopback_channel *lpb_ch;
	struct work_struct work;
};

/**
 * rx_done_completion_init() - Initialize rx_done_completion structure
 * @rx_done_comp:	The rx_done_completion structure to be initialized
 */
void rx_done_completion_init(struct rx_done_completion *rx_done_comp)
{
	init_completion(&rx_done_comp->completion);
	spin_lock_init(&rx_done_comp->rx_done_lock_lha0);
}

/**
 * cb_data_init() - Initialize notification callback data structure
 * @cb_data:	The notification callback data structure to be initialized
 */
void cb_data_init(struct ut_notify_data *cb_data)
{
	init_completion(&cb_data->cb_completion);
	cb_data_reset(cb_data);
}

/**
 * cb_data_reset() - Reset the notification callback data structure
 * @cb_data:	The notification callback data structure to be reset
 */
void cb_data_reset(struct ut_notify_data *cb_data)
{
	cb_data->rx_notify = false;
	cb_data->size = 0;
	cb_data->tx_done = false;
	cb_data->send_intent = false;
	cb_data->intent_cb_ntfy = false;
	cb_data->tx_data = NULL;
	cb_data->tx_vbuf_provider = NULL;
	cb_data->tx_pbuf_provider = NULL;
	cb_data->rx_data = NULL;
	cb_data->rx_vbuf_provider = NULL;
	cb_data->rx_pbuf_provider = NULL;

	reinit_completion(&cb_data->cb_completion);
}

/**
 * get_data() - Retrieve vector data using the physical or virtual buffer
 *		provider functions
 * @iovec:		Pointer to the vector
 * @iovec_size:		Size of the data/vector
 * @offset:		Offset from the beginning of the vector
 * @vbuf_provider:	Helper function to iterate the vector in virtual
 *			address space
 * @pbuf_provider:	Helper function to iterate the vector in physical
 *			address space
 * @data_size:		Size of the retrieved data
 */
static void *get_data(void *iovec, size_t iovec_size, size_t offset,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size),
	size_t *data_size)
{
	void *pdata;

	if (vbuf_provider) {
		return vbuf_provider(iovec, offset, data_size);
	} else if (pbuf_provider) {
		pdata = pbuf_provider(iovec, offset, data_size);
		return phys_to_virt((unsigned long)pdata);
	} else {
		*data_size = iovec_size - offset;
		return (void *)iovec + offset;
	}
	return NULL;
}

/**
 * glink_loopback_vector_cmp() - Compare two vectors
 * @cb_data:	The callback notification structure containing the vectors and
 *		the size of the data to compare
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int glink_loopback_vector_cmp(struct ut_notify_data *cb_data)
{
	size_t offset = 0;
	void *buf1;
	size_t buf1_size;
	void *buf2;
	size_t buf2_size;
	size_t cmp_size;
	char *p;
	int i;

	do {
		buf1 = get_data(cb_data->tx_data, cb_data->size, offset,
			cb_data->tx_vbuf_provider, cb_data->tx_pbuf_provider,
			&buf1_size);
		buf2 = get_data(cb_data->rx_data, cb_data->size, offset,
			cb_data->rx_vbuf_provider, cb_data->rx_pbuf_provider,
			&buf2_size);
		cmp_size = min(buf1_size, buf2_size);
		if (memcmp(buf1, buf2, cmp_size))
			break;
		offset += cmp_size;
	} while (offset != cb_data->size);

	if (offset != cb_data->size) {
		GLINK_LL_CLNT_ERR("%s: DATA Mismatch-off:sz %zu:%zu\n",
				__func__, offset, cmp_size);
		p = (char *)buf1;
		for (i = 0; i < cmp_size; i = i + 10)
			GLINK_LL_CLNT_INFO(
	"[tx line idx:%d] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				i, p[i], p[i+1], p[i+2], p[i+3], p[i+4],
				p[i+5], p[i+6], p[i+7], p[i+8], p[i+9]);
		p = (char *)buf2;
		for (i = 0; i < cmp_size; i = i + 10)
			GLINK_LL_CLNT_INFO(
	"[rx line idx:%d] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				i, p[i], p[i+1], p[i+2], p[i+3], p[i+4],
				p[i+5], p[i+6], p[i+7], p[i+8], p[i+9]);
		return -EFAULT;
	}
	return 0;
}

/**
 * glink_make_request_pkt_hdr() - Makes the loopback request packet header
 * @pkt:	The loopback request packet structure
 * @type:	The request type
 *
 * This function fills the loopback request header with the request type,
 * request size, and a request ID, which is unique for each request.
 */
static void glink_make_request_pkt_hdr(struct req *pkt, int type)
{
	GLINK_LL_CLNT_INFO("%s: type[%d]\n", __func__, type);
	pkt->hdr.req_id = atomic_inc_return(&request_id);
	pkt->hdr.req_type = type;
	switch (type) {
	case OPEN:
		pkt->hdr.req_size = sizeof(struct open_req);
		break;
	case CLOSE:
		pkt->hdr.req_size = sizeof(struct close_req);
		break;
	case QUEUE_RX_INTENT_CONFIG:
		pkt->hdr.req_size = sizeof(struct queue_rx_intent_config_req);
		break;
	case TX_CONFIG:
		pkt->hdr.req_size = sizeof(struct tx_config_req);
		break;
	case RX_DONE_CONFIG:
		pkt->hdr.req_size = sizeof(struct rx_done_config_req);
		break;
	default:
		GLINK_LL_CLNT_ERR("%s: unknown request type[%d]\n", __func__,
				type);
		break;
	}
}

/**
 * glink_loopback_send_request() - Sends the loopback request to the loopback
 *				server
 * @handle:	Handle returned by glink_loopback_open()
 * @pkt:	Loopback request packet structure
 * @type:	Type of the loopback request
 * @req_intent:	Specifies whether or not the tx() call should block waiting
 *		for an RX intent
 *
 * This function creates and sends a loopback request to the loopback server
 * and queues an RX intent to receive the response.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_send_request(void *handle, struct req *pkt, int type,
		bool req_intent)
{
	int ret;
	unsigned long flags;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;
	const char *transport = lpb_ch->open_cfg.transport;
	const char *edge = lpb_ch->open_cfg.edge;
	const char *name = lpb_ch->open_cfg.name;
	struct rx_done_completion *rx_done_comp;
	uint32_t tx_flags = req_intent ? GLINK_TX_REQ_INTENT : 0;

	rx_done_comp = kzalloc(sizeof(struct rx_done_completion), GFP_KERNEL);
	if (!rx_done_comp) {
		GLINK_LL_CLNT_ERR("%s:%s:%s: %s: %s\n",
			transport, edge, name, __func__,
			"Could not allocate rx_done_completion");
		return -ENOMEM;
	}
	rx_done_completion_init(rx_done_comp);
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: orphaned flag just after init: %d\n",
			transport, edge, name, __func__,
			rx_done_comp->orphaned);

	GLINK_LL_CLNT_INFO("%s:%s:%s: %s: Queueing rx_intent packet\n",
			transport, edge, name, __func__);
	ret = glink_queue_rx_intent(lpb_ch->handle, (void *)rx_done_comp,
			sizeof(struct resp));
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d]\n",
			transport, edge, name, __func__,
			"glink_queue_rx_intent_failed", ret);
		return ret;
	}

	glink_make_request_pkt_hdr(pkt, type);

	cb_data_reset(&lpb_ch->cb_data);
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: Sending request packet Type[%d]\n",
			transport, edge, name, __func__, type);
	ret = glink_tx(lpb_ch->handle, (void *)pkt, (void *)pkt,
						sizeof(struct req), tx_flags);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: glink_tx failed ret[%d]\n",
				transport, edge, name, __func__, ret);
		return ret;
	}
	ret = wait_for_completion_timeout(&rx_done_comp->completion,
			WAIT_TIMEOUT);

	spin_lock_irqsave(&rx_done_comp->rx_done_lock_lha0, flags);
	if (ret == 0) {
		ret = completion_done(&rx_done_comp->completion);
		if (!ret) {
			rx_done_comp->orphaned = true;
			GLINK_LL_CLNT_DBG("%s:%s:%s %s: %s\n",
					transport, edge, name, __func__,
					"Set orphaned flag to true");
		}
	}
	spin_unlock_irqrestore(&rx_done_comp->rx_done_lock_lha0, flags);

	if (ret == 0 || lpb_ch->cb_data.rx_notify != true
			|| lpb_ch->cb_data.size != sizeof(struct resp)) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d] %s[%d] size[%zu]\n",
			transport, edge, name, __func__, "Wrong Response", ret,
			"rx_notify", lpb_ch->cb_data.rx_notify,
			lpb_ch->cb_data.size);
		return -ETIMEDOUT;
	}

	glink_rx_done(lpb_ch->handle, lpb_ch->cb_data.rx_data, false);

	kfree(rx_done_comp);
	return 0;
}

/**
 * glink_loopback_rmt_rx_intent_req_worker() - Remote RX intent request worker
 *					function
 * @work:	Work structure
 *
 * This worker function queues an RX intent of the requested size on the given
 * loopback channel. The channel handle and intent size are provided in the
 * container of @work.
 */
static void glink_loopback_rmt_rx_intent_req_worker(struct work_struct *work)
{
	int ret;
	struct clnt_rx_intent_req_work *work_item =
				container_of(work,
				struct clnt_rx_intent_req_work, work);
	struct loopback_channel *lpb_ch = work_item->lpb_ch;

	ret = glink_queue_rx_intent(lpb_ch->handle, lpb_ch,
			work_item->req_intent_size);
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: Triggered with size[%zu] ret[%d]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, work_item->req_intent_size, ret);
	if (ret)
		GLINK_LL_CLNT_ERR("%s:%s:%s %s queue_rx_intent failed\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				__func__);
	kfree(work_item);
	return;
}

/**
 * glink_loopback_notify_rx_cb() - Receive notification callback
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @pkt_priv:	An rx_done_completion structure
 * @ptr:	Pointer to the receive data
 * @size:	Size of the receive data
 */
void glink_loopback_notify_rx_cb(void *handle, const void *priv,
		const void *pkt_priv, const void *ptr, size_t size)
{
	unsigned long flags;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;
	struct rx_done_completion *rx_done_comp =
		(struct rx_done_completion *)pkt_priv;

	GLINK_LL_CLNT_INFO("%s:%s:%s %s: priv[%p] data[%p] size[%zu]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, pkt_priv, (char *)ptr, size);
	lpb_ch->cb_data.size = size;
	lpb_ch->cb_data.rx_notify = true;
	lpb_ch->cb_data.rx_data = (void *)ptr;

	if (lpb_ch->ch_type == CH_CNTL_TYPE) {
		spin_lock_irqsave(&rx_done_comp->rx_done_lock_lha0, flags);
		if (rx_done_comp->orphaned) {
			GLINK_LL_CLNT_ERR("%s:%s:%s: %s: %s\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				__func__, "Receive data orphaned");
			kfree(rx_done_comp);
		} else {
			complete(&rx_done_comp->completion);
			GLINK_LL_CLNT_INFO("%s:%s:%s %s: Called complete()%s\n",
					lpb_ch->open_cfg.transport,
					lpb_ch->open_cfg.edge,
					lpb_ch->open_cfg.name,
					__func__,
					" for rx_done completion");
		}
		spin_unlock_irqrestore(&rx_done_comp->rx_done_lock_lha0, flags);
	} else {
		complete(&lpb_ch->cb_data.cb_completion);
	}
}

/**
 * glink_loopback_notify_rx_nowait_cb() - Receive notification callback for the
 *					no-wait receive mode
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @pkt_priv:	Notification callback data structure
 * @ptr:	Pointer to the receive data
 * @size:	Size of the receive data
 */
void glink_loopback_notify_rx_nowait_cb(void *handle, const void *priv,
		const void *pkt_priv, const void *ptr, size_t size)
{
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;
	struct mt_cb_data *cb_data = (struct mt_cb_data *)pkt_priv;

	atomic_inc(&cb_data->num_rx_pkts);
	GLINK_LL_CLNT_INFO_PERF(
		"%s:%s:%s %s: end (Success) RX priv[%p] data[%p] size[%zu]\n",
		lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
		lpb_ch->open_cfg.name, __func__, pkt_priv, (char *)ptr,
		size);
	if (atomic_read(&cb_data->num_rx_pkts) ==
		atomic_read(&cb_data->num_tx_pkts))
		wake_up(&cb_data->test_comp);
	glink_rx_done(lpb_ch->handle, ptr, false);
}

/**
 * glink_loopback_notify_rxv_cb() - Receive notification callback for packets
 *				transmitted in vector form
 * @handle:		Handle returned by glink_loopback_open()
 * @priv:		The loopback channel
 * @pkt_priv:		A completion used to indicate data receipt in control
 *			channels
 * @iovec:		Pointer to the vector
 * @size:		Size of data/vector
 * @vbuf_provider:	Helper function to iterate the vector in virtual address
 *			space
 * @pbuf_provider:	Helper function to iterate the vector in physical
 *			address space
 */
void glink_loopback_notify_rxv_cb(void *handle, const void *priv,
		const void *pkt_priv, void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size))
{
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;

	GLINK_LL_CLNT_INFO("%s:%s:%s %s: priv[%p] data[%p] size[%zu]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, pkt_priv, (char *)iovec, size);
	lpb_ch->cb_data.size = size;
	lpb_ch->cb_data.rx_notify = true;
	lpb_ch->cb_data.rx_data = (void *)iovec;
	lpb_ch->cb_data.rx_vbuf_provider = vbuf_provider;
	lpb_ch->cb_data.rx_pbuf_provider = pbuf_provider;
	if (lpb_ch->ch_type == CH_CNTL_TYPE)
		complete((struct completion *)pkt_priv);
	else
		complete(&lpb_ch->cb_data.cb_completion);
}

/**
 * glink_loopback_notify_tx_done_cb() - Callback used to notify the client when
 *					TX data has finished transmitting
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @pkt_priv:	Private packet data
 * @ptr:	Pointer to the data that was sent
 */
void glink_loopback_notify_tx_done_cb(void *handle, const void *priv,
		const void *pkt_priv, const void *ptr)
{
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;
	GLINK_LL_CLNT_INFO_PERF(
		"%s:%s:%s %s: ptr[%p]\n",
		lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
		lpb_ch->open_cfg.name, __func__, ptr);
	lpb_ch->cb_data.tx_done = true;
}

/**
 * glink_loopback_notify_state_cb() - Callback used to notify the client when
 *				the channel state has changed
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @event:	The new state of the loopback channel
 */
void glink_loopback_notify_state_cb(void *handle, const void *priv,
		unsigned event)
{
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: event[%d], Thread: %d\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, event, current->pid);
	if (event == GLINK_CONNECTED
		|| event == GLINK_LOCAL_DISCONNECTED) {
		lpb_ch->cb_data.event = event;
		complete(&lpb_ch->cb_data.cb_completion);
	}
}

/**
 * glink_loopback_rmt_rx_intent_req_cb() - Callback used to notify the client
 *					when the remote side has requested that
 *					an RX intent be queued
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @sz:		The size of the requested intent
 *
 * This function initiates a work item that queues the requested intent.
 */
bool glink_loopback_rmt_rx_intent_req_cb(void *handle, const void *priv,
		size_t sz)
{
	struct clnt_rx_intent_req_work *work_item;

	work_item = kmalloc(sizeof(struct clnt_rx_intent_req_work), GFP_ATOMIC);
	if (!work_item) {
		GLINK_LL_CLNT_ERR("%s failed allocate work_item\n", __func__);
		return false;
	}

	work_item->req_intent_size = sz;
	work_item->lpb_ch = (struct loopback_channel *)priv;
	INIT_WORK(&work_item->work, glink_loopback_rmt_rx_intent_req_worker);
	queue_work(glink_lbp_client_wq, &work_item->work);
	return true;
}

/**
 * glink_loopback_notify_rx_sigs_cb - Handle the event of TIOCM signal changes
 * @handle:	Handle returned by glink_loopback_open()
 * @priv:	The loopback channel
 * @old_sigs:	The old TIOCM signals
 * @new_sigs:	The new TIOCM signals
 */
void glink_loopback_notify_rx_sigs_cb(void *handle, const void *priv,
				uint32_t old_sigs, uint32_t new_sigs)
{
	struct loopback_channel *lpb_ch = (struct loopback_channel *)priv;
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: sigs [0x%x]->[0x%X], Thread: %d\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, old_sigs, new_sigs, current->pid);
	lpb_ch->cb_data.old_sigs = old_sigs;
	lpb_ch->cb_data.new_sigs = new_sigs;
	complete(&lpb_ch->cb_data.cb_completion);
}

/**
 * glink_loopback_open() - Open the loopback channel
 * @transport:	Transport of the control or data channel
 * @edge:	Edge of the control or data channel
 * @name:	Name of the control or data channel
 * @ch_type:	Type of control or data channel
 * @rx_reuse:	Reuse RX intents
 *
 * This function is used to open the loopback channel on @transport & @edge with
 * @name and waits until the GLINK_CONNECTED event is received or a timeout is
 * reached.
 *
 * Return: Handle to loopback channel or NULL in error case
 */
void *glink_loopback_open(const char *transport, const char *edge,
			  const char *name, int ch_type, int rx_type,
			  bool rx_reuse)
{
	int ret;
	struct loopback_channel *lpb_ch;

	if (!transport || !edge || !name) {
		GLINK_LL_CLNT_ERR("%s: Incorrect open configurations\n",
				__func__);
		return NULL;
	}

	lpb_ch = kzalloc(sizeof(struct loopback_channel), GFP_KERNEL);
	if (!lpb_ch) {
		GLINK_LL_CLNT_ERR("%s: No memory for allocation\n", __func__);
		return NULL;
	}

	lpb_ch->ch_type = ch_type;

	memset(&lpb_ch->open_cfg, 0, sizeof(struct glink_open_config));
	lpb_ch->open_cfg.transport = transport;
	lpb_ch->open_cfg.edge = edge;
	lpb_ch->open_cfg.name = name;
	lpb_ch->rx_reuse = rx_reuse;

	if (rx_type == LINEAR_RX || rx_type == LINEAR_VECTOR_RX)
		lpb_ch->open_cfg.notify_rx = glink_loopback_notify_rx_cb;
	if (rx_type == VECTOR_RX || rx_type == LINEAR_VECTOR_RX)
		lpb_ch->open_cfg.notify_rxv = glink_loopback_notify_rxv_cb;
	if (rx_type == LINEAR_RX_NOWAIT)
		lpb_ch->open_cfg.notify_rx = glink_loopback_notify_rx_nowait_cb;
	lpb_ch->open_cfg.notify_tx_done = glink_loopback_notify_tx_done_cb;
	lpb_ch->open_cfg.notify_state = glink_loopback_notify_state_cb;
	lpb_ch->open_cfg.notify_rx_intent_req =
		glink_loopback_rmt_rx_intent_req_cb;
	lpb_ch->open_cfg.notify_rx_sigs = glink_loopback_notify_rx_sigs_cb;
	lpb_ch->open_cfg.notify_rx_abort = NULL;
	lpb_ch->open_cfg.notify_tx_abort = NULL;
	lpb_ch->open_cfg.priv = lpb_ch;

	cb_data_init(&lpb_ch->cb_data);
	cb_data_reset(&lpb_ch->cb_data);

	lpb_ch->handle = glink_open(&lpb_ch->open_cfg);
	if (IS_ERR_OR_NULL(lpb_ch->handle)) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: unable to open channel\n",
				transport, edge, name, __func__);
		kfree(lpb_ch);
		return NULL;
	}

	ret = wait_for_completion_timeout(&lpb_ch->cb_data.cb_completion,
			WAIT_TIMEOUT);

	if (ret == 0 || lpb_ch->cb_data.event != GLINK_CONNECTED) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s Thread: %d\n",
			transport, edge, name, __func__,
			"Connected event TIMED OUT.", current->pid);
		glink_loopback_close(lpb_ch);
		return NULL;
	}

	return lpb_ch;
}

/**
 * glink_loopback_close() - Close the loopback channel
 * @handle:	Handle returned by glink_loopback_open()
 *
 * This function is used to close the loopback channel and waits until the
 * GLINK_DISCONNECTED event is received or a timeout is reached.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_close(void *handle)
{
	int ret;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;

	if (!lpb_ch) {
		GLINK_LL_CLNT_ERR("%s: NULL Pointer.\n", __func__);
		return -EINVAL;
	}

	cb_data_reset(&lpb_ch->cb_data);
	ret = glink_close(lpb_ch->handle);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name, __func__,
				"unable to close channel.", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&lpb_ch->cb_data.cb_completion,
								WAIT_TIMEOUT);

	if (ret == 0 || lpb_ch->cb_data.event != GLINK_LOCAL_DISCONNECTED) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d] event[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				"Local Disconnected event TIMED OUT",
				__func__, ret, lpb_ch->cb_data.event);
		return -ETIMEDOUT;
	}

	kfree(lpb_ch);
	return 0;
}

/**
 * glink_loopback_tx() - Send the data on loopback channel
 * @handle:	Handle returned by glink_loopback_open()
 * @data:	Data to send on the loopback channel
 * @size:	Size of the data to send on the loopback channel
 * @req_intent:	Remote intent request flag
 * @first_tx:	First tx since channel was opened
 * @rx_reuse:	Reuse the RX intents
 *
 * This function sends data on the loopback channel, queues an RX intent to
 * receive the loopback data, and waits until the loopback data is received
 * or a timeout is reached.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_tx(void *handle, void *data, size_t size, bool req_intent,
		      bool first_tx, bool rx_reuse)
{
	int ret;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;
	int i;
	char *p;
	uint32_t tx_flags = req_intent ? GLINK_TX_REQ_INTENT : 0;

	GLINK_LL_CLNT_INFO("%s:%s:%s %s: %s\n", lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge, lpb_ch->open_cfg.name,
			__func__, "Queueing rx_intent for loopback data");
	if (!rx_reuse || (rx_reuse && first_tx)) {
		ret = glink_queue_rx_intent(lpb_ch->handle, (void *)lpb_ch,
									size);
		if (ret) {
			GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge, lpb_ch->open_cfg.name,
				__func__, "glink_queue_rx_intent failed", ret);
			return ret;
		}
	}

	cb_data_reset(&lpb_ch->cb_data);
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: Sending data[%p] size[%zu]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, data, size);
	ret = glink_tx(lpb_ch->handle, (void *)data, (void *)data, size,
			tx_flags);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: glink_tx failed ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				__func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&lpb_ch->cb_data.cb_completion,
			WAIT_TIMEOUT);
	if (ret == 0 || lpb_ch->cb_data.size != size) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"data Rx TIMED out or Not exact data");
		return -ETIMEDOUT;
	}

	ret = 0;
	if (lpb_ch->ch_type == CH_DATA_TYPE) {
		if (memcmp(lpb_ch->cb_data.rx_data, data, size)) {
			GLINK_LL_CLNT_ERR("%s:%s:%s %s: DATA Mismatch\n",
					lpb_ch->open_cfg.transport,
					lpb_ch->open_cfg.edge,
					lpb_ch->open_cfg.name,
					__func__);
			p = (char *)data;
			for (i = 0; i < size; i = i + 10) {
				GLINK_LL_CLNT_INFO(
	"[tx line idx:%d] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				i, p[i], p[i+1], p[i+2], p[i+3], p[i+4],
				p[i+5], p[i+6], p[i+7], p[i+8], p[i+9]);
			}
			p = (char *)lpb_ch->cb_data.rx_data;
			for (i = 0; i < size; i = i + 10) {
				GLINK_LL_CLNT_INFO(
	"[rx line idx:%d] 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
				i, p[i], p[i+1], p[i+2], p[i+3], p[i+4],
				p[i+5], p[i+6], p[i+7], p[i+8], p[i+9]);
			}
			ret = -EFAULT;
		} else {
			GLINK_LL_CLNT_INFO("%s:%s:%s %s: DATA Matched\n",
					lpb_ch->open_cfg.transport,
					lpb_ch->open_cfg.edge,
					lpb_ch->open_cfg.name,
					__func__);
		}
	}
	glink_rx_done(lpb_ch->handle, lpb_ch->cb_data.rx_data, rx_reuse);
	return ret;
}

/**
 * glink_loopback_tx_nowait() - Send the data on loopback channel
 *				without waiting for the echo of the data.
 * @handle:	Handle returned by glink_open()
 * @data:	Data to send on the loopback channel
 * @size:	Size of the data to send on the loopback channel
 * @req_intent:	Remote intent request flag
 * @cb_data:	Callback data to be used when the data arrives back
 *
 * This function is used to send the data on the loopback channel
 * and queue the RX intent to receive the loopback data.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_tx_nowait(void *handle, void *data, size_t size,
		      bool req_intent, struct mt_cb_data *cb_data)
{
	int ret;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;
	uint32_t tx_flags = req_intent ? GLINK_TX_REQ_INTENT : 0;

	GLINK_LL_CLNT_INFO("%s:%s:%s %s: %s\n", lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge, lpb_ch->open_cfg.name,
			__func__, "Queueing rx_intent for loopback data");
	ret = glink_queue_rx_intent(lpb_ch->handle, (void *)cb_data, size);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge, lpb_ch->open_cfg.name,
				__func__, "glink_queue_rx_intent failed", ret);
		return ret;
	}

	atomic_inc(&cb_data->num_tx_pkts);
	GLINK_LL_CLNT_INFO_PERF("%s:%s:%s %s: start TX data[%p] size[%zu]\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__, data, size);
	ret = glink_tx(lpb_ch->handle, (void *)data, (void *)data, size,
			tx_flags);
	if (ret)
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: glink_tx failed ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name,
				__func__, ret);
	return ret;
}

/**
 * glink_loopback_txv() - Send data on the loopback channel in vector form
 * @handle:		Handle returned by glink_loopback_open()
 * @iovec:		Pointer to the vector
 * @size:		Size of data/vector
 * @vbuf_provider:	Client provided helper function to iterate the vector
 *			in virtual address space
 * @pbuf_provider:	Client provided helper function to iterate the vector
 *			in physical address space
 * @req_intent:		Flag indicating whether or not to request an intent
 *			from the remote side
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_txv(void *handle, void *iovec, size_t size,
	void * (*vbuf_provider)(void *iovec, size_t offset, size_t *size),
	void * (*pbuf_provider)(void *iovec, size_t offset, size_t *size),
	bool req_intent)
{
	int ret;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;
	uint32_t tx_flags = req_intent ? GLINK_TX_REQ_INTENT : 0;

	if (!vbuf_provider && !pbuf_provider)
		return -EINVAL;

	GLINK_LL_CLNT_INFO("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"Queueing rx_intent for loopback data");

	ret = glink_queue_rx_intent(lpb_ch->handle, (void *)lpb_ch, size);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s ret[%d]\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"glink_queue_rx_intent failed", ret);
		return ret;
	}

	cb_data_reset(&lpb_ch->cb_data);
	lpb_ch->cb_data.tx_data = iovec;
	lpb_ch->cb_data.tx_vbuf_provider = vbuf_provider;
	lpb_ch->cb_data.tx_pbuf_provider = pbuf_provider;
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: Sending data[%p] size[%zu]\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__, iovec, size);
	ret = glink_txv(lpb_ch->handle, (void *)iovec, (void *)iovec, size,
			vbuf_provider, pbuf_provider, tx_flags);
	if (ret) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: glink_tx failed ret[%d]\n",
				lpb_ch->open_cfg.transport,
				lpb_ch->open_cfg.edge,
				lpb_ch->open_cfg.name, __func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&lpb_ch->cb_data.cb_completion,
			WAIT_TIMEOUT);
	if (ret == 0 || lpb_ch->cb_data.size != size) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"data Rx TIMED out or Not exact data");
		return -ETIMEDOUT;
	}

	ret = glink_loopback_vector_cmp(&lpb_ch->cb_data);
	glink_rx_done(lpb_ch->handle, lpb_ch->cb_data.rx_data, false);
	return ret;
}

/**
 * glink_loopback_sigs_set() - Set the signals on a loopback channel
 * @handle:	Handle returned by glink_open()
 * @sigs:	New signals to be set to the channel
 *
 * This function is used to set the signals for a loopback channel. It waits for
 * the remote loopback channel to be set with the same signals and returns the
 * callback.
 *
 * Return: 0 on success, standard error codes otherwise
 */
int glink_loopback_sigs_set(void *handle, uint32_t sigs)
{
	int ret;
	uint32_t newsigs;
	struct loopback_channel *lpb_ch = (struct loopback_channel *)handle;

	cb_data_reset(&lpb_ch->cb_data);
	GLINK_LL_CLNT_INFO("%s:%s:%s %s: Sending sigs[%d]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, sigs);
	ret = glink_sigs_set(lpb_ch->handle, sigs);
	if (ret) {
		GLINK_LL_CLNT_ERR(
			"%s:%s:%s %s: glink_sigs_set failed ret[%d]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&lpb_ch->cb_data.cb_completion,
			WAIT_TIMEOUT);
	if (ret == 0 || lpb_ch->cb_data.new_sigs != sigs) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"set Sigs Timeout or Mis match");
		return -ETIMEDOUT;
	}

	newsigs = glink_sigs_local_get(lpb_ch->handle);
	if (newsigs < 0) {
		GLINK_LL_CLNT_ERR(
			"%s:%s:%s %s: glink_sigs_local_get failed ret[%d]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, ret);
		return ret;
	}
	if (newsigs != sigs) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"local Sigs Not set correctly");
		return -EINVAL;
	}

	newsigs = glink_sigs_remote_get(lpb_ch->handle);
	if (newsigs < 0) {
		GLINK_LL_CLNT_ERR(
			"%s:%s:%s %s: glink_sigs_remote_get failed ret[%d]\n",
			lpb_ch->open_cfg.transport,
			lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name,
			__func__, ret);
		return ret;
	}
	if (newsigs != sigs) {
		GLINK_LL_CLNT_ERR("%s:%s:%s %s: %s\n",
			lpb_ch->open_cfg.transport, lpb_ch->open_cfg.edge,
			lpb_ch->open_cfg.name, __func__,
			"Remote Sigs Not set correctly");
		return -EINVAL;
	}

	return 0;
}

/**
 * glink_loopback_client_init() - Basic loopback client init
 *
 * This function performs the basic initialization for the loopback
 * client.
 */
void glink_loopback_client_init(void)
{
	glink_lbp_client_wq = create_singlethread_workqueue("glink_lbp_client");
	if (!glink_lbp_client_wq) {
		GLINK_LL_CLNT_ERR("%s: Error creating glink_lbp_client_wq\n",
				__func__);
		return;
	}
}
