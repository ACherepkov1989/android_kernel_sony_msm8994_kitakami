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
#include <linux/ipc_logging.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <soc/qcom/glink.h>
#include "glink_core_if.h"
#include "glink_mock_xprt.h"
#include "glink_private.h"
#include "glink_xprt_if.h"

#define UINT32_MAX (0xFFFFFFFFU)

enum {
	MOCK_INFO = 1U << 0,
	MOCK_DEBUG = 1U << 1,
	MOCK_PERF = 1U << 2,
};

static unsigned glink_mock_debug_mask;
module_param_named(mock_debug_mask, glink_mock_debug_mask,
		   uint, S_IRUGO | S_IWUSR | S_IWGRP);

#define GLINK_MOCK_PERF(fmt, args...) do {	\
	if (glink_get_mock_debug_mask() & MOCK_PERF) \
		GLINK_IPC_LOG_STR("<PERF> mock:local " fmt, args); \
} while (0)

#define GLINK_MOCK_INFO(fmt, args...) do {	\
	if (glink_get_mock_debug_mask() & MOCK_INFO) \
		GLINK_IPC_LOG_STR("<MOCK> mock:local " fmt, args); \
} while (0)

#define GLINK_MOCK_INFO_PERF(fmt, args...) do {	\
	if (glink_get_mock_debug_mask() & (MOCK_INFO | MOCK_PERF)) \
		GLINK_IPC_LOG_STR("<MOCK> mock:local " fmt, args); \
} while (0)

#define GLINK_MOCK_DBG(fmt, args...) do {	\
	if (glink_get_mock_debug_mask() & MOCK_DEBUG) \
		GLINK_IPC_LOG_STR("<MOCK> mock:local " fmt, args); \
} while (0)

#define GLINK_MOCK_ERR(fmt, args...) do {	\
	if (!(glink_get_mock_debug_mask() & MOCK_PERF)) \
		pr_err("<MOCK> mock:local " fmt, args); \
	GLINK_MOCK_DBG(fmt, args);	\
} while (0)

/**
 * glink_get_mock_debug_mask - Return debug mask attribute
 *
 * Return: debug mask attribute
 */
unsigned glink_get_mock_debug_mask(void)
{
	return glink_mock_debug_mask;
}
EXPORT_SYMBOL(glink_get_mock_debug_mask);

/*
 * Exposed mock transport
 * Note - this must be unique per edge
 *
 * This mock transport is populated with the functions
 * defined in this module and returned at the user's request. It is
 * to be used a normal transport in a test environment.
 */
static struct glink_mock_xprt if_impl;

static uint32_t negotiate_features_v1(struct glink_transport_if *if_ptr,
		const struct glink_core_version *version_ptr,
		uint32_t features);
static void do_complete_all(void);
static void do_reinit_completion(void);

/* Versions data structure, used in version negotiation. */
static struct glink_core_version versions[] = {
	{1, 0x00, negotiate_features_v1},
	{2, 0x22, negotiate_features_v1},
	{5, 0x55, negotiate_features_v1},
};

/**
 * negotiate_features_v1() - Get feature set
 * @if_ptr:		Pointer to interface instance
 * @version_ptr:	Pointer to G-Link core version instance
 * @features:		Features
 *
 * Return: Features masked off by the supported features
 */
static uint32_t negotiate_features_v1(struct glink_transport_if *if_ptr,
		const struct glink_core_version *version_ptr,
		uint32_t features)
{
	GLINK_MOCK_DBG("%s: %x:%08x\n", __func__,
			1, features & version_ptr->features);
	return features & version_ptr->features;
}

/**
 * init_lists() - Initialize the linked lists in a glink_mock_xprt struct
 * @mock:	The mock transport whose lists are to be initialized
 */
static void init_lists(struct glink_mock_xprt *mock)
{
	INIT_LIST_HEAD(&mock->rx_intents);
	INIT_LIST_HEAD(&mock->tx_data);
	INIT_LIST_HEAD(&mock->tx_cmds);
	INIT_LIST_HEAD(&mock->completions);
}

/**
 * reset() - Reset mock transport data to default values
 * @mock:	Mock transport data
 *
 * All lists are emptied and completions are reset.
 */
static void reset(struct glink_mock_xprt *mock)
{
	unsigned long flags;

	spin_lock_irqsave(&mock->mock_xprt_lists_lock_lha0, flags);
	do_reinit_completion();

	if (!list_empty(&mock->completions))
		GLINK_MOCK_ERR("%s: Error: %s\n",
				"Completions list not empty at reset",
				__func__);

	while (!list_empty(&mock->tx_cmds)) {
		struct glink_mock_cmd *temp;
		temp = list_first_entry(&mock->tx_cmds,
				struct glink_mock_cmd,
				element);
		list_del(&temp->element);
		kfree(temp);
	}

	while (!list_empty(&mock->rx_intents)) {
		struct glink_mock_rx_intent *temp;
		temp = list_first_entry(&mock->rx_intents,
				struct glink_mock_rx_intent,
				element);
		list_del(&temp->element);
		kfree(temp);
	}

	while (!list_empty(&mock->tx_data)) {
		struct glink_mock_tx_data *temp;
		temp = list_first_entry(&mock->tx_data,
				struct glink_mock_tx_data,
				element);
		list_del(&temp->element);
		kfree(temp);
	}
	spin_unlock_irqrestore(&mock->mock_xprt_lists_lock_lha0, flags);
}

/**
 * ssr() - Perform any necessary cleanup at the transport level
 * @if_ptr:	Pointer to interface instance
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int ssr(struct glink_transport_if *if_ptr)
{
	if_impl.if_ptr.glink_core_if_ptr->link_down(&if_impl.if_ptr);
	return 0;
}

/**
 * tx_cmd_version() - Transmit a version command for local negotiation
 * @if_ptr:	Pointer to interface instance
 * @version:	Version
 * @features:	Features
 *
 * Expected response is glink_core_if::rx_cmd_version_ack.
 */
static void tx_cmd_version(struct glink_transport_if *if_ptr,
		uint32_t version,
		uint32_t features)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	GLINK_MOCK_DBG("%s: VERSION %x:%08x\n", __func__, version, features);
	cmd->type = VERSION;
	cmd->version.version = version;
	cmd->version.features = features;
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * tx_cmd_version_ack() - Transmit a version ack for remote negotiation
 * @if_ptr:	Pointer to interface instance
 * @version:	Version
 * @features:	Features
 */
static void tx_cmd_version_ack(struct glink_transport_if *if_ptr, uint32_t
		version, uint32_t features)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	GLINK_MOCK_DBG("%s: VERSION_ACK %x:%08x\n", __func__,
			version, features);
	cmd->type = VERSION_ACK;
	cmd->version.version = version;
	cmd->version.features = features;
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * set_version() - Signals that negotiation is complete
 * @if_ptr:	Pointer to interface instance
 * @version:	Version
 * @features:	Features
 *
 * Signals that negotiation is complete and the transport can now do version-
 * specific initialization.
 *
 * Return: The supported capabilities of the transport.
 */
static uint32_t set_version(struct glink_transport_if *if_ptr, uint32_t version,
		uint32_t features)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return 0;
	}

	/*
	 * This isn't a command that goes out over the wire - it is just used
	 * internally to configure the transport.
	 */
	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	cmd->type = SET_VERSION;
	cmd->version.version = version;
	cmd->version.features = features;
	GLINK_MOCK_DBG("%s: SET_VERSION %x:%08x\n", __func__,
			version, features);

	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_ch_open() - Sends the open command
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @name:	Channel name
 * @req_xprt:	The transport the core would like to migrate this channel to.
 *
 * The expected response is glink_core_if::rx_cmd_ch_open_ack.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_ch_open(struct glink_transport_if *if_ptr, uint32_t lcid,
		const char *name, uint16_t req_xprt)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	cmd->type = LOCAL_OPEN;
	cmd->local_open.lcid = lcid;
	strlcpy(cmd->local_open.name, name, sizeof(cmd->local_open.name));
	GLINK_MOCK_DBG("%s[%u:] %s: LOCAL_OPEN\n", name, lcid, __func__);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_ch_close() - Sends the close command
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 *
 * The expected response is glink_core_if::rx_cmd_ch_close_ack.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_ch_close(struct glink_transport_if *if_ptr, uint32_t lcid)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	cmd->type = LOCAL_CLOSE;
	cmd->local_close.lcid = lcid;
	GLINK_MOCK_DBG("lcid: %u %s: LOCAL_CLOSE\n", lcid, __func__);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_ch_remote_open_ack() - Sends the remote open ACK command
 * @if_ptr:	Pointer to interface instance
 * @rcid:	Remote channel ID
 * @xprt_resp:	The response to a transport migration request.
 *
 * Sends the remote open ACK command in response to receiving
 * glink_core_if::rx_cmd_ch_remote_open.
 */
static void tx_cmd_ch_remote_open_ack(struct glink_transport_if *if_ptr,
		uint32_t rcid, uint16_t xprt_resp)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	cmd->type = REMOTE_OPEN_ACK;
	cmd->remote_open_ack.rcid = rcid;
	GLINK_MOCK_DBG("rcid: %u %s: REMOTE_OPEN_ACK\n", rcid, __func__);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * tx_cmd_ch_remote_close_ack() - Sends the remote close ACK command
 * @if_ptr:	Pointer to interface instance
 * @rcid:	Remote channel ID
 *
 * Sends the remote open ACK command in response to receiving
 * glink_core_if::rx_cmd_ch_remote_close.
 */
static void tx_cmd_ch_remote_close_ack(struct glink_transport_if *if_ptr,
		uint32_t rcid)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	cmd->type = REMOTE_CLOSE_ACK;
	cmd->remote_close_ack.rcid = rcid;
	GLINK_MOCK_DBG("rcid: %u %s: REMOTE_CLOSE_ACK\n", rcid, __func__);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * allocate_rx_intent - Allocate/reserve space for an RX Intent
 * @if_ptr:	The transport the intent is associated with
 * @size:	Size of intent
 * @intent:	Pointer to the intent structure
 *
 * Note that returning NULL for the pointer is valid (it means that space has
 * been reserved, but the actual pointer will be provided later).
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int allocate_rx_intent(struct glink_transport_if *if_ptr, size_t size,
			      struct glink_core_rx_intent *intent)
{
	if (!intent)
		return -EINVAL;

	intent->data = NULL;
	intent->iovec = NULL;
	intent->vprovider = NULL;
	intent->pprovider = NULL;
	return 0;
}

/**
 * deallocate_rx_intent() - Deallocate space created for RX Intent
 * @if_ptr:	The transport the intent is associated with
 * @intent:	Pointer to the intent structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int deallocate_rx_intent(struct glink_transport_if *if_ptr,
				struct glink_core_rx_intent *intent)
{
	if (!intent)
		return -EINVAL;

	intent->data = NULL;
	intent->iovec = NULL;
	intent->vprovider = NULL;
	intent->pprovider = NULL;
	return 0;
}

/**
 * reuse_rx_intent() - Reuse space created for an RX Intent
 * @if_ptr:	The transport the intent is associated with
 * @intent:	Pointer to the intent structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int reuse_rx_intent(struct glink_transport_if *if_ptr,
			   struct glink_core_rx_intent *intent)
{
	if (!intent)
		return -EINVAL;

	intent->data = NULL;
	intent->iovec = NULL;
	intent->vprovider = NULL;
	intent->pprovider = NULL;
	return 0;
}

/**
 * tx_cmd_local_rx_intent() - Send RX intent ID
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @size:	Size of receive intent
 * @liid:	Local intent ID
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_local_rx_intent(struct glink_transport_if *if_ptr,
		uint32_t lcid, size_t size, uint32_t liid)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_rx_intent *intent =
		kzalloc(sizeof(struct glink_mock_rx_intent), GFP_KERNEL);
	if (!intent) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	intent->size = size;
	intent->liid = liid;
	intent->lcid = lcid;
	GLINK_MOCK_DBG("lcid: %u %s: L[%u]:%zu\n", lcid,
			__func__, liid, size);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&intent->element, &mock_ptr->rx_intents);
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_local_rx_done() - Send RX done command
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @liid:	Local intent ID
 * @reuse:	Reuse the consumed intent
 */
static void tx_cmd_local_rx_done(struct glink_transport_if *if_ptr,
				uint32_t lcid, uint32_t liid, bool reuse)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	cmd->type = LOCAL_RX_DONE;
	cmd->local_rx_done.lcid = lcid;
	cmd->local_rx_done.liid = liid;
	GLINK_MOCK_INFO("lcid: %u %s: L[%u]: LOCAL_RX_DONE\n", lcid,
			__func__, liid);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * tx() - Send a data packet
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @pctx:	Packet TX context
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx(struct glink_transport_if *if_ptr, uint32_t lcid,
		struct glink_core_tx_pkt *pctx)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_tx_data *data =
		kzalloc(sizeof(struct glink_mock_tx_data), GFP_KERNEL);

	if (!data) {
		GLINK_MOCK_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	data->data = (const char *)pctx->data;
	data->lcid = lcid;
	GLINK_MOCK_INFO("lcid: %u %s: data[%p] size[%u]\n", lcid,
			__func__, data->data, pctx->size);

	/*
	 * If a partial write is done, then only
	 * decrement size_remaining by the amount sent
	 */
	pctx->size_remaining -= pctx->size;

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&data->element, &mock_ptr->tx_data);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_remote_rx_intent_req_ack() - Transmit the ack for remote
 *				RX intent request
 * @if_ptr: Pointer to transport interface
 * @lcid: Local channel ID
 * @granted: Intent grant status
 *
 * This function transmits the ack for the remote request for an RX intent.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_remote_rx_intent_req_ack(struct glink_transport_if *if_ptr,
						uint32_t lcid, bool granted)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: Memory couldn't be allocated\n", __func__);
		return -ENOMEM;
	}

	cmd->type = REMOTE_RX_INTENT_ACK;
	cmd->remote_rx_int_ack.lcid = lcid;
	cmd->remote_rx_int_ack.granted = granted;
	GLINK_MOCK_DBG("lcid: %u %s: REMOTE_RX_INTENT_ACK granted: %d\n", lcid,
			__func__, granted);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}

/**
 * tx_cmd_rx_intent_req() - Transmit the RX intent request
 * @if_ptr: Pointer to transport interface
 * @lcid: Local channel ID
 * @size: size of the intent
 *
 * This function transmits the request for an RX intent to the
 * remote side.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_rx_intent_req(struct glink_transport_if *if_ptr,
						uint32_t lcid, size_t size)
{
	unsigned long flags;

	struct glink_mock_xprt *mock_ptr =
		(struct glink_mock_xprt *)if_ptr;
	struct glink_mock_cmd *cmd =
		kzalloc(sizeof(struct glink_mock_cmd), GFP_KERNEL);
	if (!cmd) {
		GLINK_MOCK_ERR("%s: Memory couldn't be allocated\n", __func__);
		return -ENOMEM;
	}

	cmd->type = RX_INTENT_REQ;
	cmd->intent_req.lcid = lcid;
	cmd->intent_req.size = size;
	GLINK_MOCK_DBG("lcid: %u %s: RX_INTENT_REQ size: %zu\n", lcid,
			__func__, size);

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&cmd->element, &mock_ptr->tx_cmds);
	do_complete_all();
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);

	return 0;
}


/**
 * mock_xprt_init() - Initialize the mock object
 *
 * Initialize a mock object with the functions defined in this module, to be
 * exposed to other modules for testing.
 */
void mock_xprt_init(void)
{
	struct glink_core_transport_cfg cfg;
	int ret;

	spin_lock_init(&if_impl.mock_xprt_lists_lock_lha0);

	init_lists(&if_impl);
	if_impl.if_ptr.ssr = ssr;
	if_impl.if_ptr.tx = tx;
	if_impl.if_ptr.tx_cmd_version = tx_cmd_version;
	if_impl.if_ptr.tx_cmd_version_ack = tx_cmd_version_ack;
	if_impl.if_ptr.set_version = set_version;
	if_impl.if_ptr.tx_cmd_ch_open = tx_cmd_ch_open;
	if_impl.if_ptr.tx_cmd_ch_close = tx_cmd_ch_close;
	if_impl.if_ptr.tx_cmd_ch_remote_open_ack = tx_cmd_ch_remote_open_ack;
	if_impl.if_ptr.tx_cmd_ch_remote_close_ack = tx_cmd_ch_remote_close_ack;
	if_impl.if_ptr.allocate_rx_intent = allocate_rx_intent;
	if_impl.if_ptr.deallocate_rx_intent = deallocate_rx_intent;
	if_impl.if_ptr.reuse_rx_intent = reuse_rx_intent;
	if_impl.if_ptr.tx_cmd_local_rx_intent = tx_cmd_local_rx_intent;
	if_impl.if_ptr.tx_cmd_local_rx_done = tx_cmd_local_rx_done;
	if_impl.if_ptr.tx_cmd_rx_intent_req = tx_cmd_rx_intent_req;
	if_impl.if_ptr.tx_cmd_remote_rx_intent_req_ack =
					tx_cmd_remote_rx_intent_req_ack;

	if_impl.reset = reset;
	if_impl.if_ptr.glink_core_if_ptr = NULL;

	cfg.name = "mock";
	cfg.edge = "local";
	cfg.versions = versions;
	cfg.versions_entries = ARRAY_SIZE(versions);
	cfg.max_cid = UINT32_MAX;
	cfg.max_iid = UINT32_MAX;

	ret = glink_core_register_transport(&if_impl.if_ptr, &cfg);
	if (ret)
		pr_err("%s: Unable to register transport %d\n", __func__, ret);
}

/**
 * mock_xprt_reinit() - Reinitialize the mock object
 *
 * Reinitialize a mock object with the functions defined in this module, to be
 * exposed to other modules for testing. This function is called during
 * reset; it does not reinitialize completions or lists.
 */
void mock_xprt_reinit(void)
{
	struct glink_core_transport_cfg cfg;
	int ret;

	cfg.name = "mock";
	cfg.edge = "local";
	cfg.versions = versions;
	cfg.versions_entries = ARRAY_SIZE(versions);
	cfg.max_cid = UINT32_MAX;
	cfg.max_iid = UINT32_MAX;
	ret = glink_core_register_transport(&if_impl.if_ptr, &cfg);
	if (ret)
		pr_err("%s: Unable to register transport %d\n", __func__, ret);
}

/**
 * mock_xprt_get() - Provided to expose the mock transport to other modules
 *
 * Return: An initialized mock transport
 */
struct glink_mock_xprt *mock_xprt_get(void)
{
	return &if_impl;
};

/**
 * mock_xprt_reset() - Reset the mock transport to default
 * @xprt_ptr:	Pointer to mock transport layer
 */
void mock_xprt_reset(struct glink_mock_xprt *xprt_ptr)
{
	xprt_ptr->if_ptr.glink_core_if_ptr->link_down(&xprt_ptr->if_ptr);
	reset(xprt_ptr);
	glink_core_unregister_transport(&xprt_ptr->if_ptr);

	/*
	 * The glink_mock_xprt_reinit() call is needed here
	 * since we unregister the transport before the
	 * reset() call. Without it, the transport will never
	 * be properly re-initialized. The GLINK_MOCK_DBG statement
	 * pairs with the "mock:local destroying transport" message
	 * created by the glink_core_unregister_transport() call
	 * above.
	 */
	GLINK_MOCK_INFO("%s: Re-initializing transport...\n", __func__);
	mock_xprt_reinit();
}

/**
 * mock_xprt_get_next_cmd() - Gets (and removes) the next command
 *
 * Client must call kfree() when they are done with the node to prevent memory
 * leaks.
 *
 * Return: Pointer to the next command or NULL if none
 */
struct glink_mock_cmd *mock_xprt_get_next_cmd(void)
{
	unsigned long flags;
	struct glink_mock_cmd *cmd = NULL;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	if (!list_empty(&if_impl.tx_cmds)) {
		cmd = list_first_entry(&if_impl.tx_cmds, struct glink_mock_cmd,
			element);
		list_del(&cmd->element);
	}
	do_reinit_completion();
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);

	return cmd;
}

/**
 * mock_xprt_get_next_cmd_by_cid() - Gets (and removes) the next command
 * @ch_id:	The channel ID used to lookup the command
 * @event:	Event completion, used in unit tests, which is reset in this
 *		function
 *
 * Returned pointer must be freed by caller to prevent memory leaks.
 *
 * Return: Pointer to the next command or NULL if none
 */
struct glink_mock_cmd *mock_xprt_get_next_cmd_by_cid(uint32_t ch_id,
		struct completion *event)
{
	unsigned long flags;
	struct glink_mock_cmd *cmd;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	reinit_completion(event);
	if (list_empty(&if_impl.tx_cmds)) {
		spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0,
				flags);
		GLINK_ERR("%s: tx_cmds list is empty! thread: %d\n", __func__,
				current->pid);
		return NULL;
	}

	list_for_each_entry(cmd, &if_impl.tx_cmds, element) {
		uint32_t cmd_ch_id = 0;

		switch (cmd->type) {
		case LOCAL_OPEN:
			cmd_ch_id = cmd->local_open.lcid;
			break;
		case LOCAL_CLOSE:
			cmd_ch_id = cmd->local_close.lcid;
			break;
		case LOCAL_RX_DONE:
			cmd_ch_id = cmd->local_rx_done.lcid;
			break;
		case REMOTE_OPEN_ACK:
			cmd_ch_id = cmd->remote_open_ack.rcid;
			break;
		case REMOTE_CLOSE_ACK:
			cmd_ch_id = cmd->remote_close_ack.rcid;
			break;
		case REMOTE_RX_INTENT_ACK:
			cmd_ch_id = cmd->remote_rx_int_ack.lcid;
			break;
		case RX_INTENT_REQ:
			cmd_ch_id = cmd->intent_req.lcid;
			break;
		case VERSION:
		case VERSION_ACK:
		case SET_VERSION:
			break;
		default:
			break;
		}

		if (cmd_ch_id && cmd_ch_id == ch_id) {
			list_del(&cmd->element);
			spin_unlock_irqrestore(
					&if_impl.mock_xprt_lists_lock_lha0,
					flags);
			return cmd;
		}
	}
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);
	GLINK_ERR("%s: tx_cmds list - no match found! thread: %d\n", __func__,
			current->pid);
	return NULL;
}

/**
 * mock_xprt_get_intent() - Gets (and removes) the intent
 *
 * Client must call kfree() when they are done with the node to prevent memory
 * leaks.
 *
 * Return: Pointer to the next intent or NULL if none
 */
struct glink_mock_rx_intent *mock_xprt_get_intent(void)
{
	unsigned long flags;
	struct glink_mock_rx_intent *intent = NULL;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	if (!list_empty(&if_impl.rx_intents)) {
		intent = list_first_entry(&if_impl.rx_intents,
					struct glink_mock_rx_intent, element);
		list_del(&intent->element);
	}
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);

	return intent;
}

/**
 * mock_xprt_get_intent_by_cid() - Gets (and removes) the intent with the given
 *				channel ID
 * @ch_id:	The channel ID used to lookup the intent
 * @event:	Event completion, used in unit tests, which is reset in this
 *		function
 *
 * Client must call kfree() when they are done with the node to prevent memory
 * leaks.
 *
 * Return: Pointer to the next intent or NULL if none
 */
struct glink_mock_rx_intent *mock_xprt_get_intent_by_cid(uint32_t ch_id,
		struct completion *event)
{
	unsigned long flags;
	struct glink_mock_rx_intent *intent;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	reinit_completion(event);
	if (list_empty(&if_impl.rx_intents)) {
		GLINK_MOCK_DBG("%s: intents list is empty! thread %d\n",
		__func__, current->pid);
		spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0,
				flags);
		return NULL;
	}

	list_for_each_entry(intent, &if_impl.rx_intents, element) {
		if (intent->lcid == ch_id) {
			list_del(&intent->element);
			spin_unlock_irqrestore(
					&if_impl.mock_xprt_lists_lock_lha0,
					flags);
			return intent;
		}
	}
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);
	GLINK_ERR("%s: intents list - no match found! thread: %d\n", __func__,
			current->pid);
	return NULL;
}

/**
 * mock_xprt_get_tx_data() - Gets (and removes) the data pkt from tx_data list
 *
 * Client must call kfree() when they are done with the node to prevent memory
 * leaks.
 *
 * Return: Pointer to the next data pkt or NULL if none
 */
struct glink_mock_tx_data *mock_xprt_get_tx_data(void)
{
	unsigned long flags;
	struct glink_mock_tx_data *data = NULL;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	if (!list_empty(&if_impl.tx_data)) {
		GLINK_MOCK_DBG("%s\n", __func__);
		data = list_first_entry(&if_impl.tx_data,
					struct glink_mock_tx_data, element);
		list_del(&data->element);
	}

	do_reinit_completion();
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);
	GLINK_MOCK_DBG("%s data[%p]\n", __func__, data);
	return data;
}

/**
 * mock_xprt_get_tx_data_by_cid() - Gets (and removes) a data packet from the
 *				tx_data list
 * @ch_id:	The channel ID used to lookup the data packet
 * @event:	Event completion, used in unit tests, which is reset in this
 *		function
 *
 * Client must call kfree() when they are done with the node to prevent memory
 * leaks.
 *
 * Return: Pointer to the next data pkt or NULL if none
 */
struct glink_mock_tx_data *mock_xprt_get_tx_data_by_cid(uint32_t ch_id,
		struct completion *event)
{
	unsigned long flags;
	struct glink_mock_tx_data *data;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	reinit_completion(event);
	if (list_empty(&if_impl.tx_data)) {
		spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0,
				flags);
		GLINK_MOCK_DBG("%s: tx_data list empty! thread: %d\n", __func__,
				current->pid);
		return NULL;
	}

	list_for_each_entry(data, &if_impl.tx_data, element) {
		if (data->lcid == ch_id) {
			list_del(&data->element);
			spin_unlock_irqrestore(
					&if_impl.mock_xprt_lists_lock_lha0,
					flags);
			return data;
		}
	}
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);
	GLINK_MOCK_DBG("%s: tx_data list - no match found! thread: %d\n",
	__func__, current->pid);
	return NULL;
}

/**
 * do_complete_all() - Iterates through completions list and calls
 *		complete_all() on each completion
 *
 * This function should only be called with mock_xprt_lists_lock_lha0 locked.
 */
static void do_complete_all(void)
{
	struct glink_completion_item *local_item;

	list_for_each_entry(local_item, &if_impl.completions, item) {
		complete_all(local_item->completion_ptr);
	}
}

/**
 * do_reinit_completion() - Iterates through completions list and resets each
 *			completion by calling reinit_completion()
 *
 * This function should only be called with mock_xprt_lists_lock_lha0 locked.
 */
static void do_reinit_completion(void)
{
	struct glink_completion_item *local_item;

	list_for_each_entry(local_item, &if_impl.completions, item) {
		reinit_completion(local_item->completion_ptr);
	}
}

/**
 * register_completion() - Adds a completion to the mock xprt completions list
 * @if_ptr:	Pointer to interface instance
 * @completion:	The completion to add to the list
 */
void register_completion(struct glink_transport_if *if_ptr,
		struct completion *completion)
{
	unsigned long flags;
	struct glink_completion_item *item;
	struct glink_mock_xprt *mock_ptr = (struct glink_mock_xprt *)if_ptr;

	item = kmalloc(sizeof(struct glink_completion_item), GFP_KERNEL);
	if (!item) {
		GLINK_ERR("%s: Allocation of completion failed", __func__);
		return;
	}

	item->completion_ptr = completion;

	spin_lock_irqsave(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
	list_add_tail(&item->item, &mock_ptr->completions);
	spin_unlock_irqrestore(&mock_ptr->mock_xprt_lists_lock_lha0, flags);
}

/**
 * unregister_completion() - Removes a completion from the mock xprt completions
 *			list
 * @completion:	The completion to remove from the list
 */
void unregister_completion(struct completion *completion)
{
	unsigned long flags;
	struct glink_completion_item *local_item;

	spin_lock_irqsave(&if_impl.mock_xprt_lists_lock_lha0, flags);
	list_for_each_entry(local_item, &if_impl.completions, item) {
		if (local_item->completion_ptr == completion) {
			list_del(&local_item->item);
			kfree(local_item);
			spin_unlock_irqrestore(
					&if_impl.mock_xprt_lists_lock_lha0,
					flags);
			return;
		}
	}
	spin_unlock_irqrestore(&if_impl.mock_xprt_lists_lock_lha0, flags);
}

int glink_mock_xprt_init(void)
{
	mock_xprt_init();
	return 0;
}

void glink_mock_xprt_exit(void)
{
	if_impl.if_ptr.glink_core_if_ptr->link_down(&if_impl.if_ptr);
	reset(&if_impl);
	glink_core_unregister_transport(&if_impl.if_ptr);
	pr_err("%s: Exiting\n", __func__);
}
