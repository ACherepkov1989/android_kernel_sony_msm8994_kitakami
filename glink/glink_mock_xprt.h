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

#ifndef GLINK_MOCK_XPRT_H
#define GLINK_MOCK_XPRT_H

#include <soc/qcom/glink.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include "glink_xprt_if.h"

unsigned glink_get_mock_debug_mask(void);

/**
 * struct glink_mock_xprt - Structure representing the mock transport
 * @if_ptr:			Pointer passed in from the core
 * @mock_xprt_lists_lock_lha0:	Spinlock to protect mock xprt lists
 * @completions:		List of completions
 * @rx_intents:			List of RX intents
 * @tx_data:			List of tx_data
 * @tx_cmds:			List of tx_cmds
 * @reset:			The mock transport reset() function
 *
 * The interface structure (@if_ptr) must always be the first element to allow
 * casting of the pointer passed in from the G-Link core.
 */
struct glink_mock_xprt {
	struct glink_transport_if if_ptr;
	spinlock_t mock_xprt_lists_lock_lha0;
	struct list_head completions;
	struct list_head rx_intents;
	struct list_head tx_data;
	struct list_head tx_cmds;
	void (*reset)(struct glink_mock_xprt *mock_ptr);
};

/**
 * struct glink_completion_item - Structure to aid in creating a list of
 *				completions managed by the transport
 * @item:		Used to chain the completions in a list
 * @completion_ptr:	The completion structure
 */
struct glink_completion_item {
	struct list_head item;
	struct completion *completion_ptr;
};

/**
 * struct glink_mock_rx_intent - Models an RX intent
 * @element:	Used to chain the intent structures in a list
 * @size:	Size of this intent
 * @liid:	The intent ID
 * @lcid:	The local channel ID associated with this intent
 */
struct glink_mock_rx_intent {
	struct list_head element;
	size_t size;
	uint32_t liid;
	uint32_t lcid;
};

/**
 * struct glink_mock_tx_data - Models TX data
 * @element:	Used to chain the TX data structures in a list
 * @data:	The data
 * @lcid:	The local channel ID associated with this data
 */
struct glink_mock_tx_data {
	struct list_head element;
	const char *data;
	uint32_t lcid;
};

/**
 * enum cmd_type - Representation of the commands for the mock transport
 * @LOCAL_OPEN:			Open a channel
 * @LOCAL_CLOSE:		Close a channel
 * @REMOTE_OPEN_ACK:		Response to
 *				glink_core_if::rx_cmd_ch_remote_open()
 * @REMOTE_CLOSE_ACK:		Response to
 *				glink_core_if::rx_cmd_ch_remote_close()
 * @LOCAL_RX_DONE:		Use of RX intent for a channel is complete
 * @VERSION:			Version and feature set supported
 * @VERSION_ACK:		Response to the @VERSION command
 * @SET_VERSION:		Set the version and feature set
 * @REMOTE_RX_INTENT_ACK:	RX intent for a channel was queued
 * @RX_INTENT_REQ:		Request that an RX intent be queued
 */
enum cmd_type {
	LOCAL_OPEN,
	LOCAL_CLOSE,
	REMOTE_OPEN_ACK,
	REMOTE_CLOSE_ACK,
	LOCAL_RX_DONE,
	VERSION,
	VERSION_ACK,
	SET_VERSION,
	REMOTE_RX_INTENT_ACK,
	RX_INTENT_REQ,
};

/**
 * struct glink_mock_cmd - Structure representing a mock transport command
 * @type:		The command type
 * @element:		Used to chain the command in a list
 * @local_open:		Structure to represent a LOCAL_OPEN command
 * @remote_open_ack:	Structure to represent a REMOTE_OPEN_ACK command
 * @remote_close_ack:	Structure to represent a REMOTE_CLOSE_ACK command
 * @local_close:	Structure to represent a LOCAL_CLOSE command
 * @local_rx_done:	Structure to represent a LOCAL_RX_DONE command
 * @version:		Structure to represent a VERSION command
 * @remote_rx_int_ack:	Structure to represent a REMOTE_RX_INTENT_ACK command
 * @intent_req:		Structure to represent an RX_INTENT_REQ command
 */
struct glink_mock_cmd {
	enum cmd_type type;
	struct list_head element;
	union {
		struct {
			uint32_t lcid;
			char name[GLINK_NAME_SIZE];
		} local_open;

		struct {
			uint32_t rcid;
		} remote_open_ack;

		struct {
			uint32_t rcid;
		} remote_close_ack;

		struct {
			uint32_t lcid;
		} local_close;

		struct {
			uint32_t lcid;
			uint32_t liid;
		} local_rx_done;

		struct {
			uint32_t version;
			uint32_t features;
		} version;

		struct {
			uint32_t lcid;
			bool granted;
		} remote_rx_int_ack;

		struct {
			uint32_t lcid;
			size_t size;
		} intent_req;
	};
};

int glink_mock_xprt_init(void);
void glink_mock_xprt_exit(void);

struct glink_mock_xprt *mock_xprt_get(void);
struct glink_mock_cmd *mock_xprt_get_next_cmd(void);
struct glink_mock_rx_intent *mock_xprt_get_intent(void);
struct glink_mock_tx_data *mock_xprt_get_tx_data(void);
struct glink_mock_cmd *mock_xprt_get_next_cmd_by_cid(uint32_t ch_id,
		struct completion *event);
struct glink_mock_rx_intent *mock_xprt_get_intent_by_cid(uint32_t ch_id,
		struct completion *event);
struct glink_mock_tx_data *mock_xprt_get_tx_data_by_cid(uint32_t ch_id,
		struct completion *event);
void mock_xprt_reset(struct glink_mock_xprt *ptr);
void register_completion(struct glink_transport_if *if_ptr,
		struct completion *completion);
void unregister_completion(struct completion *completion);
#endif /* GLINK_MOCK_XPRT_H */
