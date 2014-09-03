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
#ifndef GLINK_LOOPBACK_XPRT_H
#define GLINK_LOOPBACK_XPRT_H

#include <linux/completion.h>
#include "glink_xprt_if.h"

#define MAX_NAME_LEN 32
#define UINT32_MAX 0xFFFFFFFF

/**
 * struct ch_map_info - Structure to map data and control channels
 * @list:	Used to chain this mapping structure in a list
 * @ch_name:	The name which refers to both channels being mapped together
 * @ch1_lcid:	The lcid of one of the channels in the mapping
 * @ch2_lcid:	The lcid of the other channel in the mapping
 */
struct ch_map_info {
	struct list_head list;
	char ch_name[MAX_NAME_LEN];
	uint32_t ch1_lcid;
	uint32_t ch2_lcid;
};

/**
 * struct glink_loopback_xprt - Loopback transport representation structure
 * @if_ptr:		Pointer to the transport instance
 * @ll_wq:		Workqueue for loopback commands on this transport
 * @ll_cmd_work:	Work structure for loopback commands
 * @ch_map_lock:	Lock to protect changes to the channel mapping
 * @ch_map:		List containing data channel-control channel mapping
 * @tx_cmds_lock:	Lock for TX commands for this transport instance
 * @tx_cmds:		List of TX commands for this transport instance
 */
struct glink_loopback_xprt {
	struct glink_transport_if if_ptr;
	struct workqueue_struct *ll_wq;
	struct work_struct ll_cmd_work;
	struct mutex ch_map_lock;
	struct list_head ch_map;
	struct mutex tx_cmds_lock;
	struct list_head tx_cmds;
};

/**
 * enum cmd_type - Representation of the commands for the loopback transport
 * @OPEN:		Open a channel
 * @CLOSE:		Close a channel
 * @VERSION:		Version and feature set supported
 * @VERSION_ACK:	Response to @VERSION
 * @SET_VERSION:	Set the version and feature set
 * @OPEN_ACK:		Response to the @OPEN_ACK command
 * @CLOSE_ACK:		Response to the @CLOSE command
 * @RX_INTENT:		RX intent for a channel was queued
 * @RX_DONE:		Use of RX intent for a channel is complete
 * @RX_INTENT_REQ:	Request that an RX intent be queued
 * @RX_INTENT_REQ_ACK:	Response to @RX_INTENT_REQ
 * @DATA:		Start of a data transfer
 * @SIGS_SET:		Set sideband (TIOCM) signals
 */
enum cmd_type {
	OPEN,
	CLOSE,
	VERSION,
	VERSION_ACK,
	SET_VERSION,
	OPEN_ACK,
	CLOSE_ACK,
	RX_INTENT,
	RX_DONE,
	RX_INTENT_REQ,
	RX_INTENT_REQ_ACK,
	DATA,
	SIGS_SET,
};

/**
 * struct glink_loopback_xprt_cmd - Structure representing a loopback transport
 *				command
 * @type:		The command type
 * @element:		Used to chain the command in a list
 * @open:		Structure to represent an OPEN command
 * @close:		Structure to represent a CLOSE command
 * @version:		Structure to represent a VERSION command
 * @open_ack:		Structure to represent an OPEN_ACK command
 * @close_ack:		Structure to represent a CLOSE_ACK command
 * @rx_intent:		Structure to represent an RX_INTENT command
 * @rx_done:		Structure to represent an RX_DONE command
 * @rx_intent_req:	Structure to represent an RX_INTENT_REQ command
 * @rx_intent_req_ack:	Structure to represent an RX_INTENT_REQ_ACK command
 * @data:		Structure to represent a DATA command
 * @sigs_set:		Structure to represent a SIGS_SET command
 */
struct glink_loopback_xprt_cmd {
	enum cmd_type type;
	struct list_head element;
	union {
		struct {
			uint32_t lcid;
			char name[MAX_NAME_LEN];
		} open;

		struct {
			uint32_t lcid;
		} close;

		struct {
			uint32_t version;
			uint32_t features;
		} version;

		struct {
			uint32_t lcid;
		} open_ack;

		struct {
			uint32_t lcid;
		} close_ack;

		struct {
			uint32_t lcid;
			uint32_t size;
			uint32_t id;
		} rx_intent;

		struct {
			uint32_t lcid;
			uint32_t liid;
			bool reuse;
		} rx_done;

		struct {
			uint32_t lcid;
			uint32_t size;
		} rx_intent_req;

		struct {
			uint32_t lcid;
			bool granted;
		} rx_intent_req_ack;

		struct {
			uint32_t lcid;
			const char *data;
			uint32_t riid;
			uint32_t size;
			uint32_t size_remaining;
			void *iovec;
			void * (*vbuf_provider)(void *iovec, size_t offset,
				size_t *size);
			void * (*pbuf_provider)(void *iovec, size_t offset,
				size_t *size);
		} data;

		struct {
			uint32_t lcid;
			uint32_t sigs;
		} sigs_set;
	};
};
#endif
