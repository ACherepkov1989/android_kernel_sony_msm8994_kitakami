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

#include <linux/ipc_logging.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <soc/qcom/smem.h>
#include "glink_core_if.h"
#include "glink_loopback_xprt.h"
#include "glink_private.h"
#include "glink_xprt_if.h"

enum {
	INFO_MASK = 1U << 0,
	DEBUG_MASK = 1U << 1,
	PERF_MASK = 1U << 2,
};

static int glx_debug_mask;
module_param_named(xprt_debug_mask, glx_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

/*
 * SMEM and interrupt simulation.
 *
 * 0 = disable
 * >0 = enabled with specified interrupt latency in us
 */
static unsigned glx_smem_rw_int_latency_us;
module_param_named(smem_rw_enable, glx_smem_rw_int_latency_us,
		   uint, S_IRUGO | S_IWUSR | S_IWGRP);
static void *smem_item_addr;

#define GLX_PERF(fmt, args...) do {	\
	if (glx_debug_mask & PERF_MASK) \
		GLINK_IPC_LOG_STR("<LL> " fmt, args); \
} while (0)

#define GLX_INFO(x...) do { \
	if (glx_debug_mask & INFO_MASK) \
		GLINK_IPC_LOG_STR("<LL> " x); \
} while (0)

#define GLX_INFO_PERF(fmt, args...) do {	\
	if (glx_debug_mask & (INFO_MASK | PERF_MASK)) \
		GLINK_IPC_LOG_STR("<LL> " fmt, args); \
} while (0)

#define GLX_DBG(x...) do { \
	if (glx_debug_mask & DEBUG_MASK) \
		GLINK_IPC_LOG_STR("<LL> " x); \
} while (0)

#define GLX_ERR(x...) do {                              \
	if (!(glx_debug_mask & PERF_MASK)) \
		pr_err("<LL> " x); \
	GLINK_IPC_LOG_STR("<LL> " x);  \
} while (0)

static struct glink_loopback_xprt loopback_xprt;

static uint32_t negotiate_features_v1(struct glink_transport_if *if_ptr,
		const struct glink_core_version *version_ptr,
		uint32_t features);
static void deinit_loopback_xprt_lists(void);

/* Versions data structure, used in version negotiation. */
static struct glink_core_version versions[] = {
	{1, 0x00, negotiate_features_v1},
	{2, 0x22, negotiate_features_v1},
	{5, 0x55, negotiate_features_v1},
};

/**
 * struct cmd_handler_info - Structure that stores the command handler data
 * @type:		The command type
 * @cmd_handler:	Pointer to the command handler function
 */
struct cmd_handler_info {
	enum cmd_type type;
	int (*cmd_handler)(struct glink_loopback_xprt_cmd *cmd);
};

static int handle_open_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_close_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_version_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_version_ack_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_set_version_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_open_ack_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_close_ack_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_rx_intent_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_rx_done_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_rx_intent_req_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_rx_intent_req_ack_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_data_cmd(struct glink_loopback_xprt_cmd *cmd);
static int handle_sigs_set_cmd(struct glink_loopback_xprt_cmd *cmd);

struct cmd_handler_info cmd_handler_tbl[] = {
	{OPEN, handle_open_cmd},
	{CLOSE, handle_close_cmd},
	{VERSION, handle_version_cmd},
	{VERSION_ACK, handle_version_ack_cmd},
	{SET_VERSION, handle_set_version_cmd},
	{OPEN_ACK, handle_open_ack_cmd},
	{CLOSE_ACK, handle_close_ack_cmd},
	{RX_INTENT, handle_rx_intent_cmd},
	{RX_DONE, handle_rx_done_cmd},
	{RX_INTENT_REQ, handle_rx_intent_req_cmd},
	{RX_INTENT_REQ_ACK, handle_rx_intent_req_ack_cmd},
	{DATA, handle_data_cmd},
	{SIGS_SET, handle_sigs_set_cmd},
};

union fifo_mem {
	uint64_t u64;
	uint8_t u8;
};

/**
 * glink_loopback_xprt_copy_to_smem() - Copy data to SMEM
 * @src:	The buffer to copy from
 * @num_bytes:	The number of bytes to copy
 */
static void glink_loopback_xprt_copy_to_smem(void *src, size_t num_bytes)
{
	union fifo_mem *temp_dst = (union fifo_mem *)smem_item_addr;
	union fifo_mem *temp_src = (union fifo_mem *)src;
	uintptr_t mask = sizeof(union fifo_mem) - 1;

	if (unlikely(!smem_item_addr)) {
		smem_item_addr = smem_alloc(SMEM_ID_VENDOR0, 8192, 0,
					    SMEM_ANY_HOST_FLAG);
		if (!smem_item_addr) {
			GLX_ERR("%s: Cannot copy to SMEM as configured\n",
				__func__);
			return;
		}
		temp_dst = (union fifo_mem *)smem_item_addr;
	}

	if (!temp_src || !temp_dst) {
		GLX_ERR("%s: %p %p\n", __func__, temp_src, temp_dst);
		return;
	}

	GLX_INFO("%s: Copying %zu bytes to SMEM\n", __func__, num_bytes);
	while ((uintptr_t)temp_dst & mask && num_bytes) {
		__raw_writeb_no_log(temp_src->u8, temp_dst);
		temp_src = (union fifo_mem *)((uintptr_t)temp_src + 1);
		temp_dst = (union fifo_mem *)((uintptr_t)temp_dst + 1);
		num_bytes--;
	}

	while (num_bytes >= sizeof(union fifo_mem)) {
		__raw_writeq_no_log(temp_src->u64, temp_dst);
		temp_dst++;
		temp_src++;
		num_bytes -= sizeof(union fifo_mem);
	}

	while (num_bytes--) {
		__raw_writeb_no_log(temp_src->u8, temp_dst);
		temp_src = (union fifo_mem *)((uintptr_t)temp_src + 1);
		temp_dst = (union fifo_mem *)((uintptr_t)temp_dst + 1);
	}
}

/**
 * glink_loopback_xprt_copy_from_smem() - Copy data from SMEM
 * @dest:	The destination buffer
 * @num_bytes:	The number of bytes to copy
 */
static void glink_loopback_xprt_copy_from_smem(void *dest, size_t num_bytes)
{
	union fifo_mem *temp_dst = (union fifo_mem *)dest;
	union fifo_mem *temp_src = (union fifo_mem *)smem_item_addr;
	uintptr_t mask = sizeof(union fifo_mem) - 1;

	if (unlikely(!smem_item_addr))
		return;

		if (!temp_src || !temp_dst) {
			GLX_ERR("%s: %p %p\n", __func__, temp_src, temp_dst);
			return;
		}

	GLX_INFO("%s: Copying %zu bytes from SMEM\n", __func__, num_bytes);
	while ((uintptr_t)temp_src & mask && num_bytes) {
		temp_dst->u8 = __raw_readb_no_log(temp_src);
		temp_src = (union fifo_mem *)((uintptr_t)temp_src + 1);
		temp_dst = (union fifo_mem *)((uintptr_t)temp_dst + 1);
		num_bytes--;
	}

	while (num_bytes >= sizeof(union fifo_mem)) {
		temp_dst->u64 = __raw_readq_no_log(temp_src);
		temp_dst++;
		temp_src++;
		num_bytes -= sizeof(union fifo_mem);
	}

	while (num_bytes--) {
		temp_dst->u8 = __raw_readb_no_log(temp_src);
		temp_src = (union fifo_mem *)((uintptr_t)temp_src + 1);
		temp_dst = (union fifo_mem *)((uintptr_t)temp_dst + 1);
	}
}

/**
 * simulate_smem_loopback() - Simulate a buffer round trip through SMEM
 * @buffer:	The buffer to be sent in the simulation
 * @num_bytes:	The number of bytes to send
 */
static void simulate_smem_loopback(void *buffer, size_t num_bytes)
{
	if (!glx_smem_rw_int_latency_us)
		return;

	glink_loopback_xprt_copy_to_smem(buffer, num_bytes);
	udelay(glx_smem_rw_int_latency_us);
	glink_loopback_xprt_copy_from_smem(buffer, num_bytes);
}

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
	return features & version_ptr->features;
}

/**
 * lookup_ch_map_info() - Lookup channel map info given a channel name
 * @name:	The name of the channel
 *
 * Return: The channel map info if found, NULL otherwise
 */
static struct ch_map_info *lookup_ch_map_info(char *name)
{
	struct ch_map_info *temp_map;

	list_for_each_entry(temp_map, &loopback_xprt.ch_map, list) {
		if (!strncmp(temp_map->ch_name, name, MAX_NAME_LEN))
			return temp_map;
	}
	return NULL;
}

/**
 * handle_open_cmd() - Handle OPEN command
 * @cmd:	The command structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int handle_open_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	char ch_name[MAX_NAME_LEN];
	struct ch_map_info *temp_map;
	char *temp;
	int is_client = 1;
	int lcid;

	strlcpy(ch_name, cmd->open.name, MAX_NAME_LEN);
	lcid = cmd->open.lcid;
	temp = strnstr(ch_name, "_CLNT", MAX_NAME_LEN);
	if (!temp) {
		is_client = 0;
		temp = strnstr(ch_name, "_SRV", MAX_NAME_LEN);
	}

	if (!temp) {
		GLX_ERR("::%s[%d:] %s: OPEN Invalid ch_name\n", ch_name, lcid,
				__func__);
		return -EINVAL;
	}
	*temp = '\0';

	mutex_lock(&loopback_xprt.ch_map_lock);
	temp_map = lookup_ch_map_info(ch_name);
	if (!temp_map) {
		temp_map = kzalloc(sizeof(struct ch_map_info), GFP_KERNEL);
		if (!temp_map) {
			mutex_unlock(&loopback_xprt.ch_map_lock);
			GLX_ERR("::%s[%d:] %s: OPEN Error allocating ch_map\n",
				ch_name, lcid, __func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&temp_map->list);
		strlcpy(temp_map->ch_name, ch_name, MAX_NAME_LEN);
		temp_map->ch1_lcid = cmd->open.lcid;
		list_add_tail(&temp_map->list, &loopback_xprt.ch_map);
	} else {
		if (!temp_map->ch1_lcid) {
			temp_map->ch1_lcid = cmd->open.lcid;
		} else if (!temp_map->ch2_lcid) {
			temp_map->ch2_lcid = cmd->open.lcid;
		} else {
			mutex_unlock(&loopback_xprt.ch_map_lock);
			GLX_ERR("::%s[%d:] %s: OPEN Channel in invalid state\n",
				ch_name, lcid, __func__);
			return -EFAULT;
		}
	}

	if (is_client)
		strlcat(ch_name, "_SRV", MAX_NAME_LEN);
	else
		strlcat(ch_name, "_CLNT", MAX_NAME_LEN);
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_open(
		&loopback_xprt.if_ptr, cmd->open.lcid, ch_name, LLOOP_XPRT_ID);
	if (temp_map->ch1_lcid && temp_map->ch2_lcid)
		GLX_INFO("::%s[%x:] %s: OPEN Opened %x\n", temp_map->ch_name,
			   temp_map->ch1_lcid, __func__, temp_map->ch2_lcid);
	mutex_unlock(&loopback_xprt.ch_map_lock);
	return 0;
}

/**
 * handle_open_ack_cmd() - Handle OPEN_ACK command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_open_ack_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_ch_open_ack(
		&loopback_xprt.if_ptr, cmd->open_ack.lcid, LLOOP_XPRT_ID);
	return 0;
}

/**
 * handle_close_cmd() - Handle CLOSE command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_close_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	struct ch_map_info *temp_map;

	mutex_lock(&loopback_xprt.ch_map_lock);
		list_for_each_entry(temp_map, &loopback_xprt.ch_map, list) {
		if (temp_map->ch1_lcid == cmd->close.lcid) {
			temp_map->ch1_lcid = 0;
			break;
		} else if (temp_map->ch2_lcid == cmd->close.lcid) {
			temp_map->ch2_lcid = 0;
			break;
		}
	}
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_ch_remote_close(
		&loopback_xprt.if_ptr, cmd->close.lcid);
	GLX_INFO("::%s[%x:] %s CLOSE Closed %x\n", temp_map->ch_name,
		   temp_map->ch1_lcid, __func__, temp_map->ch2_lcid);
	mutex_unlock(&loopback_xprt.ch_map_lock);
	return 0;
}

/**
 * handle_close_ack_cmd() - Handle CLOSE_ACK command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_close_ack_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_ch_close_ack(
		&loopback_xprt.if_ptr, cmd->close_ack.lcid);
	return 0;
}

/**
 * handle_version_cmd() - Handle VERSION command
 * @cmd:	The command structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int handle_version_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	int count = ARRAY_SIZE(versions);

	if (cmd->version.version != versions[count-1].version &&
	    cmd->version.features != versions[count-1].features) {
		GLX_ERR("%s: VERSION %x:%x version/features mismatch\n",
			__func__, cmd->version.version, cmd->version.features);
		return -EINVAL;
	}

	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_version(
		&loopback_xprt.if_ptr, cmd->version.version,
		cmd->version.features);
	return 0;
}

/**
 * handle_version_ack_cmd() - Handle VERSION_ACK command
 * @cmd:	The command structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int handle_version_ack_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	int count = ARRAY_SIZE(versions);

	if (cmd->version.version != versions[count-1].version &&
	    cmd->version.features != versions[count-1].features) {
		GLX_ERR("%s: VERSION_ACK %x:%x version/features mismatch\n",
			__func__, cmd->version.version, cmd->version.features);
		return -EINVAL;
	}
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_version_ack(
		&loopback_xprt.if_ptr, cmd->version.version,
		cmd->version.features);
	return 0;
}

/**
 * handle_set_version_cmd() - Handle SET_VERSION command
 * @cmd:	The command structure
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int handle_set_version_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	int count = ARRAY_SIZE(versions);

	if (cmd->version.version != versions[count-1].version &&
	    cmd->version.features != versions[count-1].features) {
		GLX_ERR("%s: SET_VERSION %x:%x version/features mismatch\n",
			__func__, cmd->version.version, cmd->version.features);
		return -EINVAL;
	}
	GLX_INFO("%s: %x:%x\n", __func__, cmd->version.version,
		   cmd->version.features);
	return 0;
}

/**
 * handle_rx_intent_cmd() - Handle RX_INTENT command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_rx_intent_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_put(
		&loopback_xprt.if_ptr, cmd->rx_intent.lcid, cmd->rx_intent.id,
		cmd->rx_intent.size);
	GLX_INFO("lcid: %d %s: L[%d]:%d RX_INTENT\n", cmd->rx_intent.lcid,
			__func__, cmd->rx_intent.id, cmd->rx_intent.size);
	return 0;
}

/**
 * handle_rx_done_cmd() - Handle RX_DONE command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_rx_done_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_tx_done(
		&loopback_xprt.if_ptr, cmd->rx_done.lcid, cmd->rx_done.liid,
		cmd->rx_done.reuse);
	GLX_INFO("rcid: %u %s: RX_DONE L[%u]:\n",
		cmd->rx_done.lcid, __func__, cmd->rx_done.liid);
	return 0;
}

/**
 * handle_rx_intent_req_cmd() - Handle RX_INTENT_REQ command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_rx_intent_req_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_remote_rx_intent_req(
		&loopback_xprt.if_ptr, cmd->rx_intent_req.lcid,
		cmd->rx_intent_req.size);
	GLX_INFO("lcid: %d %s: RX_INTENT_REQ size: %d\n",
			cmd->rx_intent_req.lcid, __func__,
			cmd->rx_intent_req.size);
	return 0;
}

/**
 * handle_rx_intent_req_ack_cmd() - Handle RX_INTENT_REQ_ACK command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_rx_intent_req_ack_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_rx_intent_req_ack(
		&loopback_xprt.if_ptr, cmd->rx_intent_req_ack.lcid,
		cmd->rx_intent_req_ack.granted);
	GLX_INFO("lcid: %d %s: RX_INTENT_REQ_ACK granted: %d\n",
			cmd->rx_intent_req_ack.lcid, __func__,
			cmd->rx_intent_req_ack.granted);
	return 0;
}

/**
 * handle_data_cmd() - Handle DATA command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_data_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	struct glink_core_rx_intent *rx_intent_ptr;

	GLX_INFO_PERF("rcid: %u %s: start RX L[%u]:%u data[%p] size[%u]\n",
		cmd->data.lcid, __func__, cmd->data.riid, cmd->data.size,
		cmd->data.data, cmd->data.size);
	rx_intent_ptr = loopback_xprt.if_ptr.glink_core_if_ptr->rx_get_pkt_ctx(
		&loopback_xprt.if_ptr, cmd->data.lcid, cmd->data.riid);

	if (rx_intent_ptr->data) {
		GLX_ERR(
		    "lcid: %d %s: R[%d]:%d DATA size_rem: %d already exists\n",
		    cmd->data.lcid, __func__, cmd->data.riid, cmd->data.size,
		    cmd->data.size_remaining);
	}
	rx_intent_ptr->data = (void *)cmd->data.data;
	rx_intent_ptr->iovec = cmd->data.iovec;
	rx_intent_ptr->pkt_size = cmd->data.size;
	rx_intent_ptr->vprovider = cmd->data.vbuf_provider;
	rx_intent_ptr->pprovider = cmd->data.pbuf_provider;
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_put_pkt_ctx(
		&loopback_xprt.if_ptr, cmd->data.lcid, rx_intent_ptr,
		cmd->data.size_remaining == 0 ? true : false);
	return 0;
}

/**
 * handle_sigs_set_cmd() - Handle SIGS_SET command
 * @cmd:	The command structure
 *
 * Return: 0
 */
static int handle_sigs_set_cmd(struct glink_loopback_xprt_cmd *cmd)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->rx_cmd_remote_sigs(
		&loopback_xprt.if_ptr, cmd->sigs_set.lcid, cmd->sigs_set.sigs);
	GLX_INFO("lcid: %d %s: SIGS_SET: 0x%x\n",
			cmd->sigs_set.lcid, __func__,
			cmd->sigs_set.sigs);
	return 0;
}

/**
 * ll_cmd_worker() - Calls command handlers for commands in the command list
 * @work:	The work structure
 *
 * This worker function iterates through the list of commands for the loopback
 * transport and calls the appropriate command handler for each one.
 */
static void ll_cmd_worker(struct work_struct *work)
{
	struct glink_loopback_xprt_cmd *cmd;
	int ret;

	while (1) {
		mutex_lock(&loopback_xprt.tx_cmds_lock);
		if (list_empty(&loopback_xprt.tx_cmds)) {
			mutex_unlock(&loopback_xprt.tx_cmds_lock);
			break;
		}

		cmd = list_first_entry(&loopback_xprt.tx_cmds,
				struct glink_loopback_xprt_cmd, element);
		list_del(&cmd->element);
		mutex_unlock(&loopback_xprt.tx_cmds_lock);

		if (cmd->type < ARRAY_SIZE(cmd_handler_tbl))
			ret = cmd_handler_tbl[cmd->type].cmd_handler(cmd);
		else
			GLX_ERR("%s: Unknown cmd %d\n", __func__, cmd->type);
		kfree(cmd);
	}
}

/**
 * ssr() - Perform any necessary cleanup at the transport level
 * @if_ptr:	Pointer to interface instance
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int ssr(struct glink_transport_if *if_ptr)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->link_down(
					&loopback_xprt.if_ptr);
	deinit_loopback_xprt_lists();
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return;
	}
	GLX_DBG("%s: %x:%08x\n", __func__, version, features);
	cmd->type = VERSION;
	cmd->version.version = version;
	cmd->version.features = features;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
}

/**
 * tx_cmd_version_ack() - Transmit a version ack for remote negotiation
 * @if_ptr:	Pointer to interface instance
 * @version:	Version
 * @features:	Features
 */
static void tx_cmd_version_ack(struct glink_transport_if *if_ptr,
			       uint32_t version, uint32_t features)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	cmd->type = VERSION_ACK;
	cmd->version.version = version;
	cmd->version.features = features;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return 0;
	}

	/*
	 * This isn't a command that goes out over the wire - it is just used
	 * internally to configure the transport.
	 */
	cmd->type = SET_VERSION;
	cmd->version.version = version;
	cmd->version.features = features;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);

	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);

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
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);

	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO_PERF("::%s[%d:] %s: start OPEN\n", name, lcid, __func__);
	cmd->type = OPEN;
	cmd->open.lcid = lcid;
	strlcpy(cmd->open.name, name, MAX_NAME_LEN);

	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);

	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO_PERF("::%s[%d:] %s: finish OPEN\n", name, lcid, __func__);
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO_PERF("lcid: %d, %s: start CLOSE\n", lcid, __func__);
	cmd->type = CLOSE;
	cmd->close.lcid = lcid;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO_PERF("lcid: %d, %s: finish CLOSE\n", lcid, __func__);
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	GLX_INFO("rcid: %d, %s: start OPEN_ACK\n", rcid, __func__);
	cmd->type = OPEN_ACK;
	cmd->open_ack.lcid = rcid;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("rcid: %d, %s: finish OPEN_ACK\n", rcid, __func__);
}

/**
 * tx_cmd_ch_remote_close_ack() - Sends the remote open ACK command
 * @if_ptr:	Pointer to interface instance
 * @rcid:	Remote channel ID
 *
 * Sends the remote open ACK command in response to receiving
 * glink_core_if::rx_cmd_ch_remote_close.
 */
static void tx_cmd_ch_remote_close_ack(struct glink_transport_if *if_ptr,
				       uint32_t rcid)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	GLX_INFO("rcid: %d, %s: start CLOSE_ACK\n", rcid, __func__);
	cmd->type = CLOSE_ACK;
	cmd->close_ack.lcid = rcid;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("rcid: %d, %s: finish CLOSE_ACK\n", rcid, __func__);
}

/**
 * allocate_rx_intent - Allocate/reserve space for RX Intent
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
 * reuse_rx_intent() - Reuse space created for RX Intent
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
 * tx_cmd_local_rx_intent() - Send receive intent ID
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO("lcid: %d %s: start L[%d]:%zu RX_INTENT\n", lcid,
			__func__, liid, size);
	cmd->type = RX_INTENT;
	cmd->rx_intent.lcid = lcid;
	cmd->rx_intent.size = size;
	cmd->rx_intent.id = liid;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("lcid: %d %s: finish L[%d]:%zu RX_INTENT\n", lcid,
			__func__, liid, size);
	return 0;
}

/**
 * tx_cmd_local_rx_done() - Send receive-done command
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @liid:	Local intent ID
 * @reuse:	Reuse the consumed intent
 */
static void tx_cmd_local_rx_done(struct glink_transport_if *if_ptr,
	uint32_t lcid, uint32_t liid, bool reuse)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return;
	}

	GLX_INFO("lcid: %d %s: L[%d]:\n", lcid, __func__, liid);
	cmd->type = RX_DONE;
	cmd->rx_done.lcid = lcid;
	cmd->rx_done.liid = liid;
	cmd->rx_done.reuse = reuse;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("lcid: %d %s: end (Success) RX_DONE L[%d]:\n", lcid,
		 __func__, liid);
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
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO_PERF("lcid: %u %s: R[%u]:%u data[%p] size[%u]\n",
		lcid, __func__, pctx->riid, pctx->size, pctx->data, pctx->size);
	cmd->type = DATA;
	cmd->data.lcid = lcid;
	cmd->data.data = (const char *)pctx->data;
	cmd->data.iovec = pctx->iovec;
	cmd->data.riid = pctx->riid;
	cmd->data.size = pctx->size;
	pctx->size_remaining -= pctx->size;
	cmd->data.size_remaining = pctx->size_remaining;
	cmd->data.vbuf_provider = pctx->vprovider;
	cmd->data.pbuf_provider = pctx->pprovider;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	simulate_smem_loopback((void *)cmd->data.data, (size_t)cmd->data.size);
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO_PERF(
		"lcid: %u %s: end (Success) TX R[%u]:%u data[%p] size[%u]\n",
		lcid, __func__, pctx->riid, pctx->size, pctx->data, pctx->size);
	return 0;
}

/**
 * tx_cmd_remote_rx_intent_req_ack() - Transmit the ack for remote
 *				RX intent request
 * @if_ptr:	Pointer to transport interface
 * @lcid:	Local channel ID
 * @granted:	Intent grant status
 *
 * This function transmits the ack for the remote request for an RX intent.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_remote_rx_intent_req_ack(struct glink_transport_if *if_ptr,
						uint32_t lcid, bool granted)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO("lcid: %d %s: start RX_INTENT_REQ_ACK granted: %d\n",
		lcid, __func__, granted);
	cmd->type = RX_INTENT_REQ_ACK;
	cmd->rx_intent_req_ack.lcid = lcid;
	cmd->rx_intent_req_ack.granted = granted;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("lcid: %d %s: finish RX_INTENT_REQ_ACK granted: %d\n",
		lcid, __func__, granted);
	return 0;
}

/**
 * tx_cmd_rx_intent_req() - Transmit the RX intent request
 * @if_ptr:	Pointer to transport interface
 * @lcid:	Local channel ID
 * @size:	Size of the intent
 *
 * This function transmits the request for an RX intent to the remote side.
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_rx_intent_req(struct glink_transport_if *if_ptr,
						uint32_t lcid, size_t size)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
			(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO("lcid: %d %s: start L[]:%zu RX_INTENT_REQ\n", lcid,
		__func__, size);
	cmd->type = RX_INTENT_REQ;
	cmd->rx_intent_req.lcid = lcid;
	cmd->rx_intent_req.size = size;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO("lcid: %d %s: finish L[]:%zu RX_INTENT_REQ\n", lcid,
		__func__, size);
	return 0;
}

/**
 * tx_cmd_set_sigs() - Sends the signal set command
 * @if_ptr:	Pointer to interface instance
 * @lcid:	Local channel ID
 * @sigs:	New signals
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int tx_cmd_set_sigs(struct glink_transport_if *if_ptr, uint32_t lcid,
				uint32_t sigs)
{
	struct glink_loopback_xprt *loopback_xprt_ptr =
		(struct glink_loopback_xprt *)if_ptr;
	struct glink_loopback_xprt_cmd *cmd =
		kzalloc(sizeof(struct glink_loopback_xprt_cmd), GFP_KERNEL);
	if (!cmd) {
		GLX_ERR("%s: No memory for allocation\n", __func__);
		return -ENOMEM;
	}

	GLX_INFO_PERF("lcid: %d, %s: start SIGS_SET\n", lcid, __func__);
	cmd->type = SIGS_SET;
	cmd->sigs_set.lcid = lcid;
	cmd->sigs_set.sigs = sigs;
	mutex_lock(&loopback_xprt_ptr->tx_cmds_lock);
	simulate_smem_loopback((void *)cmd, sizeof(*cmd));
	list_add_tail(&cmd->element, &loopback_xprt_ptr->tx_cmds);
	mutex_unlock(&loopback_xprt_ptr->tx_cmds_lock);
	queue_work(loopback_xprt_ptr->ll_wq, &loopback_xprt_ptr->ll_cmd_work);
	GLX_INFO_PERF("lcid: %d, %s: finish SIGS_SET\n", lcid, __func__);
	return 0;
}

/**
 * init_loopback_xprt_lists() - Initialize loopback transport lists
 *
 * Return: 0
 */
static int init_loopback_xprt_lists(void)
{
	mutex_init(&loopback_xprt.ch_map_lock);
	mutex_init(&loopback_xprt.tx_cmds_lock);
	INIT_LIST_HEAD(&loopback_xprt.ch_map);
	INIT_LIST_HEAD(&loopback_xprt.tx_cmds);
	return 0;
}

static void deinit_loopback_xprt_lists(void)
{
	struct ch_map_info *temp_map, *temp_map1;
	struct glink_loopback_xprt_cmd *temp_cmd, *temp_cmd1;

	mutex_lock(&loopback_xprt.ch_map_lock);
	list_for_each_entry_safe(temp_map, temp_map1,
			&loopback_xprt.ch_map, list) {
		list_del(&temp_map->list);
		kfree(temp_map);
	}
	mutex_unlock(&loopback_xprt.ch_map_lock);
	mutex_lock(&loopback_xprt.tx_cmds_lock);
	list_for_each_entry_safe(temp_cmd, temp_cmd1,
			&loopback_xprt.tx_cmds, element) {
		list_del(&temp_cmd->element);
		kfree(temp_cmd);
	}
	mutex_unlock(&loopback_xprt.tx_cmds_lock);
}

/**
 * init_loopback_xprt_works() - Initialize the loopback transport workqueue
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int init_loopback_xprt_works(void)
{
	loopback_xprt.ll_wq =
	    create_singlethread_workqueue("glink_loopback_xprt");
	if (!loopback_xprt.ll_wq)
		return -ENOMEM;

	INIT_WORK(&loopback_xprt.ll_cmd_work, ll_cmd_worker);
	return 0;
}

/**
 * init_loopback_xprt_works() - Flush and destroy the loopback transport
 *				workqueue
 */
static void deinit_loopback_xprt_works(void)
{
	flush_workqueue(loopback_xprt.ll_wq);
	destroy_workqueue(loopback_xprt.ll_wq);
	loopback_xprt.ll_wq = NULL;
}

void glink_loopback_xprt_link_up(void)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->link_up(&loopback_xprt.if_ptr);
}

/* init_and_register_loopback_xprt_if() - Initialize the loopback transport
 *					and register it with the G-Link core
 *
 * Return: 0 on success, standard error codes otherwise
 */
static int init_and_register_loopback_xprt_if(void)
{
	struct glink_core_transport_cfg cfg;
	int ret;

	loopback_xprt.if_ptr.ssr = ssr;
	loopback_xprt.if_ptr.tx_cmd_version = tx_cmd_version;
	loopback_xprt.if_ptr.tx_cmd_version_ack = tx_cmd_version_ack;
	loopback_xprt.if_ptr.set_version = set_version;
	loopback_xprt.if_ptr.tx_cmd_ch_open = tx_cmd_ch_open;
	loopback_xprt.if_ptr.tx_cmd_ch_close = tx_cmd_ch_close;
	loopback_xprt.if_ptr.tx_cmd_ch_remote_open_ack =
	    tx_cmd_ch_remote_open_ack;
	loopback_xprt.if_ptr.tx_cmd_ch_remote_close_ack =
	    tx_cmd_ch_remote_close_ack;
	loopback_xprt.if_ptr.allocate_rx_intent = allocate_rx_intent;
	loopback_xprt.if_ptr.deallocate_rx_intent = deallocate_rx_intent;
	loopback_xprt.if_ptr.reuse_rx_intent = reuse_rx_intent;
	loopback_xprt.if_ptr.tx_cmd_local_rx_intent = tx_cmd_local_rx_intent;
	loopback_xprt.if_ptr.tx_cmd_local_rx_done = tx_cmd_local_rx_done;
	loopback_xprt.if_ptr.tx_cmd_rx_intent_req = tx_cmd_rx_intent_req;
	loopback_xprt.if_ptr.tx_cmd_remote_rx_intent_req_ack =
					tx_cmd_remote_rx_intent_req_ack;
	loopback_xprt.if_ptr.tx_cmd_set_sigs = tx_cmd_set_sigs;
	loopback_xprt.if_ptr.tx = tx;

	/* initialize transport */
	loopback_xprt.if_ptr.glink_core_if_ptr = NULL;

	cfg.name = "lloop";
	cfg.edge = "local";
	cfg.versions = versions;
	cfg.versions_entries = ARRAY_SIZE(versions);
	cfg.max_cid = UINT32_MAX;
	cfg.max_iid = UINT32_MAX;
	ret = glink_core_register_transport(&loopback_xprt.if_ptr, &cfg);
	if (ret)
		GLX_ERR("%s: Unable to register transport %d\n", __func__, ret);
	else
		glink_loopback_xprt_link_up();
	return ret;
}

int glink_loopback_xprt_init(void)
{
	int ret;

	ret = init_loopback_xprt_lists();
	if (ret < 0) {
		GLX_ERR("%s: Error initializing lists\n", __func__);
		return ret;
	}

	ret = init_loopback_xprt_works();
	if (ret < 0) {
		GLX_ERR("%s: Error initializing works\n", __func__);
		return ret;
	}

	ret = init_and_register_loopback_xprt_if();
	if (ret < 0) {
		deinit_loopback_xprt_works();
		GLX_ERR("%s: Error registering xprt\n", __func__);
	}
	GLX_INFO("%s: Complete\n", __func__);
	return ret;
}

void glink_loopback_xprt_exit(void)
{
	loopback_xprt.if_ptr.glink_core_if_ptr->link_down(&loopback_xprt.if_ptr);
	deinit_loopback_xprt_works();
	deinit_loopback_xprt_lists();
	glink_core_unregister_transport(&loopback_xprt.if_ptr);
}
