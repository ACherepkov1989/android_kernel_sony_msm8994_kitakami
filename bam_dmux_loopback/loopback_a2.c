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

#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/ipc_logging.h>

#include <soc/qcom/bam_dmux.h>
#include <soc/qcom/smsm.h>

#include "loopback_a2.h"
#include "loopback_smsm.h"
#include "loopback_sps.h"
#include "../../../drivers/soc/qcom/bam_dmux_private.h"

#define LOOPBACK_NETWORK_OCTET_2		192
#define LOOPBACK_NETWORK_OCTET_1		168
#define LOOPBACK_NETWORK_OCTET_0		1
#define	LOOPBACK_HOST_OCTET			3

#define MOCK_A2_IPC_PAGES			5
void *mock_a2_ipc_log_txt;

#define MOCK_A2_LOG(fmt, args...) \
do { \
	if (mock_a2_ipc_log_txt) { \
		ipc_log_string(mock_a2_ipc_log_txt, fmt, args); \
	} \
} while (0)

/**
 * struct mock_a2_lists - maintains a list of incoming and outgoing packets
 * @tx_list:		list of outgoing packets
 * @rx_list:		list of incoming packets
 * @tx_list_lock_lha2:	spinlock which must be grabbed before modifying tx_list
 * @rx_list_lock_lha1:	spinlock which must be grabbed before modifying rx_list
 */
struct mock_a2_lists {
	struct list_head tx_list;
	struct list_head rx_list;
	spinlock_t tx_list_lock_lha2;
	spinlock_t rx_list_lock_lha1;
};
static struct mock_a2_lists mock_a2;

/**
 * struct hdr_list - struct to be queued as outgoing packets to bam dmux
 * @hdr:		outgoing packet hdr
 * @pkt:		outgoing packet
 * @list_entry:		list head to handle adding and removing from lists
 */
struct hdr_list {
	struct bam_mux_hdr hdr;
	uint8_t *pkt;
	struct list_head list_entry;
};

/**
 * struct rx_list_entry - struct to be queued for incoming packets
 * @pipe:		pipe which packet was received on
 * @info:		incoming packet info
 * @list_entry:		list head to handle adding and removing from the lists
 */
struct rx_list_entry {
	void *pipe;
	struct tx_pkt_info *info;
	struct list_head list_entry;
};

static void logging_init(void);

static void toggle_mdm_ack(void);
static void mock_a2_smsm_cb(void *priv, uint32_t old_state,
		uint32_t new_state);
static void mock_a2_smsm_ack_cb(void *priv, uint32_t old_state,
				uint32_t new_state);

static int mock_a2_power_state;
static DEFINE_SPINLOCK(smsm_cb_lock);

static void mock_a2_rx_tasklet_handler(unsigned long data);
static struct tasklet_struct *mock_a2_rx_tasklet;

static void copy_to_tx(struct tx_pkt_info *info);
static void tx_open_commands_init(void);

/**
 * mock_a2_init() - helper function to initialize the mock_a2
 *
 * This function handles initializing lists, locks and workqueues, as well as
 * queueing up open commands and registering state callbacks with the
 * mock_smsm.
 *
 * Return: 0 on success, error code otherwise
 */
int mock_a2_init(void)
{
	INIT_LIST_HEAD(&(mock_a2.rx_list));
	INIT_LIST_HEAD(&(mock_a2.tx_list));
	spin_lock_init(&(mock_a2.rx_list_lock_lha1));
	spin_lock_init(&(mock_a2.tx_list_lock_lha2));

	logging_init();

	tx_open_commands_init();

	mock_a2_rx_tasklet  = kmalloc(sizeof(struct tasklet_struct),
			GFP_ATOMIC);
	if (!mock_a2_rx_tasklet) {
		MOCK_A2_LOG("%s: tasklet allocation failed", __func__);
		return -ENOMEM;
	}
	tasklet_init(mock_a2_rx_tasklet, &mock_a2_rx_tasklet_handler, 0);

	mock_smsm_state_cb_register(SMSM_APPS_STATE, SMSM_A2_POWER_CONTROL,
			mock_a2_smsm_cb, NULL);

	mock_smsm_state_cb_register(SMSM_APPS_STATE,
			SMSM_A2_POWER_CONTROL_ACK, mock_a2_smsm_ack_cb, NULL);

	return 0;
}

/**
 * logging_init() - initializes the logging for the mock_a2
 *
 * This function initializes logging context that is required for ipc logging.
 */
static void logging_init(void)
{
	mock_a2_ipc_log_txt = ipc_log_context_create(MOCK_A2_IPC_PAGES,
			"mock_a2", 0);
	if (!mock_a2_ipc_log_txt)
		pr_err("%s: failed to allocate log\n", __func__);
}

/**
 * tx_open_commands_init() - creates the open commands to be sent to bam dmux
 *
 * Creates open commands and queues them on the tx list to be sent to bam dmux
 * at a later time.
 */
static void tx_open_commands_init(void)
{
	struct hdr_list *list;
	int i;
	unsigned long flags;

	for (i = 0; i < BAM_DMUX_NUM_CHANNELS; i++) {
		list = kzalloc(sizeof(struct hdr_list), GFP_KERNEL);
		if (!list) {
			MOCK_A2_LOG("%s: list allocation failed\n", __func__);
			break;
		}

		list->hdr.magic_num = BAM_MUX_HDR_MAGIC_NO;
		list->hdr.cmd = BAM_MUX_HDR_CMD_OPEN;
		list->hdr.ch_id = i;
		list->hdr.pkt_len = 0;
		INIT_LIST_HEAD(&(list->list_entry));
		spin_lock_irqsave(&(mock_a2.tx_list_lock_lha2), flags);
		list_add_tail(&(list->list_entry), &(mock_a2.tx_list));
		spin_unlock_irqrestore(&(mock_a2.tx_list_lock_lha2), flags);

	}
	MOCK_A2_LOG("%s: complete\n", __func__);
}

/**
 * mock_a2_tx_list_pop() - pops off the first element from the tx_list
 * @hdr:	bam_dmux_hdr to store the popped header
 * @buff:	buffer to write the contents of popped pkt
 *
 * Return: 0 on success, error code otherwise
 */
int mock_a2_tx_list_pop(struct bam_mux_hdr *hdr, void *buff)
{
	struct hdr_list *popped_entry;
	unsigned long flags;

	if (hdr == NULL || buff == NULL)
		return -EINVAL;

	spin_lock_irqsave(&(mock_a2.tx_list_lock_lha2), flags);
	if (list_empty(&(mock_a2.tx_list))) {
		spin_unlock_irqrestore(&(mock_a2.tx_list_lock_lha2), flags);
		return -EAGAIN;
	}

	popped_entry = list_first_entry(&mock_a2.tx_list,
			struct hdr_list,
			list_entry);
	list_del(mock_a2.tx_list.next);
	spin_unlock_irqrestore(&(mock_a2.tx_list_lock_lha2), flags);

	if (!popped_entry)
		return -EAGAIN;

	*hdr = popped_entry->hdr;
	if (hdr->pkt_len > 0)
		memcpy(buff, popped_entry->pkt, hdr->pkt_len);

	kfree(popped_entry->pkt);
	kfree(popped_entry);

	return 0;
}

/**
 * mock_a2_rx_list_push() - push an incoming packet onto the receive list
 * @data:	incoming pipe data
 * @pkt:	incoming packet information
 *
 * Return: 0 on success, error code on failure
 */
int mock_a2_rx_list_push(void *data, struct tx_pkt_info *pkt)
{
	struct rx_list_entry *pushed_entry;
	unsigned long flags;

	pushed_entry = kmalloc(sizeof(
			struct rx_list_entry), GFP_ATOMIC);
	if (!pushed_entry) {
		MOCK_A2_LOG("%s: entry allocation failed\n", __func__);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&pushed_entry->list_entry);
	pushed_entry->info = pkt;
	pushed_entry->pipe = data;

	spin_lock_irqsave(&mock_a2.rx_list_lock_lha1, flags);
	list_add_tail(&pushed_entry->list_entry, &mock_a2.rx_list);
	tasklet_schedule(mock_a2_rx_tasklet);
	spin_unlock_irqrestore(&mock_a2.rx_list_lock_lha1, flags);

	return 0;
}

/**
 *  mock_a2_clear_lists() - removes and frees all elements in the rx/tx lists
 */
void mock_a2_clear_lists(void)
{
	struct list_head *pos;
	struct list_head *tmp;
	struct rx_list_entry *rx_entry;
	struct hdr_list *tx_entry;
	unsigned long flags;

	spin_lock_irqsave(&mock_a2.rx_list_lock_lha1, flags);
	list_for_each_safe(pos, tmp, &mock_a2.rx_list) {
		rx_entry = list_entry(pos, struct rx_list_entry, list_entry);
		list_del(pos);
		kfree(rx_entry);
	}
	spin_unlock_irqrestore(&mock_a2.rx_list_lock_lha1, flags);

	spin_lock_irqsave(&mock_a2.tx_list_lock_lha2, flags);
	list_for_each_safe(pos, tmp, &mock_a2.tx_list) {
		tx_entry = list_entry(pos, struct hdr_list, list_entry);
		list_del(pos);
		kfree(tx_entry->pkt);
		kfree(tx_entry);
	}
	spin_unlock_irqrestore(&mock_a2.tx_list_lock_lha2, flags);
}

/**
 * mock_a2_rx_tasklet_handler() - handler for processing incoming packets
 * placed on rx_list
 * @data:	needed to meet interface for a tasklet handler, but not used
 *
 * Processes each element on the rx_list, signaling sps that it received that
 * packet successfully, and perform the appropriate action depending on
 * whether the packet is a data or a command packet.
 */
static void mock_a2_rx_tasklet_handler(unsigned long data)
{
	struct rx_list_entry *entry;
	struct list_head *pos, *tmp;
	struct tx_pkt_info *info;
	unsigned long flags;

	spin_lock_irqsave(&mock_a2.rx_list_lock_lha1, flags);
	list_for_each_safe(pos, tmp, &mock_a2.rx_list) {

		entry = list_entry(pos, struct rx_list_entry,
				list_entry);
		list_del(pos);
		info = entry->info;
		if (info == NULL) {
			MOCK_A2_LOG("%s: null info\n", __func__);
			kfree(entry);
			continue;
		}
		if (info->is_cmd) {
			MOCK_A2_LOG("%s: handling command\n", __func__);
			mock_sps_trigger_event(entry->pipe, SPS_O_EOT,
					(void *)(entry->info));
		} else {
			MOCK_A2_LOG("%s: handling data\n", __func__);
			copy_to_tx(info);
			mock_sps_trigger_event(entry->pipe, SPS_O_EOT,
					(void *)(entry->info));
			mock_sps_rewrite_notify();
		}
		kfree(entry);
	}

	spin_unlock_irqrestore(&mock_a2.rx_list_lock_lha1, flags);
}

/**
 * copy_to_tx() - copies an incoming packet to the tx_list
 * @info: incoming packet to be copied
 *
 * Copies and swaps the incoming packet to the tx_list, and swaps the source and
 * destination address if it is destined for the loopback ip address.
 */
static void copy_to_tx(struct tx_pkt_info *info)
{
	struct sk_buff *skb;
	struct bam_mux_hdr *hdr;
	struct hdr_list *list;
	struct iphdr *iph;
	uint32_t source;
	uint32_t dest, dst_oct_3, dst_oct_2, dst_oct_1, dst_oct_0;
	unsigned long flags;

	if (info == NULL) {
		MOCK_A2_LOG("%s: info null\n", __func__);
		return;
	}

	skb = info->skb;
	if (skb == NULL || skb->data == NULL) {
		MOCK_A2_LOG("%s: bad skb\n", __func__);
		return;
	}
	iph = ip_hdr(skb);

	if (!iph) {
		MOCK_A2_LOG("%s: ipheader null\n", __func__);
		return;
	}

	hdr = (struct bam_mux_hdr *)(skb->data);
	list = kmalloc(sizeof(struct hdr_list), GFP_ATOMIC);
	if (!list) {
		MOCK_A2_LOG("%s: list allocation failed\n", __func__);
		return;
	}

	list->pkt = kmalloc(hdr->pkt_len, GFP_ATOMIC);
	if (list->pkt == NULL) {
		MOCK_A2_LOG("%s: pkt allocation fail\n", __func__);
		kfree(list);
		return;
	}
	list->hdr = *hdr;
	memcpy(list->pkt, iph, hdr->pkt_len);

	source = iph->saddr;
	dest = iph->daddr;
	dst_oct_0 = (dest >> 24) & 0xFF;
	dst_oct_1 = (dest >> 16) & 0xFF;
	dst_oct_2 = (dest >> 8) & 0xFF;
	dst_oct_3 = dest & 0xFF;

	if (dst_oct_3 == LOOPBACK_NETWORK_OCTET_2 &&
			dst_oct_2 == LOOPBACK_NETWORK_OCTET_1 &&
			dst_oct_1 == LOOPBACK_NETWORK_OCTET_0 &&
			dst_oct_0 == LOOPBACK_HOST_OCTET) {
		((struct iphdr *) list->pkt)->daddr = source;
		((struct iphdr *) list->pkt)->saddr = dest;
	}

	spin_lock_irqsave(&mock_a2.tx_list_lock_lha2, flags);
	INIT_LIST_HEAD(&(list->list_entry));
	list_add_tail(&(list->list_entry), &(mock_a2.tx_list));
	spin_unlock_irqrestore(&mock_a2.tx_list_lock_lha2, flags);
}

/**
 * toggle_mdm_ack() - toggle smsm_a2_power_control_ack bit
 */
static void toggle_mdm_ack(void)
{
	static unsigned int clear_bit;

	mock_smsm_change_state(SMSM_MODEM_STATE,
				clear_bit & SMSM_A2_POWER_CONTROL_ACK,
				~clear_bit & SMSM_A2_POWER_CONTROL_ACK);
	clear_bit = ~clear_bit;
}

/**
 * mock_a2_smsm_cb() - callback for changing the a2_smsm state
 * @priv:		required to meet state cb interface, but not used here
 * @old_state:		old bitmask state
 * @new_state:		new bitmask state
 */
static void mock_a2_smsm_cb(void *priv, uint32_t old_state,
		uint32_t new_state)
{
	static int last_processed_state;
	unsigned long flags;

	spin_lock_irqsave(&smsm_cb_lock, flags);
	mock_a2_power_state = new_state & SMSM_A2_POWER_CONTROL ? 1 : 0;

	if (last_processed_state == (new_state & SMSM_A2_POWER_CONTROL)) {
		MOCK_A2_LOG("%s: already processed this state\n", __func__);
		goto exit;
	}

	last_processed_state = new_state & SMSM_A2_POWER_CONTROL;

	toggle_mdm_ack();
	if (mock_a2_power_state)
		mock_smsm_change_state(SMSM_MODEM_STATE, 0,
				SMSM_A2_POWER_CONTROL);
	else
		mock_smsm_change_state(SMSM_MODEM_STATE, SMSM_A2_POWER_CONTROL,
				0);

exit:
	spin_unlock_irqrestore(&smsm_cb_lock, flags);
}

/**
 * mock_a2_smsm_ack_cb() - callback for for smsm ack'ing the state change
 *
 * Doesn't do anything, only implemented for interface purposes.
 */
static void mock_a2_smsm_ack_cb(void *priv, uint32_t old_state,
	uint32_t new_state)
{
}
