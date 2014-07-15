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

#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/ipc_logging.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "loopback_a2.h"
#include "loopback_sps.h"
#include "../../../drivers/soc/qcom/bam_dmux_private.h"

/* Client connection state */
#define SPS_STATE_DISCONNECT	0
#define SPS_STATE_CONNECT	1

#define SPS_EVENT_INDEX(e)	((e) - 1)
#define SPS_ERROR		-1

#define SPSRM_CLEAR		0xcccccccc

#define DEFAULT_IOVEC_RETRYS	1

#define MOCK_SPS_IPC_PAGES	5

void *mock_sps_ipc_log_txt;
static struct mock_sps_pipe *a2_to_bam_pipe;

#define MOCK_SPS_LOG(fmt, args...) \
do { \
	if (mock_sps_ipc_log_txt) { \
		ipc_log_string(mock_sps_ipc_log_txt, fmt, args); \
	} \
} while (0)
static struct workqueue_struct *tx_cmd_done_wq;

static struct workqueue_struct *reg_event_cb_wq;
static int reg_event_cb_wq_init(void);
struct reg_event_work_struct {
	struct work_struct work;
	void (*callback)(struct sps_event_notify *notify);
	struct sps_event_notify event;
};
static void reg_event_cb_handler(struct work_struct *work);

static struct tasklet_struct fill_bam_rx_tasklet;
static void fill_bam_rx_handler(unsigned long data);

/**
 * struct mock_sps_pipe - pipe state information
 * @available_list:	list of available buffers
 * @ready_list:		list of filled buffers
 * @list_lock:		lock that must be grabbed before modifying either lock
 * @client_state:	state of the client
 * @connect:		sps connection for this pipe
 * @event_regs:		registered events for this pipe
 * @event_regs_inited:	flag as to whether or not events are initialized
 */
struct mock_sps_pipe {
	struct list_head available_list;
	struct list_head ready_list;
	spinlock_t list_lock;
	u32 client_state;
	struct sps_connect connect;
	struct sps_register_event *event_regs[SPS_EVENT_MAX];
	int event_regs_inited;
};

/**
 * stuct mock_iov_info - IO vector
 * @iov_info_list:	used to add and remove from pipes
 * @user:		holds the packet passed in by the user parameter of
 *			transfer one
 * @size:		holds the size passed in from transfer one
 * @flags:		holds the flags passed in from transfer one
 * @retrys:		holds the number or retrys for transmitting the io
 */
struct mock_iov_info {
	struct list_head iov_info_list;
	void *user;
	u32 addr;
	u32 size;
	u32 flags;
	unsigned int retrys;
};

/**
 * enum bam_pipe_irq - enum used to describe the bits in the pipe interrupt
 * mask
 * @BAM_PIPE_IRQ_DESC_INT:	BAM finishes descriptor which has INT bit
 *				selected
 * @BAM_PIPE_IRQ_TIMER:		inactivity timer expires
 * @BAM_PIPE_IRQ_WAKE:		wakeup peripheral
 * @BAM_PIPE_IRQ_OUT_OF_DESC:	producer means no free space for descriptor or
 *				consumer means no descriptors to process
 * @BAM_PIPE_IRQ_ERROR:		pipe error
 * @BAM_PIPE_IRQ_EOT:		end of transfer
 */
enum bam_pipe_irq {
	BAM_PIPE_IRQ_DESC_INT = 0x00000001,
	BAM_PIPE_IRQ_TIMER = 0x00000002,
	BAM_PIPE_IRQ_WAKE = 0x00000004,
	BAM_PIPE_IRQ_OUT_OF_DESC = 0x00000008,
	BAM_PIPE_IRQ_ERROR = 0x00000010,
	BAM_PIPE_IRQ_EOT = 0x00000020,
};

/**
 * struct sps_bam_opt_event_table - struct copied from the real sps to
 * validate options for transfer one
 * @event_id:	the id of the event
 * @option:	the transferring option
 * @pipe_irq:	the irq of the bam pipe
 *
 * An array of this struct will be maintained to ensure that only valid
 * combinations will tried to be transferred.
 */
struct sps_bam_opt_event_table {
	enum sps_event event_id;
	enum sps_option option;
	enum bam_pipe_irq pipe_irq;
};

static const struct sps_bam_opt_event_table opt_event_table[] = {
	{SPS_EVENT_EOT, SPS_O_EOT, BAM_PIPE_IRQ_EOT},
	{SPS_EVENT_DESC_DONE, SPS_O_DESC_DONE, BAM_PIPE_IRQ_DESC_INT},
	{SPS_EVENT_WAKEUP, SPS_O_WAKEUP, BAM_PIPE_IRQ_WAKE},
	{SPS_EVENT_INACTIVE, SPS_O_INACTIVE, BAM_PIPE_IRQ_TIMER},
	{SPS_EVENT_OUT_OF_DESC, SPS_O_OUT_OF_DESC,
		BAM_PIPE_IRQ_OUT_OF_DESC},
	{SPS_EVENT_ERROR, SPS_O_ERROR, BAM_PIPE_IRQ_ERROR}
};

static int mock_sps_client_init(struct mock_sps_pipe *client);
static void mock_sps_rm_config_init(struct sps_connect *connect);

/**
 * mock_sps_connect() - mocks the call to sps_connect() to provide
 * loopback support
 * @h:		handle for the sps_pipe
 * @connect:	connection to add to the pipe
 *
 * Return: 0 on success or error code
 */
int mock_sps_connect(struct sps_pipe *h, struct sps_connect *connect)
{
	struct mock_sps_pipe *ctx = (struct mock_sps_pipe *)h;

	if (ctx == NULL) {
		MOCK_SPS_LOG("%s: pipe null\n", __func__);
		return SPS_ERROR;
	} else if (connect == NULL) {
		MOCK_SPS_LOG("%s: connect null\n", __func__);
		return SPS_ERROR;
	}

	if (ctx->client_state != SPS_STATE_DISCONNECT) {
		MOCK_SPS_LOG("%s: client not disconnected\n", __func__);
		return SPS_ERROR;
	}
	ctx->client_state = SPS_STATE_CONNECT;
	ctx->connect = *connect;
	return 0;
}

/**
 * mock_sps_disconnect() - mocks the call to sps_disconnect() to provide
 * loopback support
 * @h:		handle for the sps_pipe
 *
 * Frees IO vectors on pipe and moves the pipe to the disconnect state.
 *
 * Return: 0 on success, error code on failure
 */
int mock_sps_disconnect(struct sps_pipe *h)
{
	unsigned long flags;
	struct mock_sps_pipe *ctx = (struct mock_sps_pipe *)h;
	struct mock_iov_info *info;

	if (ctx == NULL) {
		MOCK_SPS_LOG("%s: pipe null\n", __func__);
		return SPS_ERROR;
	}

	if (ctx->client_state != SPS_STATE_CONNECT) {
		MOCK_SPS_LOG("%s: client not connected\n", __func__);
		return SPS_ERROR;
	}
	if (ctx->connect.mode == SPS_MODE_SRC) {
		mock_a2_clear_lists();
		spin_lock_irqsave(&ctx->list_lock, flags);
		while (!list_empty(&ctx->available_list)) {
			info = list_first_entry(&ctx->available_list,
					struct mock_iov_info, iov_info_list);
			list_del(ctx->available_list.next);
			kfree(info);
		}
		while (!list_empty(&ctx->ready_list)) {
			info = list_first_entry(&ctx->ready_list,
					struct mock_iov_info, iov_info_list);
			list_del(ctx->ready_list.next);
			kfree(info);
		}
		spin_unlock_irqrestore(&ctx->list_lock, flags);
	}
	ctx->client_state = SPS_STATE_DISCONNECT;
	mock_sps_rm_config_init(&(ctx->connect));
	return 0;
}

/**
 * mock_sps_register_bam_device() - mocks the call to sps_register_bam_device
 * to provide loopback support
 * @bam_props:		just verified that non-null, just needed for interface
 *			purposes
 * @dev_handle:		output parameter to receive handle to the device
 *
 * Return: 0 if parameters are non-null, SPS_ERROR otherwise
 */
int mock_sps_register_bam_device(const struct sps_bam_props *bam_props,
	unsigned long *dev_handle)
{
	if (bam_props == NULL) {
		MOCK_SPS_LOG("%s: bam props null\n", __func__);
		return SPS_ERROR;
	} else if (dev_handle == NULL) {
		MOCK_SPS_LOG("%s: dev_handle null\n", __func__);
		return SPS_ERROR;
	}

	*dev_handle = 1;

	return 0;
}

/**
 * mock_sps_deregister_bam_device() - mocks the call to
 * sps_deregister_bam_device to provide loopback support
 * @dev_handle:		handle for the device
 *
 * Return: 0 if handle is non-zero, SPS_ERROR otherwise
 */
int mock_sps_deregister_bam_device(unsigned long dev_handle)
{
	if (dev_handle == 0) {
		MOCK_SPS_LOG("%s: invalid dev_handle\n", __func__);
		return SPS_ERROR;
	}

	return 0;
}

/**
 * mock_sps_alloc_endpoint() - mocks the call to sps_alloc_endpoint() to
 * provide loopback support
 *
 * Return: an initialized mock_sps_pipe (cast as an sps_pipe for interface
 * purposes)
 */
struct sps_pipe *mock_sps_alloc_endpoint(void)
{
	struct mock_sps_pipe *ctx = NULL;
	ctx = kzalloc(sizeof(struct mock_sps_pipe), GFP_KERNEL);
	if (ctx == NULL) {
		MOCK_SPS_LOG("%s: ctx allocation failed\n", __func__);
		return NULL;
	}

	mock_sps_client_init(ctx);
	return (struct sps_pipe *)ctx;
}

/**
 * mock_sps_client_init() - helper function to help initialize the mock_sps_pipe
 * @client:	mock_sps_pipe to be initialized
 *
 * Return: 0 on success, error code otherwise
 */
static int mock_sps_client_init(struct mock_sps_pipe *client)
{
	int i;
	int is_valid = 1;
	if (client == NULL) {
		MOCK_SPS_LOG("%s: client is null\n", __func__);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&(client->available_list));
	INIT_LIST_HEAD(&(client->ready_list));
	spin_lock_init(&(client->list_lock));
	mock_sps_rm_config_init(&(client->connect));
	client->client_state = SPS_STATE_DISCONNECT;

	for (i = 0; i < SPS_EVENT_MAX; i++) {
		client->event_regs[i] = kzalloc(sizeof(
				struct sps_register_event), GFP_KERNEL);
		if (unlikely(client->event_regs[i] == NULL)) {
			is_valid = 0;
			MOCK_SPS_LOG("%s: event_reg allocing failed\n",
				__func__);
		}
	}

	if (likely(is_valid))
		client->event_regs_inited = 1;
	return 0;
}

/**
 * mock_sps_rm_config_init() - helper to configure the initialization  of the
 * connect portion of the pipe
 * @connect:	connection to be initialized
 */
static void mock_sps_rm_config_init(struct sps_connect *connect)
{
	if (connect == NULL) {
		MOCK_SPS_LOG("%s: connect null\n", __func__);
		return;
	}
	memset(connect, SPSRM_CLEAR, sizeof(*connect));
}

/**
 * mock_sps_free_endpoint() - mocks the call to sps_free_endpoint() to provide
 * loopback support
 * @h:		handle for pipe
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_free_endpoint(struct sps_pipe *h)
{
	struct mock_sps_pipe *ctx;
	int i;

	if (h == NULL) {
		MOCK_SPS_LOG("%s: pipe is null\n", __func__);
		return SPS_ERROR;
	}
	ctx = (struct mock_sps_pipe *)h;

	for (i = 0; i < SPS_EVENT_MAX; i++)
		kfree(ctx->event_regs[i]);

	kfree(ctx);

	return 0;
}

/**
 * mock_sps_set_config() - mocks the call to sps_set_config() to provide
 * loopback support
 * @h:		handle for pipe
 * @config:	connection configuration to add to pipe
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_set_config(struct sps_pipe *h, struct sps_connect *config)
{
	if (h == NULL) {
		MOCK_SPS_LOG("%s: pipe is null\n", __func__);
		return SPS_ERROR;
	} else if (config == NULL) {
		MOCK_SPS_LOG("%s: config null\n", __func__);
		return SPS_ERROR;
	}

	((struct mock_sps_pipe *)h)->connect.options = config->options;

	return 0;
}

/**
 * mock_sps_get_config() - mocks the call to sps_get_config() to provide
 * loopback support
 * @h:		handle for pipe
 * @conig:	configuration to store pipe connection in
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_get_config(struct sps_pipe *h, struct sps_connect *config)
{
	if (h == NULL) {
		MOCK_SPS_LOG("%s: pipe is null\n", __func__);
		return SPS_ERROR;
	} else if (config == NULL) {
		MOCK_SPS_LOG("%s: config is null\n", __func__);
		return SPS_ERROR;
	}

	*config = ((struct mock_sps_pipe *)h)->connect;

	return 0;
}

/**
 * mock_sps_device_reset() - mocks the call to sps_device_reset() to provide
 * loopback support
 * @dev:		handle of device to reset
 *
 * Return: SPS_ERROR if handle is 0, 0 otherwise
 */
int mock_sps_device_reset(unsigned long dev)
{
	if (dev == 0) {
		MOCK_SPS_LOG("%s: dev is null\n", __func__);
		return SPS_ERROR;
	}

	return 0;
}

/**
 * mock_sps_register_event() - mocks the call to sps_register_event() to
 * provide loopback support
 * @h:		handle for pipe to register event on
 * @reg:	event to be registered
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_register_event(struct sps_pipe *h, struct sps_register_event *reg)
{
	struct mock_sps_pipe *ctx = (struct mock_sps_pipe *)h;
	int n, index;
	struct sps_register_event *event_reg;

	if (ctx == NULL || !(ctx->event_regs_inited)) {
		MOCK_SPS_LOG("%s: ctx null or event regs not inited, ctx=%p\n",
				__func__, ctx);
		return SPS_ERROR;
	} else if (reg == NULL) {
		MOCK_SPS_LOG("%s: reg is null\n", __func__);
		return SPS_ERROR;
	}

	if (reg->xfer_done != NULL && reg->mode != SPS_TRIGGER_CALLBACK) {
		MOCK_SPS_LOG("%s: invalid reg mode, xfer_done=%p, mode=%u\n",
				__func__, reg->xfer_done, reg->mode);
		return SPS_ERROR;
	}

	for (n = 0; n < ARRAY_SIZE(opt_event_table); n++) {
		if ((reg->options & opt_event_table[n].option) == 0)
			continue;

		index = SPS_EVENT_INDEX(opt_event_table[n].event_id);
		if (index < 0) {
			MOCK_SPS_LOG("%s: invalid index %d\n", __func__,
					index);
			return SPS_ERROR;
		} else {
			event_reg = ctx->event_regs[index];
			event_reg->xfer_done = reg->xfer_done;
			event_reg->callback = reg->callback;
			event_reg->mode = reg->mode;
			event_reg->user = reg->user;
		}
	}

	return 0;
}

/**
 * mock_sps_trigger_event() - mocks the call to sps_trigger_event to provide
 * loopback support
 * @data:		pipe data to trigger event upon
 * @o_option:		options for triggered event
 * @pkt:		packet that triggered event
 */
void mock_sps_trigger_event(void *data, enum sps_option o_option, void *pkt)
{
	int ret, index, n;
	struct reg_event_work_struct *cb_work_struct;
	struct sps_register_event *reg_event;
	enum sps_event event_id;
	struct sps_connect curr_connect;
	struct mock_sps_pipe *pipe = (struct mock_sps_pipe *)data;

	if (pipe == NULL) {
		MOCK_SPS_LOG("%s: pipe null", __func__);
		return;
	}

	ret = mock_sps_get_config((struct sps_pipe *)pipe, &curr_connect);
	if (ret) {
		MOCK_SPS_LOG("%s: get_config fail\n", __func__);
		return;
	}

	if (curr_connect.options & SPS_O_POLL)
		return;

	for (n = 0; n < ARRAY_SIZE(opt_event_table); n++) {
		if (!(o_option & opt_event_table[n].option))
			continue;

		event_id = opt_event_table[n].event_id;
		index = SPS_EVENT_INDEX(event_id);
		if (index < 0) {
			MOCK_SPS_LOG("%s: invalid event index\n", __func__);
			return;
		}
		reg_event = pipe->event_regs[index];
		if (!reg_event || !(reg_event->callback)) {
			MOCK_SPS_LOG("%s: no callback\n", __func__);
			return;
		}

		cb_work_struct = kmalloc(sizeof(*cb_work_struct),GFP_ATOMIC);
		if (cb_work_struct) {
			INIT_WORK(&(cb_work_struct->work),
					reg_event_cb_handler);
			cb_work_struct->event.event_id = event_id;
			cb_work_struct->event.data.transfer.user = pkt;
			cb_work_struct->callback = reg_event->callback;
			queue_work(reg_event_cb_wq,
					&(cb_work_struct->work));
			return;
		} else {
			MOCK_SPS_LOG("%s: work_struct alloc failed\n",
				__func__);
		}
	}
}

/**
 * reg_event_cb_handler() - handler for function for register events
 * @work:		work struct contained in the event to be handled
 */
static void reg_event_cb_handler(struct work_struct *work)
{
	struct reg_event_work_struct *cb_work_struct;

	if (work == NULL) {
		MOCK_SPS_LOG("%s: work null\n", __func__);
		return;
	}

	cb_work_struct = container_of(work,
			struct reg_event_work_struct, work);

	if (cb_work_struct) {
		(*(cb_work_struct->callback))(&(cb_work_struct->event));
		kfree(cb_work_struct);
	}
}

/**
 * fill_bam_rx_handler() - handler used to defer fill of the bam receive
 * buffers to a later time
 * @data:	not used
 *
 * Fills as many available bam receive buffers by popping of packets from the
 * mock a2.
 */
static void fill_bam_rx_handler(unsigned long data)
{
	struct mock_iov_info *info;
	struct sk_buff *skb;
	struct rx_pkt_info *rx_info;
	struct bam_mux_hdr *old_hdr;
	uint8_t *buff;
	int ret;
	struct bam_mux_hdr *new_hdr;
	unsigned long lock_flags;

	if (a2_to_bam_pipe == NULL) {
		MOCK_SPS_LOG("%s: pipe is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&a2_to_bam_pipe->list_lock, lock_flags);
	while (!list_empty(&a2_to_bam_pipe->available_list)) {

		buff = kmalloc(DEFAULT_BUFFER_SIZE, GFP_ATOMIC);
		if (!buff) {
			MOCK_SPS_LOG("%s: buff alloc failed\n", __func__);
			break;
		}

		new_hdr = kmalloc(sizeof(struct bam_mux_hdr), GFP_ATOMIC);
		if (!new_hdr) {
			MOCK_SPS_LOG("%s: bam hdr alloc failed\n", __func__);
			break;
		}

		ret = mock_a2_tx_list_pop(new_hdr, buff);

		if (!ret) {
			info = list_first_entry(&a2_to_bam_pipe->available_list,
					struct mock_iov_info, iov_info_list);
			if (!info) {
				MOCK_SPS_LOG("%s: null info", __func__);
				kfree(buff);
				goto exit;
			}
			list_del_init(&info->iov_info_list);

			dma_sync_single_for_cpu(NULL,
					info->addr, info->size,
					DMA_BIDIRECTIONAL);
			rx_info = ((struct rx_pkt_info *)(info->user));
			if (!rx_info) {
				MOCK_SPS_LOG("%s: invalid packet info\n",
						__func__);
				kfree(info);
				kfree(buff);
				kfree(new_hdr);
				goto exit;
			}
			skb = rx_info->skb;
			if (!skb) {
				MOCK_SPS_LOG("%s: invalid skb\n", __func__);
				kfree(info);
				kfree(buff);
				kfree(new_hdr);
				goto exit;
			}
			old_hdr = (struct bam_mux_hdr *)(skb->data);
			if (!old_hdr) {
				MOCK_SPS_LOG("%s: invalid bam_hdr\n",
						__func__);
				kfree(info);
				kfree(buff);
				kfree(new_hdr);
				goto exit;
			}

			*old_hdr = *new_hdr;
			memcpy(old_hdr + 1, buff, new_hdr->pkt_len);
			skb_set_network_header(skb, sizeof(struct bam_mux_hdr));
			skb->ip_summed = CHECKSUM_NONE;
			kfree(buff);
			kfree(new_hdr);
			dma_sync_single_for_device(
					NULL,
					info->addr, info->size,
					DMA_BIDIRECTIONAL);
			list_add_tail(&(info->iov_info_list),
					&(a2_to_bam_pipe->ready_list));
			spin_unlock_irqrestore(&(a2_to_bam_pipe->list_lock),
					lock_flags);
			mock_sps_trigger_event((void *)a2_to_bam_pipe,
					SPS_O_EOT, NULL);
			spin_lock_irqsave(&(a2_to_bam_pipe->list_lock),
					lock_flags);
		} else {
			MOCK_SPS_LOG("%s: tx pop fail\n", __func__);
			kfree(buff);
			kfree(new_hdr);
			break;
		}
	}
exit:
	spin_unlock_irqrestore(&(a2_to_bam_pipe->list_lock), lock_flags);

}

/**
 * mock_sps_transfer_one() - mocks the call to sps_transfer_one() to provide
 * loopback support
 * @pipe:		pipe to transfer on
 * @addr:		address to be transferred
 * @size:		size to be transferred
 * @user:		pointer to user data
 * @flags:		flags used for transferring
 *
 * Used to both send data and queue up receive buffers, depending on which pipe
 * is used.
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_transfer_one(struct sps_pipe *h, phys_addr_t addr, u32 size,
	void *user, u32 flags)
{
	struct mock_sps_pipe *ctx = (struct mock_sps_pipe *)h;
	unsigned long lock_flags;
	struct mock_iov_info *info;
	int ret;

	if (ctx == NULL) {
		MOCK_SPS_LOG("%s: pipe_null\n", __func__);
		return SPS_ERROR;
	}

	if ((flags & SPS_IOVEC_FLAG_NWD) &&
			!(flags & (SPS_IOVEC_FLAG_EOT | SPS_IOVEC_FLAG_CMD))) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	} else if ((flags & SPS_IOVEC_FLAG_EOT) &&
			(flags & SPS_IOVEC_FLAG_CMD)) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	} else if (!(flags & SPS_IOVEC_FLAG_CMD) &&
			(flags & (SPS_IOVEC_FLAG_LOCK |
			SPS_IOVEC_FLAG_UNLOCK))) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	} else if ((flags & SPS_IOVEC_FLAG_LOCK) &&
			(flags & SPS_IOVEC_FLAG_UNLOCK)) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	} else if ((flags & SPS_IOVEC_FLAG_IMME) &&
			(flags & SPS_IOVEC_FLAG_CMD)) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	} else if ((flags & SPS_IOVEC_FLAG_IMME) &&
			(flags & SPS_IOVEC_FLAG_NWD)) {
		MOCK_SPS_LOG("%s: invalid flags %x\n", __func__, flags);
		return SPS_ERROR;
	}

	if (ctx->connect.mode == SPS_MODE_DEST) {
		ret = mock_a2_rx_list_push((void *)ctx,
				(struct tx_pkt_info *)user);
		if (ret)
			MOCK_SPS_LOG("%s: a2 rx push failed\n", __func__);
		return ret;
	}

	info = kmalloc(sizeof(struct mock_iov_info), GFP_ATOMIC);
	if (info == NULL) {
		MOCK_SPS_LOG("%s: info alloc failed\n", __func__);
		return -ENOMEM;
	}

	info->addr = addr;
	info->size = size;
	info->flags = flags;
	info->user = user;
	info->retrys = DEFAULT_IOVEC_RETRYS;
	INIT_LIST_HEAD(&(info->iov_info_list));

	spin_lock_irqsave(&(ctx->list_lock), lock_flags);
	list_add_tail(&(info->iov_info_list), &(ctx->available_list));
	spin_unlock_irqrestore(&(ctx->list_lock), lock_flags);
	a2_to_bam_pipe = ctx;

	tasklet_schedule(&fill_bam_rx_tasklet);
	return 0;
}

/**
 * mock_sps_get_iovec() - mocks the call to sps_get_iovec() to provide
 * loopback support
 * @h:		handle of pipe to get iovec from
 * @iovec:	where the io vector will be placed
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_get_iovec(struct sps_pipe *h, struct sps_iovec *iovec)
{
	struct mock_sps_pipe *ctx = (struct mock_sps_pipe *)h;
	struct mock_iov_info *info;
	unsigned long lock_flags;

	if (iovec == NULL) {
		MOCK_SPS_LOG("%s: iovec null\n", __func__);
		return SPS_ERROR;
	} else if (ctx == NULL) {
		MOCK_SPS_LOG("%s: pipe null\n", __func__);
		return SPS_ERROR;
	}

	MOCK_SPS_LOG("%s:\n", __func__);
	spin_lock_irqsave(&(ctx->list_lock), lock_flags);
	if (list_empty(&(ctx->ready_list))) {
		spin_unlock_irqrestore(&(ctx->list_lock), lock_flags);
		iovec->addr = 0;
		return 0;
	} else {
		info = list_first_entry(&ctx->ready_list,
				struct mock_iov_info, iov_info_list);
		info->retrys--;
		if (info->retrys == 0) {
			list_del(&info->iov_info_list);
			iovec->addr = info->addr;
			iovec->size = info->size;
			iovec->flags = info->flags;
		} else
			iovec->addr = 0;

		spin_unlock_irqrestore(&(ctx->list_lock), lock_flags);
		kfree(info);
	}

	return 0;
}

/**
 * mock_sps_get_unused_desc_num() - mocks the call to sps_get_unused_desc_num
 * to provide loopback support
 * @h:		handle for pipe
 * @desc_num:	where number of unused descriptors will be stored
 *
 * Return: 0 on success, error code otherwise
 */
int mock_sps_get_unused_desc_num(struct sps_pipe *h, u32 *desc_num)
{
	if (!h || !desc_num) {
		return -EINVAL;
	} else {
		*desc_num = 0;
		return 0;
	}
}

/**
 * mock_sps_rewrite_notify() -  this function schedules a tasklet which
 * processes all available packets to be sent
 */
void mock_sps_rewrite_notify(void)
{
	tasklet_schedule(&fill_bam_rx_tasklet);
}

/**
 * reg_event_cb_wq_init() - initializes the workqueue that processes registered
 * events
 */
static int reg_event_cb_wq_init(void)
{
	int ret = 0;

	reg_event_cb_wq = create_singlethread_workqueue(
			"reg_event_cb_wq");
	if (!reg_event_cb_wq) {
		MOCK_SPS_LOG("%s: workqueue creation failed\n", __func__);
		ret = -EFAULT;
	}

	return ret;
}

/**
 * mock_sps_init() - initializaes all of the mock_sps functionality
 *
 * Initializes all of the mock_sps tasklets, workqueues and ipc logging
 * context.
 */
int mock_sps_init(void)
{
	int ret = 0;

	mock_sps_ipc_log_txt = ipc_log_context_create(MOCK_SPS_IPC_PAGES,
			"mock_sps", 0);
	if (!mock_sps_ipc_log_txt)
		pr_err("%s: failed to allocate log\n", __func__);

	ret = reg_event_cb_wq_init();
	if (ret) {
		pr_err("%s: reg_event_cb_wq_init failed:%d\n", __func__, ret);
		return ret;
	}

	tasklet_init(&fill_bam_rx_tasklet, fill_bam_rx_handler, 0);

	tx_cmd_done_wq = create_workqueue(
			"tx_cmd_done_wq");
	if (!tx_cmd_done_wq) {
		MOCK_SPS_LOG("%s: cmd_done_wq alloc fail\n", __func__);
		ret = -EFAULT;
	}
	return ret;
}
