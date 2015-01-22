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

#include <linux/types.h>	/* u32 */
#include <linux/kernel.h>	/* pr_debug() */
#include <linux/slab.h>		/* kzalloc() */
#include <linux/mutex.h>	/* mutex */
#include <linux/list.h>		/* list_head */
#include <linux/delay.h>	/* msleep */
#include <linux/memory.h>	/* memset */
#include <linux/device.h>	/* device */
#include <linux/cdev.h>		/* cdev_alloc() */
#include <linux/fs.h>		/* alloc_chrdev_region() */
#include <linux/module.h>	/* module_init() */
#include <linux/dma-mapping.h>	/* dma_alloc_coherent() */
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/msm-sps.h>		/* SPS API*/
#include <linux/ipa.h>
#include <linux/skbuff.h>	/* sk_buff */
#include <linux/kfifo.h>  /* Kernel FIFO Implementation */
#include <linux/delay.h> /* msleep() */
#include <linux/string.h>
#include "ipa_rm_ut.h"
#include "ipa_test_module.h"

#ifndef IPA_ON_R3PC
#define IPA_ON_R3PC
#endif

/** Module name string */
#define DRV_NAME "ipa_test"
#define DRV_VERSION "3.1"

#define IPA_SUMMING_THRESHOLD 0x10
#define IPA_EVENT_THRESHOLD 0x10
#define IPA_NUM_PIPES 0x14

#define TEST_SIGNATURE 0xfacecafe
#define BAMDMA_MAX_CHANS 10
#define DFAB_ARB1_HCLK_CTL		(MSM_CLK_CTL_BASE + 0x2564)

#define DESC_FIFO_SZ 0x100
#define DATA_FIFO_SZ 0x2000

#define TX_NUM_BUFFS 16
#define TX_SZ 32768
#define TX_BUFF_SIZE ((TX_SZ)/(TX_NUM_BUFFS))

#define RX_NUM_BUFFS 16
#define RX_SZ 32768
#define RX_BUFF_SIZE ((RX_SZ)/(RX_NUM_BUFFS))

#define IPA_TEST_DMUX_HEADER_LENGTH           8
#define IPA_TEST_META_DATA_IS_VALID           1
#define IPA_TEST_DMUX_HEADER_META_DATA_OFFSET 4

#define IPA_TEST_WLAN_HDR_LEN 4

#define IPA_TEST_META_DATA_OFFSET_NONE        0

#define IPA_TEST_HDI_802_HEADER_LENGTH             22
#define IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET       11
#define IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET_VALID  1
#define IPA_TEST_HDI_802_ADD_CONST_LENGTH           0

#define IPA_TEST_HDI_RMNET_HEADER_LENGTH              6
#define IPA_TEST_HDI_RMNET_LENGTH_FIELD_OFFSET        0
#define IPA_TEST_HDI_RMNET_LENGTH_FIELD_OFFSET_VALID  0
#define IPA_TEST_HDI_RMNET_ADD_CONST_LENGTH           0

/* Settings of Exception Handling */
#define RX_DESCRIPTOR_SIZE 2048
#define EXCEPTION_DRV_NAME "ipa_exception_pipe"
#define EXCEPTION_KFIFO_SIZE (8)
#define EXCEPTION_KFIFO_SLEEP_MS (EXCEPTION_KFIFO_SLEEP_MS)
#define EXCEPTION_KFIFO_DEBUG_VERBOSE 1
#define SAVE_HEADER 1

#ifdef IPA_ON_R3PC
int ipa_sys_setup(struct ipa_sys_connect_params *sys_in, unsigned long *ipa_bam_hdl,
		  u32 *ipa_pipe_num, u32 *clnt_hdl);
int ipa_sys_teardown(u32 clnt_hdl);
#endif

enum fops_type {
	IPA_TEST_REG_CHANNEL,
	IPA_TEST_DATA_PATH_TEST_CHANNEL,
	MAX_FOPS
};

struct notify_cb_data_st {
	struct kfifo_rec_ptr_2 exception_kfifo;
};

struct exception_hdl_data {
	struct class *class;
	struct device *dev;
	struct cdev *p_cdev;
	dev_t dev_num;
	struct notify_cb_data_st notify_cb_data;
};

/*struct exception_hdl_data *p_exception_hdl_data = NULL;*/
struct exception_hdl_data *p_exception_hdl_data;

struct test_endpoint_dma {
	struct sps_pipe *sps;
	struct sps_connect connect;
	struct sps_alloc_dma_chan alloc;
	struct sps_dma_chan chan;
	u32 clnt_hdl;     /* IPA assigned handle to client */
};

struct test_endpoint_sys {
	struct sps_pipe *sps;
	struct sps_connect connect;
	struct sps_register_event reg_event; /* Check... */
	struct sps_iovec iovec; /* An array of descriptor like couples */
	struct completion xfer_done; /*A completion object for end transfer*/

};

#define MAX_CHANNEL_NAME (20)

/* A channel device is the representation of the flow of data from A5
	to IPA and vice versa.
	On Virtio the flow is A5->BAM-DMA->IPA, on R3PC A5->IPA. */
struct channel_dev {
	/*OS structures for representation of a channel.*/
	struct class *class;
	dev_t dev_num;
	struct device *dev;
	struct cdev cdev;

	/*?The representation of the connection from A5 to BAM-DMA/IPA*/
	struct test_endpoint_sys ep;
	/*?The representation of the connection from BAM-DMA to IPA*/
	struct test_endpoint_dma dma_ep;
	/*The data/desc FIFO for the A5 to IPA/BAM-DMA*/
	struct sps_mem_buffer desc_fifo;
	struct sps_mem_buffer mem;
	/*A pointer to the test context - should be deleted - TODO*/
	struct test_context *test;
	struct sps_register_event rx_event;/*SPS feature to get events*/

	int index;/*to_ipa_<index>/from_ipa_<index>*/
	char name[MAX_CHANNEL_NAME];
	int rx_pool_inited;/*check... - should be moved to contex?*/
	int ipa_client_hdl;/*returned from ipa_connect*/
};

#define MAX_CHANNEL_DEVS (10)
static struct channel_dev *to_ipa_devs[MAX_CHANNEL_DEVS/2];
/*TODO - legacy*/
static struct channel_dev *from_ipa_devs[MAX_CHANNEL_DEVS/2];

/*A handle to the BAM of the BAM-DMA HW.*/
static unsigned long dma_bam_hdl;

/*This structure holds all the data required for the test module.*/
struct test_context {

	/*OS structures for representation of the test module.*/
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;

	/*All channels that are used to read data from
	 * the IPA(Receive channel)*/
	struct channel_dev *rx_channels[MAX_CHANNEL_DEVS/2];

	/*All channels that are used to write data
	 * to the IPA(Transmit channel)*/
	struct channel_dev *tx_channels[MAX_CHANNEL_DEVS/2];
	int num_rx_channels;
	int num_tx_channels;

	/*current test case(-EINVAL is for not-configured)TODO*/
	s32 configuration_idx;
	s32 current_configuration_idx;

	u32 signature;/*Legacy*/
};

/**
 * struct ipa_tx_suspend_private_data - private data for IPA_TX_SUSPEND_IRQ use
 * @clnt_hdl: client handle assigned by IPA
 */
struct ipa_tx_suspend_private_data {
	u32 clnt_hdl;
};

static struct test_context *ipa_test;

/**
 * Allocate memory from system memory.
 *
 * @param mem
 */
static void test_alloc_mem(struct sps_mem_buffer *mem)
{
	dma_addr_t dma_addr;

	/* need to check return value in formal code */
	mem->base = dma_alloc_coherent(ipa_test->dev, mem->size, &dma_addr, GFP_KERNEL);
	mem->phys_base = dma_addr;
}

/**
 * Free memory from system memory.
 *
 * @param mem
 */
static void test_free_mem(struct sps_mem_buffer *mem)
{
	dma_addr_t dma_addr = mem->phys_base;

	if (dma_addr)
		dma_free_coherent(ipa_test->dev, mem->size, mem->base, dma_addr);

	mem->phys_base = 0;
	mem->base = NULL;
}

void print_buff(void *data, size_t size)
{
	u8 bytes_in_line = 16;
	int i, j, num_lines;
	char str[256], tmp[4];

	num_lines = size / bytes_in_line;
	if (size % bytes_in_line > 0)
		num_lines++;

	pr_debug(DRV_NAME
		":Printing buffer at address 0x%p, size = %zu:\n"
		, data, size);
	for (i = 0; i < num_lines; i++) {
		strlcpy(str, "\0", sizeof(str));
		for (j = 0; (j < bytes_in_line) &&
			((i * bytes_in_line + j) < size); j++) {
			snprintf(tmp, sizeof(tmp), "%02x ",
					((unsigned char *)data)
					[i * bytes_in_line + j]);
			strlcat(str, tmp, sizeof(str));
		}
		pr_debug(DRV_NAME ": %s\n", str);
	}
}

static int wait_xfer_completion(int use_irq,
				struct completion *xfer_done,
				struct sps_pipe *sps,
				int rx_xfers,
				u32 max_retry,
				u32 delay_ms)
{
	struct sps_event_notify dummy_event = {0};
	int i;

	if (use_irq) {
		wait_for_completion(xfer_done);
	} else {
		for (i = 0; i < rx_xfers; i++) {
			u32 retry = 0;
			while (!try_wait_for_completion(xfer_done)) {
				sps_get_event(sps, &dummy_event);
				if (retry++ >= max_retry)
					return -EBUSY;

				if (delay_ms)
					msleep(delay_ms);
			}	/* end-of-while */
		}	/* end-of-for */
	}	/* end-of-if */

	return 0;
}

static int channel_open(struct inode *inode, struct file *filp)
{
	struct channel_dev *channel_dev;

	/* Get the channel device data */
	channel_dev = container_of(inode->i_cdev, struct channel_dev, cdev);

	pr_debug(DRV_NAME
			":%s() channel_dev address = 0x%p\n",
			__func__, channel_dev);

	filp->private_data = channel_dev;

	return 0;
}

int insert_descriptors_into_rx_endpoints(u32 count)
{
	struct channel_dev *rx_channel = NULL;
	int i, j, res = 0;

	/* Insert a descriptor into the receiving end(s) */
	for (i = 0; i < ipa_test->num_rx_channels; i++) {
		rx_channel = ipa_test->rx_channels[i];
		if (!rx_channel->rx_pool_inited) {
			res = 0;
			for (j = 0; j < RX_NUM_BUFFS; j++) {
				res |= sps_transfer_one(rx_channel->ep.sps,
					rx_channel->mem.phys_base + j * count,
				       count, 0, 0);
				pr_debug(DRV_NAME ":%s() sps_transfer_one rx res = %d.\n",
					__func__, res);
			}

			if (res == 0)
				rx_channel->rx_pool_inited = 1;
		}
	}

	return res;
}

static ssize_t channel_write(struct file *filp, const char __user *buf,
			     size_t count, loff_t *f_pos)
{
	struct channel_dev *channel_dev = filp->private_data;
	u32 tx_xfer_flags = SPS_IOVEC_FLAG_EOT;
	int res = 0;
	u32 buf_index = 0;
	void *data_address = channel_dev->mem.base
			+ buf_index * TX_BUFF_SIZE;
	u32 data_phys_addr = channel_dev->mem.phys_base
			+ buf_index * TX_BUFF_SIZE;

	/* Copy the data from the user and transmit */
	res = copy_from_user(data_address, buf, count);
	if (res) {
		pr_debug(DRV_NAME ":%s() copy_from_user() failure.\n",
			__func__);
		return -EINVAL;
	}

	/* Print the data */
	print_buff(data_address, count);

	pr_debug(DRV_NAME ":%s() -----Start Transfer-----\n", __func__);

	if (count > (RX_BUFF_SIZE))
		pr_err(DRV_NAME ":%s() -----PROBLEM-----\n", __func__);

	/* Transmit */
	res = sps_transfer_one(channel_dev->ep.sps, data_phys_addr, count,
			       channel_dev->ep.sps, tx_xfer_flags);

	pr_debug(DRV_NAME ":%s() sps_transfer_one tx res = %d.\n"
			, __func__, res);
	if (res) {
		return -EINVAL;
	} else {
		buf_index = (buf_index + 1) % TX_NUM_BUFFS;
		return count;
	}
}

static ssize_t channel_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *f_pos)
{
	struct channel_dev *channel_dev = filp->private_data;
	int res = 0;
	struct sps_iovec tmp_io_vec;
	u32 offset = 0;

	pr_debug(DRV_NAME ":%s() size to read = %zu\n", __func__, count);

	res = wait_xfer_completion(false, &channel_dev->ep.xfer_done,
				   channel_dev->ep.sps, 1, 10, 5);
	if (res) {
		pr_err(DRV_NAME ":%s() transfer not completed.\n", __func__);
		return 0;
	}

	/* Get the size of the data and release the
	 * descriptor of the Rx pipe */
	res = sps_get_iovec(channel_dev->ep.sps, &channel_dev->ep.iovec);
	if (res) {
		pr_err(DRV_NAME ":%s() .sps_get_iovec() failed.\n", __func__);
		return -EBUSY;
	}

	pr_debug(DRV_NAME ":%s() sps_get_iovec() received %d bytes from 0x%8x.\n",
		__func__, channel_dev->ep.iovec.size,
		channel_dev->ep.iovec.addr);

	/* In case 0 bytes were received, bail out */
	if (0 == channel_dev->ep.iovec.size)
		return 0;

	/* Release the descriptor of the Tx pipe */
	res = sps_get_iovec(ipa_test->tx_channels[0]->ep.sps, &tmp_io_vec);
	if (res) {
		pr_err(DRV_NAME ":%s() .sps_get_iovec() failed.\n", __func__);
		return -EBUSY;
	}

	/* Copy the received data to the user buffer */
	offset = channel_dev->ep.iovec.addr - channel_dev->mem.phys_base;
	res = copy_to_user(buf,
						channel_dev->mem.base + offset,
						channel_dev->ep.iovec.size);
	if (res < 0) {
		pr_err(DRV_NAME ":%s() copy_to_user() failed.\n", __func__);
		return -EFAULT;
	}

	/* Re-insert the descriptor back to pipe */
	res = sps_transfer_one(channel_dev->ep.sps, channel_dev->ep.iovec.addr,
						   RX_BUFF_SIZE, 0, 0);
	pr_debug(DRV_NAME ":%s() sps_transfer_one rx res = %d.\n", __func__,
			res);

	pr_debug(DRV_NAME ":%s() Returning %d.\n", __func__,
		channel_dev->ep.iovec.size);

	return channel_dev->ep.iovec.size;
}

static const struct file_operations channel_dev_fops = {
	.owner = THIS_MODULE,
	.open = channel_open,
	.write = channel_write,
	.read = channel_read,
};

static ssize_t set_skb_for_user(struct file *filp, char __user *buf,
		       size_t size, loff_t *p_pos);

static ssize_t get_skb_from_user(struct file *filp, const char __user *buf,
		       size_t size, loff_t *f_pos);

static const struct file_operations data_path_fops = {
	.owner = THIS_MODULE,
	.open = channel_open,
	.read =	set_skb_for_user,
	.write = get_skb_from_user,
};


/*
 * This will create the char device named
 * "<dev_name>_<index>" and allocate data
 * FIFO(size mem_size) and descriptor
 * FIFO(size DESC_FIFO_SZ) for it.
 * In this case, we will differentiate
 * channel_dev_fops, which are used for
 * regular data transmission
 * in all the tests,and data_path_fops
 * which will be used
 * in DataPath tests for handling
 * the SKB we will transfer
 */
int create_channel_device_by_type(
			const int index,
			const char *dev_name,
			struct channel_dev **channel_dev_ptr,
			size_t mem_size,
			enum fops_type type)
{
	int ret;
	char name[MAX_CHANNEL_NAME];
	struct channel_dev *channel_dev = NULL;

	scnprintf(name, sizeof(name), "%s_%d", dev_name, index);

	pr_debug(DRV_NAME
		":Creating channel %d device, name = %s.\n"
		, index, name);

	/* Allocate memory for the device */
	*channel_dev_ptr = kzalloc(sizeof(struct channel_dev), GFP_KERNEL);
	if (NULL == *channel_dev_ptr) {
		pr_err(DRV_NAME ":kzalloc err.\n");
		ret = -ENOMEM;
		goto create_channel_device_failure;
	}

	channel_dev = *channel_dev_ptr;

	channel_dev->class = class_create(THIS_MODULE, name);
	if (NULL == channel_dev->class) {
		pr_err(DRV_NAME ":class_create() err.\n");
		ret = -ENOMEM;
		goto create_channel_device_failure;
	}

	ret = alloc_chrdev_region(&channel_dev->dev_num, 0, 1, name);
	if (ret) {
		pr_err(DRV_NAME ":alloc_chrdev_region err.\n");
		ret = -ENOMEM;
		goto create_channel_device_failure;
	}

	channel_dev->dev = device_create(channel_dev->class, NULL,
					 channel_dev->dev_num, channel_dev,
					 name);
	if (IS_ERR(channel_dev->dev)) {
		pr_err(DRV_NAME ":device_create err.\n");
		ret = -ENODEV;
		goto create_channel_device_failure;
	}

	switch (type) {
	case IPA_TEST_REG_CHANNEL:
		cdev_init(&channel_dev->cdev, &channel_dev_fops);
		break;
	case IPA_TEST_DATA_PATH_TEST_CHANNEL:
		cdev_init(&channel_dev->cdev, &data_path_fops);
		break;
	default:
		pr_debug("Wrong fops type in function %s", __func__);
		ret = -EINVAL;
		goto create_channel_device_failure;
	}
	channel_dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&channel_dev->cdev, channel_dev->dev_num, 1);
	if (ret) {
		pr_err(DRV_NAME ":cdev_add err=%d\n", -ret);
		ret = -ENODEV;
		goto create_channel_device_failure;
	}

	strlcpy(channel_dev->name, name, MAX_CHANNEL_NAME);

	/* Allocate desc FIFO of the pipe for the S2B/B2S pipes.*/
	pr_debug(DRV_NAME
		":-----Allocate desc FIFO for System2Bam/Bam2System-----\n");
	channel_dev->desc_fifo.size = DESC_FIFO_SZ;
	test_alloc_mem(&channel_dev->desc_fifo);
	if (NULL == channel_dev->desc_fifo.base) {
		pr_err(DRV_NAME "desc fifo alloc fail\n");
		ret = -ENODEV;
		goto create_channel_device_failure;
	}
	pr_debug(DRV_NAME ":mem phys=0x%pa.virt=0x%p.\n",
			&channel_dev->desc_fifo.phys_base,
			channel_dev->desc_fifo.base);
	/*To be on the safe side the descriptor fifo is cleared*/
	memset(
		channel_dev->desc_fifo.base,
		0x00, channel_dev->desc_fifo.size);

	/* Allocate memory data buffer for the pipe(S2B/B2S)*/
	pr_debug(DRV_NAME ":-----Allocate memory buffer-----\n");
	channel_dev->mem.size = mem_size;
	test_alloc_mem(&channel_dev->mem);
	if (NULL == channel_dev->mem.base) {
		pr_err(DRV_NAME "data fifo alloc fail\n");
		ret = -ENODEV;
		goto create_channel_device_failure;
	}
	pr_debug(DRV_NAME ":mem phys=0x%pa.virt=0x%p.\n",
		&channel_dev->mem.phys_base, channel_dev->mem.base);
	memset(channel_dev->mem.base, 0xbb, channel_dev->mem.size);

	/* Add a pointer from the channel device to the test context info */
	channel_dev->test = ipa_test;
	/*TODO(By Talel):is this pointer needed? the test context is shared*/

	if (0 == ret)
		pr_debug(DRV_NAME
			":Channel device:%d, name:%s created, address:0x%p.\n",
			index, name, channel_dev);

	return ret;

create_channel_device_failure:
	kfree(channel_dev);
	pr_debug(DRV_NAME
			":Channel device %d, name %s creation FAILED.\n"
			, index, name);

	return ret;
}

int create_channel_device(const int index,
			  const char *dev_name,
			  struct channel_dev **channel_dev_ptr,
			  size_t mem_size) {
	return create_channel_device_by_type(
			index,
			dev_name,
			channel_dev_ptr,
			mem_size,
			IPA_TEST_REG_CHANNEL);
}

/*
 * DataPath test definitions:
 */

#define MAX_TEST_SKB 5

#define TIME_OUT_TIME 600

struct datapath_ctx {
	struct mutex lock;
	struct kfifo_rec_ptr_2 fifo;
	struct completion write_done_completion;
	struct completion ipa_receive_completion;
};

struct datapath_ctx *p_data_path_ctx;


/*
 * Inits the kfifo needed for the    *
 * DataPath tests				     *
 */
int datapath_ds_init(void)
{
	int res = 0;

	p_data_path_ctx = kzalloc(sizeof(struct datapath_ctx), GFP_KERNEL);
	if (!p_data_path_ctx) {
		pr_err("[%s:%d, %s()]kzalloc returned error (%d)\n"
				, __FILE__, __LINE__
				, __func__, res);
		return res;
	}
	pr_debug("[%s:%d, %s()] called.\n",
		__FILE__, __LINE__, __func__);

	res = kfifo_alloc(&p_data_path_ctx->fifo
			, (sizeof(struct sk_buff *)*MAX_TEST_SKB)
			, GFP_KERNEL);
	if (0 != res) {
		pr_err("[%s:%d, %s()]kfifo_alloc returned error (%d)\n",
			__FILE__, __LINE__,
			__func__, res);
		kfree(p_data_path_ctx);
		return res;
	}

	mutex_init(&p_data_path_ctx->lock);
	init_completion(&p_data_path_ctx->ipa_receive_completion);

	pr_debug("[%s:%d, %s()] completed.(%d)\n",
		__FILE__, __LINE__, __func__, res);

	return res;
}

static struct sk_buff *datapath_create_skb(const char *buf, size_t size)
{
	struct sk_buff *skb;
	unsigned char *data;
	int err = 0;

	pr_debug(DRV_NAME ":%s():Entering\n", __func__);
	pr_debug("allocating SKB, len=%zu", size);
	skb = alloc_skb(size, GFP_KERNEL);
	if (unlikely(!skb))
		return NULL;
	pr_debug("skb allocated, skb->len=%d", skb->len);
	pr_debug("putting skb");
	data = skb_put(skb, size);
	if (unlikely(!data)) {
		kfree_skb(skb);
		return NULL;
	}
	pr_debug("skb put finish, skb->len=%d", skb->len);
	pr_debug("copying data\n");
	skb->csum = csum_and_copy_from_user(
			buf, data,
			size, 0, &err);
	if (err) {
		kfree_skb(skb);
		return NULL;
	}
	pr_debug("The following packet was created:\n");
	print_buff(skb->data, size);
	pr_debug(DRV_NAME ":%s():Exiting\n", __func__);

	return skb;
}

static int datapath_read_data(void *element, int size)
{
	int res;

	pr_debug(DRV_NAME ":%s():Entering\n", __func__);

	INIT_COMPLETION(p_data_path_ctx->ipa_receive_completion);
	pr_debug(DRV_NAME ":%s() Init completion\n", __func__);
	mutex_lock(&p_data_path_ctx->lock);
	pr_debug(DRV_NAME ":%s() Checking if kfifo is empty\n", __func__);
	if (kfifo_is_empty(&p_data_path_ctx->fifo)) {
		pr_debug(DRV_NAME ":%s() kfifo is empty\n", __func__);
		mutex_unlock(&p_data_path_ctx->lock);
		pr_debug(DRV_NAME ":%s() wait_for_ipa_receive_completion\n"
			, __func__);
		res = wait_for_completion_timeout(
			&p_data_path_ctx->ipa_receive_completion,
			TIME_OUT_TIME);
		pr_debug(DRV_NAME
			":%s() came back from wait_for_completion_timeout\n"
			, __func__);
		if (!res) {
			pr_err(DRV_NAME
			":%s() Error in wait_for_ipa_receive_completion\n"
			, __func__);
			return -EINVAL;
		}
		pr_debug(DRV_NAME ":%s() locking lock\n", __func__);
		mutex_lock(&p_data_path_ctx->lock);
	}
	res = kfifo_out(&p_data_path_ctx->fifo, element, size);
	if (res != size) {
		pr_debug(DRV_NAME
			":%s() Error in taking out an element\n",
			__func__);
		pr_debug(DRV_NAME ":%s() unlocking lock\n", __func__);
		mutex_unlock(&p_data_path_ctx->lock);
		return -EINVAL;
	}
	pr_debug(DRV_NAME
		":%s() took %d bytes out\n" , __func__, res);
	pr_debug(DRV_NAME ":%s() unlocking lock\n", __func__);
	mutex_unlock(&p_data_path_ctx->lock);
	pr_debug(DRV_NAME ":%s():Exiting\n", __func__);
	return res;
}

static int datapath_write_fifo(
		void *element,
		int size) {

	int res;

	pr_debug(DRV_NAME ":%s():Entering\n", __func__);
	mutex_lock(&p_data_path_ctx->lock);
	pr_debug(DRV_NAME ":%s() Mutex locked\n", __func__);
	pr_debug(DRV_NAME ":%s() putting %p into fifo\n", __func__, element);
	res = kfifo_in(&p_data_path_ctx->fifo, &element, size);
	pr_debug(DRV_NAME ":%s() finished kfifo in\n", __func__);
	mutex_unlock(&p_data_path_ctx->lock);
	if (res != size) {
		pr_err(DRV_NAME ":%s() Error in saving element\n", __func__);
		return -EINVAL;
	}
	pr_debug(DRV_NAME ":%s():Mutex unlocked\n", __func__);

	complete(&p_data_path_ctx->ipa_receive_completion);
	pr_debug(DRV_NAME ":%s():Completed ipa_receive_completion\n", __func__);
	return 0;
}

/*
 * Receives from the user space the buff,
 * create an SKB, and send it through
 * ipa_tx_dp that was received in the system
 */
static ssize_t get_skb_from_user(struct file *filp, const char __user *buf,
		       size_t size, loff_t *f_pos) {

	int res = 0;
	struct sk_buff *skb;

	pr_debug(DRV_NAME ":%s() Entering\n", __func__);
	/* Copy the data from the user and transmit */
	pr_debug(DRV_NAME
		":%s() -----Copy the data from the user-----\n", __func__);

	/* Print the data */
	pr_debug(DRV_NAME ":%s() Printing buff\n", __func__);
	print_buff((void *)buf, size);

	pr_debug(DRV_NAME ":%s() Creating SKB\n", __func__);

	skb = datapath_create_skb(buf, size);
	if (!skb)
		return -EINVAL;
	init_completion(&p_data_path_ctx->write_done_completion);
	pr_debug(DRV_NAME
		":%s() Starting transfer through ipa_tx_dp\n", __func__);
	res = ipa_tx_dp(IPA_CLIENT_TEST_CONS, skb,
			       NULL);
	pr_debug(DRV_NAME ":%s() ipa_tx_dp res = %d.\n"
			, __func__, res);
	res = wait_for_completion_timeout(
			&p_data_path_ctx->write_done_completion,
			TIME_OUT_TIME);
	pr_debug(DRV_NAME ":%s() timeout result = %d"
			, __func__, res);
	if (!res)
		return -EINVAL;
	pr_debug(DRV_NAME ":%s() -----Exiting-----\n", __func__);

	return size;
}

/*
 * Sends the user space the next SKB
 * that was received in the system
 */

static ssize_t set_skb_for_user(struct file *filp, char __user *buf,
		       size_t size, loff_t *p_pos)
{
	int res;
	struct sk_buff *p_skb;

	pr_debug(DRV_NAME ":%s() Entering\n", __func__);
	/* Copy the result to the user buffer */

	pr_debug(DRV_NAME ":%s() datapath_read_data\n", __func__);
	if (datapath_read_data(
			(void *)&p_skb,
			sizeof(struct sk_buff *)) < 0) {
		pr_err(DRV_NAME
			":%s() error in datapath_read_data\n", __func__);
		return -EINVAL;
	}
	print_buff(p_skb->data, size);
	pr_debug(DRV_NAME ":%s() Copying data back to user\n", __func__);
	res = copy_to_user(buf, p_skb->data, size);
	kfree_skb(p_skb);
	/* Return the number of bytes copied to the user */

	return res;
}

static void datapath_ds_clean(void)
{
	kfifo_reset(&p_data_path_ctx->fifo);
}

/*
 * Destroy the kfifo needed for the
 * DataPath tests
 */

static void datapath_exit(void)
{
	kfifo_free(&p_data_path_ctx->fifo);
	/* freeing kfifo */
	kfree(p_data_path_ctx);
	p_data_path_ctx = NULL;
}

/*
 * CB func. for the IPA_WRITE_DONE
 * event. Used in IpaTxDpTest
 */

static void notify_ipa_write_done(
		void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data) {

	pr_debug("Entering %s function\n", __func__);

	if (evt == IPA_WRITE_DONE) {
		pr_debug("evt IPA_WRITE_DONE in function %s\n", __func__);
		pr_debug("Printing received buff from IPA\n");
		print_buff(
			((struct sk_buff *)data)->data,
			((struct sk_buff *)data)->len);

		kfree_skb((struct sk_buff *)data);
		complete(&p_data_path_ctx->write_done_completion);

	} else {
		pr_debug("Error in %s, wrong event %d", __func__, evt);
	}
}

/*
 * CB func. for the IPA_RECEIVE
 * event. Used in IPAToAppsTest
 */

static void notify_ipa_received(
		void *priv,
		enum ipa_dp_evt_type evt,
		unsigned long data){

	struct sk_buff *p_skb = (struct sk_buff *)data;

	pr_debug("Entering %s function\n", __func__);

	if (evt == IPA_RECEIVE) {
		pr_debug("evt IPA_RECEIVE in function %s\n", __func__);
		pr_debug("Printing received buff from IPA\n");
		print_buff(p_skb->data, p_skb->len);
		datapath_write_fifo(p_skb, sizeof(struct sk_buff *));
	} else {
		pr_err("Error in %s, wrong event %d", __func__, evt);
	}
}

int connect_bamdma_to_apps(struct test_endpoint_sys *rx_ep,
			   u32 pipe_index,
			   struct sps_mem_buffer *desc_fifo,
			   struct sps_register_event *rx_event)
{
	int res = 0;
	u32 rx_connect_options = 0;

	rx_ep->sps = sps_alloc_endpoint();
	pr_debug(DRV_NAME ":%s():rx_ep->sps = %p\n", __func__, rx_ep->sps);

	res = sps_get_config(rx_ep->sps, &rx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to get config.\n");
		return -EINVAL;
	}

	rx_connect_options =
		SPS_O_AUTO_ENABLE | SPS_O_EOT |
		SPS_O_ACK_TRANSFERS | SPS_O_POLL;

	/*The device which will send data - dma_bam_hdl*/
	rx_ep->connect.source = dma_bam_hdl;
	rx_ep->connect.destination = SPS_DEV_HANDLE_MEM;  /* Memory */
	rx_ep->connect.mode = SPS_MODE_SRC; /* Producer pipe */
	rx_ep->connect.src_pipe_index = pipe_index;
	rx_ep->connect.options = rx_connect_options;
	rx_ep->connect.desc = *desc_fifo;
	/*Connect BAM-DMA to A5*/
	res = sps_connect(rx_ep->sps, &rx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to connect.\n");
		return -EINVAL;
	}
	pr_debug(DRV_NAME ":-----Register event for RX pipe-----\n");
	rx_event->mode = SPS_TRIGGER_WAIT;
	rx_event->options = SPS_O_EOT;
	/*The completion object will be notify for EOT.*/
	init_completion(&rx_ep->xfer_done);
	rx_event->xfer_done = &rx_ep->xfer_done;
	pr_debug(DRV_NAME ":rx_event.event=0x%p.\n",
			rx_event->xfer_done);
	/*Register our completion object to
	 * know when the connection was made*/
	res = sps_register_event(rx_ep->sps, rx_event);
	if (res) {
		pr_err(DRV_NAME ":fail to register event.\n");
		return -EINVAL;
	}
	return res;
}

int connect_ipa_to_apps(struct test_endpoint_sys *rx_ep,
			   u32 pipe_index,
			   struct sps_mem_buffer *desc_fifo,
			   struct sps_register_event *rx_event,
			   unsigned long ipa_bam_hdl)
{
	int res = 0;
	u32 rx_connect_options = 0;

	rx_ep->sps = sps_alloc_endpoint();
	pr_debug(DRV_NAME ":%s():rx_ep->sps = %p\n", __func__, rx_ep->sps);
	res = sps_get_config(rx_ep->sps, &rx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to get config.\n");
		return -EINVAL;
	}
	rx_connect_options =
		SPS_O_AUTO_ENABLE | SPS_O_EOT |
		SPS_O_ACK_TRANSFERS | SPS_O_POLL;
	rx_ep->connect.source = ipa_bam_hdl;
	rx_ep->connect.destination = SPS_DEV_HANDLE_MEM;  /* Memory */
	rx_ep->connect.mode = SPS_MODE_SRC; /* Producer pipe */
	rx_ep->connect.src_pipe_index = pipe_index;
	rx_ep->connect.options = rx_connect_options;
	rx_ep->connect.desc = *desc_fifo;
	res = sps_connect(rx_ep->sps, &rx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to connect.\n");
		return -EINVAL;
	}
	pr_debug(DRV_NAME ":-----Register event for RX pipe-----\n");
	rx_event->mode = SPS_TRIGGER_WAIT;
	rx_event->options = SPS_O_EOT;
	init_completion(&rx_ep->xfer_done);
	rx_event->xfer_done = &rx_ep->xfer_done;
	pr_debug(DRV_NAME ":rx_event.event=0x%p.\n", rx_event->xfer_done);
	res = sps_register_event(rx_ep->sps, rx_event);
	if (res) {
		pr_err(DRV_NAME ":fail to register event.\n");
		return -EINVAL;
	}
	return res;
}

int connect_apps_to_bamdma(struct test_endpoint_sys *tx_ep,
			   u32 pipe_index,
			   struct sps_mem_buffer *desc_fifo)
{
	u32 tx_connect_options = 0;
	int res = 0;

	tx_ep->sps = sps_alloc_endpoint();
	res = sps_get_config(tx_ep->sps, &tx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to get config.\n");
		return -EINVAL;
	}
	tx_connect_options =
		SPS_O_AUTO_ENABLE | SPS_O_EOT
		| SPS_O_ACK_TRANSFERS | SPS_O_POLL;
	tx_ep->connect.source = SPS_DEV_HANDLE_MEM; /* Memory */
	tx_ep->connect.destination = dma_bam_hdl;
	tx_ep->connect.mode = SPS_MODE_DEST;  /* Consumer pipe */
	tx_ep->connect.dest_pipe_index = pipe_index;
	tx_ep->connect.options = tx_connect_options;
	tx_ep->connect.desc = *desc_fifo;
	res = sps_connect(tx_ep->sps, &tx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ": fail to connect.\n");
		return -EINVAL;
	}
	return 0;
}

int connect_apps_to_ipa(struct test_endpoint_sys *tx_ep,
			   u32 pipe_index,
			   struct sps_mem_buffer *desc_fifo,
			   unsigned long ipa_bam_hdl)
{
	u32 tx_connect_options = 0;
	int res = 0;

	tx_ep->sps = sps_alloc_endpoint();
	res = sps_get_config(tx_ep->sps, &tx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ":fail to get config.\n");
		return -EINVAL;
	}
	tx_connect_options =
		SPS_O_AUTO_ENABLE | SPS_O_EOT |
		SPS_O_ACK_TRANSFERS | SPS_O_POLL;
	tx_ep->connect.source = SPS_DEV_HANDLE_MEM; /* Memory */
	tx_ep->connect.destination = ipa_bam_hdl;
	tx_ep->connect.mode = SPS_MODE_DEST;  /* Consumer pipe */
	tx_ep->connect.dest_pipe_index = pipe_index;
	tx_ep->connect.options = tx_connect_options;
	tx_ep->connect.desc = *desc_fifo;
	res = sps_connect(tx_ep->sps, &tx_ep->connect);
	if (res) {
		pr_err(DRV_NAME ": fail to connect.\n");
		return -EINVAL;
	}
	return 0;
}

int connect_ipa_to_bamdma(struct test_endpoint_dma *rx_dma_bam_ep,
			  enum ipa_client_type ipa_client,
			  struct ipa_ep_cfg *ipa_ep_cfg_ptr,
			  u32 client_ep_idx,
			  void (*notify_cb_func)(void *priv,
			  enum ipa_dp_evt_type evt, unsigned long data),
			  void *notify_cb_data)
{
	struct ipa_connect_params dst_in;
	struct ipa_sps_params dst_out;
	u32 dst_clnt_hdl;
	int res = 0;
	char pipe_name[30] = {0};

	memset(&dst_in, 0, sizeof(dst_in));
	memset(&dst_out, 0, sizeof(dst_out));
	/* IPA EP configuration */
	if (NULL != ipa_ep_cfg_ptr) {
		pr_debug(DRV_NAME
				": (NULL != ipa_ep_cfg) Executing memcpy\n");
		memcpy(&(dst_in.ipa_ep_cfg),
				ipa_ep_cfg_ptr, sizeof(struct ipa_ep_cfg));
		pr_debug(DRV_NAME
				": dst_in.ipa_ep_cfg.hdr.hdr_len = %d\n"
				, dst_in.ipa_ep_cfg.hdr.hdr_len);
	}
	dst_in.client = ipa_client;  /* this should be a CONS of some type */
	dst_in.client_bam_hdl = dma_bam_hdl;
	dst_in.client_ep_idx = client_ep_idx;
	dst_in.desc_fifo_sz = DESC_FIFO_SZ;
	dst_in.data_fifo_sz = DATA_FIFO_SZ;
	dst_in.notify = notify_cb_func; /* Installing CB Function */
	dst_in.priv = notify_cb_data;/* Installing CB Data */
	switch (ipa_client) {
	case IPA_CLIENT_TEST4_CONS:
		strlcpy(pipe_name,
				"IPA_CLIENT_TEST4_CONS",
				30);
		break;
	case IPA_CLIENT_TEST2_CONS:
		strlcpy(pipe_name,
				"IPA_CLIENT_TEST2_CONS",
				30);
		break;
	case IPA_CLIENT_TEST3_CONS:
		strlcpy(pipe_name,
				"IPA_CLIENT_TEST3_CONS",
				30);
		break;
	default:
		strlcpy(pipe_name,
				"UNKNOWN",
				30);
		break;
	}
	/*This will allocate the required desc/data buffers */
	if (ipa_connect(&dst_in, &dst_out, &dst_clnt_hdl) == 0) {
		/* use the IPA returned information to do sps_connect */
		rx_dma_bam_ep->clnt_hdl = dst_clnt_hdl;
		pr_debug(DRV_NAME
				":----clnt type %d clnt hdl %d Pipe %s "
				"connected to IPA EP %d----\n",
			dst_in.client, dst_clnt_hdl
			, pipe_name, dst_out.ipa_ep_idx);
		/* get sps configurations for the DMA
		 * endpoint - Default Config */
		rx_dma_bam_ep->sps = sps_alloc_endpoint();
		res = sps_get_config(rx_dma_bam_ep->sps
				, &rx_dma_bam_ep->connect);
		if (res) {
			pr_debug(DRV_NAME ": fail to get config.\n");
			return -EINVAL;
		}
		/* Specific Config */
		rx_dma_bam_ep->connect.mode = SPS_MODE_DEST;
		/* IPA BAM is producer */
		rx_dma_bam_ep->connect.source = dst_out.ipa_bam_hdl;
		rx_dma_bam_ep->connect.destination = dma_bam_hdl;
		rx_dma_bam_ep->connect.desc = dst_out.desc;
		rx_dma_bam_ep->connect.data = dst_out.data;
		rx_dma_bam_ep->connect.event_thresh = 0x10;
		rx_dma_bam_ep->connect.src_pipe_index = dst_out.ipa_ep_idx;
		rx_dma_bam_ep->connect.dest_pipe_index =
				rx_dma_bam_ep->chan.src_pipe_index;
		rx_dma_bam_ep->connect.options = SPS_O_AUTO_ENABLE;
		/* BAM-to-BAM */
		pr_debug(DRV_NAME
				":----Connect RX DMA BAM Endpoint----\n");
		res = sps_connect(rx_dma_bam_ep->sps, &rx_dma_bam_ep->connect);
		if (res) {
			pr_debug(DRV_NAME ":sps_connect fails.\n");
			return -EINVAL;
		}
	} else {
		/* error handling */
		pr_err(DRV_NAME ":IPA client register failed RX DMA BAM\n");
		return -EINVAL;
	}
	return 0;
}

int connect_bamdma_to_ipa(struct test_endpoint_dma *tx_dma_bam_ep,
			  struct ipa_ep_cfg *ipa_ep_cfg,
			  enum ipa_client_type ipa_client,
			  void (*notify_cb_func)(void *priv,
			  enum ipa_dp_evt_type evt, unsigned long data),
			  void *notify_cb_data)
{
	struct ipa_connect_params src_in;
	struct ipa_sps_params src_out;
	u32 src_clnt_hdl;
	int res = 0;

	memset(&src_in, 0, sizeof(src_in));
	memset(&src_out, 0, sizeof(src_out));

	/* IPA EP configuration */
	src_in.ipa_ep_cfg = *ipa_ep_cfg;
	src_in.client = ipa_client;  /* this should be a PROD of some type */
	src_in.client_bam_hdl = dma_bam_hdl;
	src_in.client_ep_idx = tx_dma_bam_ep->chan.dest_pipe_index;
	src_in.desc_fifo_sz = DESC_FIFO_SZ;
	src_in.data_fifo_sz = DATA_FIFO_SZ;
	src_in.notify = notify_cb_func; /* Installing CB Function */
	src_in.priv = notify_cb_data;/* Installing CB Data */
	if (ipa_connect(&src_in, &src_out, &src_clnt_hdl) == 0) {
		/* use the IPA returned information to do sps_connect*/
		tx_dma_bam_ep->clnt_hdl = src_clnt_hdl;
		pr_debug(DRV_NAME
				":----clnt type %d clnt hdl %d IPA EP %d----\n",
				src_in.client, src_clnt_hdl,
				src_out.ipa_ep_idx);
		/* Default Config */
		tx_dma_bam_ep->sps = sps_alloc_endpoint();
		res = sps_get_config
				(tx_dma_bam_ep->sps, &tx_dma_bam_ep->connect);
		if (res) {
			pr_debug(DRV_NAME ": fail to get config.\n");
			return -EINVAL;
		}
		/* Specific Config */
		tx_dma_bam_ep->connect.mode = SPS_MODE_SRC;
		/* TX DMA BAM is producer */
		tx_dma_bam_ep->connect.source = dma_bam_hdl;
		tx_dma_bam_ep->connect.destination = src_out.ipa_bam_hdl;
		tx_dma_bam_ep->connect.desc = src_out.desc;
		tx_dma_bam_ep->connect.data = src_out.data;
		tx_dma_bam_ep->connect.event_thresh = 0x10;
		tx_dma_bam_ep->connect.src_pipe_index =
				tx_dma_bam_ep->chan.dest_pipe_index;
		tx_dma_bam_ep->connect.dest_pipe_index = src_out.ipa_ep_idx;
		tx_dma_bam_ep->connect.options =
				SPS_O_AUTO_ENABLE; /* BAM-to-BAM */
		pr_debug(DRV_NAME ":----Connect TX DMA BAM Endpoint----\n");
		res = sps_connect(tx_dma_bam_ep->sps, &tx_dma_bam_ep->connect);
		if (res) {
			pr_debug(DRV_NAME ":sps_connect fails.\n");
			return -EINVAL;
		}
	} else {
		/* error handling */
		pr_err(DRV_NAME ":IPA client register failed TX DMA BAM\n");
		return -EINVAL;
	}
	return 0;
}

int configure_ipa_endpoint(struct ipa_ep_cfg *ipa_ep_cfg,
			   enum ipa_mode_type mode)
{
	const char *DEFAULT_ROUTING_TABLE_NAME = "LAN";
	const enum ipa_client_type DEFAULT_CLIENT = IPA_CLIENT_TEST4_CONS;
	struct ipa_ioc_add_rt_rule *rt_rule;
	struct ipa_rt_rule_add *rt_rule_entry;
	struct ipa_ioc_get_rt_tbl rt_lookup;

	memset(ipa_ep_cfg, 0, sizeof(*ipa_ep_cfg));
	/* Configure mode */
	ipa_ep_cfg->mode.mode = mode;
	/* Add default routing rule */
	rt_rule = kzalloc(sizeof(struct ipa_ioc_add_rt_rule) +
			1 * sizeof(struct ipa_rt_rule_add), GFP_KERNEL);
	if (!rt_rule) {
		pr_err(DRV_NAME
				":%s():%d:Allocation failure.\n"
				, __func__, __LINE__);
		return -EINVAL;
	}
	rt_rule->commit = 1;
	rt_rule->num_rules = 1;
	rt_rule->ip = IPA_IP_v4;
	strlcpy(rt_rule->rt_tbl_name,
			DEFAULT_ROUTING_TABLE_NAME,
			IPA_RESOURCE_NAME_MAX);
	rt_rule_entry = &rt_rule->rules[0];
	rt_rule_entry->at_rear = 1;
	rt_rule_entry->rule.dst = DEFAULT_CLIENT;
	/* Issuing a routing rule without
	 * any equations will cause a default rule
	   which catches every packet and sends it
	   to the default endpoint. */
	if (ipa_add_rt_rule(rt_rule)) {
		pr_err(DRV_NAME ":%s():%d:ipa_add_rt_rule() failure.\n"
				, __func__, __LINE__);
	}
	pr_debug(DRV_NAME ":%s():%d:rt rule hdl1=%x.\n", __func__, __LINE__,
		rt_rule_entry->rt_rule_hdl);
	/*	At this point, there is a routing table in memory,
		with one defalut rule.
		user space test application will enter more valid
		rules the default rule
		must be last on the list.
		Get a handle to the routing table which holds
		the added rule. This will
		also increment an internal reference count. */
	memset(&rt_lookup, 0, sizeof(struct ipa_ioc_get_rt_tbl));
	rt_lookup.ip = IPA_IP_v4;
	strlcpy(rt_lookup.name,
			DEFAULT_ROUTING_TABLE_NAME,
			IPA_RESOURCE_NAME_MAX);
	if (ipa_get_rt_tbl(&rt_lookup)) {
		pr_err(DRV_NAME ":%s():%d:ipa_get_rt_tbl() failure.\n"
				, __func__, __LINE__);
	}
	ipa_ep_cfg->route.rt_tbl_hdl = rt_lookup.hdl;
	/* Now release the routing table hdl. This code assumes
	   that routing table
	   will continue to exist when the endpoint
	   connection is requested. */
	if (ipa_put_rt_tbl(rt_lookup.hdl)) {
		pr_err(DRV_NAME ":%s():%d:ipa_put_rt_tbl() failure.\n"
				, __func__, __LINE__);
	}
	return 0;
}

int configure_system_0(void)
{
	int res = 0;

	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;

	/* Configure Rx endpoint and connection */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
				to_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
				&from_ipa_devs[0]->desc_fifo,
				&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Configure Tx endpoint and connection */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
				to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
				&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

fail:
	/* cleanup and tear down goes here */
	return res;
}

/*
 *  Configures the system as follows:
 * This configuration is for one input pipe
 * and one output pipe where both are USB1
 * /dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA -> BAM-DMA
 * -> MEM -> /dev/from_ipa_0
 * Those client will be configured in DMA mode thus
 * no Header removal/insertion will be
 * made on their data.
*/
int configure_system_1(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	pr_debug(DRV_NAME "Configure_system_1 was called\n");

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;
	from_ipa_devs[0]->dma_ep.chan.src_pipe_index = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;
	/* Connect Rx BAM-DMA --> A5 MEM */
	pr_debug(DRV_NAME "Connecting BAM-DMA to Application CPU\n");
	res = connect_bamdma_to_apps(&(from_ipa_devs[0]->ep),
				from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
				&(from_ipa_devs[0]->desc_fifo),
				&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;
	/* Connect IPA(USB1) -> to BAM-DMA */
	pr_debug(DRV_NAME "Connecting IPA to BAM-DMA\n");
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
				IPA_CLIENT_TEST_CONS,
				NULL,/*ipa_ep_cfg*/
				from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
				NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif
	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;
	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep, /*tx_ep*/
				     to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
				     &to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif
fail:
	/* cleanup and tear down goes here*/
	return res;
}

/*
 Configures the system with one input to IPA and 2 outputs.
 /dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA |-> BAM-DMA
 -> MEM -> /dev/from_ipa_0
|-> BAM-DMA -> MEM -> /dev/from_ipa_1
*/
int configure_system_2(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = 7;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 11;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 12;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 13;
#endif

#ifndef IPA_ON_R3PC
	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
#else
	/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect third (Default) Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
#else
	/* Connect third (Default) Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
			ipa_pipe_num,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event,
			ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect IPA -> first Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect IPA -> third (default) Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

#endif

	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	/* configure header removal on Tx */
	ipa_ep_cfg.hdr.hdr_len = ETH_HLEN;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep,
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);

#endif

	if (res)
		goto fail;

fail:
	/* cleanup and tear down goes here */
	return res;
}

int configure_system_3(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 11;
#endif

#ifndef IPA_ON_R3PC
	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Connect third (Default) Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;
#endif

#ifndef IPA_ON_R3PC
	/* Connect IPA -> first Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
			ipa_pipe_num,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event,
			ipa_bam_hdl);

#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);

#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect IPA -> third (default) Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);

#endif
	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	/* configure header removal on Tx */
	ipa_ep_cfg.hdr.hdr_len
		= IPA_TEST_DMUX_HEADER_LENGTH;
	ipa_ep_cfg.hdr.hdr_ofst_metadata_valid
		= IPA_TEST_META_DATA_IS_VALID;
	ipa_ep_cfg.hdr.hdr_ofst_metadata
			= IPA_TEST_DMUX_HEADER_META_DATA_OFFSET;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif
fail:
	/* cleanup and tear down goes here*/
	return res;
}

/*Configuration used for Header Insertion Tests*/
int configure_system_5(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

	/* configure header Insertion on Tx */

/*	ipa_ep_cfg.hdr.hdr_len  = IPA_TEST_DMUX_HEADER_LENGTH;
 !< Header length in bytes to be added/removed.
	Assuming heaser len is constant per endpoint.
	Valid for both Input and Output Pipes
	Length of Header to add / to remove
	ipa_ep_cfg.hdr.hdr_additional_const_len = 0;
	!< Defines the constant length that should be added
	to the payload length in order for IPA to update
	correctly the length field within the header
	(valid only in case Hdr_Ofst_Pkt_Size_Valid=1)
	Valid for Output Pipes (IPA Producer)
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = 0;
	!< 0: Hdr_Ofst_Pkt_Size  value is invalid, i.e.,
	no length field within the inserted header
	1: Hdr_Ofst_Pkt_Size  value is valid, i.e.,
	a packet length field resides within the header
	Valid for Output Pipes (IPA Producer)
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 0;
	!< Offset within header in which packet size
	reside. Upon Header Insertion, IPA will update this
	field within the header with the packet length .
	Assumption is that header length field size is
	constant and is 2Bytes Valid for Output Pipes (IPA Producer)
	ipa_ep_cfg.hdr.hdr_a5_mux = 0;
	0: Do not insert A5 Mux Header
	1: Insert A5 Mux Header
	!< Determines whether A5 Mux header should be
	added to the packet. This bit is valid only when
	Hdr_En=01(Header Insertion) SW should set this bit
	for IPA-to-A5 pipes.
	0: Do not insert A5 Mux Header
	1: Insert A5 Mux Header
	Valid for Output Pipes (IPA Producer) */


#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 11;

	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Connect third (Default) Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;
#endif

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.hdr.hdr_len =
			IPA_TEST_HDI_RMNET_HEADER_LENGTH;
	/* Length of Header to add / to remove */
	ipa_ep_cfg.hdr.hdr_additional_const_len =
			IPA_TEST_HDI_RMNET_ADD_CONST_LENGTH;
	/* constant length that should be added to the payload
	 * length or IPA to update correctly the length
	 *  field within the header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid
		= IPA_TEST_HDI_RMNET_LENGTH_FIELD_OFFSET_VALID;
	/*0: Hdr_Ofst_Pkt_Size  value is invalid, i.e.,
	  no length field within the inserted header
	  1: Hdr_Ofst_Pkt_Size  value is valid, i.e.,
	  a packet length field resides within the header*/
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size
		= IPA_TEST_HDI_RMNET_LENGTH_FIELD_OFFSET;
	/*	Offset within header in which packet size reside.
		Upon Header Insertion, IPA will update this field
		within the header with the packet length .
		Assumption is that header length field size
		is constant and is 2Bytes */

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			/*Configure Header Insertion as RMnet QoS*/
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.hdr.hdr_len = IPA_TEST_HDI_802_HEADER_LENGTH;
	/*Length of Header to add / to remove*/
	ipa_ep_cfg.hdr.hdr_additional_const_len
		= IPA_TEST_HDI_802_ADD_CONST_LENGTH;
	/* constant length that should be added
	 * to the payload length
	   or IPA to update correctly the
	   length field within the header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid
		= IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET_VALID;
	/*0: Hdr_Ofst_Pkt_Size  value is invalid, i.e.,
	 no length field within the inserted header
	 1: Hdr_Ofst_Pkt_Size  value is valid, i.e.
	  a packet length field resides within the header*/
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size
		= IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET;
	/* Offset within header in which packet size reside.
	   Upon Header Insertion, IPA will update this field
	   within the header with the packet length.
	   Assumption is that header length field size is constant
	   and is 2Bytes */
#ifndef IPA_ON_R3PC
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg, /* Configure Header Insertion as */
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
		/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
				&ipa_pipe_num,
				&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> third (default) Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.hdr.hdr_len
		= IPA_TEST_HDI_802_HEADER_LENGTH;
	/* Length of Header to add / to remove */
	ipa_ep_cfg.hdr.hdr_additional_const_len
		= IPA_TEST_HDI_802_ADD_CONST_LENGTH+1;
	/*  constant length that should be
	 *  added to the payload length
		or IPA to update correctly the length
		field within the header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid
		= IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET_VALID;
	/* 0: Hdr_Ofst_Pkt_Size  value is invalid, i.e.,
	   no length field within the inserted header
	   1: Hdr_Ofst_Pkt_Size  value is valid, i.e.,
	   a packet length field resides within the header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size
		= IPA_TEST_HDI_802_LENGTH_FIELD_OFFSET;
	/* Offset within header in which packet size reside.
	 * Upon Header Insertion, IPA will update this field
	 * within the header with the packet length .
	 * Assumption is that header length field size is constant
	 * and is 2Bytes */
#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			&ipa_ep_cfg, /*Configure Header Insertion as*/
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect third (Default) Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);

#endif
	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
				     to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
				     &to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here */
	return res;
}

/*Configuration Used for USB Integration (on R3PC) */
int configure_system_6(void)
{
	int res = 0;
#ifdef IPA_ON_R3PC
	struct ipa_ep_cfg ipa_ep_cfg;
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;
	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	/* Prepare an endpoint configuration structure */
	ipa_ep_cfg.mode.mode = IPA_BASIC;
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;
	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
fail:
	/* cleanup and tear down goes here */
#endif
	return res;
}


/*
 Configures the system as follows:
 This configuration is for 4 input pipes and 3 output pipes:
 /dev/to_ipa_0 -> MEM -> BAM-DMA ->
  * IPA -> BAM-DMA -> MEM -> /dev/from_ipa_0
 /dev/to_ipa_1 -> MEM -> BAM-DMA ->
  * IPA -> BAM-DMA -> MEM -> /dev/from_ipa_1
 /dev/to_ipa_2 -> MEM -> BAM-DMA ->
  * IPA -> BAM-DMA -> MEM -> /dev/from_ipa_0
 /dev/to_ipa_3 -> MEM -> BAM-DMA ->
  * IPA -> BAM-DMA -> MEM -> /dev/from_ipa_2
 to_ipa_1, to_ipa_2, from_ipa_0 &
 from_ipa_2 transfer TLP aggregated packets
 to_ipa_0, to_ipa_3 & from_ipa_1 transfer raw packets
*/
int configure_system_8(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	pr_debug(DRV_NAME "Configure_system_8 was called\n");

	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 7;
	to_ipa_devs[2]->dma_ep.chan.src_pipe_index = 8;
	to_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 9;
	to_ipa_devs[3]->dma_ep.chan.src_pipe_index = 10;
	to_ipa_devs[3]->dma_ep.chan.dest_pipe_index = 11;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index = 14;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 15;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index = 16;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 17;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index = 18;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 19;
	/* configure aggregation on Tx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_TLP;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[0]->ep),
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[0]->desc_fifo),
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[1]->ep),
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[1]->desc_fifo),
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Prepare EP configuration details - no aggregation*/
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure aggregation on Tx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_TLP;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 30;

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[2]->ep),
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[2]->desc_fifo),
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
			ipa_pipe_num,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event,
			ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST3_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep, /*tx_ep*/
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure deaggregation on Rx */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST3_CONS;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_TLP;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep, /*tx_ep*/
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure deaggregation on Rx */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_TLP;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[2]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[2]->ep, /*tx_ep*/
			to_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[2]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[2]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST2_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[3]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST4_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[3]->ep, /*tx_ep*/
			to_ipa_devs[3]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[3]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[3]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[3]->ep,
			ipa_pipe_num,
			&to_ipa_devs[3]->desc_fifo,
			ipa_bam_hdl);
	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here */
	return res;
}


/*
 Configures the system as follows:
 This configuration is for 4 input pipes and 3 output pipes:
 /dev/to_ipa_0 -> MEM -> BAM-DMA
  * -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_0
 /dev/to_ipa_1 -> MEM -> BAM-DMA
  * -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_1
 /dev/to_ipa_2 -> MEM -> BAM-DMA
  * -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_0
 /dev/to_ipa_3 -> MEM -> BAM-DMA
  * -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_2
 to_ipa_1, to_ipa_2, from_ipa_0 &
 from_ipa_2 transfer MBIM aggregated packets
 to_ipa_0, to_ipa_3 & from_ipa_1 transfer raw packets
*/
int configure_system_9(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_aggr_mode mode;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	mode = IPA_MBIM;
	res = ipa_set_aggr_mode(mode);
	if (res)
		goto fail;
	res = ipa_set_single_ndp_per_mbim(true);
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	pr_debug(DRV_NAME "Configure_system_9 was called\n");

	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 7;
	to_ipa_devs[2]->dma_ep.chan.src_pipe_index = 8;
	to_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 9;
	to_ipa_devs[3]->dma_ep.chan.src_pipe_index = 10;
	to_ipa_devs[3]->dma_ep.chan.dest_pipe_index = 11;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index = 14;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 15;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index = 16;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 17;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index = 18;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 19;

	/* configure aggregation on Tx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[0]->ep),
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[0]->desc_fifo),
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[1]->ep),
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[1]->desc_fifo),
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Prepare EP configuration details - no aggregation*/
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure aggregation on Tx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 30;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[2]->ep),
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[2]->desc_fifo),
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST3_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep, /*tx_ep*/
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure deaggregation on Rx */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST3_CONS;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep, /*tx_ep*/
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* configure deaggregation on Rx */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[2]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[2]->ep, /*tx_ep*/
			to_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[2]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[2]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST2_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[3]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST4_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[3]->ep, /*tx_ep*/
			to_ipa_devs[3]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[3]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[3]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[3]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[3]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here */
	return res;
}

/*
 Configures the system as follows:
 This configuration is for 1 input pipe and 1 output pipe:
 /dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_0
 /dev/to_ipa_1 -> MEM -> BAM-DMA -> IPA -> BAM-DMA -> MEM -> /dev/from_ipa_1
 from_ipa_0, from_ipa_1 transfer IPA_MBIM aggregated packets
 to_ipa_0, to_ipa_1 transfer raw packets
*/
int configure_system_10(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_aggr_mode mode;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	mode = IPA_MBIM;
	res = ipa_set_aggr_mode(mode);
	if (res)
		goto fail;
	res = ipa_set_single_ndp_per_mbim(false);
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	pr_debug(DRV_NAME "Configure_system_10 was called\n");

	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;

	/* configure aggregation on Tx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 0;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&(from_ipa_devs[0]->ep),
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[0]->desc_fifo),
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA's USB -> Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep, /*tx_ep*/
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here */
	return res;
}

int configure_system_11(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_aggr_mode mode;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	mode = IPA_MBIM;
	res = ipa_set_aggr_mode(mode);
	if (res)
		goto fail;
	res = ipa_set_single_ndp_per_mbim(false);
	if (res)
		goto fail;


	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = 7;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 11;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 12;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 13;
	from_ipa_devs[3]->dma_ep.chan.src_pipe_index  = 14;
	from_ipa_devs[3]->dma_ep.chan.dest_pipe_index = 15;

	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[3]->ep,
			from_ipa_devs[3]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[3]->desc_fifo,
			&from_ipa_devs[3]->rx_event);
	if (res)
		goto fail;

#endif

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
		/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 30;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 0;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[3]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[3]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[3]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[3]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[3]->desc_fifo,
				  &from_ipa_devs[3]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep,
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[1]->ipa_client_hdl))
		goto fail;
	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif
fail:
	/* cleanup and tear down goes here */
	return res;
}

int configure_system_12(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
	enum ipa_aggr_mode mode;
	char qcncm_sig[3];

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif
	mode = IPA_QCNCM;
	res = ipa_set_aggr_mode(mode);
	if (res)
		goto fail;
	res = ipa_set_single_ndp_per_mbim(false);
	if (res)
		goto fail;
	qcncm_sig[0] = 0x51;
	qcncm_sig[1] = 0x4e;
	qcncm_sig[2] = 0x44;
	res = ipa_set_qcncm_ndp_sig(qcncm_sig);
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = 7;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 11;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 12;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 13;
	from_ipa_devs[3]->dma_ep.chan.src_pipe_index  = 14;
	from_ipa_devs[3]->dma_ep.chan.dest_pipe_index = 15;

	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[3]->ep,
			from_ipa_devs[3]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[3]->desc_fifo,
			&from_ipa_devs[3]->rx_event);
	if (res)
		goto fail;

#endif

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
		/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 0;
	ipa_ep_cfg.aggr.aggr_time_limit = 30;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;
	ipa_ep_cfg.aggr.aggr_byte_limit = 0;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_len = 1;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[3]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[3]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &from_ipa_devs[3]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[3]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[3]->desc_fifo,
				  &from_ipa_devs[3]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_MBIM_16;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep,
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here */
	return res;
}

int configure_system_17(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = 7;
	to_ipa_devs[2]->dma_ep.chan.src_pipe_index    = 8;
	to_ipa_devs[2]->dma_ep.chan.dest_pipe_index   = 9;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 11;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 12;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 13;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 14;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 15;
	from_ipa_devs[3]->dma_ep.chan.src_pipe_index  = 16;
	from_ipa_devs[3]->dma_ep.chan.dest_pipe_index = 17;

	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
	if (res)
		goto fail;

	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[3]->ep,
			from_ipa_devs[3]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[3]->desc_fifo,
			&from_ipa_devs[3]->rx_event);
	if (res)
		goto fail;

#endif

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = true;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_additional_const_len = 14;
	ipa_ep_cfg.hdr.hdr_len = 58;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(
			&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	/*if (res)
		goto fail;
	res = ipa_cfg_ep_rndis_aggr(from_ipa_devs[0]->dma_ep.clnt_hdl);
	*/
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
				&ipa_bam_hdl,
				&ipa_pipe_num,
				&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	/* if (res)
		goto fail;
	  res = ipa_cfg_ep_rndis_aggr(ipa_pipe_num); */
#endif
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(
			&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
#else
		/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 30;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = true;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_additional_const_len = 14;
	ipa_ep_cfg.hdr.hdr_len = 58;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	/*if (res)
		goto fail;
	res = ipa_cfg_ep_rndis_aggr(from_ipa_devs[2]->dma_ep.clnt_hdl);*/
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(
			&from_ipa_devs[2]->ep,
			ipa_pipe_num,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event,
			ipa_bam_hdl);
	/*if (res)
		goto fail;
	res = ipa_cfg_ep_rndis_aggr(ipa_pipe_num);
	*/
#endif
	if (res)
		goto fail;

	/* Connect IPA -> first Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.aggr.aggr_byte_limit = 0;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.aggr.aggr_pkt_limit = 2;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = true;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_additional_const_len = 14;
	ipa_ep_cfg.hdr.hdr_len = 58;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(
			&from_ipa_devs[3]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[3]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	/*if (res)
		goto fail;
	res = ipa_cfg_ep_rndis_aggr(from_ipa_devs[3]->dma_ep.clnt_hdl);
	*/
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[3]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[3]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[3]->desc_fifo,
				  &from_ipa_devs[3]->rx_event,
				  ipa_bam_hdl);
	/*if (res)
		goto fail;
	res = ipa_cfg_ep_rndis_aggr(ipa_pipe_num);
	*/
#endif
	if (res)
		goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.hdr.hdr_len = 14;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(
			&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST_PROD,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(
			&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST3_PROD,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep,
				     to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
				     &to_ipa_devs[1]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.deaggr.deaggr_hdr_len = 44;
	ipa_ep_cfg.deaggr.packet_offset_valid = true;
	ipa_ep_cfg.deaggr.packet_offset_location = 8;
	ipa_ep_cfg.hdr.hdr_len = 14; /* Ethernet header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_remove_additional = false;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = 1;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = 1;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_payload_len_inc_padding = 0;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(
			&to_ipa_devs[2]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST2_PROD,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(
			&to_ipa_devs[2]->ep,
			to_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[2]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(
			&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&to_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[2]->ep,
			ipa_pipe_num,
			&to_ipa_devs[2]->desc_fifo,
			ipa_bam_hdl);

	if (res)
		goto fail;
#endif
fail:
	/* cleanup and tear down goes here */
	return res;
}


void suspend_handler(enum ipa_irq_type interrupt,
				void *private_data,
				void *interrupt_data)
{
	u32 suspend_data;
	u32 clnt_hdl;
	struct ipa_ep_cfg_ctrl ipa_to_usb_ep_cfg_ctrl;
	int res;

	suspend_data =
		((struct ipa_tx_suspend_irq_data *)interrupt_data)->endpoints;
	clnt_hdl =
		((struct ipa_tx_suspend_private_data *)private_data)->clnt_hdl;

	pr_debug("in suspend handler: interrupt=%d, private_data=%d, interrupt_data=%d\n",
			 interrupt, clnt_hdl, suspend_data);

	pr_debug("Enabling back data path for IPA_CLIENT_USB_CONS\n");
	memset(&ipa_to_usb_ep_cfg_ctrl, 0 , sizeof(struct ipa_ep_cfg_ctrl));
	ipa_to_usb_ep_cfg_ctrl.ipa_ep_suspend = false;

	res = ipa_cfg_ep_ctrl(clnt_hdl, &ipa_to_usb_ep_cfg_ctrl);
	if (res)
		pr_err("failed enabling back data path for IPA_CLIENT_USB_CONS\n");

	pr_debug("remove suspend interrupt handler:\n");
	res = ipa_remove_interrupt_handler(IPA_TX_SUSPEND_IRQ);

	if (res)
		pr_err("remove handler for suspend interrupt failed\n");

	kfree(private_data);
}
/*
 * Configures the system as follows:
 * This configuration is for one input pipe and one output pipe
 * where both are USB1
 * /dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA -> BAM-DMA-> MEM -> /dev/from_ipa_0
 * Those clients will be configured in DMA mode thus no Header removal/insertion
 * will be made on their data.
 * Then disable USB_CONS EP for creating the suspend interrupt and register
 * a handler for it.
*/
int configure_system_19(char **params)
{
	struct ipa_ep_cfg_ctrl ipa_to_usb_ep_cfg_ctrl;
	struct ipa_tx_suspend_private_data *suspend_priv_data = NULL;
	int res;
	s32 defer_work;

	res = configure_system_1();
	if (res) {
		pr_err("configure system (19) failed\n");
		goto fail;
	}

	memset(&ipa_to_usb_ep_cfg_ctrl, 0 , sizeof(struct ipa_ep_cfg_ctrl));
	ipa_to_usb_ep_cfg_ctrl.ipa_ep_suspend = true;

	res = ipa_cfg_ep_ctrl(from_ipa_devs[0]->dma_ep.clnt_hdl,
			&ipa_to_usb_ep_cfg_ctrl);
	if (res) {
		pr_err("end-point ctrl register configuration failed\n");
		goto fail;
	}
	pr_debug("end-point ctrl register configured successfully (ipa_ep_suspend = true)\n");

	res = kstrtos32(params[1], 10, (s32 *)&defer_work);
	if (res) {
		pr_err(DRV_NAME ":kstrtos32() failed. can't convert defer param\n");
		res = -EFAULT;
		goto fail;
	}
	suspend_priv_data =
			kzalloc(sizeof(*suspend_priv_data), GFP_ATOMIC);
	if (!suspend_priv_data) {
		pr_err("failed allocating suspend_priv_data\n");
		res = -ENOMEM;
		goto fail;
	}
	suspend_priv_data->clnt_hdl = from_ipa_devs[0]->dma_ep.clnt_hdl;
	res = ipa_add_interrupt_handler(IPA_TX_SUSPEND_IRQ, suspend_handler,
			(bool)defer_work, (void *)suspend_priv_data);
	if (res) {
		pr_err("register handler for suspend interrupt failed\n");
		goto fail;
	}

	return 0;

fail:

	return res;
}

int configure_system_18(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;
#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

	datapath_ds_clean();
#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;

	/* Connect Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

#endif

	/* Connect IPA -> Rx BAM-DMA */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	res = connect_ipa_to_bamdma(
			&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST_CONS,
			&ipa_ep_cfg,
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			notify_ipa_write_done,
			NULL);

#else
	/* Connect Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	sys_in.notify = notify_ipa_write_done;
	if (ipa_sys_setup(&sys_in,
				&ipa_bam_hdl,
				&ipa_pipe_num,
				&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(
			&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST_PROD,
			notify_ipa_received,
			NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;
#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	sys_in.notify = notify_ipa_received;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;

#endif
fail:
	/* cleanup and tear down goes here */
	return res;
}


/**
 * Read File.
 *
 * @note This function is used by User Mode Application
 * in order to read data from the device node /dev/ipa_exception_pipe.
 * This implementation assumes Single Reader and Single Writer.
 *
 */
ssize_t exception_kfifo_read(struct file *filp, char __user *buf,
		       size_t count, loff_t *p_pos)
{
	int ret = 0;
	size_t data_len = 0;
	unsigned int copied;

	if (kfifo_is_empty(
			&(p_exception_hdl_data->
					notify_cb_data.exception_kfifo))) {
		/* Optimization*/

		pr_debug(
				"[%s:%d, %s()] No Data in exception pipe, Sleeping...\n"
				, __FILE__, __LINE__, __func__);
		msleep(200);
		pr_debug("[%s:%d, %s()] Sleeping Done...\n"
				, __FILE__, __LINE__, __func__);
		if (kfifo_is_empty(&(p_exception_hdl_data->
				notify_cb_data.exception_kfifo))) {
			pr_debug("[%s:%d, %s()] No Data in exception pipe.Returning\n"
					, __FILE__, __LINE__, __func__);
			return 0;
		}
	}
	data_len = kfifo_peek_len
			(&(p_exception_hdl_data
					->notify_cb_data.exception_kfifo));
	if (data_len > count) {
		pr_err("[%s:%d, %s()]buffer(%zu) too small (%zu) required\n"
				, __FILE__, __LINE__,
				__func__, data_len, count);
		return -ENOSPC;
	}
	ret = kfifo_to_user(&
			(p_exception_hdl_data->
					notify_cb_data.exception_kfifo)
			, buf, data_len, &copied);
#if (EXCEPTION_KFIFO_DEBUG_VERBOSE)
	{
		int i = 0;

		pr_debug("Exception packet's length=%zu, Packet's content:\n"
					, data_len);
		if (data_len - 3 > 0) {
			for (i = 0; i < data_len-4; i += 4) {
				pr_debug("%02x %02x %02x %02x\n",
						(buf)[i], (buf)[i+1],
						(buf)[i+2], (buf)[i+3]);
			}
		}
	}
#endif
	return ret ? ret : copied;
}

/**
 * Write File.
 *
 * @note This function is used by User
 * in order to write data to the device node /dev/ipa_exception_pipe.
 * This implementation assumes Single Reader and Single Writer.
 *
 */
ssize_t exception_kfifo_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;

	ret = kfifo_from_user
			(&(p_exception_hdl_data->notify_cb_data.exception_kfifo)
					, buf, count, &copied);
	if (ret) {
		pr_err("[%s:%d, %s()](%d/%zu) Bytes were written to kfifo.\n"
				, __FILE__, __LINE__, __func__
				, copied, count);
	}
	return ret ? ret : copied;
}

static const struct file_operations exception_fops = {
	.owner = THIS_MODULE,
	.read = exception_kfifo_read,
	.write = exception_kfifo_write,
};

void notify_upon_exception(void *priv,
		enum ipa_dp_evt_type evt, unsigned long data)
{
	int i = 0;
	size_t data_len;
	int res = 0;
	char *p_data = NULL;
	struct sk_buff *p_sk_buff = (struct sk_buff *)data;
	struct notify_cb_data_st *p_notify_cb_data
		= (struct notify_cb_data_st *)priv;

	pr_debug("[%s:%d, %s()] was called, evt=%s(%d)"
			, __FILE__, __LINE__,
			__func__, (evt == IPA_RECEIVE) ?
					"IPA_RECEIVE\0" :
					"IPA_WRITE_DONE\0", evt);
	if (IPA_RECEIVE != evt) {
		pr_err("[%s:%d, %s()]unexpected value of evt == %d\n"
				, __FILE__, __LINE__,
				__func__, evt);
		return;
	}

#if (SAVE_HEADER)
	data_len = p_sk_buff->len + 8; /* store len */
	p_data = (p_sk_buff->data) - 8; /* store pointer to the data */
#else
	data_len = p_sk_buff->len; /* store len */
	p_data = p_sk_buff->data; /* store pointer to the data */
#endif

#if (EXCEPTION_KFIFO_DEBUG_VERBOSE)
		pr_debug("Exception packet length = %zu,Packet content:\n",
				data_len);
		for (i = 0; i < data_len - 4; i += 4) {
			pr_debug("%02x %02x %02x %02x",
					(p_data)[i], (p_data)[i+1],
					(p_data)[i+2], (p_data)[i+3]);
		}
#endif
	res = kfifo_in(
			&p_notify_cb_data->exception_kfifo,
			p_data , data_len);
	if (res != data_len) {
		pr_err("[%s:%d, %s()] kfifo_in copied %d Bytes instead of %zu\n"
				, __FILE__, __LINE__, __func__,
				res, data_len);
		return;
	}
}
/*
 * This function Inits the KFIFO and the Device Node of the exceptions
 */
int exception_hdl_init(void)
{
	int res = 0;
	struct notify_cb_data_st *p_notify_cb_data;

	pr_debug("[%s:%d, %s()] called.\n", __FILE__,
						__LINE__, __func__);
	if (NULL != p_exception_hdl_data) {
		pr_err("[%s:%d,%s()]p_exception_hdl_data is initialized?=(0x%p)\n"
				, __FILE__, __LINE__
				, __func__, p_exception_hdl_data);
		return -EINVAL;
	}
	p_exception_hdl_data =
			kzalloc(sizeof(struct exception_hdl_data), GFP_KERNEL);
	if (NULL == p_exception_hdl_data) {
		pr_err("[%s:%d,%s()]kzalloc return NULL(%p), can't alloc %zu Bytes\n"
				, __FILE__, __LINE__, __func__
				, p_exception_hdl_data,
				sizeof(struct exception_hdl_data));
		return -ENOMEM;
	}
	pr_debug("[%s:%d, %s()] Continue...\n", __FILE__,
			__LINE__, __func__);
	p_notify_cb_data = &(p_exception_hdl_data->notify_cb_data);

	res = kfifo_alloc(&(p_notify_cb_data->exception_kfifo)
			, EXCEPTION_KFIFO_SIZE*(sizeof(char)*RX_DESCRIPTOR_SIZE)
			, GFP_KERNEL);
	if (0 != res) {
		pr_err("[%s:%d, %s()]kfifo_alloc returned error (%d)\n"
				, __FILE__, __LINE__
				, __func__, res);
		return res;
	}
	res = alloc_chrdev_region(&p_exception_hdl_data->dev_num
			, 0, 1, EXCEPTION_DRV_NAME);
	if (0 != res) {
		pr_err("[%s:%d, %s()]alloc_chrdev_region failed (%d)\n"
				, __FILE__, __LINE__, __func__, res);
		return res;
	}
	p_exception_hdl_data->class =
			class_create(THIS_MODULE, EXCEPTION_DRV_NAME);
	p_exception_hdl_data->dev =
			device_create(p_exception_hdl_data->class
					, NULL, p_exception_hdl_data->dev_num,
				      ipa_test, EXCEPTION_DRV_NAME);
	if (IS_ERR(p_exception_hdl_data->dev)) {
		pr_err("[%s:%d, %s()]device_create returned error\n"
				, __FILE__, __LINE__, __func__);
		return -ENODEV;
	}
	p_exception_hdl_data->p_cdev = cdev_alloc();
	if (NULL == p_exception_hdl_data->p_cdev) {
		pr_err("[%s:%d, %s()]cdev_alloc() returned NULL (0x%p)\n",
				__FILE__, __LINE__, __func__
				, p_exception_hdl_data->p_cdev);
		return -EINVAL;
	}
	cdev_init(p_exception_hdl_data->p_cdev, &exception_fops);
	p_exception_hdl_data->p_cdev->owner = THIS_MODULE;
	res = cdev_add(p_exception_hdl_data->p_cdev
			, p_exception_hdl_data->dev_num, 1);
	if (0 != res) {
		pr_err("[%s:%d, %s()]cdev_add failed (%d)\n"
				, __FILE__, __LINE__, __func__, res);
		return res;
	}

	pr_debug("[%s:%d, %s()] completed.(%d)\n"
			, __FILE__, __LINE__, __func__, res);
	return res;
}
/*
 * Clear the Exception Device and KFIFO
 */
void exception_hdl_exit(void)
{

	unregister_chrdev_region
			(p_exception_hdl_data->dev_num, 1);
	kfifo_free(&(p_exception_hdl_data
				->notify_cb_data.exception_kfifo));
	/* freeing kfifo */
	memset(&(p_exception_hdl_data
				->notify_cb_data.exception_kfifo), 0,
			sizeof(p_exception_hdl_data
				->notify_cb_data.exception_kfifo));
	kfree(p_exception_hdl_data);
	p_exception_hdl_data = NULL;
}

/* Configuration used for Exception Tests */
int configure_system_7(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

	res = exception_hdl_init();
	if (0 != res) {
		pr_err("[%s:%d, %s()] exception_hdl_init() failed (%d)\n",
				__FILE__, __LINE__
				, __func__, res);
		return res;
	}

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = 11;
#endif

#ifndef IPA_ON_R3PC
	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[0]->ep,
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[0]->desc_fifo,
			&from_ipa_devs[0]->rx_event);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[1]->ep,
			from_ipa_devs[1]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[1]->desc_fifo,
			&from_ipa_devs[1]->rx_event);
#else
	/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[1]->desc_fifo,
				  &from_ipa_devs[1]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect third (Default) Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[2]->ep,
			from_ipa_devs[2]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[2]->desc_fifo,
			&from_ipa_devs[2]->rx_event);
#else
	/* Connect third (Default) Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[2]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[2]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[2]->desc_fifo,
				  &from_ipa_devs[2]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect IPA -> first Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[1]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect IPA -> third (default) Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[2]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[2]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD,
			&notify_upon_exception,
			&(p_exception_hdl_data->notify_cb_data));
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
				     to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
				     &to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	sys_in.notify = &notify_upon_exception;
	sys_in.priv = &(p_exception_hdl_data->notify_cb_data);

	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

	if (res)
		goto fail;
#endif
fail:
	/* cleanup and tear down goes here */
	return res;
}

void destroy_channel_device(struct channel_dev *channel_dev)
{
	int res = 0;

	pr_debug(DRV_NAME
			":%s():Destroying device channel_dev = 0x%p,name %s.\n",
	       __func__, channel_dev, channel_dev->name);

	if (NULL != channel_dev->ep.sps) {
		res = sps_disconnect(channel_dev->ep.sps);
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on sps_disconnect(), "
					"channel_dev = 0x%p, res = %d.\n",
			       __func__, channel_dev, res);
		}
		pr_debug(DRV_NAME ":%s(): sps disconnected.\n", __func__);

		res = sps_free_endpoint(channel_dev->ep.sps);
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on sps_free_endpoint(), "
					"channel_dev = 0x%p, res = %d.\n",
			       __func__, channel_dev, res);
		}
		pr_debug(DRV_NAME ":%s(): sps ep freed.\n", __func__);
	}

	if (NULL != channel_dev->dma_ep.sps) {
		res = ipa_disconnect(channel_dev->dma_ep.clnt_hdl);
		/*TODO check if this handle is really being saved*/
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on ipa_disconnect(), "
					"channel_dev = 0x%p, res = %d.\n",
			__func__, channel_dev, res);
		}
		pr_debug(DRV_NAME ":%s(): ipa disconnected.\n", __func__);

		res = sps_disconnect(channel_dev->dma_ep.sps);
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on sps_disconnect(), "
					"channel_dev = 0x%p, res = %d.\n",
			       __func__, channel_dev, res);
		}
		pr_debug(DRV_NAME ":%s(): bamdma sps disconnected.\n"
				, __func__);

		res = sps_free_endpoint(channel_dev->dma_ep.sps);
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on sps_free_endpoint(), "
					"channel_dev = 0x%p, res = %d.\n",
			       __func__, channel_dev, res);
		}
		pr_debug(DRV_NAME ":%s(): bamdma sps ep freed.\n", __func__);
	}
#ifdef IPA_ON_R3PC
		else {
		res = ipa_sys_teardown(channel_dev->ipa_client_hdl);
		if (res) {
			pr_err(DRV_NAME
					":%s(): Failure on ipa_sys_teardown(),"
					" channel_dev = 0x%p, res = %d.\n",
				   __func__, channel_dev, res);
		}
	}
#endif
	test_free_mem(&channel_dev->desc_fifo);
	test_free_mem(&channel_dev->mem);

	cdev_del(&channel_dev->cdev);
	device_destroy(channel_dev->class, channel_dev->dev_num);
	class_destroy(channel_dev->class);
	unregister_chrdev_region(channel_dev->dev_num, 1);

	kfree(channel_dev);
}

void destroy_channel_devices(void)
{
	pr_debug(DRV_NAME ":-----Tear Down----\n");
	while (ipa_test->num_tx_channels > 0) {
		pr_debug(DRV_NAME
				":%s():-- num_tx_channels = %d --\n",
				__func__, ipa_test->num_tx_channels);

		destroy_channel_device(
			ipa_test->tx_channels[--ipa_test->num_tx_channels]);
	}

	while (ipa_test->num_rx_channels > 0) {
		pr_debug(DRV_NAME
				":%s():-- num_rx_channels = %d --\n",
				__func__, ipa_test->num_rx_channels);
		destroy_channel_device
		(from_ipa_devs[--ipa_test->num_rx_channels]);
	}
}

int register_lan_interface(void)
{
	struct ipa_rx_intf rx_intf;
	struct ipa_tx_intf tx_intf;
	struct ipa_ioc_tx_intf_prop tx_prop;
	struct ipa_ioc_rx_intf_prop rx_prop;
	char *name = "rmnet1";
	int res;

	pr_debug(DRV_NAME ":new version\n");
	memset(&tx_prop, 0, sizeof(tx_prop));
	tx_prop.ip = IPA_IP_v6;
	tx_prop.dst_pipe = IPA_CLIENT_TEST_CONS;

	memset(&rx_prop, 0, sizeof(rx_prop));
	rx_prop.ip = IPA_IP_v6;
	rx_prop.src_pipe = IPA_CLIENT_TEST_PROD;

	memset(&rx_intf, 0, sizeof(rx_intf));
	rx_intf.num_props = 1;
	rx_intf.prop = &rx_prop;

	memset(&tx_intf, 0, sizeof(tx_intf));
	tx_intf.num_props = 1;
	tx_intf.prop = &tx_prop;

	res = ipa_register_intf(name, &tx_intf, &rx_intf);
	if (res != 0)
		goto fail;

	pr_debug(DRV_NAME ":registered interface %s !\n", name);

fail:
	return res;
}


int configure_system_16(void)
{
	int res;

	res = register_lan_interface();
	if (res)
		return res;

	res = configure_system_1();
	return res;
}

/* add wlan header to ipa */

#define IPA_TO_WLAN_HEADER_NAME "wlan0"
int add_wlan_header(void)
{

#define IPA_TO_WLAN_HEADER_LEN  34

	uint8_t hdr[IPA_TO_WLAN_HEADER_LEN + 1] = {
		/* HTC Header - 6 bytes */
		0x00, 0x00,  /* Reserved */
		0x00, 0x00, /* length to be filled by IPA,
					after adding 32 with IP Payload
					length 32 will be
					programmed while
					intializing the header */
		0x00, 0x00,  /* Reserved */
		/* WMI header - 6 bytes*/
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

		/* 802.3 header - 14 bytes*/
		0x00, 0x03, 0x7f, 0x44, 0x33, 0x89,
		/* Des. MAC to be filled by IPA */
		0x00, 0x03, 0x7f, 0x17, 0x12, 0x69,
		/* Src. MAC to be filled by IPA */
		0x00, 0x00,
		/* length can be zero */

		/* LLC SNAP header - 8 bytes */
		0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,
		0x08, 0x00  /* type value(2 bytes) to
						be filled by IPA, by reading
						from ethernet header */
		/* 0x0800 - IPV4, 0x86dd - IPV6 */
	};

	int ret = 0;
	int len = 0;
	struct ipa_ioc_add_hdr *ipa_to_wlan_header_partial;
	struct ipa_hdr_add ipa_to_wlan_header;

	memset(&ipa_to_wlan_header, 0, sizeof(ipa_to_wlan_header));
	/* Copy header name */
	memcpy(ipa_to_wlan_header.name,
				 IPA_TO_WLAN_HEADER_NAME,
				 sizeof(IPA_TO_WLAN_HEADER_NAME));

	/* poplate other fields of header add */
	ipa_to_wlan_header.hdr_len = IPA_TO_WLAN_HEADER_LEN;
	ipa_to_wlan_header.is_partial = 1;
	ipa_to_wlan_header.hdr_hdl = 0;
	ipa_to_wlan_header.status = -1;
	/* copy the parital header */
	memcpy(ipa_to_wlan_header.hdr, hdr, IPA_TO_WLAN_HEADER_LEN);

	/* Add wlan partial header to ipa */
	len = (sizeof(struct ipa_ioc_add_hdr)) +
			(1 * sizeof(struct ipa_hdr_add));
	ipa_to_wlan_header_partial = kmalloc(len, GFP_KERNEL);
	if (!ipa_to_wlan_header_partial) {
		pr_err("Memory allocation failure");
		return false;
	}

	ipa_to_wlan_header_partial->commit = 1;
	ipa_to_wlan_header_partial->num_hdrs = 1;
	memcpy(&ipa_to_wlan_header_partial->hdr[0],
		       &ipa_to_wlan_header,
		       sizeof(ipa_to_wlan_header));
	ret = ipa_add_hdr(ipa_to_wlan_header_partial);
	if (ret) {
		pr_err("unable to add wlan header %d", ret);
		goto fail;
	} else if (ipa_to_wlan_header_partial->hdr[0].status) {
		pr_err("unable to add wlan header %d", ret);
		goto fail;
	}

	pr_debug("added wlan header successfully\n");

fail:
	kfree(ipa_to_wlan_header_partial);

	return ret;
}

/* Wlan interface has 4 rx and 1 Tx endpoint */
int register_wlan_interface(void)
{
	struct ipa_rx_intf rx_intf;
	struct ipa_tx_intf tx_intf;
	struct ipa_ioc_tx_intf_prop tx_prop[4];
	struct ipa_ioc_rx_intf_prop rx_prop;
	char *name = "eth0";
	int res, index = 0;

	res = add_wlan_header();
	if (res)
		return res;

	memset(&tx_prop, 0, 4 * sizeof(struct ipa_ioc_tx_intf_prop));

	index = 0;
	tx_prop[index].ip = IPA_IP_v6;
	tx_prop[index].dst_pipe = IPA_CLIENT_TEST1_CONS;
	memcpy(tx_prop[index].hdr_name, IPA_TO_WLAN_HEADER_NAME,
		  sizeof(IPA_TO_WLAN_HEADER_NAME));

	index++;
	tx_prop[index].ip = IPA_IP_v6;
	tx_prop[index].dst_pipe = IPA_CLIENT_TEST2_CONS;
	memcpy(tx_prop[index].hdr_name, IPA_TO_WLAN_HEADER_NAME,
			sizeof(IPA_TO_WLAN_HEADER_NAME));

	index++;
	tx_prop[index].ip = IPA_IP_v6;
	tx_prop[index].dst_pipe = IPA_CLIENT_TEST3_CONS;
	memcpy(tx_prop[index].hdr_name, IPA_TO_WLAN_HEADER_NAME,
			sizeof(IPA_TO_WLAN_HEADER_NAME));

	index++;
	tx_prop[index].ip = IPA_IP_v6;
	tx_prop[index].dst_pipe = IPA_CLIENT_TEST4_CONS;
	memcpy(tx_prop[index].hdr_name, IPA_TO_WLAN_HEADER_NAME,
			sizeof(IPA_TO_WLAN_HEADER_NAME));

	memset(&rx_prop, 0, sizeof(struct ipa_ioc_rx_intf_prop));
	rx_prop.ip = IPA_IP_v6;
	rx_prop.src_pipe = IPA_CLIENT_TEST1_PROD;

	memset(&rx_intf, 0, sizeof(rx_intf));
	rx_intf.num_props = 1;
	rx_intf.prop = &rx_prop;

	memset(&tx_intf, 0, sizeof(tx_intf));
	tx_intf.num_props = 4;
	tx_intf.prop = tx_prop;

	res = ipa_register_intf(name, &tx_intf, &rx_intf);
	if (res) {
		pr_err(DRV_NAME
				":Unable to register interface %s, %d\n",
				name, res);
		return res;
	}

	pr_debug(DRV_NAME ":Registered interface %s\n", name);
	return res;
}
/*
	wlan with 4 inputs(BE, VO, VE and BG) and 1 ouput
	Configures the system with one input to IPA and 4 outputs.
	/dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA
	|-> BAM-DMA -> MEM -> /dev/from_ipa_0
	|-> BAM-DMA -> MEM -> /dev/from_ipa_1
	|-> BAM-DMA -> MEM -> /dev/from_ipa_2
	|-> BAM-DMA -> MEM -> /dev/from_ipa_3
*/

int configure_system_14(void)
{
	int res = 0, index = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#else
  int pipe_cnt = 0;
#endif

	pr_debug(DRV_NAME ":Update Version 4\n");

	res = register_wlan_interface();
	if (res)
		return res;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	pipe_cnt = 4;
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = pipe_cnt;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = (++pipe_cnt);
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = (++pipe_cnt);
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = (++pipe_cnt);

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = (++pipe_cnt);
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = (++pipe_cnt);
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = (++pipe_cnt);
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = (++pipe_cnt);
	from_ipa_devs[2]->dma_ep.chan.src_pipe_index  = (++pipe_cnt);
	from_ipa_devs[2]->dma_ep.chan.dest_pipe_index = (++pipe_cnt);
	from_ipa_devs[3]->dma_ep.chan.src_pipe_index  = (++pipe_cnt);
	from_ipa_devs[3]->dma_ep.chan.dest_pipe_index = (++pipe_cnt);
#endif

#ifndef IPA_ON_R3PC
	/* Connect first Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[index]->ep,
			from_ipa_devs[index]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[index]->desc_fifo,
			&from_ipa_devs[index]->rx_event);
#else
	/* Connect first Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST1_CONS;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[index]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[index]->desc_fifo,
				  &from_ipa_devs[index]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

index++;
#ifndef IPA_ON_R3PC
	/* Connect second Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[index]->ep,
			from_ipa_devs[index]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[index]->desc_fifo,
			&from_ipa_devs[index]->rx_event);
#else
	/* Connect second Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[index]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[index]->desc_fifo,
				  &from_ipa_devs[index]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

index++;
#ifndef IPA_ON_R3PC
	/* Connect third Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[index]->ep,
			from_ipa_devs[index]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[index]->desc_fifo,
			&from_ipa_devs[index]->rx_event);
#else
	/* Connect third Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[index]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[index]->desc_fifo,
				  &from_ipa_devs[index]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

index++;
#ifndef IPA_ON_R3PC
	/* Connect fourth Rx BAM-DMA --> A5 MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[index]->ep,
			from_ipa_devs[index]->dma_ep.chan.dest_pipe_index,
			&from_ipa_devs[index]->desc_fifo,
			&from_ipa_devs[index]->rx_event);
#else
	/* Connect fourth Rx IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST4_CONS;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[index]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[index]->desc_fifo,
				  &from_ipa_devs[index]->rx_event,
				  ipa_bam_hdl);
#endif
	if (res)
		goto fail;

index = 0;
#ifndef IPA_ON_R3PC
	/* Connect IPA -> first Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[index]->dma_ep,
			IPA_CLIENT_TEST1_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
	else
			pr_debug(DRV_NAME
				"Connect IPA -> first Rx BAM-DMA success\n");

	index++;
	/* Connect IPA -> second Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[index]->dma_ep,
			IPA_CLIENT_TEST2_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
	else
		pr_debug(DRV_NAME
				"Connect IPA -> second Rx BAM-DMA success\n");

	index++;
	/* Connect IPA -> third Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[index]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
	else
		pr_debug(DRV_NAME "Connect IPA -> third Rx BAM-DMA success\n");

	index++;
	/* Connect IPA -> fourth Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[index]->dma_ep,
			IPA_CLIENT_TEST4_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
	else
		pr_debug(DRV_NAME "Connect IPA -> fourth Rx BAM-DMA success\n");
#endif

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

index = 0;
#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[index]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST1_PROD,
			NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[index]->ep,
			to_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[index]->desc_fifo);
	if (res)
		goto fail;
	else
	  pr_debug(DRV_NAME
			  "Connect IPA -> IPA_CLIENT_TEST1_PROD success\n");

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST1_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
					&ipa_bam_hdl,
					&ipa_pipe_num,
					&to_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[index]->ep,
					ipa_pipe_num,
					&to_ipa_devs[index]->desc_fifo,
					ipa_bam_hdl);

	if (res)
		goto fail;
	else
	  pr_debug(DRV_NAME
			  "Connect IPA -> IPA_CLIENT_TEST1_PROD success\n");
#endif

index++;
#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[index]->dma_ep,
					&ipa_ep_cfg,
					IPA_CLIENT_A5_WLAN_AMPDU_PROD,
					NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[index]->ep,
			to_ipa_devs[index]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[index]->desc_fifo);
	if (res)
		goto fail;
	else
	  pr_debug(DRV_NAME
		"Connect IPA -> IPA_CLIENT_A5_WLAN_AMPDU_PROD success\n");

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_A5_WLAN_AMPDU_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in,
			&ipa_bam_hdl,
			&ipa_pipe_num,
			&to_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[index]->ep,
			ipa_pipe_num,
			&to_ipa_devs[index]->desc_fifo,
			ipa_bam_hdl);

	if (res)
		goto fail;
	else
	  pr_debug(DRV_NAME
			  "Connect IPA -> IPA_CLIENT_A5_WLAN_AMPDU_PROD success\n");
#endif

fail:
	/* cleanup and tear down goes here*/
	return res;
}


/* Wan interface has 1 rx and 1 Tx endpoint */
int register_wan_interface(void)
{
	struct ipa_rx_intf rx_intf;
	struct ipa_tx_intf tx_intf;
	struct ipa_ioc_tx_intf_prop tx_prop;
	struct ipa_ioc_rx_intf_prop rx_prop;
	char *name = "rmnet0";
	int res;

	memset(&tx_prop, 0, sizeof(tx_prop));
	tx_prop.ip = IPA_IP_v6;
	tx_prop.dst_pipe = IPA_CLIENT_TEST3_CONS;

	memset(&rx_prop, 0, sizeof(rx_prop));
	rx_prop.ip = IPA_IP_v6;
	rx_prop.src_pipe = IPA_CLIENT_TEST3_PROD;

	memset(&rx_intf, 0, sizeof(rx_intf));
	rx_intf.num_props = 1;
	rx_intf.prop = &rx_prop;

	memset(&tx_intf, 0, sizeof(tx_intf));
	tx_intf.num_props = 1;
	tx_intf.prop = &tx_prop;

	res = ipa_register_intf(name, &tx_intf, &rx_intf);
	if (res != 0)
		goto fail;

	pr_debug(DRV_NAME ":registered interface %s !\n", name);

fail:
	return res;
}

int configure_system_15(void)
{
	int res = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	res = register_wan_interface();
	if (res)
		return res;

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	pr_debug(DRV_NAME "Configure_system_1 was called\n");

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 5;
	from_ipa_devs[0]->dma_ep.chan.src_pipe_index = 6;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 7;
	/* Connect Rx BAM-DMA --> A5 MEM */
	pr_debug(DRV_NAME "Connecting BAM-DMA to Application CPU\n");
	res = connect_bamdma_to_apps(&(from_ipa_devs[0]->ep),
			from_ipa_devs[0]->dma_ep.chan.dest_pipe_index,
			&(from_ipa_devs[0]->desc_fifo),
			&from_ipa_devs[0]->rx_event);
	if (res)
		goto fail;

	/* Connect IPA(USB1) -> to BAM-DMA */
	pr_debug(DRV_NAME "Connecting IPA to BAM-DMA\n");
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
			IPA_CLIENT_TEST3_CONS,
			NULL,/*ipa_ep_cfg*/
			from_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			NULL, NULL);
	if (res)
		goto fail;
#else
	/* Connect IPA --> A5 MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&from_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &from_ipa_devs[0]->desc_fifo,
				  &from_ipa_devs[0]->rx_event,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

	/* Prepare EP configuration details */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.mode.mode = IPA_DMA;
	ipa_ep_cfg.mode.dst = IPA_CLIENT_TEST3_CONS;

#ifndef IPA_ON_R3PC
	/* Connect Tx BAM-DMA-> IPA's USB */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST3_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep, /*tx_ep*/
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);
	if (res)
		goto fail;

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl,
			&ipa_pipe_num, &to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect A5 MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);
	if (res)
		goto fail;
#endif

fail:
	/* cleanup and tear down goes here*/
	return res;
}

/*
 Configures the system with one input to IPA and 2 outputs.
 /dev/to_ipa_0 -> MEM -> BAM-DMA -> IPA |-> BAM-DMA
 -> MEM -> /dev/from_ipa_0
|-> BAM-DMA -> MEM -> /dev/from_ipa_1
*/
int configure_system_20(void)
{
	int res = 0;
	int index = 0;
	struct ipa_ep_cfg ipa_ep_cfg;

#ifdef IPA_ON_R3PC
	struct ipa_sys_connect_params sys_in;
	unsigned long ipa_bam_hdl;
	u32 ipa_pipe_num;
#endif

	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));

#ifndef IPA_ON_R3PC
	/* Setup BAM-DMA pipes */
	to_ipa_devs[0]->dma_ep.chan.src_pipe_index    = 4;
	to_ipa_devs[0]->dma_ep.chan.dest_pipe_index   = 5;
	to_ipa_devs[1]->dma_ep.chan.src_pipe_index    = 6;
	to_ipa_devs[1]->dma_ep.chan.dest_pipe_index   = 7;

	from_ipa_devs[0]->dma_ep.chan.src_pipe_index  = 8;
	from_ipa_devs[0]->dma_ep.chan.dest_pipe_index = 9;
	from_ipa_devs[1]->dma_ep.chan.src_pipe_index  = 10;
	from_ipa_devs[1]->dma_ep.chan.dest_pipe_index = 11;
#endif

#ifndef IPA_ON_R3PC
	/* Connect 0 Tx BAM-DMA --> AP MEM */
	res = connect_bamdma_to_apps(&from_ipa_devs[index]->ep,
		from_ipa_devs[index]->dma_ep.chan.dest_pipe_index,
		&from_ipa_devs[index]->desc_fifo,
		&from_ipa_devs[index]->rx_event);
#else
	/* Connect first Rx IPA --> AP MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_CONS;
	if (ipa_sys_setup(&sys_in,
		&ipa_bam_hdl, &ipa_pipe_num,
		&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(&from_ipa_devs[index]->ep,
		ipa_pipe_num,
		&from_ipa_devs[index]->desc_fifo,
		&from_ipa_devs[index]->rx_event,
		ipa_bam_hdl);
#endif
	if (res)
		goto fail;

#ifndef IPA_ON_R3PC
	/* Connect IPA -> 0 Rx BAM-DMA */
	res = connect_ipa_to_bamdma(&from_ipa_devs[0]->dma_ep,
		IPA_CLIENT_TEST2_CONS,
		NULL,/*ipa_ep_cfg*/
		from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
		NULL, NULL);
	if (res)
		goto fail;
#endif

	index++;

	/* Connect IPA -> 1 Tx BAM-DMA */
	/* RNDIS Aggregation with ETH2 header */
	memset(&ipa_ep_cfg, 0, sizeof(ipa_ep_cfg));
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_AGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.aggr.aggr_byte_limit = 1;
	ipa_ep_cfg.aggr.aggr_time_limit = 0;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = true;
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_additional_const_len = 14;
	ipa_ep_cfg.hdr.hdr_len = 58;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = true;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	/* Connect 1 Tx BAM-DMA --> AP MEM */
	res = connect_ipa_to_bamdma(
		&from_ipa_devs[index]->dma_ep,
		IPA_CLIENT_TEST3_CONS,
		&ipa_ep_cfg,
		from_ipa_devs[index]->dma_ep.chan.src_pipe_index,
		NULL,
		NULL);
#else
	/* Connect 1 Tx IPA --> AP MEM */
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST3_CONS;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(
		&sys_in,
		&ipa_bam_hdl,
		&ipa_pipe_num,
		&from_ipa_devs[index]->ipa_client_hdl))
		goto fail;

	res = connect_ipa_to_apps(
		&from_ipa_devs[index]->ep,
		ipa_pipe_num,
		&from_ipa_devs[index]->desc_fifo,
		&from_ipa_devs[index]->rx_event,
		ipa_bam_hdl);
#endif
	if (res)
		goto fail;

	/* Connect Rx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	/* configure RNDIS+ETH2 header removal on Rx */
	/* configure RNDIS de-aggregation on Rx */
	ipa_ep_cfg.aggr.aggr_en = IPA_ENABLE_DEAGGR;
	ipa_ep_cfg.aggr.aggr = IPA_GENERIC;
	ipa_ep_cfg.deaggr.deaggr_hdr_len = 44; /* RNDIS hdr */
	ipa_ep_cfg.deaggr.packet_offset_valid = true;
	ipa_ep_cfg.deaggr.packet_offset_location = 8;
	ipa_ep_cfg.hdr.hdr_len = 14; /* Ethernet header */
	ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 12;
	ipa_ep_cfg.hdr.hdr_remove_additional = false;
	ipa_ep_cfg.hdr_ext.hdr_little_endian = 1;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_valid = 1;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad = IPA_HDR_TOTAL_LEN;
	ipa_ep_cfg.hdr_ext.hdr_payload_len_inc_padding = 0;
	ipa_ep_cfg.hdr_ext.hdr_total_len_or_pad_offset = 4;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[0]->dma_ep,
			&ipa_ep_cfg, IPA_CLIENT_TEST_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect AP MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[0]->ep,
			to_ipa_devs[0]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[0]->desc_fifo);

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[0]->ipa_client_hdl))
		goto fail;

	/* Connect AP MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[0]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[0]->desc_fifo,
				  ipa_bam_hdl);

#endif

if (res)
	goto fail;

	/* Connect Tx BAM-DMA -> IPA */
	/* Prepare an endpoint configuration structure */
	res = configure_ipa_endpoint(&ipa_ep_cfg, IPA_BASIC);
	if (res)
		goto fail;

	/* configure ETH2 header removal on Tx */
	ipa_ep_cfg.hdr.hdr_len = ETH_HLEN + IPA_TEST_WLAN_HDR_LEN;

#ifndef IPA_ON_R3PC
	/* Connect using the endpoint configuration created */
	res = connect_bamdma_to_ipa(&to_ipa_devs[1]->dma_ep,
			&ipa_ep_cfg,
			IPA_CLIENT_TEST2_PROD, NULL, NULL);
	if (res)
		goto fail;

	/* Connect A5 MEM --> Tx BAM-DMA */
	res = connect_apps_to_bamdma(&to_ipa_devs[1]->ep,
			to_ipa_devs[1]->dma_ep.chan.src_pipe_index,
			&to_ipa_devs[1]->desc_fifo);

#else
	memset(&sys_in, 0, sizeof(sys_in));
	sys_in.client = IPA_CLIENT_TEST2_PROD;
	sys_in.ipa_ep_cfg = ipa_ep_cfg;
	if (ipa_sys_setup(&sys_in, &ipa_bam_hdl, &ipa_pipe_num,
			&to_ipa_devs[1]->ipa_client_hdl))
		goto fail;

	/* Connect AP MEM --> Tx IPA */
	res = connect_apps_to_ipa(&to_ipa_devs[1]->ep,
				  ipa_pipe_num,
				  &to_ipa_devs[1]->desc_fifo,
				  ipa_bam_hdl);

#endif

	if (res)
		goto fail;

fail:
	/* cleanup and tear down goes here */
	return res;
}

static char **str_split(char *str, const char *delim)
{
	char **res = NULL;
	char **tmp = NULL;
	char *p = strsep(&str, delim);
	int n_spaces = 0;

	/* split string and append tokens to 'res' */
	while (p) {
		tmp = krealloc(res, sizeof(char *) * ++n_spaces, GFP_KERNEL);

		if (tmp == NULL) {
			pr_err("krealloc failed\n");
			goto fail; /* memory allocation failed */
		}

		res = tmp;
		res[n_spaces-1] = p;
		p = strsep(&str, delim);
	}
	/* realloc one extra element for the last NULL */
	tmp = krealloc(res, sizeof(char *) * (n_spaces+1), GFP_KERNEL);
	if (tmp == NULL) {
		pr_err("krealloc failed\n");
		goto fail; /* memory allocation failed */
	}
	res = tmp;
	res[n_spaces] = 0;
	return res;
fail:
	kfree(res);
	return res;
}

/**
 * Write File.
 *
 * @note Configure the system by writing a configuration
 * index to the device node /dev/ipa_test
 */
ssize_t ipa_test_write(struct file *filp, const char __user *buf,
		       size_t size, loff_t *f_pos)
{
	int ret = 0;
	int index;

	unsigned long missing;
	char *str;
	char **params;

	str = kzalloc(size+1, GFP_KERNEL);
	if (str == NULL) {
		pr_err(DRV_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}

	missing = copy_from_user(str, buf, size);
	if (missing) {
		kfree(str);
		return -EFAULT;
	}
	pr_debug("ipa_test_write: input string= %s\n", str);
	str[size] = '\0';

	params = str_split(str, " ");
	if (params == NULL) {
		kfree(str);
		return -EFAULT;
	}

	ret = kstrtos32(params[0], 10, (s32 *)&ipa_test->configuration_idx);
	if (ret) {
		pr_err(DRV_NAME ":kstrtoul() failed.\n");
		ret = -EFAULT;
		goto bail;
	}

	pr_debug(DRV_NAME ":Invoking configuration %d.\n",
			ipa_test->configuration_idx);

	/* Setup BAM-DMA */
#ifndef IPA_ON_R3PC
	dma_bam_hdl = sps_dma_get_bam_handle();
	if (dma_bam_hdl == 0) {
		pr_err(DRV_NAME ":fail to get DMA BAM handle\n");
		ret = -ENODEV;
		goto bail;
	}
	pr_debug(DRV_NAME ":DMA BAM hdl 0x%lx -----\n", dma_bam_hdl);
#endif

	switch (ipa_test->configuration_idx) {
	case -1:
		destroy_channel_devices();
		/*exception_hdl_exit();TODO: hsnapy un-comment this*/
		break;

	case 0:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];

		ret = configure_system_0();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 1:
		index = 0;
		/*Create the device on user space and allocate
		 * buffers for its BAM connection*/
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];

		/*Create the device on user space and allocate
		 *  buffers for its BAM connection*/
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];

		/*Make all the configurations required
		 * (SPS connects and IPA connect)*/
		ret = configure_system_1();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 2:
		index = 0;
		ret = create_channel_device(index,
					    "to_ipa", &to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->tx_channels[ipa_test->num_tx_channels++] =
			to_ipa_devs[index];
		ret = create_channel_device(index, "from_ipa",
					    &from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->rx_channels[ipa_test->num_rx_channels++] =
			from_ipa_devs[index];

		index++;
		ret = create_channel_device(index,
					    "to_ipa", &to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->tx_channels[ipa_test->num_tx_channels++] =
			to_ipa_devs[index];
		ret = create_channel_device(index, "from_ipa",
					    &from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->rx_channels[ipa_test->num_rx_channels++] =
			from_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "from_ipa",
					    &from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->rx_channels[ipa_test->num_rx_channels++] =
			from_ipa_devs[index];

		ret = configure_system_2();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 3:
		index = 0;

		/*This pipe will be the A2NDUN to IPA pipe*/
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
						TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->tx_channels[ipa_test->num_tx_channels++] =
				to_ipa_devs[index];

		/*This will be the IPA to A2NDUN pipe*/
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
						RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
							from_ipa_devs[index];

		index++;
		/*This will be the IPA to USB1 pipe*/
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
						RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
							from_ipa_devs[index];

		index++;
		/*This will be the IPA to Q6_LAN pipe*/
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
						RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
							from_ipa_devs[index];

		ret = configure_system_3();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 5:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_5();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 6:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_6();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 7:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_7();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 8:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_8();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 9:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_9();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 10:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_10();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 11:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_11();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 12:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "to_ipa",
						&to_ipa_devs[index],
					    TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_12();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 13:
		/* IPA RM component unit test and regression */
		pr_err("IPA RM :: ENTER UT sequence\n");

		ret = build_rmnet_bridge_use_case_graph(
				&ipa_rm_create_resource,
				&ipa_rm_notify_completion);
		if (ret) {
			pr_err("build_rmnet_bridge_use_case_graph FAILED\n");
			goto bail;
		}
		ret = build_rmnet_bridge_use_case_dependencies
				(&ipa_rm_add_dependency);
		if (ret) {
			pr_err("build_rmnet_bridge_use_case_dependencies FAILED\n");
			goto bail;
		}
		ret = request_release_resource_sequence(
				&ipa_rm_request_resource,
					&ipa_rm_release_resource);
		if (ret) {
			pr_err("request_release_resource_sequence FAILED\n");
			goto bail;
		}
		clean_ut();
		pr_err("IPA RM ::EXIT UT\n");
		break;

	case 14:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
					&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index++;
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		/* from ipa channels creation */
		index = 0;
		ret = create_channel_device(index, "from_ipa",
				&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
					&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
					&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		index++;

		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];
		configure_system_14();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 15:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
					&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];
		ret = create_channel_device(index, "from_ipa",
					&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_15();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 16:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
					&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		ret = create_channel_device(index, "from_ipa",
				&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
				rx_channels[ipa_test->num_rx_channels++] =
						from_ipa_devs[index];

		ret = configure_system_16();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 17:
		index = 0;
		ret = create_channel_device(
				index,
				"to_ipa",
				&to_ipa_devs[index],
				TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];
		index++;

		ret = create_channel_device(
				index,
				"to_ipa",
				&to_ipa_devs[index],
				TX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];
		index++;

		ret = create_channel_device(
				index,
				"to_ipa",
				&to_ipa_devs[index],
				TX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test
		->tx_channels[ipa_test->num_tx_channels++] =
				to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(
				index,
				"from_ipa",
				&from_ipa_devs[index],
				RX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
		rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];
		index++;

		ret = create_channel_device(
				index,
				"from_ipa",
				&from_ipa_devs[index],
				RX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
		rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];
		index++;

		ret = create_channel_device(
				index,
				"from_ipa",
				&from_ipa_devs[index],
				RX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
		rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];
		index++;

		ret = create_channel_device(
				index,
				"from_ipa",
				&from_ipa_devs[index],
				RX_SZ);
		if (ret) {
			pr_err(DRV_NAME
					":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
		rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];

		ret = configure_system_17();
		if (ret) {
			pr_err(DRV_NAME
					":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;
	case 18:
		index = 0;
		ret = create_channel_device(index, "to_ipa",
					&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			return -ENODEV;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
						to_ipa_devs[index];

		index++;
		ret = create_channel_device_by_type(index, "to_ipa",
					&to_ipa_devs[index], RX_SZ,
					IPA_TEST_DATA_PATH_TEST_CHANNEL);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			return -ENODEV;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];

		index = 0;
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index], RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			return -ENODEV;
		}
		ipa_test->
			rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];

		ret = configure_system_18();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			return -ENODEV;
		}
		break;

	case 19:
		index = 0;
		/*Create the device on user space and allocate
		 * buffers for its BAM connection*/
		ret = create_channel_device(index, "to_ipa",
				&to_ipa_devs[index], TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
					to_ipa_devs[index];

		/*Create the device on user space and allocate
		 *  buffers for its BAM connection*/
		ret = create_channel_device(index, "from_ipa",
						&from_ipa_devs[index],
					    RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->rx_channels[ipa_test->num_rx_channels++] =
				from_ipa_devs[index];

		/*Make all the configurations required
		 * (SPS connects and IPA connect)*/
		ret = configure_system_19(params);
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.\n");
			ret = -ENODEV;
			goto bail;
		}
		break;

	case 20:

		/* Create producer channel 0 */
		index = 0;
		ret = create_channel_device(index,
			"to_ipa", &to_ipa_devs[index],
			TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
			to_ipa_devs[index];


		/* Create producer channel 1 */
		index++;
		ret = create_channel_device(index,
			"to_ipa", &to_ipa_devs[index],
			TX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			tx_channels[ipa_test->num_tx_channels++] =
			to_ipa_devs[index];

		/* Create consumer channel 0 */
		index = 0;
		ret = create_channel_device(index, "from_ipa",
			&from_ipa_devs[index],
			RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			rx_channels[ipa_test->num_rx_channels++] =
			from_ipa_devs[index];

		/* Create consumer channel 1 */
		index++;
		ret = create_channel_device(index, "from_ipa",
			&from_ipa_devs[index],
			RX_SZ);
		if (ret) {
			pr_err(DRV_NAME ":Channel device creation error.\n");
			ret = -ENODEV;
			goto bail;
		}
		ipa_test->
			rx_channels[ipa_test->num_rx_channels++] =
			from_ipa_devs[index];

		ret = configure_system_20();
		if (ret) {
			pr_err(DRV_NAME ":System configuration failed.");
			ret = -ENODEV;
			goto bail;
		}
		break;

	default:
		pr_err("Unsupported configuration index.\n");
		break;
	}

	/* Insert descriptors into the receiving end(s) */
	ret = insert_descriptors_into_rx_endpoints(RX_BUFF_SIZE);
	if (ret) {
		pr_debug(DRV_NAME
				":%s() Descriptor insertion into rx "
				"endpoints failed.\n", __func__);
		ret = -EINVAL;
		goto bail;
	}

	pr_debug(DRV_NAME ":System configured !\n");

	/* Set the current configuration index */
	ipa_test->current_configuration_idx	=
		ipa_test->configuration_idx;

	ret = size;

bail:
	kfree(params);
	kfree(str);
	return ret;
}

static ssize_t ipa_test_read(
				struct file *filp,
				char __user *buf,
				size_t count, loff_t *f_pos)
{
	int res, len;
	char str[10];
	if (0 != *f_pos) {
		*f_pos = 0;
		return 0;
	}

	/* Convert the configuration index to a null terminated string */
	len = snprintf(str, 10, "%d",
			ipa_test->current_configuration_idx);

	pr_debug(DRV_NAME ":str = %s, len = %d\n", str, len);

	/* Copy the result to the user buffer */
	res = copy_to_user(buf, str, len + 1);
	if (res < 0) {
		pr_err(DRV_NAME
				":%s() copy_to_user() failed.\n", __func__);
		return -EFAULT;
	}

	/* Increment the file position pointer */
	*f_pos += len;

	/* Return the number of bytes copied to the user */
	return len + 1;
}

static struct class *ipa_test_class;

static long ipa_test_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	enum ipa_hw_type retval;

	pr_debug("cmd=%x nr=%d\n", cmd, _IOC_NR(cmd));
	if (_IOC_TYPE(cmd) != IPA_TEST_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case IPA_TEST_IOC_GET_HW_TYPE:
		retval = ipa_get_hw_type();
		break;
	default:
		return -ENOTTY;
	}

	return retval;
}

static const struct file_operations ipa_test_fops = {
	.owner = THIS_MODULE,
	.write = ipa_test_write,
	.read  = ipa_test_read,
	.unlocked_ioctl = ipa_test_ioctl,
};

/**
 * Module Init.
 */
static int __init ipa_test_init(void)
{
	int ret;

	pr_debug(DRV_NAME ":SW Version = %s.\n", DRV_VERSION);

	ipa_test = kzalloc(sizeof(*ipa_test), GFP_KERNEL);
	if (ipa_test == NULL) {
		pr_err(DRV_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	ipa_test->signature = TEST_SIGNATURE;
	ipa_test->current_configuration_idx = -1;

	ipa_test_class = class_create(THIS_MODULE, DRV_NAME);

	ret = alloc_chrdev_region(&ipa_test->dev_num, 0, 1, DRV_NAME);
	if (ret) {
		pr_err(DRV_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	ipa_test->dev = device_create(ipa_test_class, NULL,
					ipa_test->dev_num,
				      ipa_test, DRV_NAME);
	if (IS_ERR(ipa_test->dev)) {
		pr_err(DRV_NAME ":device_create err.\n");
		return -ENODEV;
	}

	ipa_test->cdev = cdev_alloc();
	if (ipa_test->cdev == NULL) {
		pr_err(DRV_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}

	cdev_init(ipa_test->cdev, &ipa_test_fops);
	ipa_test->cdev->owner = THIS_MODULE;

	ret = cdev_add(ipa_test->cdev, ipa_test->dev_num, 1);
	if (ret)
		pr_err(DRV_NAME ":cdev_add err=%d\n", -ret);

	if (ret == 0)
		pr_debug(DRV_NAME
				":IPA Test init OK, waiting for configuration index.\n");
	else
		pr_debug(DRV_NAME ":IPA Test init FAIL.\n");

	ret = dma_set_mask(ipa_test->dev, DMA_BIT_MASK(32));
        if (ret){
                pr_err(DRV_NAME ":dma_set_mask failed: %d\n", ret);
        }

	ret = dma_set_coherent_mask(ipa_test->dev, DMA_BIT_MASK(32));
	if (ret){
		pr_err(DRV_NAME ":dma_set_coherent_mask failed: %d\n", ret);
	}

	ret = datapath_ds_init();
	if (ret != 0)
		pr_debug("[%s:%d, %s()] datapath_ds_init() failed (%d)\n",
				__FILE__, __LINE__
				, __func__, ret);
	return ret;
}

/**
 * Module Exit.
 */
static void __exit ipa_test_exit(void)
{
	pr_debug(DRV_NAME ":ipa_test_exit.\n");

	exception_hdl_exit(); /* Clear the Exception Device and KFIFO*/

	datapath_exit();

	destroy_channel_devices();

	cdev_del(ipa_test->cdev);
	device_destroy(ipa_test_class, ipa_test->dev_num);
	class_destroy(ipa_test_class);
	unregister_chrdev_region(ipa_test->dev_num, 1);

	kfree(ipa_test);

	pr_debug(DRV_NAME ":ipa_test_exit complete.\n");
}


module_init(ipa_test_init);
module_exit(ipa_test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IPA Testing");

