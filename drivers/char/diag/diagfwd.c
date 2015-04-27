/* Copyright (c) 2008-2015, The Linux Foundation. All rights reserved.
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
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/diagchar.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/kmemleak.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <linux/usb/usbdiag.h>
#endif
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smd.h>
#include <soc/qcom/restart.h>
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_cntl.h"
#include "diagchar_hdlc.h"
#include "diag_dci.h"
#include "diag_masks.h"
#include "diag_usb.h"
#include "diag_mux.h"

#define STM_CMD_VERSION_OFFSET	4
#define STM_CMD_MASK_OFFSET	5
#define STM_CMD_DATA_OFFSET	6
#define STM_CMD_NUM_BYTES	7

#define STM_RSP_SUPPORTED_INDEX		7
#define STM_RSP_SMD_STATUS_INDEX	8
#define STM_RSP_NUM_BYTES		9

#define SMD_DRAIN_BUF_SIZE 4096

int wrap_enabled;
uint16_t wrap_count;
static struct diag_hdlc_decode_type *hdlc_decode;

#define DIAG_NUM_COMMON_CMD	1
static uint8_t common_cmds[DIAG_NUM_COMMON_CMD] = {
	DIAG_CMD_LOG_ON_DMND
};

static uint8_t hdlc_timer_in_progress;

/* Determine if this device uses a device tree */
#ifdef CONFIG_OF
static int has_device_tree(void)
{
	struct device_node *node;

	node = of_find_node_by_path("/");
	if (node) {
		of_node_put(node);
		return 1;
	}
	return 0;
}
#else
static int has_device_tree(void)
{
	return 0;
}
#endif

int chk_config_get_id(void)
{
	switch (socinfo_get_msm_cpu()) {
	case MSM_CPU_8X60:
		return APQ8060_TOOLS_ID;
	case MSM_CPU_8960:
	case MSM_CPU_8960AB:
		return AO8960_TOOLS_ID;
	case MSM_CPU_8064:
	case MSM_CPU_8064AB:
	case MSM_CPU_8064AA:
		return APQ8064_TOOLS_ID;
	case MSM_CPU_8930:
	case MSM_CPU_8930AA:
	case MSM_CPU_8930AB:
		return MSM8930_TOOLS_ID;
	case MSM_CPU_8974:
		return MSM8974_TOOLS_ID;
	case MSM_CPU_8625:
		return MSM8625_TOOLS_ID;
	case MSM_CPU_8084:
		return APQ8084_TOOLS_ID;
	case MSM_CPU_8916:
		return MSM8916_TOOLS_ID;
	case MSM_CPU_8939:
		return MSM8939_TOOLS_ID;
	case MSM_CPU_8994:
		return MSM8994_TOOLS_ID;
	case MSM_CPU_8226:
		return APQ8026_TOOLS_ID;
	case MSM_CPU_8909:
		return MSM8909_TOOLS_ID;
	case MSM_CPU_8992:
		return MSM8992_TOOLS_ID;
	case MSM_CPU_8996:
		return MSM_8996_TOOLS_ID;
	case MSM_CPU_TELLURIUM:
		return MSMTELLURIUM_TOOLS_ID;
	default:
		if (driver->use_device_tree) {
			if (machine_is_msm8974())
				return MSM8974_TOOLS_ID;
			else if (machine_is_apq8074())
				return APQ8074_TOOLS_ID;
			else
				return 0;
		} else {
			return 0;
		}
	}
}

/*
 * This will return TRUE for targets which support apps only mode and hence SSR.
 * This applies to 8960 and newer targets.
 */
int chk_apps_only(void)
{
	if (driver->use_device_tree)
		return 1;

	switch (socinfo_get_msm_cpu()) {
	case MSM_CPU_8960:
	case MSM_CPU_8960AB:
	case MSM_CPU_8064:
	case MSM_CPU_8064AB:
	case MSM_CPU_8064AA:
	case MSM_CPU_8930:
	case MSM_CPU_8930AA:
	case MSM_CPU_8930AB:
	case MSM_CPU_8627:
	case MSM_CPU_9615:
	case MSM_CPU_8974:
		return 1;
	default:
		return 0;
	}
}

/*
 * This will return TRUE for targets which support apps as master.
 * Thus, SW DLOAD and Mode Reset are supported on apps processor.
 * This applies to 8960 and newer targets.
 */
int chk_apps_master(void)
{
	if (driver->use_device_tree)
		return 1;
	else if (soc_class_is_msm8960() || soc_class_is_msm8930() ||
		 soc_class_is_apq8064() || cpu_is_msm9615())
		return 1;
	else
		return 0;
}

int chk_polling_response(void)
{
	if (!(driver->polling_reg_flag) && chk_apps_master())
		/*
		 * If the apps processor is master and no other processor
		 * has registered to respond for polling
		 */
		return 1;
	else if (!((driver->smd_data[PERIPHERAL_MODEM].ch) &&
		 (driver->feature[PERIPHERAL_MODEM].rcvd_feature_mask)) &&
		 (chk_apps_master()))
		/*
		 * If the apps processor is not the master and the modem
		 * is not up or we did not receive the feature masks from Modem
		 */
		return 1;
	else
		return 0;
}

/*
 * This function should be called if you feel that the logging process may
 * need to be woken up. For instance, if the logging mode is MEMORY_DEVICE MODE
 * and while trying to read data from a SMD data channel there are no buffers
 * available to read the data into, then this function should be called to
 * determine if the logging process needs to be woken up.
 */
void chk_logging_wakeup(void)
{
	int i;

	/* Find the index of the logging process */
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid ==
		    driver->md_proc[DIAG_LOCAL_PROC].pid) {
			break;
		}

	if (i < driver->num_clients) {
		/* At very high logging rates a race condition can
		 * occur where the buffers containing the data from
		 * an smd channel are all in use, but the data_ready
		 * flag is cleared. In this case, the buffers never
		 * have their data read/logged.  Detect and remedy this
		 * situation.
		 */
		if ((driver->data_ready[i] & USER_SPACE_DATA_TYPE) == 0) {
			driver->data_ready[i] |= USER_SPACE_DATA_TYPE;
			pr_debug("diag: Force wakeup of logging process\n");
			wake_up_interruptible(&driver->wait_q);
		}
	}
}
int diag_add_hdlc_encoding(struct diag_smd_info *smd_info, void *buf,
			   int total_recd, uint8_t *encode_buf,
			   int *encoded_length)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct data_header {
		uint8_t control_char;
		uint8_t version;
		uint16_t length;
	};
	struct data_header *header;
	int header_size = sizeof(struct data_header);
	uint8_t *end_control_char;
	uint8_t *payload;
	uint8_t *temp_buf;
	uint8_t *temp_encode_buf;
	int src_pkt_len;
	int encoded_pkt_length;
	int max_size;
	int total_processed = 0;
	int bytes_remaining;
	int success = 1;

	temp_buf = buf;
	temp_encode_buf = encode_buf;
	bytes_remaining = *encoded_length;
	while (total_processed < total_recd) {
		header = (struct data_header *)temp_buf;
		/* Perform initial error checking */
		if (header->control_char != CONTROL_CHAR ||
			header->version != 1) {
			success = 0;
			break;
		}
		payload = temp_buf + header_size;
		end_control_char = payload + header->length;
		if (*end_control_char != CONTROL_CHAR) {
			success = 0;
			break;
		}

		max_size = 2 * header->length + 3;
		if (bytes_remaining < max_size) {
			pr_err("diag: In %s, Not enough room to encode remaining data for peripheral: %d, bytes available: %d, max_size: %d\n",
				__func__, smd_info->peripheral,
				bytes_remaining, max_size);
			success = 0;
			break;
		}

		/* Prepare for encoding the data */
		send.state = DIAG_STATE_START;
		send.pkt = payload;
		send.last = (void *)(payload + header->length - 1);
		send.terminate = 1;

		enc.dest = temp_encode_buf;
		enc.dest_last = (void *)(temp_encode_buf + max_size);
		enc.crc = 0;
		diag_hdlc_encode(&send, &enc);

		/* Prepare for next packet */
		src_pkt_len = (header_size + header->length + 1);
		total_processed += src_pkt_len;
		temp_buf += src_pkt_len;

		encoded_pkt_length = (uint8_t *)enc.dest - temp_encode_buf;
		bytes_remaining -= encoded_pkt_length;
		temp_encode_buf = enc.dest;
	}

	*encoded_length = (int)(temp_encode_buf - encode_buf);

	return success;
}

static int check_bufsize_for_encoding(struct diag_smd_info *smd_info, void *buf,
					int total_recd)
{
	int buf_size = IN_BUF_SIZE;
	int max_size = 2 * total_recd + 3;
	unsigned char *temp_buf;

	if (max_size > IN_BUF_SIZE) {
		if (max_size > MAX_IN_BUF_SIZE) {
			pr_err_ratelimited("diag: In %s, SMD sending packet of %d bytes that may expand to %d bytes, peripheral: %d\n",
				__func__, total_recd, max_size,
				smd_info->peripheral);
			max_size = MAX_IN_BUF_SIZE;
		}
		if (buf == smd_info->buf_in_1_raw) {
			/* Only realloc if we need to increase the size */
			if (smd_info->buf_in_1_size < max_size) {
				temp_buf = krealloc(smd_info->buf_in_1,
					max_size, GFP_KERNEL);
				if (temp_buf) {
					smd_info->buf_in_1 = temp_buf;
					smd_info->buf_in_1_size = max_size;
				} else {
					return -ENOMEM;
				}
			}
			buf_size = smd_info->buf_in_1_size;
		} else {
			/* Only realloc if we need to increase the size */
			if (smd_info->buf_in_2_size < max_size) {
				temp_buf = krealloc(smd_info->buf_in_2,
					max_size, GFP_KERNEL);
				if (temp_buf) {
					smd_info->buf_in_2 = temp_buf;
					smd_info->buf_in_2_size = max_size;
				} else {
					return -ENOMEM;
				}
			}
			buf_size = smd_info->buf_in_2_size;
		}
	}

	return buf_size;
}

/* Process the data read from the smd data channel */
int diag_process_smd_read_data(struct diag_smd_info *smd_info, void *buf,
			       int total_recd)
{
	int *in_busy_ptr = 0;
	int err = 0;
	int success = 0;
	int write_length = total_recd;
	int ctxt = 0;
	unsigned char *write_buf = NULL;

	unsigned long flags;

	/*
	 * Do not process data on command channel if the
	 * channel is not designated to do so
	 */
	if ((smd_info->type == TYPE_CMD) &&
	    !driver->feature[smd_info->peripheral].separate_cmd_rsp) {
		pr_debug("diag, In %s, received data on non-designated command channel: %d\n",
			__func__, smd_info->peripheral);
		return 0;
	}

	mutex_lock(&driver->hdlc_disable_mutex);

	if (!driver->feature[smd_info->peripheral].encode_hdlc) {
		/* If the data is already hdlc encoded*/
		if (smd_info->buf_in_1 == buf) {
			in_busy_ptr = &smd_info->in_busy_1;
			ctxt = smd_info->buf_in_1_ctxt;
		} else if (smd_info->buf_in_2 == buf) {
			in_busy_ptr = &smd_info->in_busy_2;
			ctxt = smd_info->buf_in_2_ctxt;
		} else {
			pr_err("diag: In %s, no match for in_busy_1, peripheral: %d\n",
				__func__, smd_info->peripheral);
			err = -EIO;
			goto end;
		}
		write_buf = buf;
		success = 1;
	} else if (driver->hdlc_disabled) {
		/* The data is raw and and on APPS side HDLC is disabled */
		if (smd_info->buf_in_1_raw == buf) {
			write_buf = smd_info->buf_in_1;
			in_busy_ptr = &smd_info->in_busy_1;
			ctxt = smd_info->buf_in_1_ctxt;
		} else if (smd_info->buf_in_2_raw == buf) {
			write_buf = smd_info->buf_in_2;
			in_busy_ptr = &smd_info->in_busy_2;
			ctxt = smd_info->buf_in_2_ctxt;
		} else {
			pr_err("diag: In %s, no match for in_busy_1, peripheral: %d\n",
				__func__, smd_info->peripheral);
			err = -EIO;
			goto end;
		}

		if (total_recd > IN_BUF_SIZE) {
			pr_err("diag: In %s, incoming data too large, total_recd: %d\n",
					__func__, total_recd);
			err = -ENOMEM;
			goto end;
		}
		write_length = total_recd;
		write_buf = buf;
		success = 1;
	} else {
		/* The data is raw and needs to be hdlc encoded */
		write_length = check_bufsize_for_encoding(smd_info, buf,
						  total_recd);
		if (write_length < 0) {
			mutex_unlock(&driver->hdlc_disable_mutex);
			return write_length;
		}
		if (smd_info->buf_in_1_raw == buf) {
			write_buf = smd_info->buf_in_1;
			in_busy_ptr = &smd_info->in_busy_1;
			ctxt = smd_info->buf_in_1_ctxt;
		} else if (smd_info->buf_in_2_raw == buf) {
			write_buf = smd_info->buf_in_2;
			in_busy_ptr = &smd_info->in_busy_2;
			ctxt = smd_info->buf_in_2_ctxt;
		} else {
			pr_err("diag: In %s, no match for in_busy_1, peripheral: %d\n",
				__func__, smd_info->peripheral);
			err = -EIO;
			goto end;
		}
		success = diag_add_hdlc_encoding(smd_info, buf,
						total_recd, write_buf,
						&write_length);
	}

	if (!success) {
		pr_err_ratelimited("diag: smd data write unsuccessful, success: %d\n",
				   success);
		goto end;
	}

	if (write_length > 0) {
		spin_lock_irqsave(&smd_info->in_busy_lock, flags);
		*in_busy_ptr = 1;
		err = diag_mux_write(DIAG_LOCAL_PROC, write_buf, write_length,
				     ctxt);
		if (err) {
			pr_err_ratelimited("diag: In %s, diag_device_write error: %d\n",
					   __func__, err);
		}
		spin_unlock_irqrestore(&smd_info->in_busy_lock, flags);
	}

end:
	mutex_unlock(&driver->hdlc_disable_mutex);
	return err;
}

static int diag_smd_resize_buf(struct diag_smd_info *smd_info, void **buf,
			       unsigned int *buf_size,
			       unsigned int requested_size)
{
	int success = 0;
	void *temp_buf = NULL;
	unsigned int new_buf_size = requested_size;

	if (!smd_info)
		return success;

	if (requested_size <= MAX_IN_BUF_SIZE) {
		pr_debug("diag: In %s, SMD peripheral: %d sending in packets up to %d bytes\n",
			__func__, smd_info->peripheral, requested_size);
	} else {
		pr_err_ratelimited("diag: In %s, SMD peripheral: %d, Packet size sent: %d, Max size supported (%d) exceeded. Data beyond max size will be lost\n",
			__func__, smd_info->peripheral, requested_size,
			MAX_IN_BUF_SIZE);
		new_buf_size = MAX_IN_BUF_SIZE;
	}

	/* Only resize if the buffer can be increased in size */
	if (new_buf_size <= *buf_size) {
		success = 1;
		return success;
	}

	temp_buf = krealloc(*buf, new_buf_size, GFP_KERNEL);

	if (temp_buf) {
		/* Match the buffer and reset the pointer and size */
		if (driver->feature[smd_info->peripheral].encode_hdlc) {
			/*
			 * This smd channel is supporting HDLC encoding
			 * on the apps
			 */
			void *temp_hdlc = NULL;
			if (*buf == smd_info->buf_in_1_raw) {
				smd_info->buf_in_1_raw = temp_buf;
				smd_info->buf_in_1_raw_size = new_buf_size;
				temp_hdlc = krealloc(smd_info->buf_in_1,
							MAX_IN_BUF_SIZE,
							GFP_KERNEL);
				if (temp_hdlc) {
					smd_info->buf_in_1 = temp_hdlc;
					smd_info->buf_in_1_size =
							MAX_IN_BUF_SIZE;
				}
			} else if (*buf == smd_info->buf_in_2_raw) {
				smd_info->buf_in_2_raw = temp_buf;
				smd_info->buf_in_2_raw_size = new_buf_size;
				temp_hdlc = krealloc(smd_info->buf_in_2,
							MAX_IN_BUF_SIZE,
							GFP_KERNEL);
				if (temp_hdlc) {
					smd_info->buf_in_2 = temp_hdlc;
					smd_info->buf_in_2_size =
							MAX_IN_BUF_SIZE;
				}
			}
		} else {
			if (*buf == smd_info->buf_in_1) {
				smd_info->buf_in_1 = temp_buf;
				smd_info->buf_in_1_size = new_buf_size;
			} else if (*buf == smd_info->buf_in_2) {
				smd_info->buf_in_2 = temp_buf;
				smd_info->buf_in_2_size = new_buf_size;
			}
		}
		*buf = temp_buf;
		*buf_size = new_buf_size;
		success = 1;
	} else {
		pr_err_ratelimited("diag: In %s, SMD peripheral: %d. packet size sent: %d, resize to support failed. Data beyond %d will be lost\n",
			__func__, smd_info->peripheral, requested_size,
			*buf_size);
	}

	return success;
}

void diag_smd_send_req(struct diag_smd_info *smd_info)
{
	void *buf = NULL, *temp_buf = NULL;
	int total_recd = 0, r = 0, pkt_len;
	int loop_count = 0, total_recd_partial = 0;
	int notify = 0;
	int buf_size = 0;
	int resize_success = 0;
	int buf_full = 0;

	if (!smd_info) {
		pr_err("diag: In %s, no smd info. Not able to read.\n",
			__func__);
		return;
	}

	/* Determine the buffer to read the data into. */
	if (smd_info->type == TYPE_DATA) {
		/* If the data is raw and not hdlc encoded */
		if (driver->feature[smd_info->peripheral].encode_hdlc) {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1_raw;
				buf_size = smd_info->buf_in_1_raw_size;
			} else if (!smd_info->in_busy_2) {
				buf = smd_info->buf_in_2_raw;
				buf_size = smd_info->buf_in_2_raw_size;
			}
		} else {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1;
				buf_size = smd_info->buf_in_1_size;
			} else if (!smd_info->in_busy_2) {
				buf = smd_info->buf_in_2;
				buf_size = smd_info->buf_in_2_size;
			}
		}
	} else if (smd_info->type == TYPE_CMD) {
		/* If the data is raw and not hdlc encoded */
		if (driver->feature[smd_info->peripheral].encode_hdlc) {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1_raw;
				buf_size = smd_info->buf_in_1_raw_size;
			}
		} else {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1;
				buf_size = smd_info->buf_in_1_size;
			}
		}
	} else if (!smd_info->in_busy_1) {
		buf = smd_info->buf_in_1;
		buf_size = smd_info->buf_in_1_size;
	}

	if (!buf)
		goto fail_return;

	if (smd_info->ch && buf) {
		int required_size = 0;
		while ((pkt_len = smd_cur_packet_size(smd_info->ch)) != 0) {
			total_recd_partial = 0;

		required_size = pkt_len + total_recd;
		if (required_size > buf_size)
			resize_success = diag_smd_resize_buf(smd_info, &buf,
						&buf_size, required_size);

		temp_buf = ((unsigned char *)buf) + total_recd;
		while (pkt_len && (pkt_len != total_recd_partial)) {
			loop_count++;
			r = smd_read_avail(smd_info->ch);
			pr_debug("diag: In %s, SMD peripheral: %d, received pkt %d %d\n",
				__func__, smd_info->peripheral, r, total_recd);
			if (!r) {
				/* Nothing to read from SMD */
				wait_event(driver->smd_wait_q,
					((smd_info->ch == 0) ||
					smd_read_avail(smd_info->ch)));
				/* If the smd channel is open */
				if (smd_info->ch) {
					pr_debug("diag: In %s, SMD peripheral: %d, return from wait_event\n",
						__func__, smd_info->peripheral);
					continue;
				} else {
					pr_debug("diag: In %s, SMD peripheral: %d, return from wait_event ch closed\n",
						__func__, smd_info->peripheral);
					goto fail_return;
				}
			}

			if (pkt_len < r) {
				pr_err("diag: In %s, SMD peripheral: %d, sending incorrect pkt\n",
					__func__, smd_info->peripheral);
				goto fail_return;
			}
			if (pkt_len > r) {
				pr_debug("diag: In %s, SMD sending partial pkt %d %d %d %d %d %d\n",
				__func__, pkt_len, r, total_recd, loop_count,
				smd_info->peripheral, smd_info->type);
			}

			/* Protect from going beyond the end of the buffer */
			if (total_recd < buf_size) {
				if (total_recd + r > buf_size) {
					r = buf_size - total_recd;
					buf_full = 1;
				}

				total_recd += r;
				total_recd_partial += r;

				/* Keep reading for complete packet */
				smd_read(smd_info->ch, temp_buf, r);
				temp_buf += r;
			} else {
				/*
				 * This block handles the very rare case of a
				 * packet that is greater in length than what
				 * we can support. In this case, we
				 * incrementally drain the remaining portion
				 * of the packet that will not fit in the
				 * buffer, so that the entire packet is read
				 * from the smd.
				 */
				int drain_bytes = (r > SMD_DRAIN_BUF_SIZE) ?
							SMD_DRAIN_BUF_SIZE : r;
				unsigned char *drain_buf = kzalloc(drain_bytes,
								GFP_KERNEL);
				if (drain_buf) {
					total_recd += drain_bytes;
					total_recd_partial += drain_bytes;
					smd_read(smd_info->ch, drain_buf,
							drain_bytes);
					kfree(drain_buf);
				} else {
					pr_err("diag: In %s, SMD peripheral: %d, unable to allocate drain buffer\n",
						__func__, smd_info->peripheral);
					break;
				}
			}
		}

		if ((smd_info->type != TYPE_CNTL &&
				smd_info->type != TYPE_CMD)
					|| buf_full)
			break;

		}

		if (total_recd > 0) {
			if (!buf) {
				pr_err("diag: In %s, SMD peripheral: %d, Out of diagmem for Modem\n",
					__func__, smd_info->peripheral);
			} else if (smd_info->process_smd_read_data) {
				/*
				 * If the buffer was totally filled, reset
				 * total_recd appropriately
				 */
				if (buf_full)
					total_recd = buf_size;

				notify = smd_info->process_smd_read_data(
						smd_info, buf, total_recd);
				/* Poll SMD channels to check for data */
				if (notify)
					diag_smd_notify(smd_info,
							SMD_EVENT_DATA);
			}
		} else {
			goto fail_return;
		}
	} else if (smd_info->ch && !buf &&
		(driver->logging_mode == MEMORY_DEVICE_MODE)) {
			chk_logging_wakeup();
	}
	return;

fail_return:
	if (smd_info->type == TYPE_DCI ||
	    smd_info->type == TYPE_DCI_CMD ||
	    driver->logging_mode == MEMORY_DEVICE_MODE)
		diag_ws_release();
	return;
}

void diag_read_smd_work_fn(struct work_struct *work)
{
	struct diag_smd_info *smd_info = container_of(work,
							struct diag_smd_info,
							diag_read_smd_work);
	diag_smd_send_req(smd_info);
}

static void pack_rsp_and_send(unsigned char *buf, int len)
{
	int err;
	int retry_count = 0;
	uint32_t write_len = 0;
	unsigned long flags;
	unsigned char *rsp_ptr = driver->encoded_rsp_buf;
	struct diag_pkt_frame_t header;

	if (!rsp_ptr || !buf)
		return;

	if (len > DIAG_MAX_RSP_SIZE || len < 0) {
		pr_err("diag: In %s, invalid len %d, permissible len %d\n",
		       __func__, len, DIAG_MAX_RSP_SIZE);
		return;
	}

	/*
	 * Keep trying till we get the buffer back. It should probably
	 * take one or two iterations. When this loops till UINT_MAX, it
	 * means we did not get a write complete for the previous
	 * response.
	 */
	while (retry_count < UINT_MAX) {
		if (!driver->rsp_buf_busy)
			break;
		/*
		 * Wait for sometime and try again. The value 10000 was chosen
		 * empirically as an optimum value for USB to complete a write
		 */
		usleep_range(10000, 10100);
		retry_count++;

		/*
		 * There can be a race conditon that clears the data ready flag
		 * for responses. Make sure we don't miss previous wakeups for
		 * draining responses when we are in Memory Device Mode.
		 */
		if (driver->logging_mode == MEMORY_DEVICE_MODE)
			chk_logging_wakeup();
	}
	if (driver->rsp_buf_busy) {
		pr_err("diag: unable to get hold of response buffer\n");
		return;
	}

	driver->rsp_buf_busy = 1;
	header.start = CONTROL_CHAR;
	header.version = 1;
	header.length = len;
	memcpy(rsp_ptr, &header, sizeof(header));
	write_len += sizeof(header);
	memcpy(rsp_ptr + write_len, buf, len);
	write_len += len;
	*(uint8_t *)(rsp_ptr + write_len) = CONTROL_CHAR;
	write_len += sizeof(uint8_t);

	err = diag_mux_write(DIAG_LOCAL_PROC, rsp_ptr, write_len,
			     driver->rsp_buf_ctxt);
	if (err) {
		pr_err("diag: In %s, unable to write to mux, err: %d\n",
		       __func__, err);
		spin_lock_irqsave(&driver->rsp_buf_busy_lock, flags);
		driver->rsp_buf_busy = 0;
		spin_unlock_irqrestore(&driver->rsp_buf_busy_lock, flags);
	}
}

static void encode_rsp_and_send(unsigned char *buf, int len)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	unsigned char *rsp_ptr = driver->encoded_rsp_buf;
	int err, retry_count = 0;
	unsigned long flags;

	if (!rsp_ptr || !buf)
		return;

	if (len > DIAG_MAX_RSP_SIZE || len < 0) {
		pr_err("diag: In %s, invalid len %d, permissible len %d\n",
		       __func__, len, DIAG_MAX_RSP_SIZE);
		return;
	}

	/*
	 * Keep trying till we get the buffer back. It should probably
	 * take one or two iterations. When this loops till UINT_MAX, it
	 * means we did not get a write complete for the previous
	 * response.
	 */
	while (retry_count < UINT_MAX) {
		if (!driver->rsp_buf_busy)
			break;
		/*
		 * Wait for sometime and try again. The value 10000 was chosen
		 * empirically as an optimum value for USB to complete a write
		 */
		usleep_range(10000, 10100);
		retry_count++;

		/*
		 * There can be a race conditon that clears the data ready flag
		 * for responses. Make sure we don't miss previous wakeups for
		 * draining responses when we are in Memory Device Mode.
		 */
		if (driver->logging_mode == MEMORY_DEVICE_MODE)
			chk_logging_wakeup();
	}

	if (driver->rsp_buf_busy) {
		pr_err("diag: unable to get hold of response buffer\n");
		return;
	}

	spin_lock_irqsave(&driver->rsp_buf_busy_lock, flags);
	driver->rsp_buf_busy = 1;
	spin_unlock_irqrestore(&driver->rsp_buf_busy_lock, flags);
	send.state = DIAG_STATE_START;
	send.pkt = buf;
	send.last = (void *)(buf + len - 1);
	send.terminate = 1;
	enc.dest = rsp_ptr;
	enc.dest_last = (void *)(rsp_ptr + DIAG_MAX_HDLC_BUF_SIZE - 1);
	diag_hdlc_encode(&send, &enc);
	driver->encoded_rsp_len = (int)(enc.dest - (void *)rsp_ptr);
	err = diag_mux_write(DIAG_LOCAL_PROC, rsp_ptr, driver->encoded_rsp_len,
			     driver->rsp_buf_ctxt);
	if (err) {
		pr_err("diag: In %s, Unable to write to device, err: %d\n",
			__func__, err);
		spin_lock_irqsave(&driver->rsp_buf_busy_lock, flags);
		driver->rsp_buf_busy = 0;
		spin_unlock_irqrestore(&driver->rsp_buf_busy_lock, flags);
	}
	memset(buf, '\0', DIAG_MAX_RSP_SIZE);
}

void diag_send_rsp(unsigned char *buf, int len)
{
	if (driver->hdlc_disabled)
		pack_rsp_and_send(buf, len);
	else
		encode_rsp_and_send(buf, len);
}

void diag_update_pkt_buffer(unsigned char *buf, uint32_t len, int type)
{
	unsigned char *ptr = NULL;
	unsigned char *temp = buf;
	int *in_busy = NULL;
	uint32_t *length = NULL;
	uint32_t max_len = 0;

	if (!buf || len == 0) {
		pr_err("diag: In %s, Invalid ptr %p and length %d\n",
		       __func__, buf, len);
		return;
	}

	switch (type) {
	case PKT_TYPE:
		ptr = driver->apps_req_buf;
		length = &driver->apps_req_buf_len;
		max_len = DIAG_MAX_REQ_SIZE;
		in_busy = &driver->in_busy_pktdata;
		break;
	case DCI_PKT_TYPE:
		ptr = driver->dci_pkt_buf;
		length = &driver->dci_pkt_length;
		max_len = DCI_BUF_SIZE;
		in_busy = &driver->in_busy_dcipktdata;
		break;
	default:
		pr_err("diag: Invalid type %d in %s\n", type, __func__);
		return;
	}

	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + max_len, len)) {
		memcpy(ptr, temp , len);
		*length = len;
		*in_busy = 1;
	} else {
		pr_alert("diag: In %s, no space for response packet, len: %d, type: %d\n",
			 __func__, len, type);
	}
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_userspace_clients(unsigned int type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid != 0)
			driver->data_ready[i] |= type;
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_sleeping_process(int process_id, int data_type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid == process_id) {
			driver->data_ready[i] |= data_type;
			break;
		}
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

static int diag_send_data(struct diag_cmd_reg_t *entry, unsigned char *buf,
			  int len)
{
	int err = 0;
	int peripheral;
	struct diag_smd_info *smd_info = NULL;

	if (!entry)
		return -EIO;

	if (entry->proc == APPS_DATA) {
		diag_update_pkt_buffer(buf, len, PKT_TYPE);
		diag_update_sleeping_process(entry->pid, PKT_TYPE);
		return 0;
	}

	peripheral = entry->proc;
	if (peripheral >= NUM_PERIPHERALS)
		return -EINVAL;

	if (!driver->feature[peripheral].rcvd_feature_mask) {
		pr_debug("diag: In %s, yet to receive feature mask from %d\n",
			 __func__, peripheral);
		return -ENODEV;
	}

	if (driver->feature[peripheral].separate_cmd_rsp)
		smd_info = &driver->smd_cmd[peripheral];
	else
		smd_info = &driver->smd_data[peripheral];

	err = diag_smd_write(smd_info, buf, len);
	if (err) {
		pr_err_ratelimited("diag: In %s, unable to write to smd, peripheral: %d, type: %d, err: %d\n",
				   __func__, smd_info->peripheral,
				   smd_info->type, err);
	}

	return err;
}

void diag_process_stm_mask(uint8_t cmd, uint8_t data_mask, int data_type)
{
	int status = 0;
	if (data_type >= PERIPHERAL_MODEM && data_type <= PERIPHERAL_SENSORS) {
		if (driver->feature[data_type].stm_support) {
			status = diag_send_stm_state(
				&driver->smd_cntl[data_type], cmd);
			if (status == 1)
				driver->stm_state[data_type] = cmd;
		}
		driver->stm_state_requested[data_type] = cmd;
	} else if (data_type == APPS_DATA) {
		driver->stm_state[data_type] = cmd;
		driver->stm_state_requested[data_type] = cmd;
	}
}

int diag_process_stm_cmd(unsigned char *buf, unsigned char *dest_buf)
{
	uint8_t version, mask, cmd;
	uint8_t rsp_supported = 0;
	uint8_t rsp_smd_status = 0;
	int i;

	if (!buf || !dest_buf) {
		pr_err("diag: Invalid pointers buf: %p, dest_buf %p in %s\n",
		       buf, dest_buf, __func__);
		return -EIO;
	}

	version = *(buf + STM_CMD_VERSION_OFFSET);
	mask = *(buf + STM_CMD_MASK_OFFSET);
	cmd = *(buf + STM_CMD_DATA_OFFSET);

	/*
	 * Check if command is valid. If the command is asking for
	 * status, then the processor mask field is to be ignored.
	 */
	if ((version != 2) || (cmd > STATUS_STM) ||
		((cmd != STATUS_STM) && ((mask == 0) || (0 != (mask >> 4))))) {
		/* Command is invalid. Send bad param message response */
		dest_buf[0] = BAD_PARAM_RESPONSE_MESSAGE;
		for (i = 0; i < STM_CMD_NUM_BYTES; i++)
			dest_buf[i+1] = *(buf + i);
		return STM_CMD_NUM_BYTES+1;
	} else if (cmd != STATUS_STM) {
		if (mask & DIAG_STM_MODEM)
			diag_process_stm_mask(cmd, DIAG_STM_MODEM,
					      PERIPHERAL_MODEM);

		if (mask & DIAG_STM_LPASS)
			diag_process_stm_mask(cmd, DIAG_STM_LPASS,
					      PERIPHERAL_LPASS);

		if (mask & DIAG_STM_WCNSS)
			diag_process_stm_mask(cmd, DIAG_STM_WCNSS,
					      PERIPHERAL_WCNSS);

		if (mask & DIAG_STM_SENSORS)
			diag_process_stm_mask(cmd, DIAG_STM_SENSORS,
						PERIPHERAL_SENSORS);

		if (mask & DIAG_STM_APPS)
			diag_process_stm_mask(cmd, DIAG_STM_APPS, APPS_DATA);
	}

	for (i = 0; i < STM_CMD_NUM_BYTES; i++)
		dest_buf[i] = *(buf + i);

	/* Set mask denoting which peripherals support STM */
	if (driver->feature[PERIPHERAL_MODEM].stm_support)
		rsp_supported |= DIAG_STM_MODEM;

	if (driver->feature[PERIPHERAL_LPASS].stm_support)
		rsp_supported |= DIAG_STM_LPASS;

	if (driver->feature[PERIPHERAL_WCNSS].stm_support)
		rsp_supported |= DIAG_STM_WCNSS;

	if (driver->feature[PERIPHERAL_SENSORS].stm_support)
		rsp_supported |= DIAG_STM_SENSORS;

	rsp_supported |= DIAG_STM_APPS;

	/* Set mask denoting STM state/status for each peripheral/APSS */
	if (driver->stm_state[PERIPHERAL_MODEM])
		rsp_smd_status |= DIAG_STM_MODEM;

	if (driver->stm_state[PERIPHERAL_LPASS])
		rsp_smd_status |= DIAG_STM_LPASS;

	if (driver->stm_state[PERIPHERAL_WCNSS])
		rsp_smd_status |= DIAG_STM_WCNSS;

	if (driver->stm_state[PERIPHERAL_SENSORS])
		rsp_smd_status |= DIAG_STM_SENSORS;

	if (driver->stm_state[APPS_DATA])
		rsp_smd_status |= DIAG_STM_APPS;

	dest_buf[STM_RSP_SUPPORTED_INDEX] = rsp_supported;
	dest_buf[STM_RSP_SMD_STATUS_INDEX] = rsp_smd_status;

	return STM_RSP_NUM_BYTES;
}

int diag_cmd_log_on_demand(unsigned char *src_buf, int src_len,
			   unsigned char *dest_buf, int dest_len)
{
	int write_len = 0;
	struct diag_log_on_demand_rsp_t header;

	if (driver->smd_cntl[PERIPHERAL_MODEM].ch &&
	    !driver->log_on_demand_support)
		return 0;

	if (!src_buf || !dest_buf || src_len <= 0 || dest_len <= 0) {
		pr_err("diag: Invalid input in %s, src_buf: %p, src_len: %d, dest_buf: %p, dest_len: %d",
		       __func__, src_buf, src_len, dest_buf, dest_len);
		return -EINVAL;
	}

	header.cmd_code = DIAG_CMD_LOG_ON_DMND;
	header.log_code = *(uint16_t *)(src_buf + 1);
	header.status = 1;
	memcpy(dest_buf, &header, sizeof(struct diag_log_on_demand_rsp_t));
	write_len += sizeof(struct diag_log_on_demand_rsp_t);

	return write_len;
}

int diag_cmd_get_mobile_id(unsigned char *src_buf, int src_len,
			   unsigned char *dest_buf, int dest_len)
{
	int write_len = 0;
	struct diag_pkt_header_t *header = NULL;
	struct diag_cmd_ext_mobile_rsp_t rsp;

	if (!src_buf || src_len != sizeof(*header) || !dest_buf ||
	    dest_len < sizeof(rsp))
		return -EIO;

	header = (struct diag_pkt_header_t *)src_buf;
	rsp.header.cmd_code = header->cmd_code;
	rsp.header.subsys_id = header->subsys_id;
	rsp.header.subsys_cmd_code = header->subsys_cmd_code;
	rsp.version = 1;
	rsp.padding[0] = 0;
	rsp.padding[1] = 0;
	rsp.padding[2] = 0;
	rsp.family = (uint32_t)socinfo_get_msm_cpu();

	memcpy(dest_buf, &rsp, sizeof(rsp));
	write_len += sizeof(rsp);

	return write_len;
}

int diag_check_common_cmd(struct diag_pkt_header_t *header)
{
	int i;

	if (!header)
		return -EIO;

	for (i = 0; i < DIAG_NUM_COMMON_CMD; i++) {
		if (header->cmd_code == common_cmds[i])
			return 1;
	}

	return 0;
}

static int diag_cmd_chk_stats(unsigned char *src_buf, int src_len,
			      unsigned char *dest_buf, int dest_len)
{
	int payload = 0;
	int write_len = 0;
	struct diag_pkt_header_t *header = NULL;
	struct diag_cmd_stats_rsp_t rsp;

	if (!src_buf || src_len < sizeof(struct diag_pkt_header_t) ||
	    !dest_buf || dest_len < sizeof(rsp))
		return -EINVAL;

	header = (struct diag_pkt_header_t *)src_buf;

	if (header->cmd_code != DIAG_CMD_DIAG_SUBSYS ||
	    header->subsys_id != DIAG_SS_DIAG)
		return -EINVAL;

	switch (header->subsys_cmd_code) {
	case DIAG_CMD_OP_GET_MSG_ALLOC:
		payload = driver->msg_stats.alloc_count;
		break;
	case DIAG_CMD_OP_GET_MSG_DROP:
		payload = driver->msg_stats.drop_count;
		break;
	case DIAG_CMD_OP_RESET_MSG_STATS:
		diag_record_stats(DATA_TYPE_F3, PKT_RESET);
		break;
	case DIAG_CMD_OP_GET_LOG_ALLOC:
		payload = driver->log_stats.alloc_count;
		break;
	case DIAG_CMD_OP_GET_LOG_DROP:
		payload = driver->log_stats.drop_count;
		break;
	case DIAG_CMD_OP_RESET_LOG_STATS:
		diag_record_stats(DATA_TYPE_LOG, PKT_RESET);
		break;
	case DIAG_CMD_OP_GET_EVENT_ALLOC:
		payload = driver->event_stats.alloc_count;
		break;
	case DIAG_CMD_OP_GET_EVENT_DROP:
		payload = driver->event_stats.drop_count;
		break;
	case DIAG_CMD_OP_RESET_EVENT_STATS:
		diag_record_stats(DATA_TYPE_EVENT, PKT_RESET);
		break;
	default:
		return -EINVAL;
	}

	memcpy(&rsp.header, header, sizeof(struct diag_pkt_header_t));
	rsp.payload = payload;
	write_len = sizeof(rsp);
	memcpy(dest_buf, &rsp, sizeof(rsp));

	return write_len;
}

static int diag_cmd_disable_hdlc(unsigned char *src_buf, int src_len,
				 unsigned char *dest_buf, int dest_len)
{
	struct diag_pkt_header_t *header = NULL;
	struct diag_cmd_hdlc_disable_rsp_t rsp;
	int write_len = 0;

	if (!src_buf || src_len < sizeof(*header) ||
	    !dest_buf || dest_len < sizeof(rsp)) {
		return -EIO;
	}

	header = (struct diag_pkt_header_t *)src_buf;
	if (header->cmd_code != DIAG_CMD_DIAG_SUBSYS ||
	    header->subsys_id != DIAG_SS_DIAG ||
	    header->subsys_cmd_code != DIAG_CMD_OP_HDLC_DISABLE) {
		return -EINVAL;
	}

	memcpy(&rsp.header, header, sizeof(struct diag_pkt_header_t));
	rsp.framing_version = 1;
	rsp.result = 0;
	write_len = sizeof(rsp);
	memcpy(dest_buf, &rsp, sizeof(rsp));

	return write_len;
}

static void diag_send_error_rsp(unsigned char *buf, int len)
{
	/* -1 to accomodate the first byte 0x13 */
	if (len > (DIAG_MAX_RSP_SIZE - 1)) {
		pr_err("diag: cannot send err rsp, huge length: %d\n", len);
		return;
	}

	*(uint8_t *)driver->apps_rsp_buf = DIAG_CMD_ERROR;
	memcpy((driver->apps_rsp_buf + sizeof(uint8_t)), buf, len);
	diag_send_rsp(driver->apps_rsp_buf, len + 1);
}

int diag_process_apps_pkt(unsigned char *buf, int len)
{
	int i;
	int mask_ret;
	int write_len = 0;
	unsigned char *temp = NULL;
	struct diag_cmd_reg_entry_t entry;
	struct diag_cmd_reg_entry_t *temp_entry = NULL;
	struct diag_cmd_reg_t *reg_item = NULL;

	if (!buf)
		return -EIO;

	/* Check if the command is a supported mask command */
	mask_ret = diag_process_apps_masks(buf, len);
	if (mask_ret > 0) {
		diag_send_rsp(driver->apps_rsp_buf, mask_ret);
		return 0;
	}

	temp = buf;
	entry.cmd_code = (uint16_t)(*(uint8_t *)temp);
	temp += sizeof(uint8_t);
	entry.subsys_id = (uint16_t)(*(uint8_t *)temp);
	temp += sizeof(uint8_t);
	entry.cmd_code_hi = (uint16_t)(*(uint16_t *)temp);
	entry.cmd_code_lo = (uint16_t)(*(uint16_t *)temp);
	temp += sizeof(uint16_t);

	pr_debug("diag: In %s, received cmd %02x %02x %02x\n",
		 __func__, entry.cmd_code, entry.subsys_id, entry.cmd_code_hi);

	temp_entry = diag_cmd_search(&entry, ALL_PROC);
	if (temp_entry) {
		reg_item = container_of(temp_entry, struct diag_cmd_reg_t,
								entry);
		return diag_send_data(reg_item, buf, len);
	}

#if defined(CONFIG_DIAG_OVER_USB)
	/* Check for the command/respond msg for the maximum packet length */
	if ((*buf == 0x4b) && (*(buf+1) == 0x12) &&
		(*(uint16_t *)(buf+2) == 0x0055)) {
		for (i = 0; i < 4; i++)
			*(driver->apps_rsp_buf+i) = *(buf+i);
		*(uint32_t *)(driver->apps_rsp_buf+4) = DIAG_MAX_REQ_SIZE;
		diag_send_rsp(driver->apps_rsp_buf, 8);
		return 0;
	} else if ((*buf == 0x4b) && (*(buf+1) == 0x12) &&
		(*(uint16_t *)(buf+2) == DIAG_DIAG_STM)) {
		len = diag_process_stm_cmd(buf, driver->apps_rsp_buf);
		if (len > 0) {
			diag_send_rsp(driver->apps_rsp_buf, len);
			return 0;
		}
		return len;
	}
	/* Check for download command */
	else if ((cpu_is_msm8x60() || chk_apps_master()) && (*buf == 0x3A)) {
		/* send response back */
		driver->apps_rsp_buf[0] = *buf;
		diag_send_rsp(driver->apps_rsp_buf, 1);
		msleep(5000);
		/* call download API */
		msm_set_restart_mode(RESTART_DLOAD);
		printk(KERN_CRIT "diag: download mode set, Rebooting SoC..\n");
		kernel_restart(NULL);
		/* Not required, represents that command isnt sent to modem */
		return 0;
	}
	/* Check for polling for Apps only DIAG */
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x03)) {
		/* If no one has registered for polling */
		if (chk_polling_response()) {
			/* Respond to polling for Apps only DIAG */
			for (i = 0; i < 3; i++)
				driver->apps_rsp_buf[i] = *(buf+i);
			for (i = 0; i < 13; i++)
				driver->apps_rsp_buf[i+3] = 0;

			diag_send_rsp(driver->apps_rsp_buf, 16);
			return 0;
		}
	}
	/* Return the Delayed Response Wrap Status */
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x04) && (*(buf+3) == 0x0)) {
		memcpy(driver->apps_rsp_buf, buf, 4);
		driver->apps_rsp_buf[4] = wrap_enabled;
		diag_send_rsp(driver->apps_rsp_buf, 5);
		return 0;
	}
	/* Wrap the Delayed Rsp ID */
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x05) && (*(buf+3) == 0x0)) {
		wrap_enabled = true;
		memcpy(driver->apps_rsp_buf, buf, 4);
		driver->apps_rsp_buf[4] = wrap_count;
		diag_send_rsp(driver->apps_rsp_buf, 6);
		return 0;
	}
	/* Log on Demand Rsp */
	else if (*buf == DIAG_CMD_LOG_ON_DMND) {
		write_len = diag_cmd_log_on_demand(buf, len,
						   driver->apps_rsp_buf,
						   DIAG_MAX_RSP_SIZE);
		if (write_len > 0)
			diag_send_rsp(driver->apps_rsp_buf, write_len);
		return 0;
	}
	/* Mobile ID Rsp */
	else if ((*buf == DIAG_CMD_DIAG_SUBSYS) &&
		(*(buf+1) == DIAG_SS_PARAMS) &&
		(*(buf+2) == DIAG_EXT_MOBILE_ID) && (*(buf+3) == 0x0))  {
			write_len = diag_cmd_get_mobile_id(buf, len,
						   driver->apps_rsp_buf,
						   DIAG_MAX_RSP_SIZE);
		if (write_len > 0) {
			diag_send_rsp(driver->apps_rsp_buf, write_len - 1);
			return 0;
		}
	}
	 /*
	  * If the apps processor is master and no other
	  * processor has registered for polling command.
	  * If modem is not up and we have not received feature
	  * mask update from modem, in that case APPS should
	  * respond for 0X7C command
	  */
	else if (chk_apps_master() &&
		 !(driver->polling_reg_flag) &&
		 !(driver->smd_data[PERIPHERAL_MODEM].ch) &&
		 !(driver->feature[PERIPHERAL_MODEM].rcvd_feature_mask)) {
		/* respond to 0x0 command */
		if (*buf == 0x00) {
			for (i = 0; i < 55; i++)
				driver->apps_rsp_buf[i] = 0;

			diag_send_rsp(driver->apps_rsp_buf, 55);
			return 0;
		}
		/* respond to 0x7c command */
		else if (*buf == 0x7c) {
			driver->apps_rsp_buf[0] = 0x7c;
			for (i = 1; i < 8; i++)
				driver->apps_rsp_buf[i] = 0;
			/* Tools ID for APQ 8060 */
			*(int *)(driver->apps_rsp_buf + 8) =
							 chk_config_get_id();
			*(unsigned char *)(driver->apps_rsp_buf + 12) = '\0';
			*(unsigned char *)(driver->apps_rsp_buf + 13) = '\0';
			diag_send_rsp(driver->apps_rsp_buf, 14);
			return 0;
		}
	}
	write_len = diag_cmd_chk_stats(buf, len, driver->apps_rsp_buf,
				       DIAG_MAX_RSP_SIZE);
	if (write_len > 0) {
		diag_send_rsp(driver->apps_rsp_buf, write_len);
		return 0;
	}
	write_len = diag_cmd_disable_hdlc(buf, len, driver->apps_rsp_buf,
					  DIAG_MAX_RSP_SIZE);
	if (write_len > 0) {
		/*
		 * This mutex lock is necessary since we need to drain all the
		 * pending buffers from peripherals which may be HDLC encoded
		 * before disabling HDLC encoding on Apps processor.
		 */
		mutex_lock(&driver->hdlc_disable_mutex);
		diag_send_rsp(driver->apps_rsp_buf, write_len);
		/*
		 * Set the value of hdlc_disabled after sending the response to
		 * the tools. This is required since the tools is expecting a
		 * HDLC encoded reponse for this request.
		 */
		pr_debug("diag: In %s, disabling HDLC encoding\n",
		       __func__);
		driver->hdlc_disabled = 1;
		diag_update_userspace_clients(HDLC_SUPPORT_TYPE);
		mutex_unlock(&driver->hdlc_disable_mutex);
		return 0;
	}
#endif

	/* We have now come to the end of the function. */
	if (chk_apps_only())
		diag_send_error_rsp(buf, len);

	return 0;
}

void diag_process_hdlc_pkt(void *data, unsigned len)
{
	int err = 0;
	int ret = 0;

	if (len > DIAG_MAX_HDLC_BUF_SIZE) {
		pr_err("diag: In %s, invalid length: %d\n", __func__, len);
		return;
	}

	mutex_lock(&driver->diag_hdlc_mutex);
	pr_debug("diag: In %s, received packet of length: %d, req_buf_len: %d\n",
		 __func__, len, driver->hdlc_buf_len);

	if (driver->hdlc_buf_len >= DIAG_MAX_REQ_SIZE) {
		pr_err("diag: In %s, request length is more than supported len. Dropping packet.\n",
		       __func__);
		goto fail;
	}

	hdlc_decode->dest_ptr = driver->hdlc_buf + driver->hdlc_buf_len;
	hdlc_decode->dest_size = DIAG_MAX_HDLC_BUF_SIZE - driver->hdlc_buf_len;
	hdlc_decode->src_ptr = data;
	hdlc_decode->src_size = len;
	hdlc_decode->src_idx = 0;
	hdlc_decode->dest_idx = 0;

	ret = diag_hdlc_decode(hdlc_decode);
	/*
	 * driver->hdlc_buf is of size DIAG_MAX_HDLC_BUF_SIZE. But the decoded
	 * packet should be within DIAG_MAX_REQ_SIZE.
	 */
	if (driver->hdlc_buf_len + hdlc_decode->dest_idx <= DIAG_MAX_REQ_SIZE) {
		driver->hdlc_buf_len += hdlc_decode->dest_idx;
	} else {
		pr_err_ratelimited("diag: In %s, Dropping packet. pkt_size: %d, max: %d\n",
				   __func__,
				   driver->hdlc_buf_len + hdlc_decode->dest_idx,
				   DIAG_MAX_REQ_SIZE);
		goto fail;
	}

	if (ret == HDLC_COMPLETE) {
		err = crc_check(driver->hdlc_buf, driver->hdlc_buf_len);
		if (err) {
			/* CRC check failed. */
			pr_err_ratelimited("diag: In %s, bad CRC. Dropping packet\n",
					   __func__);
			goto fail;
		}
		driver->hdlc_buf_len -= HDLC_FOOTER_LEN;

		if (driver->hdlc_buf_len < 1) {
			pr_err_ratelimited("diag: In %s, message is too short, len: %d, dest len: %d\n",
					   __func__, driver->hdlc_buf_len,
					   hdlc_decode->dest_idx);
			goto fail;
		}

		err = diag_process_apps_pkt(driver->hdlc_buf,
					    driver->hdlc_buf_len);
		if (err < 0)
			goto fail;
	} else {
		goto end;
	}

fail:
	driver->hdlc_buf_len = 0;
end:
	mutex_unlock(&driver->diag_hdlc_mutex);
}

void diag_reset_smd_data(int queue)
{
	int i;
	unsigned long flags;

	for (i = 0; i < NUM_PERIPHERALS; i++) {
		spin_lock_irqsave(&driver->smd_data[i].in_busy_lock, flags);
		driver->smd_data[i].in_busy_1 = 0;
		driver->smd_data[i].in_busy_2 = 0;
		spin_unlock_irqrestore(&driver->smd_data[i].in_busy_lock,
				       flags);
		if (queue)
			/* Poll SMD data channels to check for data */
			queue_work(driver->smd_data[i].wq,
				&(driver->smd_data[i].diag_read_smd_work));
	}

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_PERIPHERALS; i++) {
			spin_lock_irqsave(&driver->smd_cmd[i].in_busy_lock,
					  flags);
			driver->smd_cmd[i].in_busy_1 = 0;
			driver->smd_cmd[i].in_busy_2 = 0;
			spin_unlock_irqrestore(&driver->smd_cmd[i].in_busy_lock,
					       flags);
			if (queue)
				/* Poll SMD data channels to check for data */
				queue_work(driver->diag_wq,
					&(driver->smd_cmd[i].
						diag_read_smd_work));
		}
	}
}

static void diag_smd_reset_buf(struct diag_smd_info *smd_info, int num)
{
	unsigned long flags;
	if (!smd_info)
		return;

	spin_lock_irqsave(&smd_info->in_busy_lock, flags);
	if (num == 1)
		smd_info->in_busy_1 = 0;
	else if (num == 2)
		smd_info->in_busy_2 = 0;
	else
		pr_err_ratelimited("diag: %s invalid buf %d\n", __func__, num);
	spin_unlock_irqrestore(&smd_info->in_busy_lock, flags);

	if (smd_info->type == TYPE_DATA)
		queue_work(smd_info->wq, &(smd_info->diag_read_smd_work));
	else
		queue_work(driver->diag_wq, &(smd_info->diag_read_smd_work));

}

static int diagfwd_mux_open(int id, int mode)
{
	int i;
	unsigned long flags;

	if (driver->rsp_buf_busy) {
		/*
		 * When a client switches from callback mode to USB mode
		 * explicitly, there can be a situation when the last response
		 * is not drained to the user space application. Reset the
		 * in_busy flag in this case.
		 */
		spin_lock_irqsave(&driver->rsp_buf_busy_lock, flags);
		driver->rsp_buf_busy = 0;
		spin_unlock_irqrestore(&driver->rsp_buf_busy_lock, flags);
	}
	switch (mode) {
	case DIAG_USB_MODE:
		driver->usb_connected = 1;
		break;
	case DIAG_MEMORY_DEVICE_MODE:
		break;
	default:
		return -EINVAL;
	}

	if ((mode == DIAG_USB_MODE &&
			driver->logging_mode == MEMORY_DEVICE_MODE) ||
			(mode == DIAG_MEMORY_DEVICE_MODE &&
				driver->logging_mode == USB_MODE)) {
		/* In this case Diag shouldn't not reset the smd in_busy data.
		 * If the reset of smd in_busy values happens then this will
		 * lead to loss of data read over peripherals.
		*/
	} else {
		diag_reset_smd_data(RESET_AND_QUEUE);
	}
	for (i = 0; i < NUM_PERIPHERALS; i++) {
		/* Poll SMD CNTL channels to check for data */
		diag_smd_notify(&(driver->smd_cntl[i]), SMD_EVENT_DATA);
	}
	queue_work(driver->diag_real_time_wq, &driver->diag_real_time_work);
	return 0;
}

static int diagfwd_mux_close(int id, int mode)
{
	int i;
	unsigned long flags;
	struct diag_smd_info *smd_info = NULL;

	switch (mode) {
	case DIAG_USB_MODE:
		driver->usb_connected = 0;
		break;
	case DIAG_MEMORY_DEVICE_MODE:
		break;
	default:
		return -EINVAL;
	}

	if (driver->logging_mode == USB_MODE) {
		for (i = 0; i < NUM_PERIPHERALS; i++) {
			smd_info = &driver->smd_data[i];
			spin_lock_irqsave(&smd_info->in_busy_lock, flags);
			smd_info->in_busy_1 = 1;
			smd_info->in_busy_2 = 1;
			spin_unlock_irqrestore(&smd_info->in_busy_lock, flags);
		}

		if (driver->supports_separate_cmdrsp) {
			for (i = 0; i < NUM_PERIPHERALS; i++) {
				smd_info = &driver->smd_cmd[i];
				spin_lock_irqsave(&smd_info->in_busy_lock,
						flags);
				smd_info->in_busy_1 = 1;
				smd_info->in_busy_2 = 1;
				spin_unlock_irqrestore(&smd_info->in_busy_lock,
						flags);
			}
		}
		/* Re enable HDLC encoding */
		pr_debug("diag: In %s, re-enabling HDLC encoding\n",
		       __func__);
		driver->hdlc_disabled = 0;
	}
	queue_work(driver->diag_real_time_wq,
		   &driver->diag_real_time_work);
	return 0;
}

static uint8_t hdlc_reset;

static void hdlc_reset_timer_start(void)
{
	if (!hdlc_timer_in_progress) {
		hdlc_timer_in_progress = 1;
		mod_timer(&driver->hdlc_reset_timer,
			  jiffies + msecs_to_jiffies(200));
	}
}

static void hdlc_reset_timer_func(unsigned long data)
{

	pr_debug("diag: In %s, re-enabling HDLC encoding\n",
		       __func__);
	if (hdlc_reset) {
		driver->hdlc_disabled = 0;
		queue_work(driver->diag_wq,
			&(driver->update_user_clients));
	}
	hdlc_timer_in_progress = 0;
}

static void diag_hdlc_start_recovery(unsigned char *buf, int len)
{
	int i;
	static uint32_t bad_byte_counter;
	unsigned char *start_ptr = NULL;

	hdlc_reset = 1;
	hdlc_reset_timer_start();

	for (i = 0; i < len; i++) {
		if (buf[i] == CONTROL_CHAR && (i +
				sizeof(struct diag_pkt_frame_t)
				<= (len - 1))) {
			if (buf[i+1] == 1) {
				start_ptr = &buf[i];
				break;
			}
		}
		bad_byte_counter++;
		if (bad_byte_counter > (DIAG_MAX_REQ_SIZE +
				sizeof(struct diag_pkt_frame_t) + 1)) {
			bad_byte_counter = 0;
			pr_err("diag: In %s, re-enabling HDLC encoding\n",
					__func__);
			mutex_lock(&driver->hdlc_disable_mutex);
			driver->hdlc_disabled = 0;
			mutex_unlock(&driver->hdlc_disable_mutex);
			diag_update_userspace_clients(HDLC_SUPPORT_TYPE);
			return;
		}
	}

	if (start_ptr) {
		/* Discard any partial packet reads */
		driver->incoming_pkt.processing = 0;
		diag_process_non_hdlc_pkt(start_ptr, len - i);
	}
}

void diag_process_non_hdlc_pkt(unsigned char *buf, int len)
{
	int err = 0;
	uint16_t pkt_len = 0;
	uint32_t read_bytes = 0;
	const uint32_t header_len = sizeof(struct diag_pkt_frame_t);
	struct diag_pkt_frame_t *actual_pkt = NULL;
	unsigned char *data_ptr = NULL;
	struct diag_partial_pkt_t *partial_pkt = &driver->incoming_pkt;

	if (!buf || len <= 0)
		return;

	if (!partial_pkt->processing)
		goto start;

	if (partial_pkt->remaining > len) {
		if ((partial_pkt->read_len + len) > partial_pkt->capacity) {
			pr_err("diag: Invalid length %d, %d received in %s\n",
			       partial_pkt->read_len, len, __func__);
			goto end;
		}
		memcpy(partial_pkt->data + partial_pkt->read_len, buf, len);
		read_bytes += len;
		buf += read_bytes;
		partial_pkt->read_len += len;
		partial_pkt->remaining -= len;
	} else {
		if ((partial_pkt->read_len + partial_pkt->remaining) >
						partial_pkt->capacity) {
			pr_err("diag: Invalid length during partial read %d, %d received in %s\n",
			       partial_pkt->read_len,
			       partial_pkt->remaining, __func__);
			goto end;
		}
		memcpy(partial_pkt->data + partial_pkt->read_len, buf,
						partial_pkt->remaining);
		read_bytes += partial_pkt->remaining;
		buf += read_bytes;
		partial_pkt->read_len += partial_pkt->remaining;
		partial_pkt->remaining = 0;
	}

	if (partial_pkt->remaining == 0) {
		actual_pkt = (struct diag_pkt_frame_t *)(partial_pkt->data);
		data_ptr = partial_pkt->data + header_len;
		if (*(uint8_t *)(data_ptr + actual_pkt->length) != CONTROL_CHAR)
			diag_hdlc_start_recovery(buf, len);
		err = diag_process_apps_pkt(data_ptr,
					    actual_pkt->length);
		if (err) {
			pr_err("diag: In %s, unable to process incoming data packet, err: %d\n",
			       __func__, err);
			goto end;
		}
		partial_pkt->read_len = 0;
		partial_pkt->total_len = 0;
		partial_pkt->processing = 0;
		goto start;
	}
	goto end;

start:
	while (read_bytes < len) {
		actual_pkt = (struct diag_pkt_frame_t *)buf;
		pkt_len = actual_pkt->length;

		if (actual_pkt->start != CONTROL_CHAR) {
			diag_hdlc_start_recovery(buf, len);
			diag_send_error_rsp(buf, len);
			goto end;
		}

		if (pkt_len + header_len > partial_pkt->capacity) {
			pr_err("diag: In %s, incoming data is too large for the request buffer %d\n",
			       __func__, pkt_len);
			diag_hdlc_start_recovery(buf, len);
			break;
		}

		if ((pkt_len + header_len) > (len - read_bytes)) {
			partial_pkt->read_len = len - read_bytes;
			partial_pkt->total_len = pkt_len + header_len;
			partial_pkt->remaining = partial_pkt->total_len -
						 partial_pkt->read_len;
			partial_pkt->processing = 1;
			memcpy(partial_pkt->data, buf, partial_pkt->read_len);
			break;
		}
		data_ptr = buf + header_len;
		if (*(uint8_t *)(data_ptr + actual_pkt->length) != CONTROL_CHAR)
			diag_hdlc_start_recovery(buf, len);
		else
			hdlc_reset = 0;
		err = diag_process_apps_pkt(data_ptr,
					    actual_pkt->length);
		if (err)
			break;
		read_bytes += header_len + pkt_len + 1;
		buf += header_len + pkt_len + 1; /* advance to next pkt */
	}
end:
	return;
}

static int diagfwd_mux_read_done(unsigned char *buf, int len, int ctxt)
{
	if (!buf || len <= 0)
		return -EINVAL;

	if (!driver->hdlc_disabled)
		diag_process_hdlc_pkt(buf, len);
	else
		diag_process_non_hdlc_pkt(buf, len);

	diag_mux_queue_read(ctxt);
	return 0;
}

static int diagfwd_mux_write_done(unsigned char *buf, int len, int buf_ctxt,
				  int ctxt)
{
	struct diag_smd_info *smd_info = NULL;
	unsigned long flags;
	int peripheral = -1;
	int type = -1;
	int num = -1;

	if (!buf || len < 0)
		return -EINVAL;

	peripheral = GET_BUF_PERIPHERAL(buf_ctxt);
	type = GET_BUF_TYPE(buf_ctxt);
	num = GET_BUF_NUM(buf_ctxt);

	switch (type) {
	case TYPE_DATA:
		if (peripheral >= 0 && peripheral < NUM_PERIPHERALS) {
			smd_info = &driver->smd_data[peripheral];
			diag_smd_reset_buf(smd_info, num);
			/*
			 * Flush any work that is currently pending on the data
			 * channels. This will ensure that the next read is not
			 * missed.
			 */
			if (driver->logging_mode == MEMORY_DEVICE_MODE &&
					ctxt == DIAG_MEMORY_DEVICE_MODE) {
				flush_workqueue(smd_info->wq);
				wake_up(&driver->smd_wait_q);
			}
		} else if (peripheral == APPS_DATA) {
			diagmem_free(driver, (unsigned char *)buf,
				     POOL_TYPE_HDLC);
		} else {
			pr_err_ratelimited("diag: Invalid peripheral %d in %s, type: %d\n",
					   peripheral, __func__, type);
		}
		break;
	case TYPE_CMD:
		if (peripheral >= 0 && peripheral < NUM_PERIPHERALS &&
		    driver->supports_separate_cmdrsp) {
			smd_info = &driver->smd_cmd[peripheral];
			diag_smd_reset_buf(smd_info, num);
		} else if (peripheral == APPS_DATA) {
			spin_lock_irqsave(&driver->rsp_buf_busy_lock, flags);
			driver->rsp_buf_busy = 0;
			driver->encoded_rsp_len = 0;
			spin_unlock_irqrestore(&driver->rsp_buf_busy_lock,
					       flags);
		} else {
			pr_err_ratelimited("diag: Invalid peripheral %d in %s, type: %d\n",
					   peripheral, __func__, type);
		}
		break;
	default:
		pr_err_ratelimited("diag: Incorrect data type %d, buf_ctxt: %d in %s\n",
				   type, buf_ctxt, __func__);
		break;
	}

	return 0;
}

static struct diag_mux_ops diagfwd_mux_ops = {
	.open = diagfwd_mux_open,
	.close = diagfwd_mux_close,
	.read_done = diagfwd_mux_read_done,
	.write_done = diagfwd_mux_write_done
};

void diag_smd_notify(void *ctxt, unsigned event)
{
	struct diag_smd_info *smd_info = (struct diag_smd_info *)ctxt;
	if (!smd_info)
		return;

	if (event == SMD_EVENT_CLOSE) {
		smd_info->ch = 0;
		wake_up(&driver->smd_wait_q);
		if (smd_info->type == TYPE_DATA) {
			smd_info->notify_context = event;
			queue_work(driver->diag_cntl_wq,
				 &(smd_info->diag_notify_update_smd_work));
		} else if (smd_info->type == TYPE_DCI) {
			/* Notify the clients of the close */
			diag_dci_notify_client(smd_info->peripheral_mask,
					       DIAG_STATUS_CLOSED,
					       DCI_LOCAL_PROC);
		} else if (smd_info->type == TYPE_CNTL) {
			diag_cntl_stm_notify(smd_info,
						CLEAR_PERIPHERAL_STM_STATE);
		}
		return;
	} else if (event == SMD_EVENT_OPEN) {
		if (smd_info->ch_save)
			smd_info->ch = smd_info->ch_save;

		if (smd_info->type == TYPE_CNTL) {
			smd_info->notify_context = event;
			queue_work(driver->diag_cntl_wq,
				&(smd_info->diag_notify_update_smd_work));
		} else if (smd_info->type == TYPE_DCI) {
			smd_info->notify_context = event;
			queue_work(driver->diag_dci_wq,
				&(smd_info->diag_notify_update_smd_work));
			/* Notify the clients of the open */
			diag_dci_notify_client(smd_info->peripheral_mask,
					      DIAG_STATUS_OPEN, DCI_LOCAL_PROC);
		}
	} else if (event == SMD_EVENT_DATA) {
		if ((smd_info->type == TYPE_DCI) ||
		    (smd_info->type == TYPE_DCI_CMD) ||
		    (smd_info->type == TYPE_DATA &&
		     driver->logging_mode == MEMORY_DEVICE_MODE)) {
			diag_ws_on_notify();
		}
	}

	wake_up(&driver->smd_wait_q);

	if (smd_info->type == TYPE_DCI ||
					smd_info->type == TYPE_DCI_CMD) {
		queue_work(driver->diag_dci_wq,
				&(smd_info->diag_read_smd_work));
	} else if (smd_info->type == TYPE_DATA) {
		queue_work(smd_info->wq,
				&(smd_info->diag_read_smd_work));
	} else {
		queue_work(driver->diag_wq, &(smd_info->diag_read_smd_work));
	}
}

static int diag_smd_probe(struct platform_device *pdev)
{
	int r = 0;
	int index = -1;
	const char *channel_name = NULL;

	switch (pdev->id) {
	case SMD_APPS_MODEM:
		index = PERIPHERAL_MODEM;
		channel_name = "DIAG";
		break;
	case SMD_APPS_QDSP:
		index = PERIPHERAL_LPASS;
		channel_name = "DIAG";
		break;
	case SMD_APPS_WCNSS:
		index = PERIPHERAL_WCNSS;
		channel_name = "APPS_RIVA_DATA";
		break;
	case SMD_APPS_DSPS:
		index = PERIPHERAL_SENSORS;
		channel_name = "DIAG";
		break;
	default:
		pr_debug("diag: In %s Received probe for invalid index %d",
			__func__, pdev->id);
		return 0;

	}

	r = smd_named_open_on_edge(channel_name,
				pdev->id,
				&driver->smd_data[index].ch,
				&driver->smd_data[index],
				diag_smd_notify);
	driver->smd_data[index].ch_save = driver->smd_data[index].ch;
	diag_smd_buffer_init(&driver->smd_data[index]);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pr_debug("diag: In %s, open SMD port, Id = %d, r = %d\n",
		__func__, pdev->id, r);

	return 0;
}

static int diag_smd_cmd_probe(struct platform_device *pdev)
{
	int r = 0;
	int index = -1;
	const char *channel_name = NULL;

	if (!driver->supports_separate_cmdrsp)
		return 0;

	switch (pdev->id) {
	case SMD_APPS_MODEM:
		index = PERIPHERAL_MODEM;
		break;
	case SMD_APPS_QDSP:
		index = PERIPHERAL_LPASS;
		break;
	case SMD_APPS_WCNSS:
		index = PERIPHERAL_WCNSS;
		break;
	case SMD_APPS_DSPS:
		index = PERIPHERAL_SENSORS;
		break;
	default:
		pr_debug("diag: In %s Received probe for invalid index %d",
			__func__, pdev->id);
		return 0;

	}
	channel_name = "DIAG_CMD";
	r = smd_named_open_on_edge(channel_name,
			pdev->id,
			&driver->smd_cmd[index].ch,
			&driver->smd_cmd[index],
			diag_smd_notify);
	driver->smd_cmd[index].ch_save = driver->smd_cmd[index].ch;
	diag_smd_buffer_init(&driver->smd_cmd[index]);

	pr_debug("diag: In %s, open SMD CMD port, Id = %d, r = %d\n",
		__func__, pdev->id, r);

	return 0;
}

static int diag_smd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diag_smd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diag_smd_dev_pm_ops = {
	.runtime_suspend = diag_smd_runtime_suspend,
	.runtime_resume = diag_smd_runtime_resume,
};

static struct platform_driver msm_smd_ch1_driver = {

	.probe = diag_smd_probe,
	.driver = {
		.name = "DIAG",
		.owner = THIS_MODULE,
		.pm   = &diag_smd_dev_pm_ops,
	},
};

static struct platform_driver diag_smd_lite_driver = {

	.probe = diag_smd_probe,
	.driver = {
		.name = "APPS_RIVA_DATA",
		.owner = THIS_MODULE,
		.pm   = &diag_smd_dev_pm_ops,
	},
};

static struct platform_driver smd_lite_data_cmd_drivers = {

	.probe = diag_smd_cmd_probe,
	.driver = {
		.name = "DIAG_CMD",
		.owner = THIS_MODULE,
		.pm   = &diag_smd_dev_pm_ops,
	}
};

int device_supports_separate_cmdrsp(void)
{
	return driver->use_device_tree;
}

void diag_smd_destructor(struct diag_smd_info *smd_info)
{
	if (smd_info->type == TYPE_DATA)
		destroy_workqueue(smd_info->wq);

	if (smd_info->ch)
		smd_close(smd_info->ch);

	smd_info->ch = 0;
	smd_info->ch_save = 0;
	kfree(smd_info->buf_in_1);
	kfree(smd_info->buf_in_2);
	kfree(smd_info->buf_in_1_raw);
	kfree(smd_info->buf_in_2_raw);
}

void diag_smd_buffer_init(struct diag_smd_info *smd_info)
{
	if (!smd_info) {
		pr_err("diag: Invalid smd_info\n");
		return;
	}

	if (smd_info->inited) {
		pr_debug("diag: smd buffers are already initialized, peripheral: %d, type: %d\n",
			 smd_info->peripheral, smd_info->type);
		return;
	}

	if (smd_info->buf_in_1 == NULL) {
		smd_info->buf_in_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (smd_info->buf_in_1 == NULL)
			goto err;
		smd_info->buf_in_1_size = IN_BUF_SIZE;
		kmemleak_not_leak(smd_info->buf_in_1);
	}

	/* The smd data type needs two buffers */
	if (smd_info->type == TYPE_DATA) {
		if (smd_info->buf_in_2 == NULL) {
			smd_info->buf_in_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
			if (smd_info->buf_in_2 == NULL)
				goto err;
			smd_info->buf_in_2_size = IN_BUF_SIZE;
			kmemleak_not_leak(smd_info->buf_in_2);
		}

		if (driver->supports_apps_hdlc_encoding) {
			/* In support of hdlc encoding */
			if (smd_info->buf_in_1_raw == NULL) {
				smd_info->buf_in_1_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
				if (smd_info->buf_in_1_raw == NULL)
					goto err;
				smd_info->buf_in_1_raw_size = IN_BUF_SIZE;
				kmemleak_not_leak(smd_info->buf_in_1_raw);
			}
			if (smd_info->buf_in_2_raw == NULL) {
				smd_info->buf_in_2_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
				if (smd_info->buf_in_2_raw == NULL)
					goto err;
				smd_info->buf_in_2_raw_size = IN_BUF_SIZE;
				kmemleak_not_leak(smd_info->buf_in_2_raw);
			}
		}
	}

	if (smd_info->type == TYPE_CMD &&
		driver->supports_apps_hdlc_encoding) {
		/* In support of hdlc encoding */
		if (smd_info->buf_in_1_raw == NULL) {
			smd_info->buf_in_1_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
			if (smd_info->buf_in_1_raw == NULL)
				goto err;
			smd_info->buf_in_1_raw_size = IN_BUF_SIZE;
			kmemleak_not_leak(smd_info->buf_in_1_raw);
		}
	}
	smd_info->fifo_size = smd_write_avail(smd_info->ch);
	smd_info->inited = 1;
	return;
err:
	smd_info->inited = 0;
	kfree(smd_info->buf_in_1);
	kfree(smd_info->buf_in_2);
	kfree(smd_info->buf_in_1_raw);
	kfree(smd_info->buf_in_2_raw);
}

int diag_smd_constructor(struct diag_smd_info *smd_info, int peripheral,
			  int type)
{
	if (!smd_info)
		return -EIO;

	smd_info->peripheral = peripheral;
	smd_info->type = type;
	smd_info->inited = 0;
	mutex_init(&smd_info->smd_ch_mutex);
	spin_lock_init(&smd_info->in_busy_lock);

	switch (peripheral) {
	case PERIPHERAL_MODEM:
		smd_info->peripheral_mask = DIAG_CON_MPSS;
		break;
	case PERIPHERAL_LPASS:
		smd_info->peripheral_mask = DIAG_CON_LPASS;
		break;
	case PERIPHERAL_WCNSS:
		smd_info->peripheral_mask = DIAG_CON_WCNSS;
		break;
	case PERIPHERAL_SENSORS:
		smd_info->peripheral_mask = DIAG_CON_SENSORS;
		break;
	default:
		pr_err("diag: In %s, unknown peripheral, peripheral: %d\n",
			__func__, peripheral);
		goto err;
	}

	smd_info->ch = 0;
	smd_info->ch_save = 0;

	/* The smd data type needs separate work queues for reads */
	if (type == TYPE_DATA) {
		switch (peripheral) {
		case PERIPHERAL_MODEM:
			smd_info->wq = create_singlethread_workqueue(
						"diag_PERIPHERAL_MODEM_read_wq");
			break;
		case PERIPHERAL_LPASS:
			smd_info->wq = create_singlethread_workqueue(
						"diag_PERIPHERAL_LPASS_read_wq");
			break;
		case PERIPHERAL_WCNSS:
			smd_info->wq = create_singlethread_workqueue(
						"diag_PERIPHERAL_WCNSS_read_wq");
			break;
		case PERIPHERAL_SENSORS:
			smd_info->wq = create_singlethread_workqueue(
						"diag_PERIPHERAL_SENSORS_read_wq");
			break;
		default:
			smd_info->wq = NULL;
			break;
		}
		if (!smd_info->wq)
			goto err;
	} else {
		smd_info->wq = NULL;
	}

	INIT_WORK(&(smd_info->diag_read_smd_work), diag_read_smd_work_fn);

	/*
	 * The update function assigned to the diag_notify_update_smd_work
	 * work_struct is meant to be used for updating that is not to
	 * be done in the context of the smd notify function. The
	 * notify_context variable can be used for passing additional
	 * information to the update function.
	 */
	smd_info->notify_context = 0;
	smd_info->general_context = 0;
	switch (type) {
	case TYPE_DATA:
	case TYPE_CMD:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
						diag_clean_reg_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
						diag_cntl_smd_work_fn);
		break;
	case TYPE_CNTL:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
						diag_mask_update_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
						diag_cntl_smd_work_fn);
		break;
	case TYPE_DCI:
	case TYPE_DCI_CMD:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
					diag_update_smd_dci_work_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
					diag_cntl_smd_work_fn);
		break;
	default:
		pr_err("diag: In %s, unknown type, type: %d\n", __func__, type);
		goto err;
	}

	/*
	 * Set function ptr for function to call to process the data that
	 * was just read from the smd channel
	 */
	switch (type) {
	case TYPE_DATA:
	case TYPE_CMD:
		smd_info->process_smd_read_data = diag_process_smd_read_data;
		break;
	case TYPE_CNTL:
		smd_info->process_smd_read_data =
						diag_process_smd_cntl_read_data;
		break;
	case TYPE_DCI:
	case TYPE_DCI_CMD:
		smd_info->process_smd_read_data =
						diag_process_smd_dci_read_data;
		break;
	default:
		pr_err("diag: In %s, unknown type, type: %d\n", __func__, type);
		goto err;
	}

	smd_info->buf_in_1_ctxt = SET_BUF_CTXT(peripheral, smd_info->type, 1);
	smd_info->buf_in_2_ctxt = SET_BUF_CTXT(peripheral, smd_info->type, 2);

	return 0;
err:
	if (smd_info->wq)
		destroy_workqueue(smd_info->wq);

	return -ENOMEM;
}

static int diag_smd_write_ext(struct diag_smd_info *smd_info, void *buf,
			      int len)
{
	int err = 0;
	int offset = 0;
	int write_len = 0;
	int retry_count = 0;
	int max_retries = 3;
	uint8_t avail = 0;

	if (!smd_info || !buf || len <= 0) {
		pr_err_ratelimited("diag: In %s, invalid params, smd_info: %p, buf: %p, len: %d\n",
				   __func__, smd_info, buf, len);
		return -EINVAL;
	}

	if (!smd_info->ch)
		return -ENODEV;

	mutex_lock(&smd_info->smd_ch_mutex);
	err = smd_write_start(smd_info->ch, len);
	if (err) {
		pr_err("diag: In %s, error calling smd_write_start, peripheral: %d, err: %d\n",
		       __func__, smd_info->peripheral, err);
		goto fail;
	}

	while (offset < len) {
		retry_count = 0;
		do {
			if (smd_write_segment_avail(smd_info->ch)) {
				avail = 1;
				break;
			}
			/*
			 * The channel maybe busy - the FIFO can be full. Retry
			 * after sometime. The value of 10000 was chosen
			 * emprically as the optimal value for the peripherals
			 * to read data from the SMD channel.
			 */
			usleep_range(10000, 10100);
			retry_count++;
		} while (retry_count < max_retries);

		if (!avail) {
			err = -EAGAIN;
			goto fail;
		}

		write_len = smd_write_segment(smd_info->ch, buf + offset,
					      (len - offset));
		offset += write_len;
		write_len = 0;
	}

	err = smd_write_end(smd_info->ch);
	if (err) {
		pr_err("diag: In %s, error calling smd_write_end, peripheral: %d, err: %d\n",
		       __func__, smd_info->peripheral, err);
		goto fail;
	}

fail:
	mutex_unlock(&smd_info->smd_ch_mutex);
	return err;
}

int diag_smd_write(struct diag_smd_info *smd_info, void *buf, int len)
{
	int write_len = 0;
	int retry_count = 0;
	int max_retries = 3;

	if (!smd_info || !buf || len <= 0) {
		pr_err_ratelimited("diag: In %s, invalid params, smd_info: %p, buf: %p, len: %d\n",
				   __func__, smd_info, buf, len);
		return -EINVAL;
	}

	if (!smd_info->ch)
		return -ENODEV;

	if (len > smd_info->fifo_size)
		return diag_smd_write_ext(smd_info, buf, len);

	do {
		mutex_lock(&smd_info->smd_ch_mutex);
		write_len = smd_write(smd_info->ch, buf, len);
		mutex_unlock(&smd_info->smd_ch_mutex);
		if (write_len == len)
			break;
		/*
		 * The channel maybe busy - the FIFO can be full. Retry after
		 * sometime. The value of 10000 was chosen emprically as the
		 * optimal value for the peripherals to read data from the SMD
		 * channel.
		 */
		usleep_range(10000, 10100);
		retry_count++;
	} while (retry_count < max_retries);

	if (write_len != len)
		return -ENOMEM;

	return 0;
}

int diagfwd_init(void)
{
	int ret;
	int i;

	wrap_enabled = 0;
	wrap_count = 0;
	driver->use_device_tree = has_device_tree();
	for (i = 0; i < DIAG_NUM_PROC; i++)
		driver->real_time_mode[i] = 1;
	driver->supports_separate_cmdrsp = device_supports_separate_cmdrsp();
	driver->supports_apps_hdlc_encoding = 1;
	mutex_init(&driver->diag_hdlc_mutex);
	mutex_init(&driver->diag_cntl_mutex);
	mutex_init(&driver->mode_lock);
	driver->encoded_rsp_buf = kzalloc(DIAG_MAX_HDLC_BUF_SIZE, GFP_KERNEL);
	if (!driver->encoded_rsp_buf)
		goto err;
	kmemleak_not_leak(driver->encoded_rsp_buf);
	hdlc_decode = kzalloc(sizeof(struct diag_hdlc_decode_type),
			      GFP_KERNEL);
	if (!hdlc_decode)
		goto err;
	setup_timer(&driver->hdlc_reset_timer, hdlc_reset_timer_func, 0);
	kmemleak_not_leak(hdlc_decode);
	driver->encoded_rsp_len = 0;
	driver->rsp_buf_busy = 0;
	spin_lock_init(&driver->rsp_buf_busy_lock);
	driver->user_space_data_busy = 0;
	driver->hdlc_buf_len = 0;
	INIT_LIST_HEAD(&driver->cmd_reg_list);
	driver->cmd_reg_count = 0;
	mutex_init(&driver->cmd_reg_mutex);

	for (i = 0; i < NUM_PERIPHERALS; i++) {
		driver->feature[i].separate_cmd_rsp = 0;
		driver->feature[i].stm_support = DISABLE_STM;
		driver->feature[i].rcvd_feature_mask = 0;
		driver->feature[i].peripheral_buffering = 0;
		driver->feature[i].encode_hdlc = 0;
		driver->feature[i].mask_centralization = 0;
		driver->buffering_mode[i].peripheral = i;
		driver->buffering_mode[i].mode = DIAG_BUFFERING_MODE_STREAMING;
		driver->buffering_mode[i].high_wm_val = DEFAULT_HIGH_WM_VAL;
		driver->buffering_mode[i].low_wm_val = DEFAULT_LOW_WM_VAL;
	}

	for (i = 0; i < NUM_STM_PROCESSORS; i++) {
		driver->stm_state_requested[i] = DISABLE_STM;
		driver->stm_state[i] = DISABLE_STM;
	}

	for (i = 0; i < NUM_PERIPHERALS; i++) {
		ret = diag_smd_constructor(&driver->smd_data[i], i,
							TYPE_DATA);
		if (ret)
			goto err;
	}

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_PERIPHERALS; i++) {
			ret = diag_smd_constructor(&driver->smd_cmd[i], i,
								TYPE_CMD);
			if (ret)
				goto err;
		}
	}

	if (driver->hdlc_buf == NULL) {
		driver->hdlc_buf = kzalloc(DIAG_MAX_HDLC_BUF_SIZE, GFP_KERNEL);
		if (!driver->hdlc_buf)
			goto err;
		kmemleak_not_leak(driver->hdlc_buf);
	}
	if (driver->user_space_data_buf == NULL)
		driver->user_space_data_buf = kzalloc(USER_SPACE_DATA,
							GFP_KERNEL);
	if (driver->user_space_data_buf == NULL)
		goto err;
	kmemleak_not_leak(driver->user_space_data_buf);
	if (driver->client_map == NULL &&
	    (driver->client_map = kzalloc
	     ((driver->num_clients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->client_map);
	if (driver->data_ready == NULL &&
	     (driver->data_ready = kzalloc(driver->num_clients * sizeof(int)
							, GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->data_ready);
	if (driver->apps_req_buf == NULL) {
		driver->apps_req_buf = kzalloc(DIAG_MAX_REQ_SIZE, GFP_KERNEL);
		if (!driver->apps_req_buf)
			goto err;
		kmemleak_not_leak(driver->apps_req_buf);
	}
	if (driver->dci_pkt_buf == NULL) {
		driver->dci_pkt_buf = kzalloc(DCI_BUF_SIZE, GFP_KERNEL);
		if (!driver->dci_pkt_buf)
			goto err;
		kmemleak_not_leak(driver->dci_pkt_buf);
	}
	if (driver->apps_rsp_buf == NULL) {
		driver->apps_rsp_buf = kzalloc(DIAG_MAX_RSP_SIZE, GFP_KERNEL);
		if (driver->apps_rsp_buf == NULL)
			goto err;
		kmemleak_not_leak(driver->apps_rsp_buf);
	}
	driver->diag_wq = create_singlethread_workqueue("diag_wq");
	if (!driver->diag_wq)
		goto err;
	ret = diag_mux_register(DIAG_LOCAL_PROC, DIAG_LOCAL_PROC,
				&diagfwd_mux_ops);
	if (ret) {
		pr_err("diag: Unable to register with USB, err: %d\n", ret);
		goto err;
	}
	platform_driver_register(&msm_smd_ch1_driver);
	platform_driver_register(&diag_smd_lite_driver);

	if (driver->supports_separate_cmdrsp)
		platform_driver_register(&smd_lite_data_cmd_drivers);

	return 0;
err:
	pr_err("diag: In %s, couldn't initialize diag\n", __func__);

	for (i = 0; i < NUM_PERIPHERALS; i++)
		diag_smd_destructor(&driver->smd_data[i]);

	for (i = 0; i < NUM_PERIPHERALS; i++)
		diag_smd_destructor(&driver->smd_cmd[i]);

	diag_usb_exit(DIAG_USB_LOCAL);
	kfree(driver->encoded_rsp_buf);
	kfree(driver->hdlc_buf);
	kfree(driver->client_map);
	kfree(driver->data_ready);
	kfree(driver->apps_req_buf);
	kfree(driver->dci_pkt_buf);
	kfree(driver->apps_rsp_buf);
	kfree(hdlc_decode);
	kfree(driver->user_space_data_buf);
	if (driver->diag_wq)
		destroy_workqueue(driver->diag_wq);
	return -ENOMEM;
}

void diagfwd_exit(void)
{
	int i;

	for (i = 0; i < NUM_PERIPHERALS; i++)
		diag_smd_destructor(&driver->smd_data[i]);

	platform_driver_unregister(&msm_smd_ch1_driver);
	platform_driver_unregister(&diag_smd_lite_driver);

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_PERIPHERALS; i++)
			diag_smd_destructor(&driver->smd_cmd[i]);
		platform_driver_unregister(
			&smd_lite_data_cmd_drivers);
	}

	kfree(driver->encoded_rsp_buf);
	kfree(driver->hdlc_buf);
	kfree(hdlc_decode);
	kfree(driver->client_map);
	kfree(driver->data_ready);
	kfree(driver->apps_req_buf);
	kfree(driver->dci_pkt_buf);
	kfree(driver->apps_rsp_buf);
	kfree(driver->user_space_data_buf);
	destroy_workqueue(driver->diag_wq);
}
