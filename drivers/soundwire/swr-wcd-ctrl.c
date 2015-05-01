/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
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

#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/soundwire/soundwire.h>
#include <linux/soundwire/swr-wcd.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include "swrm_registers.h"
#include "swr-wcd-ctrl.h"

static u8 mstr_ports[] = {100, 101, 102, 103, 104, 105, 106, 107};
static u8 mstr_port_type[] = {SWR_DAC_PORT, SWR_COMP_PORT, SWR_BOOST_PORT,
			      SWR_DAC_PORT, SWR_COMP_PORT, SWR_BOOST_PORT,
			      SWR_VISENSE_PORT, SWR_VISENSE_PORT};

#define SWR_NUM_SLV_DEVICES		3 /* This includes dev_num_0 */

struct usecase uc[] = {
	{0, 0, 0},		/* UC0: no ports */
	{1, 1, 2400},		/* UC1: Spkr */
	{1, 4, 600},		/* UC2: Compander */
	{1, 2, 300},		/* UC3: Smart Boost */
	{1, 2, 1200},		/* UC4: VI Sense */
	{4, 9, 4500},		/* UC5: Spkr + Comp + SB + VI */
	{8, 18, 9000},		/* UC6: 2*(Spkr + Comp + SB + VI) */
	{2, 2, 4800},		/* UC7: 2*Spkr */
	{2, 5, 3000},		/* UC8: Spkr + Comp */
	{4, 10, 6000},		/* UC9: 2*(Spkr + Comp) */
	{3, 7, 3300},		/* UC10: Spkr + Comp + SB */
	{6, 14, 6600},		/* UC11: 2*(Spkr + Comp + SB) */
	{2, 3, 2700},		/* UC12: Spkr + SB */
	{4, 6, 5400},		/* UC13: 2*(Spkr + SB) */
};
#define MAX_USECASE	ARRAY_SIZE(uc)

struct port_params pp[MAX_USECASE][SWR_MSTR_PORT_LEN] = {
	/* UC 0 */
	{
		{0, 0, 0},
	},
	/* UC 1 */
	{
		{7, 1, 0},
	},
	/* UC 2 */
	{
		{31, 2, 0},
	},
	/* UC 3 */
	{
		{63, 12, 31},
	},
	/* UC 4 */
	{
		{15, 7, 0},
	},
	/* UC 5 */
	{
		{7, 1, 0},
		{31, 2, 0},
		{63, 12, 31},
		{15, 7, 0},
	},
	/* UC 6 */
	{
		{7, 1, 0},
		{31, 2, 0},
		{63, 12, 31},
		{15, 7, 0},
		{7, 6, 0},
		{31, 18, 0},
		{63, 13, 31},
		{15, 10, 0},
	},
	/* UC 7 */
	{
		{7, 1, 0},
		{7, 6, 0},

	},
	/* UC 8 */
	{
		{7, 1, 0},
		{31, 2, 0},
	},
	/* UC 9 */
	{
		{7, 1, 0},
		{31, 2, 0},
		{7, 6, 0},
		{31, 18, 0},
	},
	/* UC 10 */
	{
		{7, 1, 0},
		{31, 2, 0},
		{63, 12, 31},
	},
	/* UC 11 */
	{
		{7, 1, 0},
		{31, 2, 0},
		{63, 12, 31},
		{7, 6, 0},
		{31, 18, 0},
		{63, 13, 31},
	},
	/* UC 12 */
	{
		{7, 1, 0},
		{63, 12, 31},
	},
	/* UC 13 */
	{
		{7, 1, 0},
		{63, 12, 31},
		{7, 6, 0},
		{63, 13, 31},
	},
};

enum {
	SWR_NOT_PRESENT,
	SWR_ATTACHED_OK,
	SWR_ALERT,
	SWR_RESERVED,
};

#define SWR_MSTR_MAX_REG_ADDR	0x1740
#define SWR_MSTR_START_REG_ADDR	0x00
#define SWR_MSTR_MAX_BUF_LEN     20
#define BYTES_PER_LINE          12
#define SWR_MSTR_RD_BUF_LEN      8
#define SWR_MSTR_WR_BUF_LEN      32

static struct swr_mstr_ctrl *dbgswrm;
static struct dentry *debugfs_swrm_dent;
static struct dentry *debugfs_peek;
static struct dentry *debugfs_poke;
static struct dentry *debugfs_reg_dump;
static unsigned int read_data;

static int swrm_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, u32 *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");
	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (kstrtou32(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else
			return -EINVAL;
	}
	return 0;
}

static ssize_t swrm_reg_show(char __user *ubuf, size_t count,
					  loff_t *ppos)
{
	int i, reg_val, len;
	ssize_t total = 0;
	char tmp_buf[SWR_MSTR_MAX_BUF_LEN];

	if (!ubuf || !ppos)
		return 0;

	for (i = (((int) *ppos / BYTES_PER_LINE) + SWR_MSTR_START_REG_ADDR);
		i <= SWR_MSTR_MAX_REG_ADDR; i += 4) {
		reg_val = dbgswrm->read(dbgswrm->handle, i);
		len = snprintf(tmp_buf, 25, "0x%.3x: 0x%.2x\n", i, reg_val);
		if ((total + len) >= count - 1)
			break;
		if (copy_to_user((ubuf + total), tmp_buf, len)) {
			pr_err("%s: fail to copy reg dump\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		*ppos += len;
		total += len;
	}

copy_err:
	return total;
}

static ssize_t swrm_debug_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[SWR_MSTR_RD_BUF_LEN];
	char *access_str;
	ssize_t ret_cnt;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	access_str = file->private_data;
	if (*ppos < 0)
		return -EINVAL;

	if (!strcmp(access_str, "swrm_peek")) {
		snprintf(lbuf, sizeof(lbuf), "0x%x\n", read_data);
		ret_cnt = simple_read_from_buffer(ubuf, count, ppos, lbuf,
					       strnlen(lbuf, 7));
	} else if (!strcmp(access_str, "swrm_reg_dump")) {
		ret_cnt = swrm_reg_show(ubuf, count, ppos);
	} else {
		pr_err("%s: %s not permitted to read\n", __func__, access_str);
		ret_cnt = -EPERM;
	}
	return ret_cnt;
}

static ssize_t swrm_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_MSTR_WR_BUF_LEN];
	int rc;
	u32 param[5];
	char *access_str;

	if (!filp || !ppos || !ubuf)
		return -EINVAL;

	access_str = filp->private_data;
	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	if (!strcmp(access_str, "swrm_poke")) {
		/* write */
		rc = get_parameters(lbuf, param, 2);
		if ((param[0] <= SWR_MSTR_MAX_REG_ADDR) &&
			(param[1] <= 0xFFFFFFFF) &&
			(rc == 0))
			rc = dbgswrm->write(dbgswrm->handle, param[0],
					    param[1]);
		else
			rc = -EINVAL;
	} else if (!strcmp(access_str, "swrm_peek")) {
		/* read */
		rc = get_parameters(lbuf, param, 1);
		if ((param[0] <= SWR_MSTR_MAX_REG_ADDR) && (rc == 0))
			read_data = dbgswrm->read(dbgswrm->handle, param[0]);
		else
			rc = -EINVAL;
	}
	if (rc == 0)
		rc = cnt;
	else
		pr_err("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations swrm_debug_ops = {
	.open = swrm_debug_open,
	.write = swrm_debug_write,
	.read = swrm_debug_read,
};

static int swrm_set_ch_map(struct swr_mstr_ctrl *swrm, void *data)
{
	struct swr_mstr_port *pinfo = (struct swr_mstr_port *)data;

	swrm->mstr_port = kzalloc(sizeof(struct swr_mstr_port), GFP_KERNEL);
	if (swrm->mstr_port == NULL)
		return -ENOMEM;
	swrm->mstr_port->num_port = pinfo->num_port;
	swrm->mstr_port->port = kzalloc((pinfo->num_port * sizeof(u8)),
					GFP_KERNEL);
	if (!swrm->mstr_port->port)
		return -ENOMEM;

	memcpy(swrm->mstr_port->port, pinfo->port, pinfo->num_port);
	return 0;
}

int swrm_notify(struct platform_device *pdev, u32 id, void *data)
{
	struct swr_mstr_ctrl *swrm;
	int ret = 0;

	if (!pdev) {
		pr_err("%s: pdev is NULL\n", __func__);
		return -EINVAL;
	}
	swrm = platform_get_drvdata(pdev);
	if (!swrm) {
		dev_err(&pdev->dev, "%s: swrm is NULL\n", __func__);
		return -EINVAL;
	}
	switch (id) {
	case SWR_CH_MAP:
		if (!data) {
			dev_err(swrm->dev, "%s: data is NULL\n", __func__);
			ret = -EINVAL;
		} else {
			ret = swrm_set_ch_map(swrm, data);
		}
		break;
	case SWR_DEVICE_DOWN:
		dev_dbg(swrm->dev, "%s: swr master down called\n", __func__);
		break;
	case SWR_DEVICE_UP:
		dev_dbg(swrm->dev, "%s: swr master up called\n", __func__);
		break;
	default:
		dev_err(swrm->dev, "%s: swr master unknow id %d\n",
			__func__, id);
		break;
	}
	return ret;
}
EXPORT_SYMBOL(swrm_notify);

static int swrm_get_port_config(struct swr_master *master)
{
	u32 ch_rate = 0;
	u32 num_ch = 0;
	int i, uc_idx;
	u32 portcount = 0;

	for (i = 0; i < SWR_MSTR_PORT_LEN; i++) {
		if (master->port[i].port_en) {
			ch_rate += master->port[i].ch_rate;
			num_ch += master->port[i].num_ch;
			portcount++;
		}
	}
	for (i = 0; i < ARRAY_SIZE(uc); i++) {
		if ((uc[i].num_port == portcount) &&
		    (uc[i].num_ch == num_ch) &&
		    (uc[i].chrate == ch_rate)) {
			uc_idx = i;
			break;
		}
	}

	if (i >= ARRAY_SIZE(uc)) {
		dev_err(&master->dev,
			"%s: usecase port:%d, num_ch:%d, chrate:%d not found\n",
			__func__, master->num_port, num_ch, ch_rate);
		return -EINVAL;
	}
	for (i = 0; i < SWR_MSTR_PORT_LEN; i++) {
		if (master->port[i].port_en) {
			master->port[i].sinterval = pp[uc_idx][i].si;
			master->port[i].offset1 = pp[uc_idx][i].off1;
			master->port[i].offset2 = pp[uc_idx][i].off2;
		}
	}
	return 0;
}

static int swrm_get_master_port(u8 *mstr_port_id, u8 slv_port_id)
{
	int i;

	for (i = 0; i < SWR_MSTR_PORT_LEN; i++) {
		if (mstr_ports[i] == slv_port_id) {
			*mstr_port_id = i;
			return 0;
		}
	}
	return -EINVAL;
}

static int swrm_cmd_fifo_rd_cmd(struct swr_mstr_ctrl *swrm, u32 *cmd_data,
				 u8 dev_addr, u8 cmd_id, u16 reg_addr,
				 u32 len)
{
	u32 val;
	int ret = 0;

	if (!cmd_id) {
		if (swrm->wcmd_id < 14)
			swrm->wcmd_id += 1;
		else
			swrm->wcmd_id = 0;

		cmd_id = swrm->wcmd_id;
	}
	val = (reg_addr | (cmd_id << 16) | (dev_addr << 20) |
		(len << 24));

	ret = swrm->write(swrm->handle, SWRM_CMD_FIFO_RD_CMD, val);
	if (ret < 0) {
		dev_err(swrm->dev, "%s: reg 0x%x write failed, err:%d\n",
			__func__, val, ret);
		goto err;
	}
	*cmd_data = swrm->read(swrm->handle, SWRM_CMD_FIFO_RD_FIFO_ADDR);
	dev_dbg(swrm->dev,
		"%s: reg: 0x%x, cmd_id: 0x%x, dev_id: 0x%x, cmd_data: 0x%x\n",
		__func__, reg_addr, cmd_id, dev_addr, *cmd_data);
err:
	return ret;
}

static int swrm_cmd_fifo_wr_cmd(struct swr_mstr_ctrl *swrm, u8 cmd_data,
				 u8 dev_addr, u8 cmd_id, u16 reg_addr)
{
	u32 val;
	int ret = 0;

	if (!cmd_id) {
		if (swrm->wcmd_id < 14)
			swrm->wcmd_id += 1;
		else
			swrm->wcmd_id = 0;

		cmd_id = swrm->wcmd_id;
	}
	val = (reg_addr | (cmd_id << 16) | (dev_addr << 20) |
		(cmd_data << 24));

	dev_dbg(swrm->dev,
		"%s: reg: 0x%x, cmd_id: 0x%x, dev_id: 0x%x, cmd_data: 0x%x\n",
		__func__, reg_addr, cmd_id, dev_addr, cmd_data);
	ret = swrm->write(swrm->handle, SWRM_CMD_FIFO_WR_CMD, val);
	if (ret < 0) {
		dev_err(swrm->dev, "%s: reg 0x%x write failed, err:%d\n",
			__func__, val, ret);
		goto err;
	}
	if (cmd_id == 0xF)
		wait_for_completion_timeout(&swrm->broadcast, (2 * HZ/10));
err:
	return ret;
}

static int swrm_read(struct swr_master *master, u8 dev_num, u32 reg_addr,
		u32 *buf, u32 len)
{
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(master);

	if (dev_num) {
		swrm_cmd_fifo_rd_cmd(swrm, buf, dev_num, 0, reg_addr,
					len);
	} else {
		if (swrm->read)
			buf[0] = swrm->read(swrm->handle, reg_addr);
		else {
			dev_err(&master->dev, "%s: handle is NULL 0x%x\n",
				__func__, reg_addr);
			return -EINVAL;
		}
	}
	return 0;
}

static int swrm_write(struct swr_master *master, u8 dev_num, u32 reg_addr,
		u32 *buf)
{
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(master);
	int ret = 0;

	if (dev_num) {
		ret = swrm_cmd_fifo_wr_cmd(swrm, buf[0], dev_num, 0, reg_addr);
	} else {
		if (swrm->write) {
			ret = swrm->write(swrm->handle, reg_addr, buf[0]);
		} else {
			dev_err(&master->dev, "%s: handle is NULL 0x%x\n",
				__func__, reg_addr);
			return -EINVAL;
		}
	}
	return ret;
}

static u8 get_inactive_bank_num(struct swr_mstr_ctrl *swrm)
{
	return (swrm->read(swrm->handle, SWRM_MCP_STATUS) &
		SWRM_MCP_STATUS_BANK_NUM_MASK) ? 0 : 1;
}

static void enable_bank_switch(struct swr_mstr_ctrl *swrm, u8 bank,
				u8 row, u8 col)
{
	swrm_cmd_fifo_wr_cmd(swrm, ((row << 3) | col), 0xF, 0xF,
			SWRS_SCP_FRAME_CTRL_BANK(bank));
}

static struct swr_port_info *swrm_get_port(struct swr_master *master,
					   u8 port_id)
{
	int i;
	struct swr_port_info *port = NULL;

	for (i = 0; i < SWR_MSTR_PORT_LEN; i++) {
		port = &master->port[i];
		if ((port->port_id == port_id) && (port->port_en == true))
			break;
	}
	if (i == SWR_MSTR_PORT_LEN)
		port = NULL;
	return port;
}

static void swrm_apply_port_config(struct swr_master *master)
{
	u32 value;
	struct swr_port_info *port;
	u8 bank;
	int i;
	int port_type;
	struct swrm_mports *mport;
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(master);
	int mask = (SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_BMSK |
		SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_BMSK);

	bank = get_inactive_bank_num(swrm);
	dev_dbg(swrm->dev, "%s: enter bank: %d master_ports: %d\n",
		__func__, bank, master->num_port);

	/* set Row = 48 and col = 16 */
	value = swrm->read(swrm->handle, SWRM_MCP_FRAME_CTRL_BANK_ADDR(bank));
	value &= (~mask);
	value |= ((0 << SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_SHFT) |
		(7 << SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_SHFT));
	swrm->write(swrm->handle, SWRM_MCP_FRAME_CTRL_BANK_ADDR(bank), value);
	dev_dbg(swrm->dev, "%s: regaddr: 0x%x, value: 0x%x\n", __func__,
		SWRM_MCP_FRAME_CTRL_BANK_ADDR(bank), value);

	swrm_cmd_fifo_wr_cmd(swrm, 0x01, 0xF, 0x00,
			SWRS_SCP_HOST_CLK_DIV2_CTL_BANK(bank));

	mport = list_first_entry_or_null(&swrm->mport_list,
					struct swrm_mports,
					list);
	if (!mport) {
		dev_err(swrm->dev, "%s: list is empty\n", __func__);
		return;
	}
	for (i = 0; i < master->num_port; i++) {

		port = swrm_get_port(master, mstr_ports[mport->id]);
		if (!port)
			continue;
		port_type = mstr_port_type[mport->id];
		if (!port->dev_id || (port->dev_id >= SWR_NUM_SLV_DEVICES)) {
			dev_dbg(swrm->dev, "%s: invalid device id = %d\n",
				__func__, port->dev_id);
			continue;
		}
		value = ((port->ch_en)
				<< SWRM_DP_PORT_CTRL_EN_CHAN_SHFT);
		value |= ((port->offset2)
				<< SWRM_DP_PORT_CTRL_OFFSET2_SHFT);
		value |= ((port->offset1)
				<< SWRM_DP_PORT_CTRL_OFFSET1_SHFT);
		value |= port->sinterval;
		swrm->write(swrm->handle,
			    SWRM_DP_PORT_CTRL_BANK((mport->id+1), bank),
			    value);
		dev_dbg(swrm->dev, "%s: mport :%d, reg: 0x%x, val: 0x%x\n",
			__func__, mport->id,
			(SWRM_DP_PORT_CTRL_BANK((mport->id+1), bank)), value);

		swrm_cmd_fifo_wr_cmd(swrm, port->ch_en, port->dev_id, 0x0,
				SWRS_DP_CHANNEL_ENABLE_BANK(port_type, bank));
		swrm_cmd_fifo_wr_cmd(swrm, port->sinterval, port->dev_id, 0x0,
				SWRS_DP_SAMPLE_CONTROL_1_BANK(port_type, bank));
		swrm_cmd_fifo_wr_cmd(swrm, port->offset1, port->dev_id, 0x0,
				SWRS_DP_OFFSET_CONTROL_1_BANK(port_type, bank));

		if (port_type != 0)
			swrm_cmd_fifo_wr_cmd(swrm, port->offset2,
				port->dev_id, 0x00,
				SWRS_DP_OFFSET_CONTROL_2_BANK(port_type, bank));

		mport = list_next_entry(mport, list);
		if (!mport) {
			dev_err(swrm->dev, "%s: end of list\n", __func__);
			break;
		}
	}
	enable_bank_switch(swrm, bank, SWR_MAX_ROW, SWR_MAX_COL);
}

static int swrm_connect_port(struct swr_master *master,
			struct swr_params *portinfo)
{
	int i;
	struct swr_port_info *port;
	int ret = 0;
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(master);
	struct swrm_mports *mport;

	dev_dbg(&master->dev, "%s: enter\n", __func__);
	if (!portinfo)
		return -EINVAL;

	mutex_lock(&swrm->mlock);
	for (i = 0; i < portinfo->num_port; i++) {
		mport = kzalloc(sizeof(struct swrm_mports), GFP_KERNEL);
		if (!mport) {
			ret = -ENOMEM;
			goto mem_fail;
		}
		ret = swrm_get_master_port(&mport->id,
						portinfo->port_id[i]);
		if (ret < 0) {
			dev_err(&master->dev,
				"%s: mstr portid for slv port %d not found\n",
				__func__, portinfo->port_id[i]);
			goto port_fail;
		}
		list_add(&mport->list, &swrm->mport_list);
		port = &master->port[(master->num_port+i)];
		port->dev_id = portinfo->dev_id;
		port->port_id = portinfo->port_id[i];
		port->num_ch = portinfo->num_ch[i];
		port->ch_rate = portinfo->ch_rate[i];
		port->ch_en = portinfo->ch_en[i];
		port->port_en = true;
		dev_dbg(&master->dev,
			"%s: mstr port %d, slv port %d ch_rate %d num_ch %d\n",
			__func__, mport->id, port->port_id, port->ch_rate,
			port->num_ch);
	}
	master->num_port += portinfo->num_port;
	if (master->num_port >= SWR_MSTR_PORT_LEN)
		master->num_port = SWR_MSTR_PORT_LEN;

	swrm_get_port_config(master);
	swr_port_response(master, portinfo->tid);
	swrm_apply_port_config(master);
	mutex_unlock(&swrm->mlock);
	return 0;

port_fail:
	kfree(mport);
mem_fail:
	mutex_unlock(&swrm->mlock);
	return ret;
}

static int swrm_disconnect_port(struct swr_master *master,
			struct swr_params *portinfo)
{
	int i;
	struct swr_port_info *port;
	struct swrm_mports *mport;
	struct list_head *ptr, *next;
	u8 bank;
	int ret = 0;
	u8 mport_id = 0;
	int port_type = 0;
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(master);

	if (!portinfo) {
		dev_err(&master->dev, "%s: portinfo is NULL\n", __func__);
		return -EINVAL;
	}
	mutex_lock(&swrm->mlock);
	bank = get_inactive_bank_num(swrm);
	for (i = 0; i < portinfo->num_port; i++) {
		ret = swrm_get_master_port(&mport_id,
						portinfo->port_id[i]);
		if (ret < 0) {
			dev_err(&master->dev,
				"%s: mstr portid for slv port %d not found\n",
				__func__, portinfo->port_id[i]);
			mutex_unlock(&swrm->mlock);
			return -EINVAL;
		}
		port = swrm_get_port(master, portinfo->port_id[i]);
		if (!port) {
			dev_dbg(&master->dev, "%s: port %d already disabled\n",
				__func__, portinfo->port_id[i]);
			continue;
		}
		port_type = mstr_port_type[mport_id];
		port->dev_id = portinfo->dev_id;
		port->port_en = false;
		port->ch_en = 0;

		swrm->write(swrm->handle,
			    SWRM_DP_PORT_CTRL_BANK((mport_id+1), bank),
			    0);
		swrm_cmd_fifo_wr_cmd(swrm, 0x00, port->dev_id, 0x00,
				SWRS_DP_CHANNEL_ENABLE_BANK(port_type, bank));
		list_for_each_safe(ptr, next, &swrm->mport_list) {
			mport = list_entry(ptr, struct swrm_mports, list);
			if (mport->id == mport_id) {
				list_del(&mport->list);
				kfree(mport);
			}
		}
	}
	enable_bank_switch(swrm, bank, SWR_MAX_ROW, SWR_MAX_COL);
	if (master->num_port >= SWR_MSTR_PORT_LEN)
		master->num_port = SWR_MSTR_PORT_LEN;

	master->num_port -= portinfo->num_port;
	swr_port_response(master, portinfo->tid);
	mutex_unlock(&swrm->mlock);
	return 0;
}

static int swrm_check_slave_change_status(struct swr_mstr_ctrl *swrm,
					int status, u8 *devnum)
{
	int i;
	int new_sts = status;
	int ret = SWR_NOT_PRESENT;

	if (status != swrm->slave_status) {
		for (i = 0; i < SWR_NUM_SLV_DEVICES; i++) {
			if ((status & SWRM_MCP_SLV_STATUS_MASK) !=
			    (swrm->slave_status & SWRM_MCP_SLV_STATUS_MASK)) {
				ret = (status & SWRM_MCP_SLV_STATUS_MASK);
				*devnum = i;
				break;
			}
			status >>= 2;
			swrm->slave_status >>= 2;
		}
		swrm->slave_status = new_sts;
	}
	return ret;
}

static irqreturn_t swr_mstr_interrupt(int irq, void *dev)
{
	struct swr_mstr_ctrl *swrm = dev;
	u32 value, intr_sts;
	int status, chg_sts, i;
	u8 devnum;
	int ret = IRQ_HANDLED;

	intr_sts = swrm->read(swrm->handle, SWRM_INTERRUPT_STATUS);
	intr_sts &= SWRM_INTERRUPT_STATUS_RMSK;
	for (i = 0; i < SWRM_INTERRUPT_MAX; i++) {
		value = intr_sts & (1 << i);
		if (!value)
			continue;

		swrm->write(swrm->handle, SWRM_INTERRUPT_CLEAR, value);
		switch (value) {
		case SWRM_INTERRUPT_STATUS_SLAVE_PEND_IRQ:
			dev_dbg(swrm->dev, "SWR slave pend irq\n");
			break;
		case SWRM_INTERRUPT_STATUS_NEW_SLAVE_ATTACHED:
			dev_dbg(swrm->dev, "SWR new slave attached\n");
			break;
		case SWRM_INTERRUPT_STATUS_CHANGE_ENUM_SLAVE_STATUS:
			status = swrm->read(swrm->handle, SWRM_MCP_SLV_STATUS);
			chg_sts = swrm_check_slave_change_status(swrm, status,
								&devnum);
			switch (chg_sts) {
			case SWR_NOT_PRESENT:
				dev_dbg(swrm->dev, "device %d got detached\n",
					devnum);
				break;
			case SWR_ATTACHED_OK:
				dev_dbg(swrm->dev, "device %d got attached\n",
					devnum);
				break;
			case SWR_ALERT:
				dev_dbg(swrm->dev, "device %d has pending interrupt\n",
					devnum);
				break;
			}
			break;
		case SWRM_INTERRUPT_STATUS_MASTER_CLASH_DET:
			dev_err(swrm->dev, "SWR bus clash detected\n");
			break;
		case SWRM_INTERRUPT_STATUS_RD_FIFO_OVERFLOW:
			dev_dbg(swrm->dev, "SWR read FIFO overflow\n");
			break;
		case SWRM_INTERRUPT_STATUS_RD_FIFO_UNDERFLOW:
			dev_dbg(swrm->dev, "SWR read FIFO underflow\n");
			break;
		case SWRM_INTERRUPT_STATUS_WR_CMD_FIFO_OVERFLOW:
			dev_dbg(swrm->dev, "SWR write FIFO overflow\n");
			break;
		case SWRM_INTERRUPT_STATUS_CMD_ERROR:
			value = swrm->read(swrm->handle, SWRM_CMD_FIFO_STATUS);
			dev_err(swrm->dev,
			"SWR CMD error, CMD fifo status 0x%x, flushing fifo\n",
			value);
			swrm->write(swrm->handle, SWRM_CMD_FIFO_CMD, 0x1);
			break;
		case SWRM_INTERRUPT_STATUS_DOUT_PORT_COLLISION:
			dev_dbg(swrm->dev, "SWR Port collision detected\n");
			break;
		case SWRM_INTERRUPT_STATUS_READ_EN_RD_VALID_MISMATCH:
			dev_dbg(swrm->dev, "SWR read enable valid mismatch\n");
			break;
		case SWRM_INTERRUPT_STATUS_SPECIAL_CMD_ID_FINISHED:
			complete(&swrm->broadcast);
			dev_dbg(swrm->dev, "SWR cmd id finished\n");
			break;
		case SWRM_INTERRUPT_STATUS_NEW_SLAVE_AUTO_ENUM_FINISHED:
			break;
		case SWRM_INTERRUPT_STATUS_AUTO_ENUM_FAILED:
			break;
		case SWRM_INTERRUPT_STATUS_AUTO_ENUM_TABLE_IS_FULL:
			break;
		case SWRM_INTERRUPT_STATUS_BUS_RESET_FINISHED:
			complete(&swrm->reset);
			break;
		case SWRM_INTERRUPT_STATUS_CLK_STOP_FINISHED:
			break;
		default:
			dev_err(swrm->dev, "SWR unkown interrupt\n");
			ret = IRQ_NONE;
			break;
		}
	}
	return ret;
}

static int swrm_get_device_status(struct swr_mstr_ctrl *swrm, u8 devnum)
{
	u32 val;

	swrm->slave_status = swrm->read(swrm->handle, SWRM_MCP_SLV_STATUS);
	val = (swrm->slave_status >> (devnum * 2));
	val &= SWRM_MCP_SLV_STATUS_MASK;
	return val;
}

static int swrm_get_logical_dev_num(struct swr_master *mstr, u64 dev_id,
				u8 *dev_num)
{
	int i;
	u64 id;
	int ret = -EINVAL;
	struct swr_mstr_ctrl *swrm = swr_get_ctrl_data(mstr);

	for (i = 1; i < SWR_NUM_SLV_DEVICES; i++) {
		id = ((u64)(swrm->read(swrm->handle,
			    SWRM_ENUMERATOR_SLAVE_DEV_ID_2(i))) << 32);
		id |= swrm->read(swrm->handle,
			    SWRM_ENUMERATOR_SLAVE_DEV_ID_1(i));
		if (id == dev_id) {
			if (swrm_get_device_status(swrm, i) == 0x01) {
				*dev_num = i;
				ret = 0;
			} else {
				dev_err(swrm->dev, "%s: device is not ready\n",
					 __func__);
			}
			goto found;
		}
	}
	dev_err(swrm->dev, "%s: device id does not match\n", __func__);
found:
	return ret;
}

static int swrm_master_init(struct swr_mstr_ctrl *swrm)
{
	int ret = 0;
	u32 mask, val;
	u8 row_ctrl = SWR_MAX_ROW;
	u8 col_ctrl = SWR_MIN_COL;
	u8 ping_val = 0;
	u8 retry_cmd_num = 3;

	/* enable swr clock */
	swrm->clk(swrm->handle, true);

	/* Clear Rows and Cols */
	mask = (SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_BMSK |
		SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_BMSK);
	val = swrm->read(swrm->handle, SWRM_MCP_FRAME_CTRL_BANK_ADDR(0));
	val &= (~mask);
	val |= ((row_ctrl << SWRM_MCP_FRAME_CTRL_BANK_ROW_CTRL_SHFT) |
		(col_ctrl << SWRM_MCP_FRAME_CTRL_BANK_COL_CTRL_SHFT));
	swrm->write(swrm->handle, SWRM_MCP_FRAME_CTRL_BANK_ADDR(0), val);

	/* Set Auto enumeration flag */
	swrm->write(swrm->handle, SWRM_ENUMERATOR_CFG_ADDR, 1);

	/* Mask soundwire interrupts */
	swrm->write(swrm->handle, SWRM_INTERRUPT_MASK_ADDR, 0x1FFFD);

	/* Configure NO_PINGS */
	val = swrm->read(swrm->handle, SWRM_MCP_CFG_ADDR);
	val &= ~SWRM_MCP_CFG_MAX_NUM_OF_CMD_NO_PINGS_BMSK;
	val |= (ping_val << SWRM_MCP_CFG_MAX_NUM_OF_CMD_NO_PINGS_SHFT);
	swrm->write(swrm->handle, SWRM_MCP_CFG_ADDR, val);

	/* Configure number of retries of a read/write cmd */
	val = swrm->read(swrm->handle, SWRM_CMD_FIFO_CFG_ADDR);
	val &= ~SWRM_CMD_FIFO_CFG_NUM_OF_CMD_RETRY_BMSK;
	val |= (retry_cmd_num << SWRM_CMD_FIFO_CFG_NUM_OF_CMD_RETRY_SHFT);
	swrm->write(swrm->handle, SWRM_CMD_FIFO_CFG_ADDR, val);

	/* Set IRQ to PULSE */
	swrm->write(swrm->handle, SWRM_COMP_CFG_ADDR, 0x02);
	swrm->write(swrm->handle, SWRM_MCP_BUS_CTRL_ADDR, 0x2);
	swrm->write(swrm->handle, SWRM_COMP_CFG_ADDR, 0x03);
	return ret;
}

static int swrm_probe(struct platform_device *pdev)
{
	struct swr_mstr_ctrl *swrm;
	struct swr_ctrl_platform_data *pdata;
	int ret;

	/* Allocate soundwire master driver structure */
	swrm = kzalloc(sizeof(struct swr_mstr_ctrl), GFP_KERNEL);
	if (!swrm) {
		dev_err(&pdev->dev, "%s: no memory for swr mstr controller\n",
			 __func__);
		ret = -ENOMEM;
		goto err_memory_fail;
	}
	swrm->dev = &pdev->dev;
	platform_set_drvdata(pdev, swrm);
	swr_set_ctrl_data(&swrm->master, swrm);
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "%s: pdata from parent is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->handle = (void *)pdata->handle;
	if (!swrm->handle) {
		dev_err(&pdev->dev, "%s: swrm->handle is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->read = pdata->read;
	if (!swrm->read) {
		dev_err(&pdev->dev, "%s: swrm->read is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->write = pdata->write;
	if (!swrm->write) {
		dev_err(&pdev->dev, "%s: swrm->write is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->clk = pdata->clk;
	if (!swrm->clk) {
		dev_err(&pdev->dev, "%s: swrm->clk is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->reg_irq = pdata->reg_irq;
	if (!swrm->reg_irq) {
		dev_err(&pdev->dev, "%s: swrm->reg_irq is NULL\n",
			__func__);
		ret = -EINVAL;
		goto err_pdata_fail;
	}
	swrm->master.read = swrm_read;
	swrm->master.write = swrm_write;
	swrm->master.get_logical_dev_num = swrm_get_logical_dev_num;
	swrm->master.connect_port = swrm_connect_port;
	swrm->master.disconnect_port = swrm_disconnect_port;
	swrm->master.dev.parent = &pdev->dev;
	swrm->master.dev.of_node = pdev->dev.of_node;
	swrm->master.num_port = 0;
	swrm->num_enum_slaves = 0;
	swrm->rcmd_id = 0;
	swrm->wcmd_id = 0;
	swrm->slave_status = 0;
	init_completion(&swrm->reset);
	init_completion(&swrm->broadcast);
	mutex_init(&swrm->mlock);
	INIT_LIST_HEAD(&swrm->mport_list);

	ret = swrm->reg_irq(swrm->handle, swr_mstr_interrupt, swrm,
			    SWR_IRQ_REGISTER);
	if (ret) {
		dev_err(&pdev->dev, "%s: IRQ register failed ret %d\n",
			__func__, ret);
		goto err_irq_fail;
	}

	ret = swr_register_master(&swrm->master);
	if (ret) {
		dev_err(&pdev->dev, "%s: error adding swr master\n", __func__);
		goto err_mstr_fail;
	}

	if (pdev->dev.of_node)
		of_register_swr_devices(&swrm->master);

	/* Add devices registered with board-info as the
	   controller will be up now
	 */
	swr_master_add_boarddevices(&swrm->master);
	mutex_lock(&swrm->mlock);
	ret = swrm_master_init(swrm);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"%s: Error in master Initializaiton, err %d\n",
			__func__, ret);
		mutex_unlock(&swrm->mlock);
		goto err_mstr_fail;
	}
	mutex_unlock(&swrm->mlock);

	dbgswrm = swrm;
	debugfs_swrm_dent = debugfs_create_dir(dev_name(&pdev->dev), 0);
	if (!IS_ERR(debugfs_swrm_dent)) {
		debugfs_peek = debugfs_create_file("swrm_peek",
				S_IFREG | S_IRUGO, debugfs_swrm_dent,
				(void *) "swrm_peek", &swrm_debug_ops);

		debugfs_poke = debugfs_create_file("swrm_poke",
				S_IFREG | S_IRUGO, debugfs_swrm_dent,
				(void *) "swrm_poke", &swrm_debug_ops);

		debugfs_reg_dump = debugfs_create_file("swrm_reg_dump",
				   S_IFREG | S_IRUGO, debugfs_swrm_dent,
				   (void *) "swrm_reg_dump",
				   &swrm_debug_ops);
	}

	return 0;
err_mstr_fail:
	swrm->reg_irq(swrm->handle, swr_mstr_interrupt,
			swrm, SWR_IRQ_FREE);
err_irq_fail:
err_pdata_fail:
	kfree(swrm);
err_memory_fail:
	return ret;
}

static int swrm_remove(struct platform_device *pdev)
{
	struct swr_mstr_ctrl *swrm = platform_get_drvdata(pdev);

	swrm->reg_irq(swrm->handle, swr_mstr_interrupt,
			swrm, SWR_IRQ_FREE);
	if (swrm->mstr_port) {
		kfree(swrm->mstr_port->port);
		kfree(swrm->mstr_port);
	}
	swr_unregister_master(&swrm->master);
	mutex_destroy(&swrm->mlock);
	kfree(swrm);
	return 0;
}

static struct of_device_id swrm_dt_match[] = {
	{
		.compatible = "qcom,swr-wcd",
	},
	{}
};

static struct platform_driver swr_mstr_driver = {
	.probe = swrm_probe,
	.remove = swrm_remove,
	.driver = {
		.name = SWR_WCD_NAME,
		.owner = THIS_MODULE,
		.of_match_table = swrm_dt_match,
	},
};

static int __init swrm_init(void)
{
	return platform_driver_register(&swr_mstr_driver);
}
subsys_initcall(swrm_init);

static void __exit swrm_exit(void)
{
	platform_driver_unregister(&swr_mstr_driver);
}
module_exit(swrm_exit);


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("WCD SoundWire Controller");
MODULE_ALIAS("platform:swr-wcd");
