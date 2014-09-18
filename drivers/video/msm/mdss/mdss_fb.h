/* Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef MDSS_FB_H
#define MDSS_FB_H

#include <linux/msm_ion.h>
#include <linux/list.h>
#include <linux/msm_mdp.h>
#include <linux/types.h>

#include "mdss_panel.h"

#define MSM_FB_DEFAULT_PAGE_SIZE 2
#define MFD_KEY  0x11161126
#define MSM_FB_MAX_DEV_LIST 32

#define MSM_FB_ENABLE_DBGFS
#define WAIT_FENCE_FIRST_TIMEOUT MSEC_PER_SEC
#define WAIT_FENCE_FINAL_TIMEOUT (10 * MSEC_PER_SEC)
/* Display op timeout should be greater than total timeout */
#define WAIT_DISP_OP_TIMEOUT ((WAIT_FENCE_FIRST_TIMEOUT + \
		WAIT_FENCE_FINAL_TIMEOUT) * MDP_MAX_FENCE_FD)

#ifndef MAX
#define  MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif

#ifndef MIN
#define  MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

struct disp_info_type_suspend {
	int op_enable;
	int panel_power_on;
};

struct disp_info_notify {
	int type;
	struct timer_list timer;
	struct completion comp;
	struct mutex lock;
};

struct msm_fb_data_type;

struct msm_mdp_interface {
	int (*fb_mem_alloc_fnc)(struct msm_fb_data_type *mfd);
	int (*init_fnc)(struct msm_fb_data_type *mfd);
	int (*on_fnc)(struct msm_fb_data_type *mfd);
	int (*off_fnc)(struct msm_fb_data_type *mfd);
	int (*kickoff_fnc)(struct msm_fb_data_type *mfd);
	int (*ioctl_handler)(struct msm_fb_data_type *mfd, u32 cmd, void *arg);
	void (*dma_fnc)(struct msm_fb_data_type *mfd);
	int (*cursor_update)(struct msm_fb_data_type *mfd,
				struct fb_cursor *cursor);
	int (*lut_update)(struct msm_fb_data_type *mfd, struct fb_cmap *cmap);
	int (*do_histogram)(struct msm_fb_data_type *mfd,
				struct mdp_histogram *hist);
	int (*panel_register_done)(struct mdss_panel_data *pdata);
	u32 (*fb_stride)(u32 fb_index, u32 xres, int bpp);
	void *private1;
};

struct msm_fb_data_type {
	u32 key;
	u32 index;
	u32 ref_cnt;
	u32 fb_page;

	struct panel_id panel;
	struct mdss_panel_info *panel_info;
	int split_display;

	u32 dest;
	struct fb_info *fbi;

	int op_enable;
	u32 fb_imgType;
	int panel_reconfig;

	u32 dst_format;
	int panel_power_on;
	struct disp_info_type_suspend suspend;

	struct ion_handle *ihdl;
	unsigned long iova;
	void *cursor_buf;
	unsigned long cursor_buf_phys;
	unsigned long cursor_buf_iova;

	u32 bl_level;
	u32 bl_scale;
	u32 bl_min_lvl;
	struct mutex lock;

	struct platform_device *pdev;

	u32 mdp_fb_page_protection;

	struct disp_info_notify update;
	struct disp_info_notify no_update;

	struct msm_mdp_interface mdp;

	u32 acq_fen_cnt;
	struct sync_fence *acq_fen[MDP_MAX_FENCE_FD];
	int cur_rel_fen_fd;
	struct sync_pt *cur_rel_sync_pt;
	struct sync_fence *cur_rel_fence;
	struct sync_fence *last_rel_fence;
	struct sw_sync_timeline *timeline;
	int timeline_value;
	u32 last_acq_fen_cnt;
	struct sync_fence *last_acq_fen[MDP_MAX_FENCE_FD];
	struct mutex sync_mutex;
	/* for non-blocking */
	struct completion commit_comp;
	u32 is_committing;
	struct work_struct commit_work;
	void *msm_fb_backup;
	struct completion power_set_comp;
	u32 is_power_setting;
};

struct msm_fb_backup_type {
	struct fb_info info;
	struct mdp_display_commit disp_commit;
};

int mdss_fb_get_phys_info(unsigned long *start, unsigned long *len, int fb_num);
void mdss_fb_set_backlight(struct msm_fb_data_type *mfd, u32 bkl_lvl);
void mdss_fb_update_backlight(struct msm_fb_data_type *mfd);
void mdss_fb_wait_for_fence(struct msm_fb_data_type *mfd);
void mdss_fb_signal_timeline(struct msm_fb_data_type *mfd);
int mdss_fb_register_mdp_instance(struct msm_mdp_interface *mdp);
#endif /* MDSS_FB_H */
