/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#ifndef MDSS_MDP_ROTATOR_H
#define MDSS_MDP_ROTATOR_H

#include <linux/types.h>

#include "mdss_mdp.h"

#define MDSS_MDP_ROT_SESSION_MASK	0x40000000

struct mdss_mdp_rotator_session {
	u32 session_id;
	u32 ref_cnt;
	u32 params_changed;

	u32 format;
	u32 flags;

	u16 img_width;
	u16 img_height;
	struct mdss_mdp_img_rect src_rect;

	u32 bwc_mode;
	struct mdss_mdp_pipe *pipe;

	struct mutex lock;
	struct completion comp;
	u8 busy;
	u8 no_wait;

	struct list_head head;
};

static inline u32 mdss_mdp_get_rotator_dst_format(u32 in_format)
{
	switch (in_format) {
	case MDP_RGB_565:
	case MDP_BGR_565:
		return MDP_RGB_888;
	case MDP_Y_CBCR_H2V2_VENUS:
	case MDP_Y_CB_CR_H2V2:
	case MDP_Y_CR_CB_GH2V2:
	case MDP_Y_CR_CB_H2V2:
		return MDP_Y_CRCB_H2V2;
	default:
		return in_format;
	}
}

struct mdss_mdp_rotator_session *mdss_mdp_rotator_session_alloc(void);
struct mdss_mdp_rotator_session *mdss_mdp_rotator_session_get(u32 session_id);

int mdss_mdp_rotator_queue(struct mdss_mdp_rotator_session *rot,
			   struct mdss_mdp_data *src_data,
			   struct mdss_mdp_data *dst_data);

int mdss_mdp_rotator_release(u32 ndx);
int mdss_mdp_rotator_release_all(void);

#endif /* MDSS_MDP_ROTATOR_H */
