/* Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <mach/iommu_domains.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_mdp_rotator.h"

#define VSYNC_PERIOD 16
#define BORDERFILL_NDX	0x0BF000BF
#define CHECK_BOUNDS(offset, size, max_size) \
	(((size) > (max_size)) || ((offset) > ((max_size) - (size))))

static atomic_t ov_active_panels = ATOMIC_INIT(0);
static int mdss_mdp_overlay_free_fb_pipe(struct msm_fb_data_type *mfd);

static int mdss_mdp_overlay_get(struct msm_fb_data_type *mfd,
				struct mdp_overlay *req)
{
	struct mdss_mdp_pipe *pipe;

	pipe = mdss_mdp_pipe_get_locked(req->id);
	if (pipe == NULL) {
		pr_err("invalid pipe ndx=%x\n", req->id);
		return -ENODEV;
	}

	*req = pipe->req_data;
	mdss_mdp_pipe_unlock(pipe);

	return 0;
}

static int mdss_mdp_overlay_req_check(struct msm_fb_data_type *mfd,
				      struct mdp_overlay *req,
				      struct mdss_mdp_format_params *fmt)
{
	u32 xres, yres;

	xres = mfd->fbi->var.xres;
	yres = mfd->fbi->var.yres;

	if (req->z_order >= MDSS_MDP_MAX_STAGE) {
		pr_err("zorder %d out of range\n", req->z_order);
		return -ERANGE;
	}

	if (req->src.width > MAX_IMG_WIDTH ||
	    req->src.height > MAX_IMG_HEIGHT ||
	    req->src_rect.w == 0 || req->src_rect.h == 0 ||
	    CHECK_BOUNDS(req->src_rect.x, req->src_rect.w, req->src.width) ||
	    CHECK_BOUNDS(req->src_rect.y, req->src_rect.h, req->src.height)) {
		pr_err("invalid source image img wh=%dx%d rect=%d,%d,%d,%d\n",
		       req->src.width, req->src.height,
		       req->src_rect.x, req->src_rect.y,
		       req->src_rect.w, req->src_rect.h);
		return -EOVERFLOW;
	}

	if (req->dst_rect.w < MIN_DST_W || req->dst_rect.h < MIN_DST_H ||
	    req->dst_rect.w > MAX_DST_W || req->dst_rect.h > MAX_DST_H) {
		pr_err("invalid destination resolution (%dx%d)",
		       req->dst_rect.w, req->dst_rect.h);
		return -EOVERFLOW;
	}

	if (req->flags & MDSS_MDP_ROT_ONLY) {
		/* dst res should match src res in rotation only mode*/
		req->dst_rect.w = req->src_rect.w;
		req->dst_rect.h = req->src_rect.h;
	} else {
		u32 dst_w, dst_h;

		if ((CHECK_BOUNDS(req->dst_rect.x, req->dst_rect.w, xres) ||
		     CHECK_BOUNDS(req->dst_rect.y, req->dst_rect.h, yres))) {
			pr_err("invalid destination rect=%d,%d,%d,%d\n",
			       req->dst_rect.x, req->dst_rect.y,
			       req->dst_rect.w, req->dst_rect.h);
			return -EOVERFLOW;
		}

		if (req->flags & MDP_ROT_90) {
			dst_h = req->dst_rect.w;
			dst_w = req->dst_rect.h;
		} else {
			dst_w = req->dst_rect.w;
			dst_h = req->dst_rect.h;
		}

		if ((req->src_rect.w * MAX_UPSCALE_RATIO) < dst_w) {
			pr_err("too much upscaling Width %d->%d\n",
			       req->src_rect.w, req->dst_rect.w);
			return -EINVAL;
		}

		if ((req->src_rect.h * MAX_UPSCALE_RATIO) < dst_h) {
			pr_err("too much upscaling. Height %d->%d\n",
			       req->src_rect.h, req->dst_rect.h);
			return -EINVAL;
		}

		if (req->src_rect.w > (dst_w * MAX_DOWNSCALE_RATIO)) {
			pr_err("too much downscaling. Width %d->%d\n",
			       req->src_rect.w, req->dst_rect.w);
			return -EINVAL;
		}

		if (req->src_rect.h > (dst_h * MAX_DOWNSCALE_RATIO)) {
			pr_err("too much downscaling. Height %d->%d\n",
			       req->src_rect.h, req->dst_rect.h);
			return -EINVAL;
		}

		if ((fmt->chroma_sample == MDSS_MDP_CHROMA_420 ||
		     fmt->chroma_sample == MDSS_MDP_CHROMA_H2V1) &&
		    ((req->src_rect.w * (MAX_UPSCALE_RATIO / 2)) < dst_w)) {
			pr_err("too much YUV upscaling Width %d->%d\n",
			       req->src_rect.w, req->dst_rect.w);
			return -EINVAL;
		}

		if ((fmt->chroma_sample == MDSS_MDP_CHROMA_420 ||
		     fmt->chroma_sample == MDSS_MDP_CHROMA_H1V2) &&
		    (req->src_rect.h * (MAX_UPSCALE_RATIO / 2)) < dst_h) {
			pr_err("too much YUV upscaling Height %d->%d\n",
			       req->src_rect.h, req->dst_rect.h);
			return -EINVAL;
		}
	}

	if (fmt->is_yuv) {
		if ((req->src_rect.x & 0x1) || (req->src_rect.y & 0x1) ||
		    (req->src_rect.w & 0x1) || (req->src_rect.h & 0x1)) {
			pr_err("invalid odd src resolution or coordinates\n");
			return -EINVAL;
		}
		if ((req->dst_rect.w & 0x1) || (req->dst_rect.h & 0x1)) {
			pr_err("invalid odd dst resolution\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int mdss_mdp_overlay_rotator_setup(struct msm_fb_data_type *mfd,
					  struct mdp_overlay *req)
{
	struct mdss_mdp_rotator_session *rot;
	struct mdss_mdp_format_params *fmt;
	int ret = 0;

	pr_debug("rot ctl=%u req id=%x\n", mfd->ctl->num, req->id);

	fmt = mdss_mdp_get_format_params(req->src.format);
	if (!fmt) {
		pr_err("invalid rot format %d\n", req->src.format);
		return -EINVAL;
	}

	ret = mdss_mdp_overlay_req_check(mfd, req, fmt);
	if (ret)
		return ret;

	if (req->id == MSMFB_NEW_REQUEST) {
		rot = mdss_mdp_rotator_session_alloc();

		if (!rot) {
			pr_err("unable to allocate rotator session\n");
			return -ENOMEM;
		}
	} else if (req->id & MDSS_MDP_ROT_SESSION_MASK) {
		rot = mdss_mdp_rotator_session_get(req->id);

		if (!rot) {
			pr_err("rotator session=%x not found\n", req->id);
			return -ENODEV;
		}
	} else {
		pr_err("invalid rotator session id=%x\n", req->id);
		return -EINVAL;
	}

	/* keep only flags of interest to rotator */
	rot->flags = req->flags & (MDP_ROT_90 | MDP_FLIP_LR | MDP_FLIP_UD |
				   MDP_SECURE_OVERLAY_SESSION);

	rot->format = fmt->format;
	rot->img_width = req->src.width;
	rot->img_height = req->src.height;
	rot->src_rect.x = req->src_rect.x;
	rot->src_rect.y = req->src_rect.y;
	rot->src_rect.w = req->src_rect.w;
	rot->src_rect.h = req->src_rect.h;

	if (req->flags & MDP_DEINTERLACE) {
		rot->flags |= MDP_DEINTERLACE;
		rot->src_rect.h /= 2;
	}

	rot->params_changed++;

	req->id = rot->session_id;

	return ret;
}

static int mdss_mdp_overlay_pipe_setup(struct msm_fb_data_type *mfd,
				       struct mdp_overlay *req,
				       struct mdss_mdp_pipe **ppipe)
{
	struct mdss_mdp_format_params *fmt;
	struct mdss_mdp_pipe *pipe;
	struct mdss_mdp_mixer *mixer = NULL;
	u32 pipe_type, mixer_mux;
	int ret;

	if (mfd == NULL || mfd->ctl == NULL)
		return -ENODEV;

	if (req->flags & MDSS_MDP_RIGHT_MIXER)
		mixer_mux = MDSS_MDP_MIXER_MUX_RIGHT;
	else
		mixer_mux = MDSS_MDP_MIXER_MUX_LEFT;

	pr_debug("pipe ctl=%u req id=%x mux=%d\n", mfd->ctl->num, req->id,
			mixer_mux);

	if (req->flags & MDP_ROT_90) {
		pr_err("unsupported inline rotation\n");
		return -ENOTSUPP;
	}

	fmt = mdss_mdp_get_format_params(req->src.format);
	if (!fmt) {
		pr_err("invalid pipe format %d\n", req->src.format);
		return -EINVAL;
	}

	ret = mdss_mdp_overlay_req_check(mfd, req, fmt);
	if (ret)
		return ret;

	pipe = mdss_mdp_mixer_stage_pipe(mfd->ctl, mixer_mux, req->z_order);
	if (pipe && pipe->ndx != req->id) {
		pr_err("stage %d taken by pnum=%d\n", req->z_order, pipe->num);
		return -EBUSY;
	}


	if (req->id == MSMFB_NEW_REQUEST) {
		mixer = mdss_mdp_mixer_get(mfd->ctl, mixer_mux);
		if (!mixer) {
			pr_err("unable to get mixer\n");
			return -ENODEV;
		}

		if (fmt->is_yuv || (req->flags & MDP_OV_PIPE_SHARE))
			pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
		else
			pipe_type = MDSS_MDP_PIPE_TYPE_RGB;

		pipe = mdss_mdp_pipe_alloc_locked(pipe_type);

		/* VIG pipes can also support RGB format */
		if (!pipe && pipe_type == MDSS_MDP_PIPE_TYPE_RGB) {
			pipe_type = MDSS_MDP_PIPE_TYPE_VIG;
			pipe = mdss_mdp_pipe_alloc_locked(pipe_type);
		}

		if (pipe == NULL) {
			pr_err("error allocating pipe\n");
			return -ENOMEM;
		}

		mutex_lock(&mfd->lock);
		list_add(&pipe->used_list, &mfd->pipes_used);
		mutex_unlock(&mfd->lock);
		pipe->mixer = mixer;
		pipe->mfd = mfd;
	} else {
		pipe = mdss_mdp_pipe_get_locked(req->id);
		if (pipe == NULL) {
			pr_err("invalid pipe ndx=%x\n", req->id);
			return -ENODEV;
		}
	}

	pipe->flags = req->flags;

	pipe->img_width = req->src.width & 0x3fff;
	pipe->img_height = req->src.height & 0x3fff;
	pipe->src.x = req->src_rect.x;
	pipe->src.y = req->src_rect.y;
	pipe->src.w = req->src_rect.w;
	pipe->src.h = req->src_rect.h;
	pipe->dst.x = req->dst_rect.x;
	pipe->dst.y = req->dst_rect.y;
	pipe->dst.w = req->dst_rect.w;
	pipe->dst.h = req->dst_rect.h;

	pipe->src_fmt = fmt;

	pipe->mixer_stage = req->z_order;
	pipe->is_fg = req->is_fg;
	pipe->alpha = req->alpha;
	pipe->transp = req->transp_mask;
	pipe->overfetch_disable = fmt->is_yuv;

	pipe->req_data = *req;

	if (pipe->flags & MDP_OVERLAY_PP_CFG_EN) {
		if (pipe->num <= MDSS_MDP_SSPP_VIG2)
			memcpy(&pipe->pp_cfg, &req->overlay_pp_cfg,
					sizeof(struct mdp_overlay_pp_params));
		else
			pr_debug("%s: RGB Pipes don't support CSC/QSEED\n",
								__func__);
	}

	if (pipe->flags & MDP_DEINTERLACE) {
		if (pipe->flags & MDP_SOURCE_ROTATED_90) {
			pipe->src.w /= 2;
			pipe->img_width /= 2;
		} else {
			pipe->src.h /= 2;
		}
	}

	pipe->params_changed++;

	req->id = pipe->ndx;

	*ppipe = pipe;

	mdss_mdp_pipe_unlock(pipe);

	return ret;
}

static int mdss_mdp_overlay_set(struct msm_fb_data_type *mfd,
				struct mdp_overlay *req)
{
	int ret = 0;

	ret = mutex_lock_interruptible(&mfd->ov_lock);
	if (ret)
		return ret;

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->ov_lock);
		return -EPERM;
	}

	if (req->flags & MDSS_MDP_ROT_ONLY) {
		ret = mdss_mdp_overlay_rotator_setup(mfd, req);
	} else if (req->src.format == MDP_RGB_BORDERFILL) {
		req->id = BORDERFILL_NDX;
	} else {
		struct mdss_mdp_pipe *pipe;

		/* userspace zorder start with stage 0 */
		req->z_order += MDSS_MDP_STAGE_0;

		ret = mdss_mdp_overlay_pipe_setup(mfd, req, &pipe);

		req->z_order -= MDSS_MDP_STAGE_0;
	}

	mutex_unlock(&mfd->ov_lock);

	return ret;
}

static inline int mdss_mdp_overlay_get_buf(struct msm_fb_data_type *mfd,
					   struct mdss_mdp_data *data,
					   struct msmfb_data *planes,
					   int num_planes,
					   u32 flags)
{
	int i;

	memset(data, 0, sizeof(*data));
	for (i = 0; i < num_planes; i++) {
		data->p[i].flags = flags;
		mdss_mdp_get_img(&planes[i], &data->p[i]);
		if (data->p[0].len == 0)
			break;
	}

	if (i != num_planes) {
		for (; i >= 0; i--)
			mdss_mdp_put_img(&data->p[i]);
		return -ENOMEM;
	}

	data->num_planes = num_planes;

	return 0;
}

static inline int mdss_mdp_overlay_free_buf(struct mdss_mdp_data *data)
{
	int i;
	for (i = 0; i < data->num_planes && data->p[i].len; i++)
		mdss_mdp_put_img(&data->p[i]);

	data->num_planes = 0;

	return 0;
}

static int mdss_mdp_overlay_cleanup(struct msm_fb_data_type *mfd)
{
	struct mdss_mdp_pipe *pipe, *tmp;
	LIST_HEAD(destroy_pipes);

	mutex_lock(&mfd->lock);
	list_for_each_entry_safe(pipe, tmp, &mfd->pipes_cleanup, cleanup_list) {
		list_move(&pipe->cleanup_list, &destroy_pipes);
		mdss_mdp_overlay_free_buf(&pipe->back_buf);
		mdss_mdp_overlay_free_buf(&pipe->front_buf);
	}

	list_for_each_entry(pipe, &mfd->pipes_used, used_list) {
		if (pipe->back_buf.num_planes) {
			/* make back buffer active */
			mdss_mdp_overlay_free_buf(&pipe->front_buf);
			swap(pipe->back_buf, pipe->front_buf);
		}
	}
	mutex_unlock(&mfd->lock);
	list_for_each_entry_safe(pipe, tmp, &destroy_pipes, cleanup_list)
		mdss_mdp_pipe_destroy(pipe);

	return 0;
}

int mdss_mdp_overlay_kickoff(struct mdss_mdp_ctl *ctl)
{
	struct msm_fb_data_type *mfd = ctl->mfd;
	int ret;

	mutex_lock(&mfd->ov_lock);

	if (mfd->kickoff_fnc)
		ret = mfd->kickoff_fnc(ctl);
	else
		ret = mdss_mdp_display_commit(ctl, NULL);
	if (IS_ERR_VALUE(ret)) {
		mutex_unlock(&mfd->ov_lock);
		return ret;
	}

	complete(&mfd->update.comp);
	mutex_lock(&mfd->no_update.lock);
	if (mfd->no_update.timer.function)
		del_timer(&(mfd->no_update.timer));

	mfd->no_update.timer.expires = jiffies + (2 * HZ);
	add_timer(&mfd->no_update.timer);
	mutex_unlock(&mfd->no_update.lock);

	ret = mdss_mdp_overlay_cleanup(mfd);

	mutex_unlock(&mfd->ov_lock);

	return ret;
}

static int mdss_mdp_overlay_release(struct msm_fb_data_type *mfd, int ndx)
{
	struct mdss_mdp_pipe *pipe;
	u32 pipe_ndx, unset_ndx = 0;
	int i;

	for (i = 0; unset_ndx != ndx && i < MDSS_MDP_MAX_SSPP; i++) {
		pipe_ndx = BIT(i);
		if (pipe_ndx & ndx) {
			unset_ndx |= pipe_ndx;
			pipe = mdss_mdp_pipe_get_locked(pipe_ndx);
			if (!pipe) {
				pr_warn("unknown pipe ndx=%x\n", pipe_ndx);
				continue;
			}
			mutex_lock(&mfd->lock);
			list_del(&pipe->used_list);
			list_add(&pipe->cleanup_list, &mfd->pipes_cleanup);
			mutex_unlock(&mfd->lock);
			mdss_mdp_mixer_pipe_unstage(pipe);
		}
	}
	return 0;
}

static int mdss_mdp_overlay_unset(struct msm_fb_data_type *mfd, int ndx)
{
	int ret = 0;

	if (!mfd || !mfd->ctl)
		return -ENODEV;

	ret = mutex_lock_interruptible(&mfd->ov_lock);
	if (ret)
		return ret;

	if (ndx == BORDERFILL_NDX) {
		pr_debug("borderfill disable\n");
		mfd->borderfill_enable = false;
		return 0;
	}

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->ov_lock);
		return -EPERM;
	}

	pr_debug("unset ndx=%x\n", ndx);

	if (ndx & MDSS_MDP_ROT_SESSION_MASK)
		ret = mdss_mdp_rotator_release(ndx);
	else
		ret = mdss_mdp_overlay_release(mfd, ndx);

	mutex_unlock(&mfd->ov_lock);

	return ret;
}

static int mdss_mdp_overlay_release_all(struct msm_fb_data_type *mfd)
{
	struct mdss_mdp_pipe *pipe;
	u32 unset_ndx = 0;
	int cnt = 0;

	mutex_lock(&mfd->ov_lock);
	mutex_lock(&mfd->lock);
	list_for_each_entry(pipe, &mfd->pipes_used, used_list) {
		unset_ndx |= pipe->ndx;
		cnt++;
	}
	mutex_unlock(&mfd->lock);

	if (unset_ndx) {
		pr_debug("%d pipes need cleanup (%x)\n", cnt, unset_ndx);
		mdss_mdp_overlay_release(mfd, unset_ndx);
	}
	mutex_unlock(&mfd->ov_lock);

	if (cnt)
		mdss_mdp_overlay_kickoff(mfd->ctl);

	return 0;
}

static int mdss_mdp_overlay_play_wait(struct msm_fb_data_type *mfd,
				      struct msmfb_overlay_data *req)
{
	int ret;

	if (!mfd || !mfd->ctl)
		return -ENODEV;

	ret = mdss_mdp_overlay_kickoff(mfd->ctl);
	if (!ret)
		pr_err("error displaying\n");

	return ret;
}

static int mdss_mdp_overlay_rotate(struct msm_fb_data_type *mfd,
				   struct msmfb_overlay_data *req)
{
	struct mdss_mdp_rotator_session *rot;
	struct mdss_mdp_data src_data, dst_data;
	int ret;
	u32 flgs;

	rot = mdss_mdp_rotator_session_get(req->id);
	if (!rot) {
		pr_err("invalid session id=%x\n", req->id);
		return -ENOENT;
	}

	flgs = rot->flags & MDP_SECURE_OVERLAY_SESSION;

	ret = mdss_mdp_overlay_get_buf(mfd, &src_data, &req->data, 1, flgs);
	if (ret) {
		pr_err("src_data pmem error\n");
		goto rotate_done;
	}

	ret = mdss_mdp_overlay_get_buf(mfd, &dst_data, &req->dst_data, 1, flgs);
	if (ret) {
		pr_err("dst_data pmem error\n");
		goto rotate_done;
	}

	ret = mdss_mdp_rotator_queue(rot, &src_data, &dst_data);
	if (ret) {
		pr_err("rotator queue error session id=%x\n", req->id);
		goto rotate_done;
	}

rotate_done:
	mdss_mdp_overlay_free_buf(&dst_data);
	mdss_mdp_overlay_free_buf(&src_data);

	return 0;
}

static int mdss_mdp_overlay_queue(struct msm_fb_data_type *mfd,
				  struct msmfb_overlay_data *req)
{
	struct mdss_mdp_ctl *ctl;
	struct mdss_mdp_pipe *pipe;
	struct mdss_mdp_data *src_data;
	int ret;
	u32 flags;

	pipe = mdss_mdp_pipe_get_locked(req->id);
	if (pipe == NULL) {
		pr_err("pipe ndx=%x doesn't exist\n", req->id);
		return -ENODEV;
	}

	pr_debug("ov queue pnum=%d\n", pipe->num);

	flags = (pipe->flags & MDP_SECURE_OVERLAY_SESSION);

	src_data = &pipe->back_buf;
	if (src_data->num_planes) {
		pr_warn("dropped buffer pnum=%d play=%d addr=0x%x\n",
			pipe->num, pipe->play_cnt, src_data->p[0].addr);
		mdss_mdp_overlay_free_buf(src_data);
	}

	ret = mdss_mdp_overlay_get_buf(mfd, src_data, &req->data, 1, flags);
	if (IS_ERR_VALUE(ret)) {
		pr_err("src_data pmem error\n");
	} else {
		ret = mdss_mdp_pipe_queue_data(pipe, src_data);
		if (IS_ERR_VALUE(ret))
			mdss_mdp_overlay_free_buf(src_data);
	}
	ctl = pipe->mixer->ctl;
	mdss_mdp_pipe_unlock(pipe);

	return ret;
}

static int mdss_mdp_overlay_play(struct msm_fb_data_type *mfd,
				 struct msmfb_overlay_data *req)
{
	int ret = 0;

	pr_debug("play req id=%x\n", req->id);

	ret = mutex_lock_interruptible(&mfd->ov_lock);
	if (ret)
		return ret;

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->ov_lock);
		return -EPERM;
	}

	if (req->id & MDSS_MDP_ROT_SESSION_MASK) {
		ret = mdss_mdp_overlay_rotate(mfd, req);
	} else if (req->id == BORDERFILL_NDX) {
		pr_debug("borderfill enable\n");
		mfd->borderfill_enable = true;
		ret = mdss_mdp_overlay_free_fb_pipe(mfd);
	} else {
		ret = mdss_mdp_overlay_queue(mfd, req);

		if ((ret == 0) && (mfd->panel.type == WRITEBACK_PANEL)) {
			mutex_unlock(&mfd->ov_lock);
			ret = mdss_mdp_overlay_kickoff(mfd->ctl);
			return ret;
		}
	}

	mutex_unlock(&mfd->ov_lock);

	return ret;
}

static int mdss_mdp_overlay_free_fb_pipe(struct msm_fb_data_type *mfd)
{
	struct mdss_mdp_pipe *pipe;
	u32 fb_ndx = 0;

	pipe = mdss_mdp_mixer_stage_pipe(mfd->ctl, MDSS_MDP_MIXER_MUX_LEFT,
					 MDSS_MDP_STAGE_BASE);
	if (pipe)
		fb_ndx |= pipe->ndx;

	pipe = mdss_mdp_mixer_stage_pipe(mfd->ctl, MDSS_MDP_MIXER_MUX_RIGHT,
					 MDSS_MDP_STAGE_BASE);
	if (pipe)
		fb_ndx |= pipe->ndx;

	if (fb_ndx) {
		pr_debug("unstaging framebuffer pipes %x\n", fb_ndx);
		mdss_mdp_overlay_unset(mfd, fb_ndx);
		mdss_mdp_overlay_kickoff(mfd->ctl);
	}
	return 0;
}

static int mdss_mdp_overlay_get_fb_pipe(struct msm_fb_data_type *mfd,
					struct mdss_mdp_pipe **ppipe,
					int mixer_mux)
{
	struct mdss_mdp_pipe *pipe;

	pipe = mdss_mdp_mixer_stage_pipe(mfd->ctl, mixer_mux,
					 MDSS_MDP_STAGE_BASE);
	if (pipe == NULL) {
		struct mdp_overlay req;
		struct fb_info *fbi = mfd->fbi;
		struct mdss_mdp_mixer *mixer;
		int ret, bpp;

		mixer = mdss_mdp_mixer_get(mfd->ctl, MDSS_MDP_MIXER_MUX_LEFT);
		if (!mixer) {
			pr_err("unable to retrieve mixer\n");
			return -ENODEV;
		}

		memset(&req, 0, sizeof(req));

		bpp = fbi->var.bits_per_pixel / 8;
		req.id = MSMFB_NEW_REQUEST;
		req.src.format = mfd->fb_imgType;
		req.src.height = fbi->var.yres;
		req.src.width = fbi->fix.line_length / bpp;
		if (mixer_mux == MDSS_MDP_MIXER_MUX_RIGHT) {
			if (req.src.width <= mixer->width) {
				pr_warn("right fb pipe not needed\n");
				return -EINVAL;
			}

			req.flags |= MDSS_MDP_RIGHT_MIXER;
			req.src_rect.x = mixer->width;
			req.src_rect.w = fbi->var.xres - mixer->width;
		} else {
			req.src_rect.x = 0;
			req.src_rect.w = MIN(fbi->var.xres, mixer->width);
		}

		req.src_rect.y = 0;
		req.src_rect.h = req.src.height;
		req.dst_rect.x = 0;
		req.dst_rect.y = 0;
		req.dst_rect.w = req.src_rect.w;
		req.dst_rect.h = req.src_rect.h;
		req.z_order = MDSS_MDP_STAGE_BASE;

		pr_debug("allocating base pipe mux=%d\n", mixer_mux);

		ret = mdss_mdp_overlay_pipe_setup(mfd, &req, &pipe);
		if (ret)
			return ret;

		pr_debug("ctl=%d pnum=%d\n", mfd->ctl->num, pipe->num);
	}

	*ppipe = pipe;
	return 0;
}

static void mdss_mdp_overlay_pan_display(struct msm_fb_data_type *mfd)
{
	struct mdss_mdp_data data;
	struct mdss_mdp_pipe *pipe;
	struct fb_info *fbi;
	u32 offset;
	int bpp, ret;

	if (!mfd || !mfd->ctl)
		return;

	fbi = mfd->fbi;

	if (fbi->fix.smem_len == 0 || mfd->borderfill_enable) {
		mdss_mdp_overlay_kickoff(mfd->ctl);
		return;
	}

	if (mutex_lock_interruptible(&mfd->ov_lock))
		return;

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->ov_lock);
		return;
	}

	memset(&data, 0, sizeof(data));

	bpp = fbi->var.bits_per_pixel / 8;
	offset = fbi->var.xoffset * bpp +
		 fbi->var.yoffset * fbi->fix.line_length;

	if (offset > fbi->fix.smem_len) {
		pr_err("invalid fb offset=%u total length=%u\n",
		       offset, fbi->fix.smem_len);
		return;
	}

	if (is_mdss_iommu_attached())
		data.p[0].addr = mfd->iova;
	else
		data.p[0].addr = fbi->fix.smem_start;

	data.p[0].addr += offset;
	data.p[0].len = fbi->fix.smem_len - offset;
	data.num_planes = 1;

	ret = mdss_mdp_overlay_get_fb_pipe(mfd, &pipe, MDSS_MDP_MIXER_MUX_LEFT);
	if (ret) {
		pr_err("unable to allocate base pipe\n");
		return;
	}

	mdss_mdp_pipe_lock(pipe);
	ret = mdss_mdp_pipe_queue_data(pipe, &data);
	mdss_mdp_pipe_unlock(pipe);
	if (ret) {
		pr_err("unable to queue data\n");
		return;
	}

	if (fbi->var.xres > MAX_MIXER_WIDTH) {
		ret = mdss_mdp_overlay_get_fb_pipe(mfd, &pipe,
						   MDSS_MDP_MIXER_MUX_RIGHT);
		if (ret) {
			pr_err("unable to allocate right base pipe\n");
			return;
		}
		mdss_mdp_pipe_lock(pipe);
		ret = mdss_mdp_pipe_queue_data(pipe, &data);
		mdss_mdp_pipe_unlock(pipe);
		if (ret) {
			pr_err("unable to queue right data\n");
			return;
		}
	}
	mutex_unlock(&mfd->ov_lock);

	if (fbi->var.activate & FB_ACTIVATE_VBL)
		mdss_mdp_overlay_kickoff(mfd->ctl);
}

/* function is called in irq context should have minimum processing */
static void mdss_mdp_overlay_handle_vsync(struct mdss_mdp_ctl *ctl, ktime_t t)
{
	struct msm_fb_data_type *mfd = ctl->mfd;
	if (!mfd) {
		pr_warn("Invalid handle for vsync\n");
		return;
	}

	pr_debug("vsync on fb%d play_cnt=%d\n", mfd->index, ctl->play_cnt);

	spin_lock(&mfd->vsync_lock);
	mfd->vsync_time = t;
	complete(&mfd->vsync_comp);
	spin_unlock(&mfd->vsync_lock);
}

int mdss_mdp_overlay_vsync_ctrl(struct msm_fb_data_type *mfd, int en)
{
	struct mdss_mdp_ctl *ctl = mfd->ctl;
	unsigned long flags;
	int rc;

	if (!ctl)
		return -ENODEV;
	if (!ctl->set_vsync_handler)
		return -ENOTSUPP;

	if (!ctl->power_on) {
		pr_debug("fb%d vsync pending first update en=%d\n",
				mfd->index, en);
		mfd->vsync_pending = en;
		return 0;
	}

	pr_debug("fb%d vsync en=%d\n", mfd->index, en);

	spin_lock_irqsave(&mfd->vsync_lock, flags);
	INIT_COMPLETION(mfd->vsync_comp);
	if (en && ctl->play_cnt == 0) {
		mfd->vsync_time = ktime_get();
		complete(&mfd->vsync_comp);
	}
	spin_unlock_irqrestore(&mfd->vsync_lock, flags);

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	if (en)
		rc = ctl->set_vsync_handler(ctl, mdss_mdp_overlay_handle_vsync);
	else
		rc = ctl->set_vsync_handler(ctl, NULL);
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);

	return rc;
}

static ssize_t mdss_mdp_vsync_show_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	unsigned long flags;
	u64 vsync_ticks;
	int ret;

	if (!mfd->ctl || !mfd->ctl->power_on)
		return 0;

	ret = wait_for_completion_interruptible_timeout(&mfd->vsync_comp,
			msecs_to_jiffies(VSYNC_PERIOD * 5));
	if (ret <= 0) {
		pr_warn("vsync wait on fb%d interrupted (%d)\n",
			mfd->index, ret);
		return -EBUSY;
	}

	spin_lock_irqsave(&mfd->vsync_lock, flags);
	vsync_ticks = ktime_to_ns(mfd->vsync_time);
	spin_unlock_irqrestore(&mfd->vsync_lock, flags);

	pr_debug("fb%d vsync=%llu", mfd->index, vsync_ticks);
	ret = snprintf(buf, PAGE_SIZE, "VSYNC=%llu", vsync_ticks);

	return ret;
}

static DEVICE_ATTR(vsync_event, S_IRUGO, mdss_mdp_vsync_show_event, NULL);

static struct attribute *vsync_fs_attrs[] = {
	&dev_attr_vsync_event.attr,
	NULL,
};

static struct attribute_group vsync_fs_attr_group = {
	.attrs = vsync_fs_attrs,
};

static int mdss_mdp_hw_cursor_update(struct msm_fb_data_type *mfd,
				     struct fb_cursor *cursor)
{
	struct mdss_mdp_mixer *mixer;
	struct fb_image *img = &cursor->image;
	u32 blendcfg;
	int off, ret = 0;

	if (!mfd->cursor_buf && (cursor->set & FB_CUR_SETIMAGE)) {
		mfd->cursor_buf = dma_alloc_coherent(NULL, MDSS_MDP_CURSOR_SIZE,
					(dma_addr_t *) &mfd->cursor_buf_phys,
					GFP_KERNEL);
		if (!mfd->cursor_buf) {
			pr_err("can't allocate cursor buffer\n");
			return -ENOMEM;
		}

		ret = msm_iommu_map_contig_buffer(mfd->cursor_buf_phys,
			mdss_get_iommu_domain(MDSS_IOMMU_DOMAIN_UNSECURE),
			0, MDSS_MDP_CURSOR_SIZE, SZ_4K, 0,
			&(mfd->cursor_buf_iova));
		if (IS_ERR_VALUE(ret)) {
			dma_free_coherent(NULL, MDSS_MDP_CURSOR_SIZE,
					  mfd->cursor_buf,
					  (dma_addr_t) mfd->cursor_buf_phys);
			pr_err("unable to map cursor buffer to iommu(%d)\n",
			       ret);
			return -ENOMEM;
		}
	}

	mixer = mdss_mdp_mixer_get(mfd->ctl, MDSS_MDP_MIXER_MUX_DEFAULT);
	off = MDSS_MDP_REG_LM_OFFSET(mixer->num);

	if ((img->width > MDSS_MDP_CURSOR_WIDTH) ||
	    (img->height > MDSS_MDP_CURSOR_HEIGHT) ||
	    (img->depth != 32))
		return -EINVAL;

	pr_debug("mixer=%d enable=%x set=%x\n", mixer->num, cursor->enable,
			cursor->set);

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	blendcfg = MDSS_MDP_REG_READ(off + MDSS_MDP_REG_LM_CURSOR_BLEND_CONFIG);

	if (cursor->set & FB_CUR_SETPOS)
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_START_XY,
				   (img->dy << 16) | img->dx);

	if (cursor->set & FB_CUR_SETIMAGE) {
		int calpha_en, transp_en, alpha, size, cursor_addr;
		ret = copy_from_user(mfd->cursor_buf, img->data,
				     img->width * img->height * 4);
		if (ret)
			return ret;

		if (is_mdss_iommu_attached())
			cursor_addr = mfd->cursor_buf_iova;
		else
			cursor_addr = mfd->cursor_buf_phys;

		if (img->bg_color == 0xffffffff)
			transp_en = 0;
		else
			transp_en = 1;

		alpha = (img->fg_color & 0xff000000) >> 24;

		if (alpha)
			calpha_en = 0x0; /* xrgb */
		else
			calpha_en = 0x2; /* argb */

		size = (img->height << 16) | img->width;
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_IMG_SIZE, size);
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_SIZE, size);
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_STRIDE,
				   img->width * 4);
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_BASE_ADDR,
				   cursor_addr);

		wmb();

		blendcfg &= ~0x1;
		blendcfg |= (transp_en << 3) | (calpha_en << 1);
		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_BLEND_CONFIG,
				   blendcfg);
		if (calpha_en)
			MDSS_MDP_REG_WRITE(off +
					   MDSS_MDP_REG_LM_CURSOR_BLEND_PARAM,
					   alpha);

		if (transp_en) {
			MDSS_MDP_REG_WRITE(off +
				   MDSS_MDP_REG_LM_CURSOR_BLEND_TRANSP_LOW0,
				   ((img->bg_color & 0xff00) << 8) |
				   (img->bg_color & 0xff));
			MDSS_MDP_REG_WRITE(off +
				   MDSS_MDP_REG_LM_CURSOR_BLEND_TRANSP_LOW1,
				   ((img->bg_color & 0xff0000) >> 16));
			MDSS_MDP_REG_WRITE(off +
				   MDSS_MDP_REG_LM_CURSOR_BLEND_TRANSP_HIGH0,
				   ((img->bg_color & 0xff00) << 8) |
				   (img->bg_color & 0xff));
			MDSS_MDP_REG_WRITE(off +
				   MDSS_MDP_REG_LM_CURSOR_BLEND_TRANSP_HIGH1,
				   ((img->bg_color & 0xff0000) >> 16));
		}
	}

	if (!cursor->enable != !(blendcfg & 0x1)) {
		if (cursor->enable) {
			pr_debug("enable hw cursor on mixer=%d\n", mixer->num);
			blendcfg |= 0x1;
		} else {
			pr_debug("disable hw cursor on mixer=%d\n", mixer->num);
			blendcfg &= ~0x1;
		}

		MDSS_MDP_REG_WRITE(off + MDSS_MDP_REG_LM_CURSOR_BLEND_CONFIG,
				   blendcfg);

		mixer->cursor_enabled = cursor->enable;
		mixer->params_changed++;
	}

	mixer->ctl->flush_bits |= BIT(6) << mixer->num;
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);

	return 0;
}

static int mdss_mdp_overlay_ioctl_handler(struct msm_fb_data_type *mfd,
					  u32 cmd, void __user *argp)
{
	struct mdp_overlay req;
	int val, ret = -ENOSYS;

	switch (cmd) {
	case MSMFB_OVERLAY_GET:
		ret = copy_from_user(&req, argp, sizeof(req));
		if (!ret) {
			ret = mdss_mdp_overlay_get(mfd, &req);

			if (!IS_ERR_VALUE(ret))
				ret = copy_to_user(argp, &req, sizeof(req));
		}

		if (ret) {
			pr_debug("OVERLAY_GET failed (%d)\n", ret);
			ret = -EFAULT;
		}
		break;

	case MSMFB_OVERLAY_SET:
		ret = copy_from_user(&req, argp, sizeof(req));
		if (!ret) {
			ret = mdss_mdp_overlay_set(mfd, &req);

			if (!IS_ERR_VALUE(ret))
				ret = copy_to_user(argp, &req, sizeof(req));
		}
		if (ret) {
			pr_debug("OVERLAY_SET failed (%d)\n", ret);
			ret = -EFAULT;
		}
		break;


	case MSMFB_OVERLAY_UNSET:
		if (!IS_ERR_VALUE(copy_from_user(&val, argp, sizeof(val))))
			ret = mdss_mdp_overlay_unset(mfd, val);
		break;

	case MSMFB_OVERLAY_PLAY_ENABLE:
		if (!copy_from_user(&val, argp, sizeof(val))) {
			mfd->overlay_play_enable = val;
		} else {
			pr_err("OVERLAY_PLAY_ENABLE failed (%d)\n", ret);
			ret = -EFAULT;
		}
		break;

	case MSMFB_OVERLAY_PLAY:
		if (mfd->overlay_play_enable) {
			struct msmfb_overlay_data data;

			ret = copy_from_user(&data, argp, sizeof(data));
			if (!ret) {
				ret = mdss_mdp_overlay_play(mfd, &data);
				if (!IS_ERR_VALUE(ret))
					mdss_fb_update_backlight(mfd);
			}

			if (ret) {
				pr_debug("OVERLAY_PLAY failed (%d)\n", ret);
				ret = -EFAULT;
			}
		} else {
			ret = 0;
		}
		break;

	case MSMFB_OVERLAY_PLAY_WAIT:
		if (mfd->overlay_play_enable) {
			struct msmfb_overlay_data data;

			ret = copy_from_user(&data, argp, sizeof(data));
			if (!ret)
				ret = mdss_mdp_overlay_play_wait(mfd, &data);

			if (ret) {
				pr_err("OVERLAY_PLAY_WAIT failed (%d)\n", ret);
				ret = -EFAULT;
			}
		} else {
			ret = 0;
		}
		break;

	case MSMFB_VSYNC_CTRL:
	case MSMFB_OVERLAY_VSYNC_CTRL:
		if (!copy_from_user(&val, argp, sizeof(val))) {
			ret = mdss_mdp_overlay_vsync_ctrl(mfd, val);
		} else {
			pr_err("MSMFB_OVERLAY_VSYNC_CTRL failed (%d)\n", ret);
			ret = -EFAULT;
		}
		break;
	case MSMFB_OVERLAY_COMMIT:
		mdss_fb_wait_for_fence(mfd);
		ret = mdss_mdp_overlay_kickoff(mfd->ctl);
		mdss_fb_signal_timeline(mfd);
		break;
	default:
		if (mfd->panel.type == WRITEBACK_PANEL)
			ret = mdss_mdp_wb_ioctl_handler(mfd, cmd, argp);
		break;
	}

	return ret;
}

static int mdss_mdp_overlay_on(struct msm_fb_data_type *mfd)
{
	int rc;

	rc = mdss_mdp_ctl_on(mfd);
	if (rc == 0)
		atomic_inc(&ov_active_panels);

	return rc;
}

static int mdss_mdp_overlay_off(struct msm_fb_data_type *mfd)
{
	int rc;

	mdss_mdp_overlay_release_all(mfd);

	rc = mdss_mdp_ctl_off(mfd);
	if (rc == 0) {
		if (!mfd->ref_cnt)
			mfd->borderfill_enable = false;

		if (atomic_dec_return(&ov_active_panels) == 0)
			mdss_mdp_rotator_release_all();
	}

	return rc;
}

int mdss_mdp_overlay_init(struct msm_fb_data_type *mfd)
{
	struct device *dev = mfd->fbi->dev;
	int rc;

	mfd->on_fnc = mdss_mdp_overlay_on;
	mfd->off_fnc = mdss_mdp_overlay_off;
	mfd->hw_refresh = true;
	mfd->do_histogram = NULL;
	mfd->overlay_play_enable = true;
	mfd->cursor_update = mdss_mdp_hw_cursor_update;
	mfd->dma_fnc = mdss_mdp_overlay_pan_display;
	mfd->ioctl_handler = mdss_mdp_overlay_ioctl_handler;

	if (mfd->panel.type == WRITEBACK_PANEL)
		mfd->kickoff_fnc = mdss_mdp_wb_kickoff;

	INIT_LIST_HEAD(&mfd->pipes_used);
	INIT_LIST_HEAD(&mfd->pipes_cleanup);
	init_completion(&mfd->vsync_comp);
	spin_lock_init(&mfd->vsync_lock);
	mutex_init(&mfd->ov_lock);

	rc = sysfs_create_group(&dev->kobj, &vsync_fs_attr_group);
	if (rc) {
		pr_err("vsync sysfs group creation failed, ret=%d\n", rc);
		return rc;
	}

	kobject_uevent(&dev->kobj, KOBJ_ADD);
	pr_debug("vsync kobject_uevent(KOBJ_ADD)\n");

	return rc;
}
