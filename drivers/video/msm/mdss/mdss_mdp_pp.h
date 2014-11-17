/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#ifndef MDSS_MDP_PP_DEBUG_H
#define MDSS_MDP_PP_DEBUG_H

#include <linux/msm_mdp.h>

#define MDSS_BLOCK_DISP_NUM (MDP_BLOCK_MAX - MDP_LOGICAL_BLOCK_DISP_0)

/* PP STS related flags */
#define PP_STS_ENABLE	0x1

/* Demo mode macros */
#define MDSS_SIDE_NONE	0
#define MDSS_SIDE_LEFT	1
#define MDSS_SIDE_RIGHT	2
/* size calculated for c0,c1_c2 for 4 tables */
#define GAMUT_COLOR_COEFF_SIZE_V1_7 (2 * MDP_GAMUT_TABLE_V1_7_SZ * 4)
/* 16 entries for c0,c1,c2 */
#define GAMUT_SCALE_OFFSET_SIZE_V1_7 (3 * MDP_GAMUT_SCALE_OFF_SZ)
#define GAMUT_TOTAL_TABLE_SIZE_V1_7 (GAMUT_COLOR_COEFF_SIZE_V1_7 + \
				  GAMUT_SCALE_OFFSET_SIZE_V1_7)

/* PP Feature Operations */
enum pp_features {
	IGC,
	PCC,
	GC,
	PA,
	GAMUT,
	CSC,
	DITHER,
	QSEED,
	HIST_LUT,
	PP_FEATURE_MAX
};

enum pp_block_opmodes {
	PP_OPMODE_VIG = 1,
	PP_OPMODE_DSPP,
	PP_OPMODE_MAX
};

enum pp_config_block {
	SSPP_RGB = 1,
	SSPP_DMA,
	SSPP_VIG,
	DSPP,
	LM
};

struct mdp_pp_feature_ops {
	u32 feature;
	int (*pp_get_config)(char __iomem *base_addr, void *cfg_data,
			u32 block_type, u32 disp_num);
	int (*pp_set_config)(char __iomem *base_addr,
		struct pp_sts_type *pp_sts, void *cfg_data,
		u32 block_type);
};

struct mdp_pp_driver_ops {
	struct mdp_pp_feature_ops pp_ops[PP_FEATURE_MAX];
	void (*pp_opmode_config)(int location, struct pp_sts_type *pp_sts,
			u32 *opmode, int side);
};

struct mdss_pp_res_type_v1_7 {
	struct mdp_gamut_data_v1_7 gamut_v17_data[MDSS_BLOCK_DISP_NUM];
};

#ifdef CONFIG_ARCH_MSMTHULIUM
void *pp_get_driver_ops(struct mdp_pp_driver_ops *ops);
#else
static inline void *pp_get_driver_ops(struct mdp_pp_driver_ops *ops)
{
	memset(ops, 0, sizeof(struct mdp_pp_driver_ops));
	return NULL;
}
#endif

static inline void pp_sts_set_split_bits(u32 *sts, u32 bits)
{
	u32 tmp = *sts;
	tmp &= ~MDSS_PP_SPLIT_MASK;
	tmp |= bits & MDSS_PP_SPLIT_MASK;
	*sts = tmp;
}

static inline bool pp_sts_is_enabled(u32 sts, int side)
{
	bool ret = false;
	/*
	 * If there are no sides, or if there are no split mode bits set, the
	 * side can't be disabled via split mode.
	 *
	 * Otherwise, if the side being checked opposes the split mode
	 * configuration, the side is disabled.
	 */
	if ((side == MDSS_SIDE_NONE) || !(sts & MDSS_PP_SPLIT_MASK))
		ret = true;
	else if ((sts & MDSS_PP_SPLIT_RIGHT_ONLY) && (side == MDSS_SIDE_RIGHT))
		ret = true;
	else if ((sts & MDSS_PP_SPLIT_LEFT_ONLY) && (side == MDSS_SIDE_LEFT))
		ret = true;

	return ret && (sts & PP_STS_ENABLE);
}

/* Debug related functions */
void pp_print_lut(void *data, int size, char *tab, uint32_t type);
void pp_print_uint16_lut(uint16_t *data, int size, char *tab);
void pp_print_pcc_coeff(struct mdp_pcc_coeff *pcc_coeff, int tab_depth);
void pp_print_pcc_cfg_data(struct mdp_pcc_cfg_data *pcc_data, int tab_depth);
void pp_print_csc_cfg(struct mdp_csc_cfg *data, int tab_depth);
void pp_print_csc_cfg_data(struct mdp_csc_cfg_data *data, int tab_depth);
void pp_print_igc_lut_data(struct mdp_igc_lut_data *data, int tab_depth);
void pp_print_ar_gc_lut_data(struct mdp_ar_gc_lut_data *data, int tab_depth);
void pp_print_pgc_lut_data(struct mdp_pgc_lut_data *data, int tab_depth);
void pp_print_hist_lut_data(struct mdp_hist_lut_data *data, int tab_depth);
void pp_print_lut_cfg_data(struct mdp_lut_cfg_data *data, int tab_depth);
void pp_print_qseed_cfg(struct mdp_qseed_cfg *data, int tab_depth);
void pp_print_qseed_cfg_data(struct mdp_qseed_cfg_data *data, int tab_depth);
void pp_print_pa_cfg(struct mdp_pa_cfg *data, int tab_depth);
void pp_print_pa_cfg_data(struct mdp_pa_cfg_data *data, int tab_depth);
void pp_print_mem_col_cfg(struct mdp_pa_mem_col_cfg *data, int tab_depth);
void pp_print_pa_v2_data(struct mdp_pa_v2_data *data, int tab_depth);
void pp_print_pa_v2_cfg_data(struct mdp_pa_v2_cfg_data *data, int tab_depth);
void pp_print_dither_cfg_data(struct mdp_dither_cfg_data *data, int tab_depth);
void pp_print_gamut_cfg_data(struct mdp_gamut_cfg_data *data, int tab_depth);
void pp_print_ad_init(struct mdss_ad_init *data, int tab_depth);
void pp_print_ad_cfg(struct mdss_ad_cfg *data, int tab_depth);
void pp_print_ad_init_cfg(struct mdss_ad_init_cfg *data, int tab_depth);
void pp_print_ad_input(struct mdss_ad_input *data, int tab_depth);
void pp_print_histogram_cfg(struct mdp_histogram_cfg *data, int tab_depth);
void pp_print_sharp_cfg(struct mdp_sharp_cfg *data, int tab_depth);
void pp_print_calib_config_data(struct mdp_calib_config_data *data,
				int tab_depth);
void pp_print_calib_config_buffer(struct mdp_calib_config_buffer *data,
				int tab_depth);
void pp_print_calib_dcm_state(struct mdp_calib_dcm_state *data, int tab_depth);
void pp_print_mdss_calib_cfg(struct mdss_calib_cfg *data, int tab_depth);

#endif
