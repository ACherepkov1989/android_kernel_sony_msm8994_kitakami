/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MSM_IOMMU_CONFIG__
#define __MSM_IOMMU_CONFIG__

static struct cats_reg cats_reg_v1 = {
	.phys_smmu_local_base = 0x1EE0000,
	.cats_128_bit_base_addr = 0x20000000,
	.cats_64_bit_base_addr = 0x24000000,
	.tbu_id_shift = 0x1,
	.va_remap_shift = 0x6,
	.enable_sid_shift = 0xc,
	.sid_shift = 0xd,
	.sid_mask = 0x3FF,
};

static struct cats_reg cats_reg_8937 = {
	.phys_smmu_local_base = 0x1EE0000,
	.cats_128_bit_base_addr = 0x0e000000,
	.cats_64_bit_base_addr = 0x0f000000,
	.tbu_id_shift = 0x1,
	.va_remap_shift = 0x6,
	.enable_sid_shift = 0xe,
	.sid_shift = 0xf,
	.sid_mask = 0x3FF,
};

static struct cats_reg cats_reg_titanium = {
	.phys_smmu_local_base = 0x1EE0000,
	.cats_128_bit_base_addr = 0x0e000000,
	.cats_64_bit_base_addr = 0x0f000000,
	.tbu_id_shift = 0x1,
	.va_remap_shift = 0x6,
	.enable_sid_shift = 0xe,
	.sid_shift = 0xf,
	.sid_mask = 0x3FF,
};

static struct target_config tc[] = {
	{
		.ts = {
			.name = "msm8952",
		},
		.cr = &cats_reg_v1,
	},
	{
		.ts = {
			.name = "msm8937",
		},
		.cr = &cats_reg_8937,
	},
	{
		.ts = {
			.name = "msmtita",
		},
		.cr = &cats_reg_titanium,
	},
	{
		.ts = {
			.name = "",
		},
	},
};

#endif
