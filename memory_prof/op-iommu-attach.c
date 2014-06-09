/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution ande in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may beed to endorse or promote products derived
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

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "memory_prof.h"
#include "memory_prof_util.h"
#include "memory_prof_module.h"
#include "alloc_profiles.h"

struct iommu_attach_op {
	struct mp_iommu_attach_test_args args;
	char flags_string[MAX_FLAGS_STRING_LEN];
};

#define LINE_IDX_CTX_NAME 1
#define LINE_IDX_ITERATIONS 2
#define LINE_IDX_FLAGS 3

static int memory_prof_fd;

static int op_iommu_attach_global_setup(
	struct alloc_profile_entry entries[] __unused)
{
	memory_prof_fd = open(MEMORY_PROF_DEV, O_RDONLY);
	if (memory_prof_fd < 0)
		err(1, "couldn't open " MEMORY_PROF_DEV);
	return 0;
}

static void op_iommu_attach_global_teardown(void)
{
	close(memory_prof_fd);
}

static int op_iommu_attach_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct iommu_attach_op *op = entry->priv;
	STRNCPY_SAFE(op->args.ctx_name, li->words[LINE_IDX_CTX_NAME],
		MAX_IOMMU_CTX_NAME);
	op->args.iterations = strtoul(li->words[LINE_IDX_ITERATIONS], NULL, 10);
	op->args.flags = parse_flags(li->words[LINE_IDX_FLAGS]);
	STRNCPY_SAFE(op->flags_string, li->words[LINE_IDX_FLAGS],
		MAX_FLAGS_STRING_LEN);
	return 0;
}

static int op_iommu_attach_run(struct alloc_profile_entry *entry)
{
	struct iommu_attach_op *op = entry->priv;
	if (ioctl(memory_prof_fd, MEMORY_PROF_IOC_IOMMU_ATTACH_TEST,
			&op->args)) {
		warn("Couldn't do MEMORY_PROF_IOC_IOMMU_ATTACH_TEST");
		return 1;
	}
	printf(ST_PREFIX_DATA_ROW
		" iommu_attach_device time %llu us, ave: %llu us, min: %llu, max: %llu, ctx: %s iterations: %u flags: %s\n",
		op->args.time_elapsed_us,
		op->args.time_elapsed_us / op->args.iterations,
		op->args.time_elapsed_min_us,
		op->args.time_elapsed_max_us,
		op->args.ctx_name, op->args.iterations, op->flags_string);
	return 0;
}

static struct alloc_profile_ops iommu_attach_ops = {
	.parse = op_iommu_attach_parse,
	.run = op_iommu_attach_run,
	.global_setup = op_iommu_attach_global_setup,
	.global_teardown = op_iommu_attach_global_teardown,
};

ALLOC_PROFILE_OP_SIZED(&iommu_attach_ops, iommu_attach,
		sizeof(struct iommu_attach_op));
