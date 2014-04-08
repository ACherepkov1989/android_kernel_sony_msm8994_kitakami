/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#include "memory_prof.h"
#include "memory_prof_util.h"
#include "memory_prof_module.h"
#include "alloc_profiles.h"

struct alloc_pages_op {
	unsigned int order;
	uint64_t gfp;
	char gfp_string[MAX_FLAGS_STRING_LEN];
};

static int memory_prof_fd;

static int op_simple_alloc_global_setup(
	struct alloc_profile_entry entries[] __unused)
{
	memory_prof_fd = open(MEMORY_PROF_DEV, O_RDONLY);
	if (memory_prof_fd < 0)
		err(1, "couldn't open " MEMORY_PROF_DEV);
	return 0;
}

static void op_simple_alloc_global_teardown(void)
{
	if (ioctl(memory_prof_fd, MEMORY_PROF_IOC_CLEANUP_ALLOC_PAGES)) {
		warn("Couldn't do MEMORY_PROF_IOC_CLEANUP_ALLOC_PAGES");
	}
	close(memory_prof_fd);
}

enum alloc_pages_op_line_idx {
	LINE_IDX_ORDER = 1,
	LINE_IDX_GFP,
};

static int op_alloc_pages_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct alloc_pages_op *op = (struct alloc_pages_op *) entry->priv;
	STRTOL(op->order, li->words[LINE_IDX_ORDER], 0);

	op->gfp = parse_flags(li->words[LINE_IDX_GFP]);
	STRNCPY_SAFE(op->gfp_string, li->words[LINE_IDX_GFP],
		MAX_FLAGS_STRING_LEN);

	return 0;
}

static int op_alloc_pages_run(struct alloc_profile_entry *entry)
{
	struct alloc_pages_op *op = (struct alloc_pages_op *) entry->priv;
	struct mp_alloc_pages_args args = {
		.order = op->order,
		.gfp = op->gfp,
	};
	if (ioctl(memory_prof_fd, MEMORY_PROF_IOC_TEST_ALLOC_PAGES, &args)) {
		warn("Couldn't do MEMORY_PROF_IOC_TEST_ALLOC_PAGES");
		return 1;
	}
	printf(ST_PREFIX_DATA_ROW " alloc_pages took %llu ms order:%d gfp:%s\n",
		args.time_elapsed_us / 1000, op->order, op->gfp_string);
	return 0;
}

static struct alloc_profile_ops alloc_pages_ops = {
	.parse = op_alloc_pages_parse,
	.run = op_alloc_pages_run,
	.global_setup = op_simple_alloc_global_setup,
	.global_teardown = op_simple_alloc_global_teardown,
};

ALLOC_PROFILE_OP_SIZED(&alloc_pages_ops, alloc_pages,
		sizeof(struct alloc_pages_op));
