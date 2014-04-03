/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <stdio.h>

#include <linux/msm_ion.h>
#include "memory_prof.h"
#include "memory_prof_util.h"
#include "alloc_profiles.h"

struct alloc_op {
	int reps;
	unsigned int heap_id;
	char heap_id_string[MAX_HEAP_ID_STRING_LEN];
	unsigned int flags;
	char flags_string[MAX_FLAGS_STRING_LEN];
	unsigned long size;
	char size_string[MAX_SIZE_STRING_LEN];
	bool quiet;
	bool profile_mmap;
	bool profile_memset;
};

enum alloc_op_line_idx {
	LINE_IDX_REPS = 1,
	LINE_IDX_HEAP_ID,
	LINE_IDX_FLAGS,
	LINE_IDX_ALLOC_SIZE,
	LINE_IDX_ALLOC_SIZE_LABEL,
	LINE_IDX_QUIET_ON_FAILURE,
	LINE_IDX_PROFILE_MMAP,
	LINE_IDX_PROFILE_MEMSET,
};

static double *alloc_stats;
static double *map_stats;
static double *memset_stats;
static double *free_stats;

static int op_alloc_global_setup(struct alloc_profile_entry entries[])
{
	int max_reps = 0;
	struct alloc_profile_entry *entry;
	/*
	 * Rather than malloc'ing and free'ing just enough space on
	 * each iteration, let's get the max needed up front. This
	 * will help keep things a little more equal during the actual
	 * timings.
	 */
	for_each_alloc_profile_entry_op(entry, entries, "alloc") {
		struct alloc_op *op = (struct alloc_op *) entry->priv;
		max_reps = MAX(max_reps, op->reps);
	}

	if (max_reps) {
		MALLOC(double *, alloc_stats, sizeof(double) * max_reps);
		MALLOC(double *, map_stats, sizeof(double) * max_reps);
		MALLOC(double *, memset_stats, sizeof(double) * max_reps);
		MALLOC(double *, free_stats, sizeof(double) * max_reps);
	}

	return 0;
}

static void op_alloc_global_teardown(void)
{
	free(alloc_stats);
	free(map_stats);
	free(memset_stats);
	free(free_stats);
}

static int op_alloc_parse(struct alloc_profile_entry *entry,
			struct line_info *li)
{
	unsigned int heap_id;
	struct alloc_op *op = (struct alloc_op *) entry->priv;
	char **words = li->words;

	STRTOL(op->reps, words[LINE_IDX_REPS], 0);

	if (find_heap_id_value(words[LINE_IDX_HEAP_ID], &heap_id)) {
		warnx("Unknown heap_id: %s", words[LINE_IDX_HEAP_ID]);
		return 1;
	}

	op->heap_id = ION_HEAP(heap_id);
	STRNCPY_SAFE(op->heap_id_string,
		words[LINE_IDX_HEAP_ID], MAX_HEAP_ID_STRING_LEN);

	op->flags = (unsigned int)parse_flags(words[LINE_IDX_FLAGS]);
	STRNCPY_SAFE(op->flags_string,
		words[LINE_IDX_FLAGS], MAX_FLAGS_STRING_LEN);

	if (parse_size_string(words[LINE_IDX_ALLOC_SIZE], &op->size)) {
		warnx("Couldn't parse alloc_size %s into a size",
			words[LINE_IDX_ALLOC_SIZE]);
		return 1;
	}
	STRNCPY_SAFE(op->size_string,
		words[LINE_IDX_ALLOC_SIZE_LABEL], MAX_SIZE_STRING_LEN);

	op->quiet = parse_bool(words[LINE_IDX_QUIET_ON_FAILURE]);
	op->profile_mmap = parse_bool(words[LINE_IDX_PROFILE_MMAP]);
	op->profile_memset = parse_bool(words[LINE_IDX_PROFILE_MEMSET]);

	return 0;
}

static int op_alloc_run(struct alloc_profile_entry *entry)
{
	int i;
	struct alloc_op *op = (struct alloc_op *) entry->priv;
	for (i = 0; i < op->reps; ++i) {
		int ret = profile_alloc_for_heap(
			op->heap_id,
			op->flags,
			op->size,
			&alloc_stats[i],
			op->profile_mmap
			? &map_stats[i] : NULL,
			op->profile_memset
			? &memset_stats[i] : NULL,
			&free_stats[i]);
		if (ret && !op->quiet)
			warn("Couldn't allocate %s from %s",
				op->size_string,
				op->heap_id_string);
	}
	print_a_bunch_of_stats_results(
		op->heap_id_string,
		op->flags_string,
		op->size_string,
		alloc_stats,
		op->profile_mmap
		? map_stats : NULL,
		op->profile_memset
		? memset_stats : NULL,
		free_stats,
		op->reps);
	putchar('\n');
	fflush(stdout);
	return 0;
}

static struct alloc_profile_ops alloc_ops = {
	.parse = op_alloc_parse,
	.run = op_alloc_run,
	.global_setup = op_alloc_global_setup,
	.global_teardown = op_alloc_global_teardown,
};

ALLOC_PROFILE_OP_SIZED(&alloc_ops, alloc, sizeof(struct alloc_op));
