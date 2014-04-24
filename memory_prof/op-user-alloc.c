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
#include <sys/mman.h>

#include "memory_prof.h"
#include "memory_prof_util.h"
#include "alloc_profiles.h"

enum user_allocator {
	ALLOCATOR_MALLOC,
	ALLOCATOR_MMAP,
};

enum usage_fn {
	USAGE_FN_NOP,
	USAGE_FN_MEMSET_N,
};

struct user_alloc_op {
	enum user_allocator user_allocator;
	enum usage_fn usage_fn;
	unsigned int memset_n_arg; /* only for USAGE_FN_MEMSET_N */
	unsigned long alloc_bytes;
	unsigned long usage_bytes;

	char user_allocator_string[MAX_ALLOC_PROFILE_WORD_LEN];
	char usage_fn_string[MAX_ALLOC_PROFILE_WORD_LEN];
	char alloc_bytes_string[MAX_ALLOC_PROFILE_WORD_LEN];
	char usage_bytes_string[MAX_ALLOC_PROFILE_WORD_LEN];
};

#define LINE_IDX_ALLOCATOR	1
#define LINE_IDX_ALLOC_BYTES	2
#define LINE_IDX_USAGE_BYTES	3
#define LINE_IDX_USAGE_FN	4

static int op_user_alloc_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct user_alloc_op *op = (struct user_alloc_op *) entry->priv;
	char **words = li->words;
	char *cur;

	cur = words[LINE_IDX_ALLOCATOR];
	if (!strcmp("malloc", cur)) {
		op->user_allocator = ALLOCATOR_MALLOC;
	} else if (!strcmp("mmap", cur)) {
		op->user_allocator = ALLOCATOR_MMAP;
	} else {
		warnx("Unknown allocator: %s", cur);
		return 1;
	}
	STRNCPY_SAFE(op->user_allocator_string, cur,
		MAX_ALLOC_PROFILE_WORD_LEN);

	cur = words[LINE_IDX_ALLOC_BYTES];
	if (parse_size_string(cur, &op->alloc_bytes)) {
		warnx("Couldn't parse alloc_bytes %s into a size",
			cur);
		return 1;
	}
	STRNCPY_SAFE(op->alloc_bytes_string, cur, MAX_ALLOC_PROFILE_WORD_LEN);

	cur = words[LINE_IDX_USAGE_BYTES];
	if (parse_size_string(cur, &op->usage_bytes)) {
		warnx("Couldn't parse usage_bytes %s into a size", cur);
		return 1;
	}
	STRNCPY_SAFE(op->usage_bytes_string, cur, MAX_ALLOC_PROFILE_WORD_LEN);

	cur = words[LINE_IDX_USAGE_FN];
	if (!strcmp("nop", cur)) {
		op->usage_fn = USAGE_FN_NOP;
	} else if (!strncmp("memset-", cur, strlen("memset-"))) {
		op->usage_fn = USAGE_FN_MEMSET_N;
		STRTOL(op->memset_n_arg, cur + strlen("memset-"), 0);
	} else {
		warnx("Unknown usage_fn: %s\n", cur);
		return 1;
	}
	STRNCPY_SAFE(op->usage_fn_string, cur, MAX_ALLOC_PROFILE_WORD_LEN);

	return 0;
}

static void *do_mmap_alloc(size_t len)
{
	void *buf = mmap(NULL, len,
			PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS,
			-1, 0);
	if (buf == MAP_FAILED) {
		warn("Couldn't allocate buffer with mmap\n");
		return NULL;
	}
	return buf;
}

static void do_munmap(void *buf, size_t len)
{
	munmap(buf, len);
}

static void do_free(void *buf, size_t len __unused)
{
	free(buf);
}

static int memset_n_arg;
static void do_memset_n(void *buf, size_t len)
{
	memset(buf, memset_n_arg, len);
}

static void identity(void *buf __unused, size_t len __unused)
{
}

static int op_user_alloc_run(struct alloc_profile_entry *entry)
{
	int rc, rc2, i;
	struct timeval tv_before, tv_after;
	void *buf;
	struct user_alloc_op *op = (struct user_alloc_op *) entry->priv;
	double alloc_timing, usage_timing, free_timing;
	void *(*alloc_fn)(size_t len);
	void (*free_fn)(void *, size_t);
	void (*usage_fn)(void *, size_t) = NULL;
	struct munmapper *unmapper;

	switch (op->user_allocator) {
	case ALLOCATOR_MALLOC:
		alloc_fn = malloc;
		free_fn = do_free;
		break;
	case ALLOCATOR_MMAP:
		alloc_fn = do_mmap_alloc;
		free_fn = do_munmap;
		break;
	default:
		errx(1, "Backwards cheeseburgers found. Calling it a day.");
		return 1;
	}

	switch (op->usage_fn) {
	case USAGE_FN_NOP:
		usage_fn = identity;
		break;
	case USAGE_FN_MEMSET_N:
		usage_fn = do_memset_n;
		memset_n_arg = op->memset_n_arg;
		break;
	default:
		errx(1, "usage_fn cheeseburg'd. Calling it a day.");
		return 1;
	}

	rc = gettimeofday(&tv_before, NULL);
	buf = alloc_fn(op->alloc_bytes);
	rc2 = gettimeofday(&tv_after, NULL);
	if (!buf) {
		warn("Couldn't allocate memory!");
		return 1;
	}
	if (rc || rc2) {
		warn("Couldn't get time of day!");
		free(buf);
		return 1;
	}
	alloc_timing = timeval_ms_diff(tv_after, tv_before);

	rc = gettimeofday(&tv_before, NULL);
	usage_fn(buf, op->usage_bytes);
	rc2 = gettimeofday(&tv_after, NULL);
	if (rc || rc2) {
		warn("Couldn't get time of day!");
		return 1;
	}
	usage_timing = timeval_ms_diff(tv_after, tv_before);

	rc = gettimeofday(&tv_before, NULL);
	free_fn(buf, op->alloc_bytes);
	rc2 = gettimeofday(&tv_after, NULL);

	if (rc || rc2) {
		warn("Couldn't get time of day!");
		return 1;
	}
	free_timing = timeval_ms_diff(tv_after, tv_before);

	printf(ST_PREFIX_DATA_ROW
		" user_alloc %s: %g %s: %g alloc_size: %s usage_size: %s free: %g\n",
		op->user_allocator_string, alloc_timing,
		op->usage_fn_string, usage_timing,
		op->alloc_bytes_string,
		op->usage_bytes_string,
		free_timing);

	return 0;
}

static struct alloc_profile_ops user_alloc_ops = {
	.parse = op_user_alloc_parse,
	.run = op_user_alloc_run,
};

ALLOC_PROFILE_OP_SIZED(&user_alloc_ops, user_alloc,
		sizeof(struct user_alloc_op));
