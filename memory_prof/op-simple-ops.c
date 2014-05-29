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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/queue.h>

#include <linux/msm_ion.h>
#include "memory_prof.h"
#include "memory_prof_util.h"
#include "alloc_profiles.h"

static LIST_HEAD(simple_alloc_list, simple_alloc_node) simple_alloc_head;

struct simple_alloc_node {
	char *alloc_id;
	ion_user_handle_t handle;
	struct simple_alloc_op *simple_alloc_op;
	LIST_ENTRY(simple_alloc_node) nodes;
};

struct simple_alloc_op {
	char alloc_id[MAX_ALLOC_ID_STRING_LEN];
	unsigned int heap_id;
	char heap_id_string[MAX_HEAP_ID_STRING_LEN];
	unsigned int flags;
	char flags_string[MAX_FLAGS_STRING_LEN];
	unsigned long size;
	char size_string[MAX_SIZE_STRING_LEN];
};

enum simple_alloc_op_line_idx {
	SA_LINE_IDX_ALLOC_ID = 1,
	SA_LINE_IDX_HEAP_ID,
	SA_LINE_IDX_FLAGS,
	SA_LINE_IDX_ALLOC_SIZE_BYTES,
	SA_LINE_IDX_ALLOC_SIZE_LABEL,
};

int simple_ion_fd;

static int op_simple_alloc_global_setup(
	struct alloc_profile_entry entries[] __unused)
{
	LIST_INIT(&simple_alloc_head);

	simple_ion_fd = open(ION_DEV, O_RDONLY);
	if (simple_ion_fd < 0)
		err(1, "couldn't open " ION_DEV);
	return 0;
}

static void op_simple_alloc_global_teardown(void)
{
	close(simple_ion_fd);
}

static int op_simple_alloc_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	unsigned int heap_id;
	struct simple_alloc_op *op = (struct simple_alloc_op *) entry->priv;
	char **words = li->words;

	STRNCPY_SAFE(op->alloc_id, words[SA_LINE_IDX_ALLOC_ID],
		MAX_ALLOC_ID_STRING_LEN);

	if (find_heap_id_value(words[SA_LINE_IDX_HEAP_ID], &heap_id))
		warn("Unknown heap_id: %s", words[SA_LINE_IDX_HEAP_ID]);

	op->heap_id = ION_HEAP(heap_id);
	STRNCPY_SAFE(op->heap_id_string, words[SA_LINE_IDX_HEAP_ID],
		MAX_HEAP_ID_STRING_LEN);

	op->flags = (unsigned int)parse_flags(words[SA_LINE_IDX_FLAGS]);
	STRNCPY_SAFE(op->flags_string, words[SA_LINE_IDX_FLAGS],
		MAX_FLAGS_STRING_LEN);

	if (parse_size_string(words[SA_LINE_IDX_ALLOC_SIZE_BYTES],
				&op->size)) {
		warnx("Couldn't parse alloc_bytes %s into a size",
			words[SA_LINE_IDX_ALLOC_SIZE_BYTES]);
		return 1;
	}
	STRNCPY_SAFE(op->size_string, words[SA_LINE_IDX_ALLOC_SIZE_LABEL],
		MAX_SIZE_STRING_LEN);

	return 0;
}

static int op_simple_alloc_run(struct alloc_profile_entry *entry)
{
	struct simple_alloc_node *np;
	struct simple_alloc_op *op = (struct simple_alloc_op *) entry->priv;
	struct ion_allocation_data alloc_data = {
		.align	   = SZ_4K,
		.len	   = op->size,
		.heap_id_mask = op->heap_id,
		.flags	   = op->flags,
	};

	MALLOC(struct simple_alloc_node *,
		np, sizeof(struct simple_alloc_node));

	if (alloc_me_up_some_ion(simple_ion_fd, &alloc_data)) {
		warn("Couldn't do Ion alloc");
		return 1;
	}

	np->handle = alloc_data.handle;
	np->alloc_id = op->alloc_id;
	np->simple_alloc_op = op;
	LIST_INSERT_HEAD(&simple_alloc_head, np, nodes);
	return 0;
}

static struct alloc_profile_ops simple_alloc_ops = {
	.parse = op_simple_alloc_parse,
	.run = op_simple_alloc_run,
	.global_setup = op_simple_alloc_global_setup,
	.global_teardown = op_simple_alloc_global_teardown,
};

ALLOC_PROFILE_OP_SIZED(&simple_alloc_ops, simple_alloc,
		sizeof(struct simple_alloc_op));

struct simple_free_op {
	char alloc_id[MAX_ALLOC_ID_STRING_LEN];
};

static int op_simple_free_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct simple_free_op *op = (struct simple_free_op *) entry->priv;
	STRNCPY_SAFE(op->alloc_id, li->words[1], MAX_ALLOC_ID_STRING_LEN);
	return 0;
}

static int op_simple_free_run(struct alloc_profile_entry *entry)
{
	struct simple_free_op *op = (struct simple_free_op *) entry->priv;
	struct simple_alloc_node *np;

	for (np = simple_alloc_head.lh_first;
	     np != NULL;
	     np = np->nodes.le_next) {
		struct ion_handle_data data;
		if (strcmp(np->alloc_id, op->alloc_id))
			continue;
		data.handle = np->handle;
		if (ioctl(simple_ion_fd, ION_IOC_FREE, &data))
			err(1, "Couldn't do ION_IOC_FREE");
		LIST_REMOVE(np, nodes);
		free(np);
	}

	return 0;
}

static struct alloc_profile_ops simple_free_ops = {
	.parse = op_simple_free_parse,
	.run = op_simple_free_run,
};

ALLOC_PROFILE_OP_SIZED(&simple_free_ops, simple_free,
		sizeof(struct simple_free_op));

struct simple_profile_op {
	char alloc_id[MAX_ALLOC_ID_STRING_LEN];
};

static int op_simple_profile_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct simple_profile_op *op = (struct simple_profile_op *) entry->priv;
	STRNCPY_SAFE(op->alloc_id, li->words[1], MAX_ALLOC_ID_STRING_LEN);
	return 0;
}

static struct simple_alloc_node *find_by_alloc_id(char *alloc_id)
{
	struct simple_alloc_node *np;
	/* find the parameters that were used for allocation */
	for (np = simple_alloc_head.lh_first;
	     np != NULL;
	     np = np->nodes.le_next) {
		if (!strcmp(np->alloc_id, alloc_id))
			return np;
	}
	return NULL;
}

static int op_simple_profile_run(struct alloc_profile_entry *entry)
{
	int i, ret;
	double map_ms, memset_ms;
	unsigned long size;
	ion_user_handle_t handle;
	char sbuf[70], *heap_id_string = NULL, *flags_string, *size_string;
	struct simple_profile_op *op = (struct simple_profile_op *) entry->priv;
	struct simple_alloc_node *np;

	heap_id_string = flags_string = size_string = NULL;

	/* find the parameters that were used for allocation */
	np = find_by_alloc_id(op->alloc_id);
	if (!np) {
		warnx("Couldn't find a simple allocation with id %s. Bailing.",
			op->alloc_id);
		return 1;
	}
	heap_id_string = np->simple_alloc_op->heap_id_string;
	flags_string = np->simple_alloc_op->flags_string;
	size_string = np->simple_alloc_op->size_string;
	size = np->simple_alloc_op->size;
	handle = np->handle;

	ret = do_profile_alloc_for_heap(
		0, 0, size, NULL, &map_ms, &memset_ms, NULL,
		false, handle, simple_ion_fd, false);

	if (ret) {
		warn("Couldn't profile %s. Bailing.",
			op->alloc_id);
		return 1;
	}

	snprintf(sbuf, 70, "mmap %s", heap_id_string);
	print_stats_results(sbuf, flags_string, size_string, &map_ms, 1);

	snprintf(sbuf, 70, "memset %s", heap_id_string);
	print_stats_results(sbuf, flags_string, size_string, &memset_ms, 1);

	putchar('\n');
	fflush(stdout);
	return 0;
}

static struct alloc_profile_ops simple_profile_ops = {
	.parse = op_simple_profile_parse,
	.run = op_simple_profile_run,
};

ALLOC_PROFILE_OP_SIZED(&simple_profile_ops, simple_profile,
		sizeof(struct simple_profile_op));

struct simple_basic_sanity_op {
	char alloc_id[MAX_ALLOC_ID_STRING_LEN];
};

static int op_simple_basic_sanity_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct simple_basic_sanity_op *op =
		(struct simple_basic_sanity_op *) entry->priv;
	STRNCPY_SAFE(op->alloc_id, li->words[1], MAX_ALLOC_ID_STRING_LEN);
	return 0;
}

static int op_simple_basic_sanity_run(struct alloc_profile_entry *entry)
{
	struct simple_basic_sanity_op *op =
		(struct simple_basic_sanity_op *) entry->priv;
	struct simple_alloc_node *np;

	np = find_by_alloc_id(op->alloc_id);
	if (!np) {
		warnx("Couldn't find a simple allocation with id %s. Bailing.",
			op->alloc_id);
		return 1;
	}

	if (do_basic_ion_sanity_test(simple_ion_fd, np->handle,
					np->simple_alloc_op->size)) {
		warnx("Basic sanity test on %s failed!", op->alloc_id);
		return 1;
	}

	return 0;
}

static struct alloc_profile_ops simple_basic_sanity_ops = {
	.parse = op_simple_basic_sanity_parse,
	.run = op_simple_basic_sanity_run,
};

ALLOC_PROFILE_OP_SIZED(&simple_basic_sanity_ops, simple_basic_sanity,
		sizeof(struct simple_basic_sanity_op));
