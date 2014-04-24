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
#include <sys/queue.h>

#include <linux/msm_ion.h>
#include "memory_prof.h"
#include "memory_prof_util.h"
#include "alloc_profiles.h"


static LIST_HEAD(create_unused_client_list, unused_client_node) unused_client_head;

struct unused_client_node {
	int fd;
	LIST_ENTRY(unused_client_node) nodes;
};

static int op_create_unused_client_run(
	struct alloc_profile_entry *entry __unused)
{
	int fd;
	struct unused_client_node *np;
	MALLOC(struct unused_client_node *,
		np, sizeof(struct unused_client_node));

	fd = open(ION_DEV, O_RDONLY);
	if (fd < 0) {
		warn("couldn't open " ION_DEV);
		return 1;
	}

	np->fd = fd;
	LIST_INSERT_HEAD(&unused_client_head, np, nodes);
	return 0;
}

static struct alloc_profile_ops simple_create_unused_client_ops = {
	.run = op_create_unused_client_run,
};

ALLOC_PROFILE_OP(&simple_create_unused_client_ops, create_unused_client);

static int op_free_all_unused_clients_run(
	struct alloc_profile_entry *entry __unused)
{
	struct unused_client_node *np;
	for (np = unused_client_head.lh_first;
	     np != NULL;
	     np = np->nodes.le_next)
		close(np->fd);
	return 0;
}

static struct alloc_profile_ops free_all_unused_clients_ops = {
	.run = op_free_all_unused_clients_run,
};

ALLOC_PROFILE_OP(&free_all_unused_clients_ops, free_all_unused_clients);
