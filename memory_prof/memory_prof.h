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

#ifndef __MEMORY_PROF_H__
#define __MEMORY_PROF_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <err.h>

#include <linux/msm_ion.h>
#include "memory_prof_util.h"

#define NUM_REPS_FOR_HEAP_PROFILING 50
#define NUM_REPS_FOR_REPEATABILITY 100
#define ION_PRE_ALLOC_SIZE_DEFAULT 0 /* 0 MB */
#define MAX_PRE_ALLOC_SIZE 5000 /* 5000 MB */
#define MAX_ALLOC_PROFILE_LINE_LEN 500
#define MAX_ALLOC_PROFILE_FIELDS 20
#define MAX_ALLOC_PROFILE_WORD_LEN 80
#define MAX_HEAP_ID_STRING_LEN 40
#define MAX_PRINT_STRING_LEN (MAX_ALLOC_PROFILE_LINE_LEN - 10)
#define MAX_ALLOC_ID_STRING_LEN 40
#define MAX_FLAGS_STRING_LEN 100
#define MAX_FLAGS 15
#define MAX_SIZE_STRING_LEN 15

#define MEMORY_PROF_DEV "/dev/memory_prof"
#define ION_DEV		"/dev/ion"

#define xstr(s) str(s)
#define str(s) #s

#ifndef ALLOC_PROFILES_PATH
#error Please define ALLOC_PROFILES_PATH!
#endif

#define ALLOC_PROFILES_PATH_STRING xstr(ALLOC_PROFILES_PATH)

/*
 * Don't change the format of the following line strings. We need to
 * rely on them for parsing.
 */
#define ST_PREFIX_DATA_ROW	"=>"
#define ST_PREFIX_PREALLOC_SIZE "==>"

struct alloc_profile_handler;

struct alloc_profile_entry {
	void *priv;
	struct alloc_profile_handler *handler;
};

enum size_type {
	ST_BYTES,
	ST_KB,
	ST_MB,
	ST_GB,
};

struct size_suffix_size_type_mapping {
	char *suffix;
	enum size_type size_type;
	unsigned long multiplier;
};

#ifndef __unused
#define __unused __attribute__((unused))
#endif

/**
 * A set of callbacks to be used for reading allocation profiles from
 * various sources.
 *
 * @getline - Reads a line from the input stream. Should return NULL
 *            when there's nothing more to read.
 * @priv    - A private storage area.
 */
struct alloc_profile_reader {
	const char * (*getline)(struct alloc_profile_reader *reader);
	void *priv;
};

struct alloc_profile_entry *get_alloc_profile(
	struct alloc_profile_reader *reader);
extern int ion_pre_alloc_size;
int do_profile_alloc_for_heap(unsigned int heap_id_mask,
			unsigned int flags, unsigned int size,
			double *alloc_ms, double *map_ms,
			double *memset_ms, double *free_ms,
			bool do_pre_alloc, ion_user_handle_t handle,
			int ionfd, bool do_free);
int profile_alloc_for_heap(unsigned int heap_id_mask,
			unsigned int flags, unsigned int size,
			double *alloc_ms, double *map_ms,
			double *memset_ms, double *free_ms);
int profile_ion_cache_ops_for_heap(unsigned int heap_id_mask,
                                        unsigned int flags, unsigned int size,
                                        double *time_elapsed_flush_ms,
                                        bool cache_clean, bool cache_inv);
void print_stats_results(const char *name, const char *flags_label,
			const char *size_string,
			double stats[], int reps);
void print_a_bunch_of_stats_results(const char *name,
				const char *flags_label,
				const char *size_string,
				double alloc_stats[],
				double map_stats[],
				double memset_stats[],
				double free_stats[],
				int reps);
int alloc_me_up_some_ion(int ionfd,
			struct ion_allocation_data *alloc_data);
int do_basic_ion_sanity_test(int ionfd, ion_user_handle_t handle,
			unsigned long size);
void compute_stats(double stats[], int num,
		double *average, double *std_dev);

#endif /* __MEMORY_PROF_H__ */
