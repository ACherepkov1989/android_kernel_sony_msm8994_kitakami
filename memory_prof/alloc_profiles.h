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

#ifndef __ALLOC_PROFILES_H__
#define __ALLOC_PROFILES_H__

#include <stdint.h>
#include "memory_prof.h"

/**
 * New allocation profile handlers should be registered with
 * ALLOC_PROFILE_OP, which takes as arguments an instance of
 * `alloc_profile_ops' (described below) and the `keyword'
 * corresponding to the first field of an allocation profile text file
 * that should trigger the handler to be used. For example:
 *
 *     ALLOC_PROFILE_OP(simple_sleep_ops, simple_sleep);
 *
 * would correspond to the following line in an allocation profile
 * text file:
 *
 *     simple_sleep,5000
 *
 * The alloc_profile_ops structure consists of the following
 * callbacks:
 *
 *     - ctor
 *     - dtor
 *     - global_setup
 *     - global_teardown
 *     - parse (required)
 *     - run (required)
 *
 * The lifecycle of the allocation profile handlers is as follows:
 *
 *     1. Space is allocated by calling the `ctor' callback if not
 *        NULL and malloc()'ing `priv_size' bytes from the heap.
 *     2. The `parse' callback is called with the current line of
 *        text, split into words.
 *     3. `global_setup' callbacks are all called for all alloc profile
 *        handlers that are in use for this run (e.g. if there isn't a
 *        line in the allocation profile text file for `simple_sleep'
 *        the simple_sleep global_setup callback will *not* be
 *        called). The global_setup callback is only called *once* per
 *        alloc profile handler.
 *     4. `run' callbacks are all called.
 *     5. `dtor' callbacks are called if not NULL. If memory was
 *        allocated with the priv_size method then that is free()'d.
 *     6. `global_teardown' callbacks are all called (see note about
 *        `global_setup' callbacks above for info on when they run).
 */

struct line_info {
	char **words;
	int nwords;
};

struct alloc_profile_ops {
	int	(*ctor)(struct alloc_profile_entry *entry);
	void	(*dtor)(struct alloc_profile_entry *entry);
	int	(*parse)(struct alloc_profile_entry *entry,
		struct line_info *li);
	int	(*run)(struct alloc_profile_entry *entry);
	int	(*global_setup)(struct alloc_profile_entry entries[]);
	void	(*global_teardown)(void);
};

struct alloc_profile_handler {
	struct alloc_profile_ops *ops;
	size_t priv_size;	/* alternative to manual .ctor/.dtor */
	char *keyword;
};

/**
 * We implement a "pluging"-based system using gcc's built-in rules
 * for custom ELF section variables. By putting our variables into a
 * custom ELF section with a valid C variable name, the linker
 * automatically creates __start_SECTION_NAME and __stop__SECTION_NAME
 * variables. We can then treat the data between those variables as an
 * array.
 */

#define ALLOC_PROFILE_OP_SIZED(ops_, kword, priv_size_)			\
	static struct alloc_profile_handler __profile_op_ ## kword	\
	__attribute__((__section__("alloc_profile_handlers")))		\
	__attribute__((__used__)) = {					\
		.ops = ops_,						\
		.priv_size = priv_size_,				\
		.keyword = #kword,					\
	}

#define ALLOC_PROFILE_OP(ops_, kword) ALLOC_PROFILE_OP_SIZED(ops_, kword, 0)

extern struct alloc_profile_handler __start_alloc_profile_handlers;
extern struct alloc_profile_handler __stop_alloc_profile_handlers;

/*** Some convenience iterators: ***/

/* iterate over our custom elf section that contains the handlers */
#define for_each_alloc_profile_handler(__iter)		\
	for (__iter = &__start_alloc_profile_handlers;	\
	     __iter < &__stop_alloc_profile_handlers;	\
	     ++__iter)

/* iterate over the parsed alloc profile entries */
#define for_each_alloc_profile_entry(__entry, __alloc_profile)	\
	for (__entry = &__alloc_profile[0];			\
	     __entry->handler;					\
	     ++__entry)

/* iterate over the parsed alloc profile entries of a particular keyword */
#define for_each_alloc_profile_entry_op(__entry, __alloc_profile, __op)	\
	for_each_alloc_profile_entry(__entry, __alloc_profile)		\
		if (!strcmp(__entry->handler->keyword, __op))

int split_string(const char * const string, char delim, char *output[],
		int output_sizes);
bool endswith(const char * const string, const char * const suffix);
int parse_size_string_smap(const char * const size_string,
			unsigned long *bytes,
			struct size_suffix_size_type_mapping *size_map);
int parse_size_string(const char * const size_string,
		unsigned long *bytes);
int find_heap_id_value(const char * const heap_id_string, unsigned int *val);
int find_flag_value(const char * const flag, int *val);
uint64_t parse_flags(const char * const word);
bool parse_bool(const char * const word);
void alloc_profile_free_priv(struct alloc_profile_entry *entry);

extern const struct size_suffix_size_type_mapping
size_suffix_to_size_type_mappings[];

#endif /* __ALLOC_PROFILES_H__ */

