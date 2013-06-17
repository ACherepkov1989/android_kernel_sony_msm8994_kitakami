/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#define NUM_REPS_FOR_HEAP_PROFILING 50
#define NUM_REPS_FOR_REPEATABILITY 100
#define ION_PRE_ALLOC_SIZE_DEFAULT 0 /* 0 MB */
#define MAX_PRE_ALLOC_SIZE 5000 /* 5000 MB */
#define MAX_ALLOC_PROFILE_LINE_LEN 500
#define MAX_ALLOC_PROFILE_FIELDS 20
#define MAX_ALLOC_PROFILE_WORD_LEN 80
#define MAX_HEAP_ID_STRING_LEN 40
#define MAX_FLAGS_STRING_LEN 100
#define MAX_FLAGS 15
#define MAX_SIZE_STRING_LEN 15

#define MEMORY_PROF_DEV "/dev/memory_prof"
#define ION_DEV		"/dev/ion"

/*
 * Don't change the format of the following line strings. We need to
 * rely on them for parsing.
 */
#define ST_PREFIX_DATA_ROW	"=>"
#define ST_PREFIX_PREALLOC_SIZE "==>"

#define SZ_1K				0x00000400
#define SZ_2K				0x00000800
#define SZ_4K				0x00001000
#define SZ_8K				0x00002000
#define SZ_16K				0x00004000
#define SZ_32K				0x00008000
#define SZ_64K				0x00010000
#define SZ_128K				0x00020000
#define SZ_256K				0x00040000
#define SZ_512K				0x00080000

#define SZ_1M				0x00100000
#define SZ_2M				0x00200000
#define SZ_4M				0x00400000
#define SZ_8M				0x00800000
#define SZ_16M				0x01000000
#define SZ_32M				0x02000000
#define SZ_64M				0x04000000
#define SZ_128M				0x08000000
#define SZ_256M				0x10000000
#define SZ_512M				0x20000000

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define _MALLOC(ptr_type, ptr, size, file, line) do {			\
		ptr = (ptr_type) malloc(size);				\
		if (!ptr)						\
			err(1, "Couldn't get memory for malloc! %s:%d", file, line); \
	} while(0)

#define _REALLOC(ptr_type, ptr, size, file, line) do {			\
		ptr = (ptr_type) realloc(ptr, size);			\
		if (!ptr)						\
			err(1, "Couldn't get memory for realloc! %s:%d", file, line); \
	} while(0)

#define _STRTOL(var, word, base, file, line) do {			\
		char *endchar;						\
		var = strtol(word, &endchar, base);			\
		if (!endchar || endchar == word)			\
			errx(1, "Couldn't convert %s to a number. %s:%d", word, file, line); \
	} while(0)

#define MALLOC(ptr_type, ptr, size) _MALLOC(ptr_type, ptr, size, __FILE__, __LINE__)
#define REALLOC(ptr_type, ptr, size) _REALLOC(ptr_type, ptr, size, __FILE__, __LINE__)
#define STRTOL(var, word, base) _STRTOL(var, word, base, __FILE__, __LINE__)
/*
 * strncpy is considered "unsafe" and strlcpy doesn't exist on all
 * systems (notably glibc-based ones). Here's a strncpy that
 * guarantees null termination.
 */
#define STRNCPY_SAFE(dst, src, n) do {				\
		char *p;					\
		int l = MIN((int) n, (int) strlen(src));	\
		p = (char *) memcpy(dst, src, l);		\
		*(p + l) = '\0';				\
	} while (0)

enum alloc_op_enum {
	OP_NONE,
	OP_ALLOC,
	OP_SLEEP,
};

struct alloc_profile_entry {
	enum alloc_op_enum op;
	union {
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
		} alloc_op;
		struct sleep_op {
			unsigned int time_us;
		} sleep_op;
	} u;
};

struct alloc_profile_entry *get_alloc_profile(const char *alloc_profile_path);
struct alloc_profile_entry *get_default_alloc_profile(void);

#endif /* __MEMORY_PROF_H__ */
