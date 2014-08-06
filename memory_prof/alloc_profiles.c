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

#include <stdio.h>
#include <string.h>
#include <err.h>
#include <stdint.h>
#include <linux/msm_ion.h>
#include "memory_prof.h"
#include "memory_prof_util.h"
#include "memory_prof_module.h"
#include "alloc_profiles.h"

/**
 * split_string() - split a string by a delimiter
 *
 * @string - string to split
 * @delim - delimiter char
 * @output - output array of C-strings. Must have enough room for us.
 * @output_lens - how much room we have in each entry of @output
 *
 * Returns the number of entries found
 */
int split_string(const char * const string, char delim, char *output[],
		int output_sizes)
{
	char *word, *string_cpy;
	int nentries = 0;
	size_t len = strlen(string) + 1;
	const char delim_string[2] = { delim, '\0' };

	MALLOC(char *, string_cpy, len);
	STRNCPY_SAFE(string_cpy, string, len);

	word = strtok(string_cpy, delim_string);
	if (!word) {
		warnx("Malformed line: %s", string_cpy);
		return 0;
	}
	STRNCPY_SAFE(*output, word, output_sizes);
	output++;
	nentries++;
	for (word = strtok(NULL, delim_string);
	     word;
	     word = strtok(NULL, delim_string)) {
		STRNCPY_SAFE(*output, word, output_sizes);
		output++;
		nentries++;
	}

	free(string_cpy);

	return nentries;
}

bool endswith(const char * const string, const char * const suffix)
{
	int s1 = strlen(string);
	int s2 = strlen(suffix);
	int len = MIN(s1, s2);
	return 0 == strcmp(string + (s1 - len), suffix + (s2 - len));
}

int parse_size_string_smap(const char * const size_string,
		unsigned long *bytes,
		struct size_suffix_size_type_mapping *size_map)
{
	const struct size_suffix_size_type_mapping *smap;
	char *scopy;
	for (smap = &size_suffix_to_size_type_mappings[0];
	     smap->suffix;
	     smap++) {
		if (endswith(size_string, smap->suffix))
			break;
	}
	if (!smap->suffix) {
		STRTOL(*bytes, size_string, 0);
		return 0;
	}
	STRDUP(scopy, size_string);
	/* chop off the suffix */
	scopy[strlen(scopy) - strlen(smap->suffix)] = '\0';
	*size_map = *smap;
	STRTOL(*bytes, scopy, 0);
	*bytes *= smap->multiplier;
	free(scopy);
	return 0;
}

int parse_size_string(const char * const size_string,
	unsigned long *bytes)
{
	struct size_suffix_size_type_mapping smap;
	return parse_size_string_smap(size_string, bytes, &smap);
}

const struct size_suffix_size_type_mapping size_suffix_to_size_type_mappings[] =
{
	{.suffix = "KB", .size_type=ST_KB, .multiplier = 1024},
	{.suffix = "MB", .size_type=ST_MB, .multiplier = 1024 * 1024},
	{.suffix = "GB", .size_type=ST_GB, .multiplier = 1024 * 1024 * 1024},
	{.suffix = NULL},
};

#define MAKE_HEAP_INFO(heap) { .heap_id = heap, .heap_id_string = #heap }

struct heap_info {
	unsigned int heap_id;
	const char * const heap_id_string;
} heap_info[] = {
	MAKE_HEAP_INFO(ION_CP_MM_HEAP_ID),
	MAKE_HEAP_INFO(ION_CP_MFC_HEAP_ID),
	MAKE_HEAP_INFO(ION_CP_WB_HEAP_ID),
	MAKE_HEAP_INFO(ION_CAMERA_HEAP_ID),
	MAKE_HEAP_INFO(ION_SYSTEM_CONTIG_HEAP_ID),
	MAKE_HEAP_INFO(ION_ADSP_HEAP_ID),
	MAKE_HEAP_INFO(ION_PIL1_HEAP_ID),
	MAKE_HEAP_INFO(ION_SF_HEAP_ID),
	MAKE_HEAP_INFO(ION_IOMMU_HEAP_ID),
	MAKE_HEAP_INFO(ION_PIL2_HEAP_ID),
	MAKE_HEAP_INFO(ION_QSECOM_HEAP_ID),
	MAKE_HEAP_INFO(ION_AUDIO_HEAP_ID),
	MAKE_HEAP_INFO(ION_MM_FIRMWARE_HEAP_ID),
	MAKE_HEAP_INFO(ION_SYSTEM_HEAP_ID),
	MAKE_HEAP_INFO(ION_HEAP_ID_RESERVED),
	/* sentinel */
	{ .heap_id_string = NULL }
};

int find_heap_id_value(const char * const heap_id_string,
			unsigned int *val)
{
	struct heap_info *h;
	for (h = &heap_info[0]; h->heap_id_string; ++h) {
		if (0 == strcmp(heap_id_string, h->heap_id_string)) {
			*val = h->heap_id;
			return 0;
		}
	}
	return 1;
}

#define MAKE_FLAG_INFO(flag) { .flag_value = flag, .flag_string = #flag }

struct flag_info {
	unsigned int flag_value;
	const char * const flag_string;
} flag_info[] = {
	MAKE_FLAG_INFO(ION_FLAG_CACHED),
	MAKE_FLAG_INFO(ION_FLAG_CACHED_NEEDS_SYNC),
	MAKE_FLAG_INFO(ION_SECURE),
	MAKE_FLAG_INFO(ION_FLAG_SECURE),
	MAKE_FLAG_INFO(MP_GFP_KERNEL),
	MAKE_FLAG_INFO(MP_GFP_HIGHMEM),
	MAKE_FLAG_INFO(MP_GFP_ZERO),
	MAKE_FLAG_INFO(MP_GFP_HIGHUSER),
	MAKE_FLAG_INFO(MP_GFP_NOWARN),
	MAKE_FLAG_INFO(MP_GFP_NORETRY),
	MAKE_FLAG_INFO(MP_GFP_NO_KSWAPD),
	MAKE_FLAG_INFO(MP_GFP_WAIT),
	MAKE_FLAG_INFO(MP_GFPNOT_WAIT),
	MAKE_FLAG_INFO(MP_IOMMU_WRITE),
	MAKE_FLAG_INFO(MP_IOMMU_READ),
	MAKE_FLAG_INFO(MP_IOMMU_CACHE),
	MAKE_FLAG_INFO(MP_IOMMU_SECURE),
	MAKE_FLAG_INFO(MP_IOMMU_ATTACH),
	/* sentinel */
	{ .flag_string = NULL }
};

int find_flag_value(const char * const flag, int *val)
{
	struct flag_info *f;
	if (0 == strcmp(flag, "0")) {
		*val = 0;
		return 0;
	}
	for (f = &flag_info[0]; f->flag_string; ++f) {
		if (0 == strcmp(flag, f->flag_string)) {
			*val = f->flag_value;
			return 0;
		}
	}
	return 1;
}

uint64_t parse_flags(const char * const word)
{
	uint64_t ret = 0;
	int i, nflags;
	char *flags_words[MAX_FLAGS];
	size_t len = strlen(word) + 1;

	for (i = 0; i < MAX_FLAGS; ++i)
		MALLOC(char *, flags_words[i], MAX_FLAGS_STRING_LEN);

	nflags = split_string(word, '|', flags_words, MAX_FLAGS_STRING_LEN);

	for (i = 0; i < nflags; ++i) {
		int f;
		if (find_flag_value(flags_words[i], &f))
			warnx("Unknown flag: %s\n", flags_words[i]);
		else
			ret |= f;
	}

	for (i = 0; i < MAX_FLAGS; ++i)
		free(flags_words[i]);

	return ret;
}

bool parse_bool(const char * const word)
{
	return 0 == strcmp("true", word);
}

/* how many more alloc profile entries to {re-,m}alloc when we need more */
#define MORE_PROFILE_ENTRIES 30

/**
 * get_alloc_profile() - Get allocation profile
 *
 * @reader: struct alloc_profile_reader to use for reading lines.
 */
struct alloc_profile_entry *get_alloc_profile(
	struct alloc_profile_reader *reader)
{
	char *buf = NULL;
	char *words[MAX_ALLOC_PROFILE_FIELDS];
	int i;
	struct alloc_profile_entry *current = NULL, *base = NULL, new;
	int nentries = 0;
	int current_capacity = 0;
	const size_t more_alloc_size = sizeof(struct alloc_profile_entry)
		* MORE_PROFILE_ENTRIES;

	for (i = 0; i < MAX_ALLOC_PROFILE_FIELDS; ++i)
		MALLOC(char *, words[i], MAX_ALLOC_PROFILE_WORD_LEN);

	for (;;) {
		struct line_info line_info;
		int nwords;
		struct alloc_profile_handler *iter;
		const char *tmp;
		/*
		 * On the first iteration, buf = NULL. On subsequent
		 * interations buf is a pointer that was returned by
		 * strdup. Free it.
		 */
		free(buf);

		memset(&new, 0, sizeof(new));

		if (current_capacity == nentries) {
			size_t current_size = (current_capacity
					* sizeof(struct alloc_profile_entry));
			REALLOC(struct alloc_profile_entry *,
				base, current_size + more_alloc_size);
			current = base + nentries;
			memset(current, 0, more_alloc_size);
			current_capacity += MORE_PROFILE_ENTRIES;
		}

		tmp = reader->getline(reader);
		if (!tmp)
			break;

		STRDUP(buf, tmp);

		nwords = split_string(buf, ',', words,
				MAX_ALLOC_PROFILE_WORD_LEN);
		if (nwords < 1) {
			warn("Malformed line: `%s'", buf);
			continue;
		}

		for_each_alloc_profile_handler(iter) {
			if (strcmp(words[0], iter->keyword))
				continue;
			new.handler = iter;
			break;
		}
		if (!new.handler) {
			warnx("Couldn't find parser for %s\n",
				words[0]);
			continue;
		}
		if (iter->ops->ctor)
			iter->ops->ctor(&new);

		if (iter->priv_size)
			MALLOC(void *, new.priv, iter->priv_size);

		line_info.words = &words[0];
		line_info.nwords = nwords;
		if (iter->ops->parse)
			if (iter->ops->parse(&new, &line_info)) {
				warnx("Error in parser for %s\n", words[0]);
				continue;
			}

		*current++ = new;
		nentries++;
	}

	/* sentinel */
	new.handler = NULL;
	*current = new;

	for (i = 0; i < MAX_ALLOC_PROFILE_FIELDS; ++i)
		free(words[i]);

	return nentries ? base : NULL;
}
