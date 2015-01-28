/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <libgen.h>		/* for basename */
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/queue.h>
#include <getopt.h>
#include <linux/msm_ion.h>
#include "memory_prof.h"
#include "memory_prof_module.h"
#include "memory_prof_util.h"
#include "alloc_profiles.h"

static unsigned int sleepiness;
static int verbosity;

static void sleepy()
{
	usleep(sleepiness);
}

static void print_n_slow(char *st)
{
	puts(st);
	sleepy();
}

static void hr()
{
	puts("---------------");
}

/**
 * @ionfd [out] ion fd. On success, user must close.
 * @alloc_data [in/out] alloc data. On success, user must free.
 * returns 0 on success, 1 otherwise
 */
int alloc_me_up_some_ion(int ionfd,
			struct ion_allocation_data *alloc_data)
{
	int rc = 0;

	rc = ioctl(ionfd, ION_IOC_ALLOC, alloc_data);
	if (rc)
		perror("couldn't do ion alloc");

	return rc;
}

/**
 * mmaps an Ion buffer to *buf.
 *
 * @ionfd - the ion fd
 * @handle - the handle to the Ion buffer to map
 * @len - the length of the buffer
 * @buf - [output] the userspace buffer where the ion buffer will be
 * mapped
 * @map_fd - [output] the fd corresponding to the ION_IOC_MAP call
 * will be saved in *map_fd. *You should close it* when you're done
 * with it (and have munmap'd) *buf.
 *
 * Returns 0 on success, 1 otherwise
 */
static int mmap_my_ion_buffer(int ionfd, ion_user_handle_t handle, size_t len,
			char **buf, int *map_fd)
{
	struct ion_fd_data fd_data;
	int rc;

	fd_data.handle = handle;

	rc = ioctl(ionfd, ION_IOC_MAP, &fd_data);
	if (rc) {
		perror("couldn't do ION_IOC_MAP");
		return 1;
	}

	*map_fd = fd_data.fd;

	*buf = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED,
		*map_fd, 0);

	return 0;
}

/**
 * Allocates an ion buffer and mmaps it into *buf. See
 * alloc_me_up_some_ion and mmap_my_ion_buffer for an explanation of
 * the parameters here.
 */
static int alloc_and_map_some_ion(int ionfd,
				struct ion_allocation_data *alloc_data,
				char **buf, int *map_fd)
{
	int rc;

	rc = alloc_me_up_some_ion(ionfd, alloc_data);
	if (rc)
		return rc;
	rc = mmap_my_ion_buffer(ionfd, alloc_data->handle, alloc_data->len,
				buf, map_fd);
	if (rc) {
		rc |= ioctl(ionfd, ION_IOC_FREE, &alloc_data->handle);
	}
	return rc;
}

int do_basic_ion_sanity_test(int ionfd, ion_user_handle_t handle,
			unsigned long size)
{
	int rc;
	unsigned long i, squelched = 0;
	struct ion_fd_data fd_data;
	struct ion_flush_data flush_data;
	struct ion_custom_data custom_data;
	uint8_t *buf;
	bool integrity_good = true;
	int error_vals[256];

	memset(error_vals, 0, sizeof(error_vals));

	fd_data.handle = handle;

	rc = ioctl(ionfd, ION_IOC_MAP, &fd_data);
	if (rc) {
		perror("couldn't do ION_IOC_MAP");
		goto err_out;
	}

	buf = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
		fd_data.fd, 0);

	if (buf == MAP_FAILED) {
		perror("couldn't do mmap");
		rc = 1;
		goto err_close_fd_data;
	}

	for (i = 0; i < size; ++i)
		if (buf[i]) {
			printf("Buffer wasn't zero'd at offset %ld! got: %x\n",
				i, buf[i]);
			rc = 1;
			goto err_munmap;
		}

	memset(buf, 0xA5, size);
	flush_data.handle = handle;
	flush_data.vaddr = buf;
	flush_data.length = size;
	custom_data.cmd = ION_IOC_CLEAN_INV_CACHES;
	custom_data.arg = (unsigned long) &flush_data;
	if (ioctl(ionfd, ION_IOC_CUSTOM, &custom_data)) {
		perror("Couldn't flush caches");
		rc = 1;
		goto err_munmap;
	} else {
		puts("flushed caches");
	}

	for (i = 0; i < size; ++i) {
		if (buf[i] != 0xA5) {
			error_vals[buf[i]]++;
			if (!integrity_good) {
				squelched++;
				if (verbosity < 2)
					continue;
			}
			printf("  Data integrity error at offset 0x%lx from 0x%p (%x != 0xa5))!!!!\n",
				i, buf, buf[i]);
			integrity_good = false;
			rc = 1;
		}
	}
	if (squelched)
		printf("  (squelched %ld additional integrity error%s)\n",
			squelched, squelched == 1 ? "" : "s");

	if (integrity_good)
		printf("  Buffer integrity check succeeded\n");
	else if (verbosity)
		for (i = 0; i < 255; ++i)
			if (error_vals[i])
				printf("Saw %d instead of 0xA5 %d times\n",
					(int) i, error_vals[i]);

err_munmap:
	if (munmap(buf, size)) {
		rc = 1;
		perror("couldn't do munmap");
	}

err_close_fd_data:
	close(fd_data.fd);
err_out:
	return rc;
}

static int basic_ion_sanity_test(struct ion_allocation_data alloc_data)
{
	int ionfd, rc = 0;
	unsigned long size = alloc_data.len;

	ionfd = open(ION_DEV, O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open " ION_DEV);
		rc = ionfd;
		goto out;
	}

	rc = alloc_me_up_some_ion(ionfd, &alloc_data);
	if (rc)
		goto err0;

	rc |= do_basic_ion_sanity_test(ionfd, alloc_data.handle, size);

err1:
	rc |= ioctl(ionfd, ION_IOC_FREE, &alloc_data.handle);
err0:
	close(ionfd);
out:
	return rc;
}

static int basic_sanity_tests(unsigned long size)
{
	int lrc, rc = 0;

	struct ion_allocation_data system_alloc_data = {
		.align	   = SZ_4K,
		.len	   = size,
		.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID),
		.flags	   = 0,
	};

	struct ion_allocation_data system_contig_alloc_data = {
		.align	   = SZ_4K,
		.len	   = size,
		.heap_id_mask = ION_HEAP(ION_SYSTEM_CONTIG_HEAP_ID),
		.flags	   = 0,
	};

	puts("testing system without caching...");
	lrc = basic_ion_sanity_test(system_alloc_data);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system with caching...");
	system_alloc_data.flags |= ION_FLAG_CACHED;
	lrc = basic_ion_sanity_test(system_alloc_data);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system contig without caching...");
	lrc = basic_ion_sanity_test(system_contig_alloc_data);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system contig with caching...");
	system_contig_alloc_data.flags |= ION_FLAG_CACHED;
	lrc = basic_ion_sanity_test(system_contig_alloc_data);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	if (rc)
		puts("BASIC SANITY TESTS FAILED!!!!!!!! WOOOWWWW!!!!!!");
	else
		puts("All basic sanity tests passed");

	return rc;
}

/**
 * Free memory in alloc_list
 */
void free_mem_list(char **alloc_list)
{
	int i = 0;
	for (i = 0; i < MAX_PRE_ALLOC_SIZE; i++) {
		if (alloc_list[i] != NULL) {
			free(alloc_list[i]);
			alloc_list[i] = NULL;
		}
	}
}


/**
 * Allocate sizemb in alloc_list
 */
int alloc_mem_list(char **alloc_list, int sizemb)
{
	int i = 0;
	int alloc_size = 1*1024*1024 * sizeof(char);

	if (sizemb > MAX_PRE_ALLOC_SIZE) {
		return -1;
	}

	// Break allocation into 1 MB pieces to ensure
	// we easily find enough virtually contigous memory
	for (i = 0; i < sizemb; i++)
	{
		alloc_list[i] =(char *) malloc(alloc_size);
		if (alloc_list[i] == NULL)
		{
		    perror("couldn't allocate 1MB");
		    free_mem_list(alloc_list);
		    return -1;
		}

		// Memory must be accessed for it to be page backed.
		// We may want to randomize the data in the future
		// to prevent features such as KSM returning memory
		// to the system.
		memset(alloc_list[i], 1, alloc_size);
	}
	return 0;
}


/**
 * Returns the total time taken for ION_IOC_ALLOC
 *
 * @heap_id_mask - passed to ION_IOC_ALLOC
 * @flags - passed to ION_IOC_ALLOC
 * @size - passed to ION_IOC_ALLOC
 * @alloc_ms - [output] time taken (in MS) to complete the ION_IOC_ALLOC
 * @map_ms - [output] time taken (in MS) to complete the ION_IOC_MAP + mmap
 * @memset_ms - [output] time taken (in MS) to memset the Ion buffer
 * @free_ms - [output] time taken (in MS) to complete the ION_IOC_FREE
 * @do_pre_alloc - whether or not we should do a "pre-allocation"
 * @handle - an Ion handle to use for profiling. If 0, a buffer will be
 *           allocated for you.
 * @ionfd - fd to /dev/ion to be used (needed if you're passing in a
 *          handle). Pass -1 to open /dev/ion and get a new client.
 * @do_free - whether or not we should free the handle when we're done with it
 *
 * Returns 0 on success, 1 on failure.
 */
int do_profile_alloc_for_heap(unsigned int heap_id_mask,
			unsigned int flags, unsigned int size,
			double *alloc_ms, double *map_ms,
			double *memset_ms, double *free_ms,
			bool do_pre_alloc, ion_user_handle_t handle,
			int ionfd, bool do_free)
{
	int rc = 0, rc2;
	uint8_t *buf = MAP_FAILED;
	struct ion_fd_data fd_data;
	struct timeval tv_before, tv_after, tv_result;
	char **pre_alloc_list = NULL;
	bool need_to_close_ionfd = true;

	if (do_pre_alloc) {
		char *plist[MAX_PRE_ALLOC_SIZE];
		memset(plist, 0, MAX_PRE_ALLOC_SIZE * sizeof(char *));
		if (alloc_mem_list(plist, ion_pre_alloc_size) < 0) {
			rc = 1;
			perror("couldn't create pre-allocated buffer");
			goto out;
		}
		pre_alloc_list = plist;
	}

	if (alloc_ms)
		*alloc_ms = 0;
	if (free_ms)
		*free_ms = 0;
	if (map_ms) *map_ms = 0;
	if (memset_ms) *memset_ms = 0;

	if (ionfd < 0) {
		ionfd = open(ION_DEV, O_RDONLY);
		if (ionfd < 0) {
			rc = 1;
			perror("couldn't open " ION_DEV);
			goto free_mem_list;
		}
	} else {
		need_to_close_ionfd = false;
	}


	if (!handle) {
		struct ion_allocation_data alloc_data = {
			.align	   = SZ_4K,
			.len	   = size,
			.heap_id_mask = heap_id_mask,
			.flags	   = flags,
		};

		rc = gettimeofday(&tv_before, NULL);
		if (rc) {
			perror("couldn't get time of day");
			goto close_ion;
		}

		rc2 = ioctl(ionfd, ION_IOC_ALLOC, &alloc_data);
		rc = gettimeofday(&tv_after, NULL);
		if (rc2) {
			rc = rc2;
			goto close_ion;
		}
		if (rc) {
			perror("couldn't get time of day");
			goto free;
		}

		handle = alloc_data.handle;

		if (alloc_ms)
			*alloc_ms = timeval_ms_diff(tv_after, tv_before);
	}

	if (map_ms) {
		rc = gettimeofday(&tv_before, NULL);
		if (rc) {
			perror("couldn't get time of day");
			goto free;
		}

		fd_data.handle = handle;

		rc = ioctl(ionfd, ION_IOC_MAP, &fd_data);
		if (rc) {
			perror("couldn't do ION_IOC_MAP");
			goto free;
		}

		buf = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd_data.fd, 0);

		rc2 = gettimeofday(&tv_after, NULL);

		if (buf == MAP_FAILED) {
			perror("couldn't do mmap");
			goto fd_data_close;
		}

		if (rc2) {
			perror("couldn't get time of day");
			goto munmap;
		}

		*map_ms = timeval_ms_diff(tv_after, tv_before);
	}

	if (memset_ms && buf != MAP_FAILED) {
		rc = gettimeofday(&tv_before, NULL);
		if (rc) {
			perror("couldn't get time of day");
			goto munmap;
		}

		memset(buf, 0xA5, size);
		rc2 = gettimeofday(&tv_after, NULL);
		if (rc2) {
			perror("couldn't get time of day");
			goto munmap;
		}
		*memset_ms = timeval_ms_diff(tv_after, tv_before);
	}

munmap:
	if (buf != MAP_FAILED)
		if (munmap(buf, size))
			perror("couldn't do munmap");

fd_data_close:
	close(fd_data.fd);

	/*
	 * Okay, things are about to get messy. We're profiling our
	 * cleanup, but we might fail some stuff and need to
	 * cleanup... during cleanup.
	 */

	rc = gettimeofday(&tv_before, NULL);
	if (rc)
		perror("couldn't get time of day");
free:
	if (do_free) {
		struct ion_handle_data handle_data;
		handle_data.handle = handle;
		ioctl(ionfd, ION_IOC_FREE, &handle_data);
		rc2 = gettimeofday(&tv_after, NULL);
		if (!rc) {
			if (rc2) {
				perror("couldn't get time of day");
				goto close_ion;
			}
			if (free_ms)
				*free_ms = timeval_ms_diff(tv_after, tv_before);
		}
	}
close_ion:
	if (need_to_close_ionfd)
		close(ionfd);
free_mem_list:
	if (do_pre_alloc)
		free_mem_list(pre_alloc_list);
out:
	return rc;
}

/**
 * Allocates an Ion buffer and profiles it. See
 * `do_profile_alloc_for_heap'.
 *
 * Returns 0 on success, 1 on failure.
 */
int profile_alloc_for_heap(unsigned int heap_id_mask,
			unsigned int flags, unsigned int size,
			double *alloc_ms, double *map_ms,
			double *memset_ms, double *free_ms)
{
	return do_profile_alloc_for_heap(
		heap_id_mask, flags, size, alloc_ms, map_ms, memset_ms, free_ms,
		true, 0, -1, true);
}

static int profile_kernel_alloc(void)
{
	int rc, memory_prof_fd;

	memory_prof_fd = open(MEMORY_PROF_DEV, 0);
	if (memory_prof_fd < 0) {
		perror("couldn't open " MEMORY_PROF_DEV);
		return 1;
	}
	rc = ioctl(memory_prof_fd, MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS);
	if (rc)
		perror("couldn't do MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS");
	close(memory_prof_fd);

	return rc;
}

void compute_stats(double stats[], int num,
		double *average, double *std_dev)
{
	int i;
	double sum = 0, sum_of_squares = 0;
	for (i = 0; i < num; ++i)
		sum += stats[i];
	*average = sum / num;

	for (i = 0; i < num; ++i)
		sum_of_squares += pow(stats[i] - *average, 2);
	*std_dev = sqrt(sum_of_squares / num);
}

void print_stats_results(const char *name, const char *flags_label,
			const char *size_string,
			double stats[], int reps)
{
	int i;
	struct timeval tv;
	unsigned long long time_ms;
	double average, std_dev;

	compute_stats(stats, reps, &average, &std_dev);

	gettimeofday(&tv, NULL);
	time_ms = TV_TO_MS(tv);
	printf("%s %llu %s %s %s average: %.2f std_dev: %.2f reps: %d\n",
		ST_PREFIX_DATA_ROW, time_ms,
		name, flags_label, size_string, average, std_dev, reps);
}

void print_a_bunch_of_stats_results(const char *name,
				const char *flags_label,
				const char *size_string,
				double alloc_stats[],
				double map_stats[],
				double memset_stats[],
				double free_stats[],
				int reps)
{
	char sbuf[70];
	snprintf(sbuf, 70, "ION_IOC_ALLOC %s", name);
	print_stats_results(sbuf, flags_label, size_string, alloc_stats, reps);
	if (map_stats) {
		snprintf(sbuf, 70, "mmap %s", name);
		print_stats_results(sbuf, flags_label, size_string,
				map_stats, reps);
	}
	if (memset_stats) {
		snprintf(sbuf, 70, "memset %s", name);
		print_stats_results(sbuf, flags_label, size_string,
				memset_stats, reps);
	}
	snprintf(sbuf, 70, "ION_IOC_FREE %s", name);
	print_stats_results(sbuf, flags_label, size_string, free_stats, reps);
}

struct string_reader_getline_data {
	char **lines;
	int nlines;
	int linum;
};

static const char *alloc_profile_string_reader_getline(
	struct alloc_profile_reader *reader)
{
	struct string_reader_getline_data *data = reader->priv;
	if (data->linum > data->nlines)
		return NULL;
	return data->lines[data->linum++ - 1];
}

static struct alloc_profile_entry *get_alloc_profile_from_string(
	const char *eval_program)
{
	const char *tmp;
	char **lines;
	struct alloc_profile_entry *alloc_profile;
	struct alloc_profile_reader reader;
	struct string_reader_getline_data reader_data;
	int nlines = 1, i;

	for (tmp = eval_program; *tmp != '\0'; ++tmp) {
		if (*tmp == ';')
			nlines++;
	}

	MALLOC(char **, lines, sizeof(*lines) * nlines);

	for (i = 0; i < nlines; ++i)
		MALLOC(char *, lines[i], MAX_ALLOC_PROFILE_LINE_LEN);

	i = split_string(eval_program, ';', lines, MAX_ALLOC_PROFILE_LINE_LEN);
	if (i != nlines)
		errx(1,
			"Error parsing --eval program! Expected %d lines, got %d (check for trailing ;)",
			nlines, i);

	reader.getline = alloc_profile_string_reader_getline;
	reader.priv = &reader_data;
	reader_data.lines = lines;
	reader_data.nlines = nlines;
	reader_data.linum = 1;

	/**
	 * get_alloc_profile will run the necessary .ctor and
	 * .parse callbacks to build up an array of
	 * alloc_profile_entry objects
	 */
	alloc_profile = get_alloc_profile(&reader);

	if (!alloc_profile)
		errx(1, "Couldn't parse --eval program");

	for (i = 0; i < nlines; ++i)
		free(lines[i]);

	free(lines);

	return alloc_profile;
}

struct file_reader_getline_data {
	char *buf;
	FILE *fp;
	int linum;
	const char const *path;
};

static const char *alloc_profile_file_reader_getline(
	struct alloc_profile_reader *reader)
{
	char *buf;
	struct file_reader_getline_data *data = reader->priv;

	for (;;) {
		buf = fgets(data->buf, MAX_ALLOC_PROFILE_LINE_LEN, data->fp);
		if (!buf) {
			if (!feof(data->fp))
				err(1, "Error reading line %d from %s",
					data->linum, data->path);
			return NULL;
		}
		if (!(buf[0] == '#' || buf[0] == '\n' || buf[0] == '\r'))
			break;
	}
	data->linum++;
	/* strip off trailing newline */
	data->buf[strlen(data->buf) - 1] = '\0';
	return data->buf;
}

static struct alloc_profile_entry *get_alloc_profile_from_file(
	const char *alloc_profile_path)
{
	struct alloc_profile_entry *alloc_profile;
	struct file_reader_getline_data reader_data;
	struct alloc_profile_reader reader;
	bool using_stdin = alloc_profile_path &&
		!strcmp(alloc_profile_path, "-");
	int rc = 0;

	reader.getline = alloc_profile_file_reader_getline;
	reader.priv = &reader_data;
	reader_data.linum = 1;
	reader_data.path = alloc_profile_path;

	if (!alloc_profile_path)
		alloc_profile_path = ALLOC_PROFILES_PATH_STRING "/builtin.txt";

	printf("Using allocation profile: %s\n",
		using_stdin ? "(STDIN)" : alloc_profile_path);

	MALLOC(char *, reader_data.buf, MAX_ALLOC_PROFILE_LINE_LEN);

	if (using_stdin) {
		reader_data.fp = stdin;
	} else {
		reader_data.fp = fopen(alloc_profile_path, "r");
		if (!reader_data.fp)
			err(1, "Couldn't read %s\n", alloc_profile_path);
	}

	/**
	 * get_alloc_profile will run the necessary .ctor and
	 * .parse callbacks to build up an array of
	 * alloc_profile_entry objects
	 */
	alloc_profile = get_alloc_profile(&reader);

	if (!alloc_profile)
		errx(1, "Couldn't parse allocation profile: %s\n",
			alloc_profile_path);

	if (!using_stdin)
		fclose(reader_data.fp);

	return alloc_profile;
}

static int heap_profiling(struct alloc_profile_entry *alloc_profile)
{
	struct alloc_profile_entry *entry;
	struct alloc_profile_handler *iter;
	int rc = 0;

	/**
	 * Run .global_setup callbacks for any handlers being used in
	 * this allocation profile
	 */
	for_each_alloc_profile_handler(iter) {
		int (*gsetup)(struct alloc_profile_entry entries[]) = NULL;
		for_each_alloc_profile_entry(entry, alloc_profile) {
			if (entry->handler == iter) {
				gsetup = iter->ops->global_setup;
				break;
			}
		}
		if (gsetup)
			if (gsetup(alloc_profile))
				warn("global_setup failed for %s!",
					iter->keyword);
	}

	puts("All times are in milliseconds unless otherwise indicated");
	printf(ST_PREFIX_PREALLOC_SIZE " pre-alloc size (MB): %d\n",
		ion_pre_alloc_size);

	/* Run .run callbacks */
	for_each_alloc_profile_entry(entry, alloc_profile)
		rc |= entry->handler->ops->run(entry);

	/**
	 * Run .global_teardown callbacks for any handlers being used
	 * in this allocation profile
	 */
	for_each_alloc_profile_handler(iter) {
		struct alloc_profile_entry *entry;
		void (*gteardown)(void) = NULL;
		for_each_alloc_profile_entry(entry, alloc_profile) {
			if (entry->handler == iter) {
				gteardown = iter->ops->global_teardown;
				break;
			}
		}
		if (gteardown)
			gteardown();
	}

	/* Run .dtor callbacks and free allocations made with priv_size */
	for_each_alloc_profile_entry(entry, alloc_profile) {
		if (entry->handler->ops->dtor)
			entry->handler->ops->dtor(entry);
		if (entry->handler->priv_size)
			free(entry->priv);
	}

	return rc;
}

static int oom_test(void)
{
	int rc, ionfd, cnt = 0;
	struct ion_allocation_data alloc_data = {
		.len	   = SZ_8M,
		.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID),
		.flags	   = 0,
	};

	LIST_HEAD(handle_list, ion_handle_node) head;
	struct handle_list *headp;
	struct ion_handle_node {
		ion_user_handle_t handle;
		LIST_ENTRY(ion_handle_node) nodes;
	} *np;
	LIST_INIT(&head);

	ionfd = open(ION_DEV, O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open " ION_DEV);
		return 1;
	}


	for (;; cnt++) {
		rc = ioctl(ionfd, ION_IOC_ALLOC, &alloc_data);
		if (rc) {
			/* game over! */
			break;
		} else {
			MALLOC(struct ion_handle_node *,
				np, sizeof(struct ion_handle_node));
			np->handle = alloc_data.handle;
			LIST_INSERT_HEAD(&head, np, nodes);
		}
		printf("Allocated %d MB so far...\n", cnt * 8);
	}

	printf("Did %d 8M allocations before dying\n", cnt);

	while (head.lh_first != NULL) {
		np = head.lh_first;
		ioctl(ionfd, ION_IOC_FREE, np->handle);
		LIST_REMOVE(head.lh_first, nodes);
		free(np);
	}

	return 0;
}

static int leak_test(void)
{
	int ionfd;
	struct ion_fd_data fd_data;
	struct ion_allocation_data alloc_data = {
		.len	   = SZ_4K,
		.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID),
		.flags	   = 0,
	};

	puts("About to leak a handle...");
	fflush(stdout);

	ionfd = open(ION_DEV, O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open " ION_DEV);
		return 1;
	}

	alloc_me_up_some_ion(ionfd, &alloc_data);

	fd_data.handle = alloc_data.handle;

	if (ioctl(ionfd, ION_IOC_MAP, &fd_data))
		perror("Couldn't ION_IOC_MAP the buffer");

	close(ionfd);

	puts("closed ionfd w/o free'ing handle.");
	puts("If you have CONFIG_ION_LEAK_CHECK turned on in");
	puts("your kernel and have already done:");
	puts("echo 1 > /debug/ion/check_leaks_on_destroy");
	puts("then you should have seen a warning on your");
	puts("console.");
	puts("We will now sleep for 5 seconds for you to check");
	puts("<debugfs>/ion/check_leaked_fds");
	sleep(5);

	return 0;
}

/**
 * Allocate two ion buffers, mmap them, memset each of them to
 * something different, memcpy from one to the other, then check that
 * they are equal.

 * We time the memcpy operation and return the number of milliseconds
 * it took in *time_elapsed_memcpy_ms. When time_elapsed_flush_ms is
 * not NULL, we flush the dst buffer and return the number of
 * milliseconds it took to do so in *time_elapsed_flush_ms.
 *
 * @src_alloc_data - source allocation data
 * @dst_alloc_data - source allocation data
 * @time_elapsed_memcpy_ms - [output] time taken (in MS) to do the memcpy
 * @time_elapsed_flush_ms - [output] time taken (in MS) to do the
 * cache flush. Pass NULL if you don't want to do the flushing.
 *
 */
static int do_ion_memcpy(struct ion_allocation_data src_alloc_data,
			struct ion_allocation_data dst_alloc_data,
			double *time_elapsed_memcpy_ms,
			double *time_elapsed_flush_ms)
{
	int ionfd, i, src_fd, dst_fd, fail_index = 0, rc = 0;
	char *src, *dst;
	struct timeval tv;
	size_t len = src_alloc_data.len;

	ionfd = open(ION_DEV, O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open " ION_DEV);
		rc = 1;
		goto out;
	}

	if (alloc_and_map_some_ion(ionfd, &src_alloc_data, &src, &src_fd)) {
		perror("Couldn't alloc and map src buffer");
		rc = 1;
		goto close_ionfd;
	}
	if (alloc_and_map_some_ion(ionfd, &dst_alloc_data, &dst, &dst_fd)) {
		perror("Couldn't alloc and map dst buffer");
		rc = 1;
		goto free_src;
	}

	memset(src, 0x5a, len);
	memset(dst, 0xa5, len);
	mprof_tick(&tv);
	memcpy(dst, src, len);
	*time_elapsed_memcpy_ms = mprof_tock(&tv);

	if (time_elapsed_flush_ms) {
		struct ion_flush_data flush_data;
		struct ion_custom_data custom_data;
		flush_data.handle = dst_alloc_data.handle;
		flush_data.vaddr = dst;
		flush_data.length = len;
		custom_data.cmd = ION_IOC_CLEAN_CACHES;
		custom_data.arg = (unsigned long) &flush_data;
		mprof_tick(&tv);
		if (ioctl(ionfd, ION_IOC_CUSTOM, &custom_data)) {
			perror("Couldn't flush caches");
			rc = 1;
		}
		*time_elapsed_flush_ms = mprof_tock(&tv);
	}

	if (!buffers_are_equal(src, dst, len, &fail_index)) {
		printf("WARNING: buffer integrity check failed\n"
			"dst[%d]=0x%x, src[%d]=0x%x\n",
			fail_index, dst[fail_index],
			fail_index, src[fail_index]);
		rc = 1;
	}

munmap_and_close:
	munmap(src, len);
	munmap(dst, len);
	close(src_fd);
	close(dst_fd);
free_dst:
	ioctl(ionfd, ION_IOC_FREE, &dst_alloc_data.handle);
free_src:
	ioctl(ionfd, ION_IOC_FREE, &src_alloc_data.handle);
close_ionfd:
	close(ionfd);
out:
	return rc;
}

int profile_ion_cache_ops_for_heap(unsigned int heap_id_mask,
					unsigned int flags, unsigned int size,
					double *time_elapsed_flush_ms,
					bool cache_clean, bool cache_inv)
{
	struct ion_allocation_data alloc_data;
	int ionfd, data_fd, rc = 0;
	char *buffer;
	struct timeval tv;
	struct ion_flush_data flush_data;
	struct ion_custom_data custom_data;

	ionfd = open(ION_DEV, O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open " ION_DEV);
		rc = 1;
		goto out;
	}

	alloc_data.len = size;
	alloc_data.align = SZ_4K;
	alloc_data.heap_id_mask = heap_id_mask;
	alloc_data.flags = flags;

	if (alloc_and_map_some_ion(ionfd, &alloc_data, &buffer, &data_fd)) {
		perror("Couldn't alloc and map buffer");
		rc = 1;
		goto close_ionfd;
	}

	memset(buffer, 0x5a, size);

	flush_data.handle = alloc_data.handle;
	flush_data.vaddr = buffer;
	flush_data.length = size;

	if (cache_clean && cache_inv)
		custom_data.cmd = ION_IOC_CLEAN_INV_CACHES;
	else if (cache_clean)
		custom_data.cmd = ION_IOC_CLEAN_CACHES;
	else if (cache_inv)
		custom_data.cmd = ION_IOC_INV_CACHES;
	else {
		perror("Not cleaning or invalidating caches");
		goto munmap_and_close;
	}
	custom_data.arg = (unsigned long) &flush_data;
	mprof_tick(&tv);
	if (ioctl(ionfd, ION_IOC_CUSTOM, &custom_data)) {
		perror("Couldn't flush caches");
		rc = 1;
	}

	*time_elapsed_flush_ms = mprof_tock(&tv);

munmap_and_close:
	munmap(buffer, size);
	close(data_fd);
free_src:
	ioctl(ionfd, ION_IOC_FREE, &alloc_data.handle);
close_ionfd:
	close(ionfd);
out:
	return rc;
}
/**
 * Profiles the time it takes to copy between various types of Ion
 * buffers. Namely, copies between every permutation of
 * cached/uncached buffer (4 permutations total). Also profiles the
 * amount of time it takes to flush the destination buffer on a
 * cached->cached copy.
 */
static int ion_memcpy_test(void)
{
	struct ion_allocation_data src_alloc_data, dst_alloc_data;
	struct timeval tv;
	double time_elapsed_memcpy_ms = 0,
		time_elapsed_flush_ms = 0;

	src_alloc_data.len = SZ_4M;
	src_alloc_data.align = SZ_4K;
	src_alloc_data.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID);
	dst_alloc_data.len = SZ_4M;
	dst_alloc_data.align = SZ_4K;
	dst_alloc_data.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID);

	src_alloc_data.flags = 0;
	dst_alloc_data.flags = 0;
	if (do_ion_memcpy(src_alloc_data, dst_alloc_data,
				&time_elapsed_memcpy_ms, NULL)) {
		printf("Uncached -> Uncached: FAIL\n");
	} else {
		printf("Uncached -> Uncached: %.3fms\n", time_elapsed_memcpy_ms);
	}

	src_alloc_data.flags = 0;
	dst_alloc_data.flags = ION_FLAG_CACHED;
	if (do_ion_memcpy(src_alloc_data, dst_alloc_data,
				&time_elapsed_memcpy_ms, NULL)) {
		printf("Uncached -> Cached: FAIL\n");
	} else {
		printf("Uncached -> Cached: %.3fms\n", time_elapsed_memcpy_ms);
	}

	src_alloc_data.flags = ION_FLAG_CACHED;
	dst_alloc_data.flags = 0;
	if (do_ion_memcpy(src_alloc_data, dst_alloc_data,
				&time_elapsed_memcpy_ms, NULL)) {
		printf("Cached -> Uncached: FAIL\n");
	} else {
		printf("Cached -> Uncached: %.3fms\n", time_elapsed_memcpy_ms);
	}

	src_alloc_data.flags = ION_FLAG_CACHED;
	dst_alloc_data.flags = ION_FLAG_CACHED;
	if (do_ion_memcpy(src_alloc_data, dst_alloc_data,
				&time_elapsed_memcpy_ms,
				&time_elapsed_flush_ms)) {
		printf("Cached -> Cached: FAIL\n");
	} else {
		printf("Cached -> Cached: %.3fms (cache flush took %.3fms)\n",
			time_elapsed_memcpy_ms, time_elapsed_flush_ms);
	}

	return 0;
}

/**
 * Memory throughput test. Print some stats.
 */
static int mmap_memcpy_test(void)
{
	int rc = 0;
	char *chunk, *src, *dst;
	struct timeval tv;
	double *data_rates;
	double sum = 0, sum_of_squares = 0, average, std_dev, dmin, dmax;
	int iters = 1000, memcpy_iters = 100;
	int i, j;
	const size_t chunk_len = SZ_1M;

	MALLOC(double *, data_rates, iters * sizeof(double));

	chunk = mmap(NULL, chunk_len,
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS,
		-1, 0);
	if (chunk == MAP_FAILED) {
		perror("Couldn't allocate 1MB buffer with mmap\n");
		rc = 1;
		goto free_data_rates;
	}

	/*
	 * Some systems appear to do this trick where they mprotect
	 * the first page of a large allocation with
	 * PROT_NONE. Possibly to protect against someone else
	 * overflowing into their buffer? We do it here just to
	 * emulate the "real-world" a little closer.
	 */
	if (mprotect(chunk, SZ_4K, PROT_NONE)) {
		perror("Couldn't mprotect the first page of the 1MB buffer\n");
		rc = 1;
		goto munmap_chunk;
	}

	src = chunk + SZ_4K;
	dst = chunk + SZ_512K;
	memset(src, 0x5a, SZ_64K);
	memset(dst, 0xa5, SZ_64K);

	for (i = 0; i < iters; ++i) {
		float elapsed_ms;
		mprof_tick(&tv);
		for (j = 0; j < memcpy_iters; ++j)
			memcpy(dst, src, SZ_64K);
		elapsed_ms = mprof_tock(&tv);
		/* units: MB/s */
		data_rates[i] = ((SZ_64K * memcpy_iters) / SZ_1M)
			/ (elapsed_ms / 1000);
	}

	dmin = dmax = data_rates[0];
	for (i = 0; i < iters; ++i) {
		sum += data_rates[i];
		dmin = MIN(dmin, data_rates[i]);
		dmax = MAX(dmax, data_rates[i]);
	}
	average = sum / iters;

	for (i = 0; i < iters; ++i)
		sum_of_squares += pow(data_rates[i] - average, 2);
	std_dev = sqrt(sum_of_squares / iters);

	printf("average: %.3f MB/s, min: %.3f MB/s, max: %.3f MB/s, std_dev: %.3f MB/s\n",
		average, dmin, dmax, std_dev);

munmap_chunk:
	munmap(chunk, chunk_len);
free_data_rates:
	free(data_rates);

	return rc;
}

static int file_exists(const char const *fname)
{
	struct stat tmp;
	return stat(fname, &tmp) == 0;
}

#define USAGE_STRING							\
	"Usage: %s [options]\n"						\
	"\n"								\
	"Supported options:\n"						\
	"\n"								\
	"  -h         Print this message and exit\n"			\
	"  -a         Do the adversarial test (same as -l)\n"		\
	"  -b         Do basic sanity tests\n"				\
	"  -z         Size (in bytes) of buffer for basic sanity tests\n" \
	"             (default=65536 (64KB))\n"			\
	"  -e         Do Ion heap profiling.\n"				\
	"  -i file    Input `alloc profile' for heap profiling (-e)\n"	\
	"             (Runs a general default profile if omitted).\n"	\
	"             If - is given, the allocation profile is read\n"	\
	"             from stdin.\n"					\
	"  -k         Do kernel alloc profiling (requires kernel module)\n" \
	"  -l         Do leak test (leak an ion handle)\n"		\
	"  -n         Do the nominal test (same as -b)\n"		\
	"  -o         Do OOM test (alloc from Ion system heap until OOM)\n" \
	"  -p MS      Sleep for MS milliseconds between stuff (for debugging)\n" \
	"  -r         Do the repeatability test\n"			\
	"  -s         Do the stress test (same as -e)\n"		\
	"  -t MB      Size (in MB) of temp buffer pre-allocated before Ion allocations (default 0 MB)\n" \
	"  -v         Increase verbosity. Pass multiple times to increase verbosity\n" \
	"  --ion-memcpy-test\n" \
	"             Does some memcpy's between various types of Ion buffers\n" \
	"  --mmap-memcpy-test\n" \
	"             Does some memcpy's between some large buffers allocated\n" \
	"             by mmap\n"					\
	"  --eval PROGRAM\n"						\
	"             Evaluates PROGRAM as an allocation profile.\n"	\
	"             Multiple commands should be separated by a semi-colon (;).\n"

static void usage(char *progname)
{
	printf(USAGE_STRING, progname);
}

static struct option memory_prof_options[] = {
	{"ion-memcpy-test", no_argument, 0, 0},
	{"mmap-memcpy-test", no_argument, 0, 0},
	{"eval", required_argument, 0, 0},
	{0, 0, 0, 0}
};

int ion_pre_alloc_size = ION_PRE_ALLOC_SIZE_DEFAULT;

int main(int argc, char *argv[])
{
	int rc = 0, i, opt;
	unsigned long basic_sanity_size = SZ_64K;
	bool do_basic_sanity_tests = false;
	bool do_heap_profiling = false;
	bool do_kernel_alloc_profiling = false;
	bool do_oom_test = false;
	bool do_leak_test = false;
	bool do_ion_memcpy_test = false;
	bool do_mmap_memcpy_test = false;
	bool did_something = false;
	char *alloc_profile_path = NULL;
	int num_reps = 1;
	int option_index = 0;
	char *eval_program = NULL;

	while (-1 != (opt = getopt_long(
				argc,
				argv,
				"abe::hi:klmnop:rs::t:vz:",
				memory_prof_options,
				&option_index))) {
		switch (opt) {
		case 0:
			if (strcmp("ion-memcpy-test",
					memory_prof_options[option_index].name)
				== 0) {
				do_ion_memcpy_test = true;
				break;
			}
			if (strcmp("mmap-memcpy-test",
					memory_prof_options[option_index].name)
				== 0) {
				do_mmap_memcpy_test = true;
				break;
			}
			if (strcmp("eval",
					memory_prof_options[option_index].name)
				== 0) {
				eval_program = strdup(optarg);
				break;
			}
			printf("Unhandled longopt: %s\n",
				memory_prof_options[option_index].name);
			break;
		case 't':
			ion_pre_alloc_size = atoi(optarg);
			break;
		case 'n':
		case 'b':
			do_basic_sanity_tests = true;
			break;
		case 's':
		case 'e':
			do_heap_profiling = true;
			break;
		case 'i':
			STRDUP(alloc_profile_path, optarg);
			if (strcmp(alloc_profile_path, "-") &&
				!file_exists(alloc_profile_path))
				err(1, "Can't read alloc profile file: %s",
					alloc_profile_path);
			break;
		case 'k':
			do_kernel_alloc_profiling = true;
			break;
		case 'a':
		case 'l':
			do_leak_test = true;
			break;
		case 'o':
			do_oom_test = true;
			break;
		case 'r':
			num_reps = NUM_REPS_FOR_REPEATABILITY;
			break;
		case 'p':
			/* ms to us */
			sleepiness = atoi(optarg) * 1000;
			break;
		case 'v':
			verbosity++;
			break;
		case 'z':
			basic_sanity_size = atoi(optarg);
			break;
		case 'h':
		default:
			usage(basename(argv[0]));
			exit(1);
		}
	}

	/* make sure we don't get killed: */
	set_oom_score_adj_self(-1000);
	set_oom_score_adj_parent(-1000);

	if (do_basic_sanity_tests) {
		for (i = 0; i < num_reps; ++i)
			rc |= basic_sanity_tests(basic_sanity_size);
		did_something = true;
	}
	if (do_heap_profiling) {
		struct alloc_profile_entry *alloc_profile
			= get_alloc_profile_from_file(alloc_profile_path);
		for (i = 0; i < num_reps; ++i)
			rc |= heap_profiling(alloc_profile);
		did_something = true;
	}
	if (eval_program) {
		struct alloc_profile_entry *alloc_profile
			= get_alloc_profile_from_string(eval_program);
		rc |= heap_profiling(alloc_profile);
		did_something = true;
	}
	if (do_kernel_alloc_profiling) {
		for (i = 0; i < num_reps; ++i)
			rc |= profile_kernel_alloc();
		did_something = true;
	}
	if (do_oom_test) {
		for (i = 0; i < num_reps; ++i)
			rc |= oom_test();
		did_something = true;
	}
	if (do_leak_test) {
		for (i = 0; i < num_reps; ++i)
			rc |= leak_test();
		did_something = true;
	}

	if (do_ion_memcpy_test) {
		rc |= ion_memcpy_test();
		did_something = true;
	}

	if (do_mmap_memcpy_test) {
		rc |= mmap_memcpy_test();
		did_something = true;
	}

	if (!did_something)
		printf("Nothing to do. Try %s -h\n", argv[0]);

	free(eval_program);
	free(alloc_profile_path);

	return rc;
}
