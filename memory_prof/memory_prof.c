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
#include <linux/msm_ion.h>
#include <memory_prof_module.h>

#define NUM_REPS_FOR_HEAP_PROFILING 100
#define NUM_REPS_FOR_REPEATABILITY 100

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

static unsigned int sleepiness;

static int memory_prof_fd;

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
static int alloc_me_up_some_ion(int *ionfd,
				struct ion_allocation_data *alloc_data)
{
	int rc = 0;
	*ionfd = open("/dev/ion", O_RDONLY);
	if (*ionfd < 0) {
		perror("couldn't open /dev/ion");
		return *ionfd;
	}

	rc = ioctl(*ionfd, ION_IOC_ALLOC, alloc_data);
	if (rc) {
		perror("couldn't do ion alloc");
		close(*ionfd);
	}

	return rc;
}

static int basic_ion_sanity_test(struct ion_allocation_data alloc_data,
				unsigned long size_mb)
{
	uint8_t *buf;
	int ionfd, rc;
	unsigned long i, squelched = 0;
	bool integrity_good = true;
	struct ion_fd_data fd_data;
	struct ion_custom_data custom_data;
	struct ion_flush_data flush_data;

	rc = alloc_me_up_some_ion(&ionfd, &alloc_data);
	if (rc)
		goto out;

	fd_data.handle = alloc_data.handle;

	rc = ioctl(ionfd, ION_IOC_MAP, &fd_data);
	if (rc) {
		perror("couldn't do ION_IOC_MAP");
		goto err1;
	}

	buf = mmap(NULL, size_mb, PROT_READ | PROT_WRITE, MAP_SHARED,
		fd_data.fd, 0);

	if (buf == MAP_FAILED) {
		perror("couldn't do mmap");
		rc = (int) MAP_FAILED;
		goto err2;
	}

	memset(buf, 0xA5, size_mb);
	flush_data.handle = alloc_data.handle;
	flush_data.vaddr = buf;
	flush_data.length = size_mb;
	custom_data.cmd = ION_IOC_CLEAN_INV_CACHES;
	custom_data.arg = (unsigned long) &flush_data;
	if (ioctl(ionfd, ION_IOC_CUSTOM, &custom_data)) {
		perror("Couldn't flush caches");
		rc = 1;
		goto err3;
	} else {
		puts("flushed caches");
	}

	for (i = 0; i < size_mb; ++i) {
		if (buf[i] != 0xA5) {
			if (!integrity_good) {
				squelched++;
				continue;
			}
			printf("  Data integrity error at "
				"offset 0x%x from 0x%p!!!!\n", i, buf);
			integrity_good = false;
		}
	}
	if (squelched)
		printf("  (squelched %d additional integrity error%s)\n",
			squelched, squelched == 1 ? "" : "s");

	if (integrity_good)
		printf("  Buffer integrity check succeeded\n");

err3:
	if (munmap(buf, size_mb)) {
		rc = 1;
		perror("couldn't do munmap");
	}

err2:
	close(fd_data.fd);
err1:
	rc |= ioctl(ionfd, ION_IOC_FREE, &alloc_data.handle);
err0:
	close(ionfd);
out:
	return rc;
}

static void basic_sanity_tests(unsigned long size_mb)
{
	int lrc, rc = 0;

	struct ion_allocation_data iommu_alloc_data = {
		.align	   = SZ_1M,
		.len	   = SZ_1M,
		.heap_mask = ION_HEAP(ION_IOMMU_HEAP_ID),
		.flags	   = 0,
	};

	struct ion_allocation_data system_alloc_data = {
		.align	   = SZ_1M,
		.len	   = SZ_1M,
		.heap_mask = ION_HEAP(ION_SYSTEM_HEAP_ID),
		.flags	   = 0,
	};

	struct ion_allocation_data system_contig_alloc_data = {
		.align	   = SZ_1M,
		.len	   = SZ_1M,
		.heap_mask = ION_HEAP(ION_SYSTEM_CONTIG_HEAP_ID),
		.flags	   = 0,
	};

	puts("testing IOMMU without caching...");
	lrc = basic_ion_sanity_test(iommu_alloc_data, size_mb);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing IOMMU with caching...");
	iommu_alloc_data.flags |= ION_FLAG_CACHED;
	lrc = basic_ion_sanity_test(iommu_alloc_data, size_mb);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system without caching (should fail)...");
	lrc = !basic_ion_sanity_test(system_alloc_data, size_mb);
	puts(lrc ? "FAILED! (failed to fail)" : "PASSED (successfully failed)");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system with caching...");
	system_alloc_data.flags |= ION_FLAG_CACHED;
	lrc = basic_ion_sanity_test(system_alloc_data, size_mb);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system contig without caching (should fail)...");
	lrc = !basic_ion_sanity_test(system_contig_alloc_data, size_mb);
	puts(lrc ? "FAILED! (failed to fail)" : "PASSED (successfully failed)");
	hr();
	sleepy();
	rc |= lrc;

	puts("testing system contig with caching...");
	system_contig_alloc_data.flags |= ION_FLAG_CACHED;
	lrc = basic_ion_sanity_test(system_contig_alloc_data, size_mb);
	puts(lrc ? "FAILED!" : "PASSED");
	hr();
	sleepy();
	rc |= lrc;

	if (rc)
		puts("BASIC SANITY TESTS FAILED!!!!!!!! WOOOWWWW!!!!!!");
	else
		puts("All basic sanity tests passed");

}

static int do_map_extra_test(void)
{
	int rc = 0;
	int ionfd;
	struct memory_prof_map_extra_args args;
	size_t buffer_length = SZ_1M;
	struct ion_allocation_data alloc_data = {
		.align	   = SZ_1M,
		.len	   = buffer_length,
		.heap_mask = ION_HEAP(ION_IOMMU_HEAP_ID),
		.flags	   = 0,
	};
	struct ion_fd_data fd_data;

	rc = alloc_me_up_some_ion(&ionfd, &alloc_data);
	if (rc)
		goto out;

	fd_data.handle = alloc_data.handle;

	rc = ioctl(ionfd, ION_IOC_SHARE, &fd_data);
	if (rc) {
		perror("Couldn't do ION_IOC_SHARE");
		goto err0;
	}

	args.ionfd = fd_data.fd;
	args.iommu_map_len = buffer_length * 2;

	print_n_slow("Doing MEMORY_PROF_IOC_TEST_MAP_EXTRA");
	rc = ioctl(memory_prof_fd, MEMORY_PROF_IOC_TEST_MAP_EXTRA, &args);
	if (rc) {
		perror("couldn't do MEMORY_PROF_IOC_TEST_MAP_EXTRA");
		goto err0;
	}

err0:
	rc |= ioctl(ionfd, ION_IOC_FREE, &alloc_data.handle);
	close(ionfd);
out:
	return rc;
}


#define US_TO_MS(us) (us / 1000)
#define MS_TO_S(ms) (ms / 1000)
#define S_TO_MS(s) (s * 1000)
#define MS_TO_US(ms) (ms * 1000)
#define S_TO_US(s) (s * 1000 * 1000)

static struct timeval timeval_sub(struct timeval t1, struct timeval t2)
{
	struct timeval diff;

	diff.tv_sec = t1.tv_sec - t2.tv_sec;

	if (t1.tv_usec < t2.tv_usec) {
		diff.tv_usec = t1.tv_usec + S_TO_US(1) - t2.tv_usec;
		diff.tv_sec--;
	} else {
		diff.tv_usec = t1.tv_usec - t2.tv_usec;
	}

	return diff;
}

/**
 * Returns the total time taken for ION_IOC_ALLOC
 */
static long profile_alloc_for_heap(const char *name, const char *size_string,
				unsigned int heap_mask,
				unsigned int flags, unsigned int size,
				bool quiet)
{
	long ms = -1;
	int ionfd, rc, rc_ioctl;
	struct timeval tv_before, tv_after, tv_result;
	struct ion_allocation_data alloc_data = {
		.align	   = SZ_4K,
		.len	   = size,
		.heap_mask = heap_mask,
		.flags	   = flags,
	};
	ionfd = open("/dev/ion", O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open /dev/ion");
		goto out;
	}

	rc = gettimeofday(&tv_before, NULL);
	if (rc) {
		perror("couldn't get time of day");
		goto close_ion;
	}

	rc_ioctl = ioctl(ionfd, ION_IOC_ALLOC, &alloc_data);
	rc = gettimeofday(&tv_after, NULL);
	if (rc) {
		perror("couldn't get time of day");
		goto close_ion;
	}
	if (rc_ioctl) {
		if (!quiet)
			perror("couldn't do ion alloc");
		rc = rc_ioctl;
		goto close_ion;
	}

	tv_result = timeval_sub(tv_after, tv_before);

	ms = US_TO_MS(tv_result.tv_usec) + S_TO_MS(tv_result.tv_sec);

free:
	ioctl(ionfd, ION_IOC_FREE, &alloc_data.handle);
close_ion:
	close(ionfd);
out:
	return ms;
}

static void map_extra_test(void)
{
	int rc = 0;
	puts("testing msm_iommu_map_extra...");
	rc = do_map_extra_test();
	if (rc) puts("FAILED!");
	hr();
}

static void profile_kernel_alloc(void)
{
	int rc;
	rc = ioctl(memory_prof_fd, MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS);
	if (rc)
		perror("couldn't do MEMORY_PROF_IOC_TEST_KERNEL_ALLOCS");
}

static void print_stats_results(const char *name, const char *cached,
				const char *size_string,
				long stats[])
{
	int i;
	float sum = 0, sum_of_squares = 0, average, std_dev;

	for (i = 0; i < NUM_REPS_FOR_HEAP_PROFILING; ++i) {
		sum += stats[i];
	}
	average = sum / NUM_REPS_FOR_HEAP_PROFILING;

	for (i = 0; i < NUM_REPS_FOR_HEAP_PROFILING; ++i) {
		sum_of_squares += pow(stats[i] - average, 2);
	}
	std_dev = sqrt(sum_of_squares / NUM_REPS_FOR_HEAP_PROFILING);

	printf(" > %s %s %s average: %.2f std_dev: %.2f\n",
		name, cached, size_string, average, std_dev);
}

static void heap_profiling(void)
{
	int i;
	const int nreps = NUM_REPS_FOR_HEAP_PROFILING;
	struct sizes_struct {
		unsigned long size;
		const char *sMB;
		bool quiet;
	};
	struct sizes_struct *szp;
	struct sizes_struct sizes[] = {
		{
			.size = SZ_1M * 1,
			.sMB = "1MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 3,
			.sMB = "3MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 5,
			.sMB = "5MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 8,
			.sMB = "8MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 10,
			.sMB = "10MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 13,
			.sMB = "13MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 20,
			.sMB = "20MB",
			.quiet = false,
		}, {
			.size = SZ_1M * 50,
			.sMB = "50MB",
			.quiet = true,
		}, {
			.size = SZ_1M * 100,
			.sMB = "100MB",
			.quiet = true,
		}, {
			/* SENTINEL */
			.size = 0,
		}
	};

	puts("All times are in milliseconds unless otherwise indicated");
	/*
	 * Don't change the format of the following line string. We
	 * need to rely on it for parsing.
	 */
	printf("===>>> num reps: %d\n", nreps);

	for (szp = &sizes[0]; szp->size; ++szp) {
		unsigned int sz = szp->size;
		const char *sMB = szp->sMB;
		bool quiet = szp->quiet;
		long stats[NUM_REPS_FOR_HEAP_PROFILING];
		long *statsp;
		printf("\n============PROFILING FOR %s MB=============\n",
			sMB);
		for (i = 0, statsp = &stats[0]; i < nreps; ++i) {
			*statsp++ = profile_alloc_for_heap(
				"ION_CP_MM_HEAP_ID uncached",
				sMB,
				ION_HEAP(ION_CP_MM_HEAP_ID),
				ION_SECURE, sz, quiet);
		}
		print_stats_results("ION_CP_MM_HEAP_ID", "uncached", sMB,
				stats);

		for (i = 0, statsp = &stats[0]; i < nreps; ++i) {
			*statsp++ = profile_alloc_for_heap(
				"ION_IOMMU_HEAP_ID uncached",
				sMB,
				ION_HEAP(ION_IOMMU_HEAP_ID),
				0, sz, quiet);
		}
		print_stats_results("ION_IOMMU_HEAP_ID", "uncached", sMB,
				stats);

		for (i = 0, statsp = &stats[0]; i < nreps; ++i) {
			*statsp++ = profile_alloc_for_heap(
				"ION_IOMMU_HEAP_ID cached",
				sMB,
				ION_HEAP(ION_IOMMU_HEAP_ID),
				ION_FLAG_CACHED, sz, quiet);
		}
		print_stats_results("ION_IOMMU_HEAP_ID", "cached", sMB, stats);

		for (i = 0, statsp = &stats[0]; i < nreps; ++i) {
			*statsp++ = profile_alloc_for_heap(
				"ION_SYSTEM_HEAP_ID cached",
				sMB,
				ION_HEAP(ION_SYSTEM_HEAP_ID),
				ION_FLAG_CACHED, sz, quiet);
		}
		print_stats_results("ION_SYSTEM_HEAP_ID", "cached", sMB, stats);

		for (i = 0, statsp = &stats[0]; i < nreps; ++i) {
			*statsp++ = profile_alloc_for_heap(
				"ION_SYSTEM_HEAP_ID uncached",
				sMB,
				ION_HEAP(ION_SYSTEM_HEAP_ID),
				0, sz, quiet);
		}
		print_stats_results("ION_SYSTEM_HEAP_ID", "uncached", sMB, stats);

	}
}

static void oom_test(void)
{
	int rc, ionfd, cnt = 0;
	struct ion_allocation_data alloc_data = {
		.len	   = SZ_8M,
		.heap_mask = ION_HEAP(ION_IOMMU_HEAP_ID),
		.flags	   = 0,
	};

	LIST_HEAD(handle_list, ion_handle_node) head;
	struct handle_list *headp;
	struct ion_handle_node {
		struct ion_handle *handle;
		LIST_ENTRY(ion_handle_node) nodes;
	} *np;
	LIST_INIT(&head);

	ionfd = open("/dev/ion", O_RDONLY);
	if (ionfd < 0) {
		perror("couldn't open /dev/ion");
		return;
	}


	for (;; cnt++) {
		rc = ioctl(ionfd, ION_IOC_ALLOC, &alloc_data);
		if (rc) {
			/* game over! */
			break;
		} else {
			np = malloc(sizeof(struct ion_handle_node));
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
}

static void leak_test(void)
{
	int ionfd;
	struct ion_fd_data fd_data;
	struct ion_allocation_data alloc_data = {
		.len	   = SZ_4K,
		.heap_mask = ION_HEAP(ION_SYSTEM_HEAP_ID),
		.flags	   = 0,
	};

	puts("About to leak a handle...");
	fflush(stdout);

	alloc_me_up_some_ion(&ionfd, &alloc_data);

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
}

#define USAGE_STRING							\
	"Usage: %s [options]\n"						\
	"\n"								\
	"Supported options:\n"						\
	"\n"								\
	"  -h         Print this message and exit\n"			\
	"  -a         Do the adversarial test (same as -l)\n"		\
	"  -b         Do basic sanity tests\n"				\
	"  -z         Size (in MB) of buffer for basic sanity tests (default=1)\n" \
	"  -e         Do Ion heap profiling\n"				\
	"  -k         Do kernel alloc profiling\n"			\
	"  -l         Do leak test (leak an ion handle)\n"		\
	"  -m         Do map extra test\n"				\
	"  -n         Do the nominal test (same as -b)\n"		\
	"  -o         Do OOM test (alloc from Ion Iommu heap until OOM)\n" \
	"  -p MS      Sleep for MS milliseconds between stuff (for debugging)\n" \
	"  -r         Do the repeatability test\n"			\
	"  -s         Do the stress test (same as -e)\n"

static void usage(char *progname)
{
	printf(USAGE_STRING, progname);
}

int main(int argc, char *argv[])
{
	int rc = 0, i, opt;
	unsigned long basic_sanity_size_mb = SZ_1M;
	bool do_basic_sanity_tests = false;
	bool do_heap_profiling = false;
	bool do_kernel_alloc_profiling = false;
	bool do_map_extra_test = false;
	bool do_oom_test = false;
	bool do_leak_test = false;
	int num_reps = 1;

	while (-1 != (opt = getopt(argc, argv, "abehklmnop:rsz:"))) {
		switch (opt) {
		case 'n':
		case 'b':
			do_basic_sanity_tests = true;
			break;
		case 's':
		case 'e':
			do_heap_profiling = true;
			break;
		case 'k':
			do_kernel_alloc_profiling = true;
			break;
		case 'a':
		case 'l':
			do_leak_test = true;
			break;
		case 'm':
			do_map_extra_test = true;
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
		case 'z':
			basic_sanity_size_mb = atoi(optarg) * SZ_1M;
			break;
		case 'h':
		default:
			usage(basename(argv[0]));
			exit(1);
		}
	}

	memory_prof_fd = open("/dev/memory_prof", 0);
	if (memory_prof_fd < 0) {
		perror("couldn't open /dev/memory_prof");
		return memory_prof_fd;
	}

	rc = ioctl(memory_prof_fd, MEMORY_PROF_IOC_CLIENT_CREATE);
	if (rc) {
		perror("couldn't do MEMORY_PROF_IOC_CLIENT_CREATE");
		goto err0;
	}

	if (do_basic_sanity_tests)
		for (i = 0; i < num_reps; ++i)
			basic_sanity_tests(basic_sanity_size_mb);
	if (do_map_extra_test)
		for (i = 0; i < num_reps; ++i)
			map_extra_test();
	if (do_heap_profiling)
		for (i = 0; i < num_reps; ++i)
			heap_profiling();
	if (do_kernel_alloc_profiling)
		for (i = 0; i < num_reps; ++i)
			profile_kernel_alloc();
	if (do_oom_test)
		for (i = 0; i < num_reps; ++i)
			oom_test();
	if (do_leak_test)
		for (i = 0; i < num_reps; ++i)
			leak_test();

	rc = ioctl(memory_prof_fd, MEMORY_PROF_IOC_CLIENT_DESTROY);
	if (rc)
		perror("couldn't do MEMORY_PROF_IOC_CLIENT_DESTROY");

err0:
	close(memory_prof_fd);
	return rc;
}
