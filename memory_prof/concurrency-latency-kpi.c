/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#include <signal.h>
#include <stdlib.h>
#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <float.h>
#include <sys/select.h>
#include <string.h>
#include <err.h>
#include "memory_prof_util.h"
#include "alloc_profiles.h"

enum test_types {
	GEOMETRIC,
	ARITHMETIC,
	CONSTANT,
	MAX_TEST_TYPES
};

static int verbose;

#define debug(x...)			\
	do {				\
                if (verbose)      	\
                        printf(x); 	\
        } while (0)

#define progress()			\
	do {				\
		if (!verbose) {		\
			printf("\b=>");	\
			fflush(stdout);	\
		}			\
	} while (0)

#define TEST_GEOMETRIC_START_PAGES 1
#define TEST_ARITHMETIC_START_PAGES 256
#define TEST_ARITHMETIC_COMMON_DIFF 2560

#define DEFAULT_FILE_DIR "/data/"
#define MAX_FILE_NAME_SIZE 200
#define FILE_PAGES (256 * 30)

struct conc_op {
	int type;
	int adj;
	int writers;
	int debug;
	int repeat;
	int pages;
	int delay;
	int count;
};

enum conc_op_line_idx {
	LINE_IDX_TYPE = 1,
	LINE_IDX_ADJ,
	LINE_IDX_WRITERS,
	LINE_IDX_DEBUG,
	LINE_IDX_REPEAT,
	LINE_IDX_PAGES,
	LINE_IDX_DELAY,
	LINE_IDX_COUNT,
	LINE_IDX_MAX,
};

static char *file_dir = DEFAULT_FILE_DIR;

enum {
	ANON_BG,
	ANON_CACHED,
	ANON_FILE_WRITE_BG,
	ANON_FILE_WRITE_CACHED,
};

static int ckilled;
static int child_get_mem;
static int last_pid;
static int n_children;
static int vmstat_depth;
static char **vmstat_strings;

struct meminfo {
	long swap_avg;
	long swap_max;
};

static int stop_stat;

static void vmstat_init(void)
{
	FILE *fp;
	ssize_t read;
	char *line = NULL;
	char *tok;
	size_t tok_len;
	size_t len = 0;
	int i = 0;

	fp = fopen("/proc/vmstat", "r");

	if (!fp)
		err(1, "opening vmstat failed");

	while ((read = getline(&line, &len, fp)) != -1)
		vmstat_depth++;

	MALLOC(char **, vmstat_strings, sizeof(char *) * vmstat_depth);

	rewind(fp);

	while ((read = getline(&line, &len, fp)) != -1) {
		tok = strtok(line, " ");
		tok_len = strlen(tok);
		MALLOC(char *, vmstat_strings[i], tok_len);
		strncpy(vmstat_strings[i], tok, tok_len);
		i++;
	}

	free(line);
	fclose(fp);
}

static void free_vmstat(void)
{
	int i;

	for (i = 0; i < vmstat_depth; i++)
		free(vmstat_strings[i]);

	free(vmstat_strings);
	vmstat_strings = NULL;
}

static long *get_vmstat(void)
{
	FILE *fp;
	ssize_t read;
	char *line = NULL;
	size_t len;
	long *v;
	char *tok;
	int i = 0;

	MALLOC(long *, v, sizeof(long) * vmstat_depth);

	fp = fopen("/proc/vmstat", "r");

	if (!fp)
		err(1, "opening vmstat failed");

	while ((read = getline(&line, &len, fp)) != -1) {
		strtok(line, " ");
		tok = strtok(NULL, " ");
		v[i++] = strtol(tok, NULL, 0);
	}

	free(line);
	fclose(fp);

	return v;
}

static void vmstat_diff(long *v1, long *v2)
{
	int i;

	printf("vmstat diff:");

	for (i = 0; i < vmstat_depth; i++)
		printf("%s: %ld\n", vmstat_strings[i], v2[i] - v1[i]);

	printf("\n\n");
}

static void set_int_path(const char *path, int integer)
{
	char *val;
	int fd;

	ASPRINTF(&val, "%d", integer);

	OPEN(path, O_WRONLY, fd);

	if (write(fd, val, strlen(val)) < 0)
		err(1, "Couldn't write to %s", path);

	close(fd);
	free(val);
}

static void drop_cache(void)
{
	sync();
	set_int_path("/proc/sys/vm/drop_caches", 3);
}

static void *alloc_pages(int ps, int n)
{
	void *p;

	p = mmap(NULL, ps * n, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (p == MAP_FAILED) {
		printf("alloc_page failed to allocate %d pages\n", ps * n);
		return NULL;
	}

	memset(p, ~0, ps * n);

	return p;
}

static int alloc_page(int ps)
{
	void *p;

	p = mmap(NULL, ps, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	if (p == MAP_FAILED) {
		printf("alloc_page failed to allocate a page (%d)\n", ps);
		return -ENOMEM;
	}

	memset(p, ~0, ps);

	return 0;
}

static void child_killed(int snum)
{
	int pages;
	pid_t pid;

	/* Get the memory details from last task */
	debug("Sending message to %d\n", last_pid);

	if (last_pid)
		kill(last_pid, SIGUSR1);

	pid = wait(NULL);

	n_children--;

	debug("Received signal %s, pid:%d, nc:%d\n",
			strsignal(snum), pid, n_children);
	ckilled = 1;
}

static void get_mem_child(int snum)
{
	debug("%s, %d, %s\n", __func__, getpid(), strsignal(snum));
	child_get_mem = 1;
}

static void exit_child(int snum)
{
	debug("%d received %s\n", getpid(), strsignal(snum));
	exit(EXIT_SUCCESS);
}

pid_t write_file_forever(int writers)
{
	char file[MAX_FILE_NAME_SIZE];
	int fd;
	int ps = getpagesize();
	size_t oz = 512;
	void *buf = malloc(oz);
	size_t sz = FILE_PAGES * ps;
	pid_t pid;
	int i;
	struct sigaction act;

	for (i = 0; i < writers; i++) {

		pid = fork();

		if (pid > 0) {
			if (i == (writers - 1))
				return pid;
			else
				continue;
		} else if (pid < 0) {
			err(1, "failed forking %s", __func__);
		} else {
			break;
		}
	}

	act.sa_handler = exit_child;
	/* Otherwise we may fail to kill the task on D state */
	act.sa_flags = 0;
	if (0 < sigaction(SIGUSR2, &act, NULL))
		err(1, "%s: child sigaction failed for SIGUSR2", __func__);

	if (!buf)
		err(1, "malloc failed %s", __func__);

	memset(buf, 0xff, oz);

	if (0 > sprintf(file, "%s/S-%d.kpi", file_dir, getpid()))
		err(1, "sprintf failed");

restart:
	fd = open(file, O_CREAT|O_RDWR, S_IRWXU | S_IRWXG | S_IRWXO);

	if (0 > fd)
		err(1, "open failed: %s", file);

	debug("file %s opened by %d\n", file, getpid());

	while (1) {
		if (0 > write(fd, buf, oz))
			break;
		sleep(0);
	}

	close(fd);
	unlink(file);

	goto restart;
}

static void stop_proc(int snum)
{
	debug("%s, %d, %s\n", __func__, getpid(), strsignal(snum));

	if (snum == SIGUSR1)
		stop_stat += 1;
	else if (snum == SIGUSR2)
		stop_stat += 2;
}

pid_t stat_proc(void)
{
	pid_t pid;
	struct sigaction act1, act2;
	long swp_tot = 0, swp_free = 0;
	struct meminfo memi;
	FILE *fp;
	ssize_t read;
	char *line = NULL;
	size_t len;

	pid = fork();

	if (pid > 0)
		return pid;
	else if (pid < 0)
		err(1, "failed forking %s", __func__);

	act1.sa_handler = stop_proc;
	act1.sa_flags = 0;
	if (0 < sigaction(SIGUSR1, &act1, NULL))
		err(1, "child sigaction failed for SIGUSR1");

	act2.sa_handler = stop_proc;
	/* Otherwise we may fail to kill the task on D state */
	act2.sa_flags = 0;
	if (0 < sigaction(SIGUSR2, &act2, NULL))
		err(1, "child sigaction failed for SIGUSR2");

	memi.swap_max = memi.swap_avg = 0;

	while (1) {
		if (stop_stat == 1) {
			sleep(0);
			continue;
		} else if (stop_stat >= 2) {
			printf("[meminfo]{Max swap usage (kB): %ld,"
				" Avg swap usage (kB): %ld}\n\n",
				memi.swap_max, memi.swap_avg);
			exit(EXIT_SUCCESS);
		}

		fp = fopen("/proc/meminfo", "r");
		if (!fp)
			err(1, "opening /proc/meminfo failed");

		while ((read = getline(&line, &len, fp)) != -1) {
			if (startswith(line, "SwapTotal:"))
				sscanf(line, "SwapTotal: %ld kB\n", &swp_tot);
			else if (startswith(line, "SwapFree:"))
				sscanf(line, "SwapFree: %ld kB\n", &swp_free);
		}

		if (memi.swap_max < (swp_tot - swp_free))
			memi.swap_max = (swp_tot - swp_free);

		if (!memi.swap_avg)
			memi.swap_avg = swp_tot - swp_free;
		else
			memi.swap_avg = (memi.swap_avg +
				(swp_tot - swp_free)) / 2;

		fclose(fp);

		sleep(0);
	}

	free(line);
}

static void child_func(int pages, int fd, int is_lat)
{
	int i;
	FILE* stream;
	struct sigaction act1, act2;
	int ps = getpagesize();
	int ret;
	struct timeval tv;
	double time;

	/*
	 * To receive information on any other child being
	 * killed, so that the current number of pages allocated
	 * can be passed to parent for KPI calculation.
	 */
	act1.sa_handler = get_mem_child;
	act1.sa_flags = 0;
	if (0 < sigaction(SIGUSR1, &act1, NULL))
		err(1, "child sigaction failed for SIGUSR1");

	act2.sa_handler = exit_child;
	/* Otherwise we may fail to kill the task on D state */
	act2.sa_flags = 0;
	if (0 < sigaction(SIGUSR2, &act2, NULL))
		err(1, "child sigaction failed for SIGUSR2");

	if (is_lat)
		mprof_tick(&tv);

	for (i = 0; i < pages; i++) {
		if (0 > alloc_page(ps)) {
			debug("%s: alloc failed, sending %d\n", __func__, i);
			break;
		} else if (child_get_mem) {
			child_get_mem = 0;
			debug("pid %d sending alloc details on req from"
				" parent -> %d\n", getpid(), i);
			break;
		}
	}

	if (is_lat)
		time = mprof_tock(&tv);

	debug("%d Allocated %d pages\n", getpid(), i);

	ret = write(fd, &i, sizeof(i));
	if (0 > ret)
		err(1, "failed to write to parent");

	debug("%d provided page dteails\n", getpid());
	if (is_lat) {
		ret = write(fd, &time, sizeof(time));
		if (0 > ret)
			err(1, "failed to write to parent");
	}
	debug("%d RIP\n", getpid());

	while (1)
		sleep(1);
}

static void report_conc_kpi_test(int num_children,
		int failed_alloc_pages, int type, int adj, int w)
{
	long total_pages;
	int ps = getpagesize();

	if (type == GEOMETRIC) {
		int actual_geometric_sum;
		/* Sum of geometric series */
		actual_geometric_sum =
			(TEST_GEOMETRIC_START_PAGES *
				(1 - (1 << num_children))) / -1;
		/* Sum excluding the last process */
		total_pages =
			(TEST_GEOMETRIC_START_PAGES *
				(1 - (1 << (num_children - 1)))) / -1;

		if (!failed_alloc_pages)
			total_pages = actual_geometric_sum;

		printf("\n----------------------------------\n");
		printf("Geometric Concurrency test: @adj %d + %d writers\n",
					adj, w);
		printf("----------------------------------\n");
		printf("Concurrency KPI: %ld\n",
			((total_pages + failed_alloc_pages) * ps) /
					(1024 * 1024));
		printf("Concurrency in pages: %ld\n",
			total_pages + failed_alloc_pages);
		printf("Actual geometric sum: %d\n",
				actual_geometric_sum);
		printf("Number of forks: %d\n", num_children);
		printf("----------------------------------\n");
	} else if (type == ARITHMETIC) {
		int actual_arithmetic_sum;
		/* Sum of arithmetic series */
		actual_arithmetic_sum =
			(num_children * ((2 * TEST_ARITHMETIC_START_PAGES)
			+ ((num_children - 1) *
			TEST_ARITHMETIC_COMMON_DIFF))) / 2;

		/* Sum excluding the last process */
		total_pages =
			((num_children - 1) *
			((2 * TEST_ARITHMETIC_START_PAGES) +
			((num_children - 2) *
			TEST_ARITHMETIC_COMMON_DIFF))) / 2;

		if (!failed_alloc_pages)
			total_pages = actual_arithmetic_sum;

		printf("\n----------------------------------\n");
		printf("Arithmetic Concurrency test: @adj %d + %d writers\n",
					adj, w);
		printf("----------------------------------\n");
		printf("Concurrency KPI: %ld\n",
			((total_pages + failed_alloc_pages) * ps) /
					(1024 * 1024));
		printf("Concurrency in pages: %ld\n",
			total_pages + failed_alloc_pages);
		printf("Actual arithmetic sum: %d\n",
				actual_arithmetic_sum);
		printf("Number of forks: %d\n", num_children);
		printf("----------------------------------\n");
	} else {
		err(1, "%s: wrong type", __func__);
	}
}

static void report_lat_kpi_test(int num_children,
		int failed_alloc_pages, int type,
		double avg_time, int adj, int w)
{
	long total_pages;
	int ps = getpagesize();

	if (type == GEOMETRIC) {
		int actual_geometric_sum;

		actual_geometric_sum =
			(TEST_GEOMETRIC_START_PAGES *
				(1 - (1 << num_children))) / -1;
		/* Sum excluding the last process */
		total_pages =
			(TEST_GEOMETRIC_START_PAGES *
				(1 - (1 << (num_children - 1)))) / -1;

		if (!failed_alloc_pages)
			total_pages = actual_geometric_sum;

		printf("\n----------------------------------\n");
		printf("Geometric Latency test: @adj %d + %d writers\n",
					adj, w);
		printf("----------------------------------\n");
		printf("Latency KPI: %f (ms for 1MB)\n", avg_time);
		printf("Total pages allocated: %ld\n",
			total_pages + failed_alloc_pages);
		printf("Actual geometric sum (pages): %d\n",
				actual_geometric_sum);
		printf("Number of forks: %d\n", num_children);
		printf("----------------------------------\n");
	} else if (type == ARITHMETIC) {
		int actual_arithmetic_sum;
		/* Sum of arithmetic series */
		actual_arithmetic_sum =
			(num_children * ((2 * TEST_ARITHMETIC_START_PAGES)
			+ ((num_children - 1) *
			TEST_ARITHMETIC_COMMON_DIFF))) / 2;

		/* Sum excluding the last process */
		total_pages =
			((num_children - 1) *
			((2 * TEST_ARITHMETIC_START_PAGES) +
			((num_children - 2) *
			TEST_ARITHMETIC_COMMON_DIFF))) / 2;

		if (!failed_alloc_pages)
			total_pages = actual_arithmetic_sum;

		printf("\n----------------------------------\n");
		printf("Arithmetic Latency test: @ adj %d + %d writers\n",
					adj, w);
		printf("----------------------------------\n");
		printf("Latency KPI: %f (ms for 1MB)\n", avg_time);
		printf("Total pages allocated: %ld\n",
			total_pages + failed_alloc_pages);
		printf("Actual arithmetic sum (pages): %d\n",
				actual_arithmetic_sum);
		printf("Number of forks: %d\n", num_children);
		printf("----------------------------------\n");
	} else {
		err(1, "%s: wrong type", __func__);
	}
}

static int get_page_count(int num_children, int type)
{
	if (type == GEOMETRIC)
		/* nth term of geometric sequence */
		return TEST_GEOMETRIC_START_PAGES << (num_children - 1);
	else if (type == ARITHMETIC)
		/* nth term of arithmetic sequence */
		return TEST_ARITHMETIC_START_PAGES +
			((num_children - 1) * TEST_ARITHMETIC_COMMON_DIFF);
	else
		err(1, "%s: wrong type\n", __func__);
}

void cleanup()
{
	char *buf;
	signal(SIGCHLD, SIG_IGN);
	signal(SIGUSR2, SIG_IGN);

	debug("%s starts", __func__);
	/* This will also let stat_proc dump meminfo */
	/* A sigkill will kill the parent also, so let children exit normally */
	kill(0, SIGUSR2);

	waitpid(-1, NULL, __WALL);

	signal(SIGCHLD, SIG_DFL);
	signal(SIGUSR2, SIG_DFL);

	/* We do not track file names, so better do this than unlink */
	ASPRINTF(&buf, "rm -rf %s/*.kpi", file_dir);
	system(buf);
	free(buf);

	drop_cache();

	debug("%s ends", __func__);
}

/*
 * Concurrency test:
 * In user terms, concurrency is the ability to sustain multiple apps in memory.
 * This is usually measured for e.g. like number of browser tabs that can be
 * sustained in background while we play a memory hungry game. Running a memory
 * hungry game translates to, running a memory hungry application in foreground.
 * So to simulate this what conc_test does is to fork tasks sequentially, each
 * forked task allocating a certain amount of memory. Forking is done till a
 * task is killed. So concurrency index is calculated as the total memory that
 * could be allocated by all tasks till the first task was killed. The amount
 * of memory each task allocates is calculated using a mathematical progression.
 * conc_test uses geometric and arithmetic progressions. Geometric progression
 * simulates higher rates of allocation, while arithmetic progression simulates
 * gradually increasing rate of allocation. These tests are performed multiple
 * times with varying enviroments. for e.g. The newly forked tasks allocates
 * memory while being in foreground (adj 0), and are then moved to background.
 * So there are 2 variants where the tasks are either moved to background or
 * cached apps adj levels. This is done to simulate situations to test features
 * like adaptive LMK. Also, some of the tests are performed with a parallel
 * background write. This is to include the effetcs of page cache on concurency.
 *
 * There are 2 recommened ways of running the test.
 * 1) Standalone test where the concurrency test is executed in a non-android
 *    environment (with adb shell stop, or kdev).
 * 2) Perform the concurrency tets with a stress test like monkey in parallel.
 *    This is improtant to verify the effects of features like adaptive LMK,
 *    where simulating the enviroment that creates the scenario is difficult.
 */
static void test_conc(struct conc_op *op)
{
	pid_t pid = 0;
	struct sigaction act;
	int ret;
	int num_children = 0;
	int pages;
	int fds[2];
	int failed_alloc_pages;
	char buf[MAX_FILE_NAME_SIZE];
	pid_t w_pid = 0, stat_pid = 0;
	long *v1, *v2;

	/* The current process simulates a foreground task */
	set_oom_score_adj_self(0);
	/*
	 * Register handler to get notified on child exit.
	 * In the handler we request the currently running
	 * child (N) to provide the number of pages allocated
	 * till that point. This is added to the geometric
	 * sum of (N-1) tasks to calculate the concurrency.
	 */
	act.sa_handler = child_killed;
	act.sa_flags = SA_RESTART;
	if (0 < sigaction(SIGCHLD, &act, NULL))
		err(1, "parent sigaction failed");

	/*
	 * Pipe to synchronize the allocations by each child
	 * and to get the max pages allocated from last child
	 * as described above.
	 */
	if (0 > pipe(fds))
		err(1, "pipe open failed");

	v1 = get_vmstat();

	while (1) {
		if (ckilled) {
			debug("Late kill of child\n");
			failed_alloc_pages = 0;
			goto out;
		} else {
			num_children++;
			pages = get_page_count(num_children, op->type);
		}

		/* Moving task to background is done very late to
		 * avoid the task being killed while performing
		 * allocation.
		 */
		if (pid)
			set_oom_score_adj_pid(pid, op->adj);

		pid = fork();
		if (!pid) {
			debug("child %d (pid:%d) allocating %d pages\n",
					num_children, getpid(), pages);
			child_func(pages, fds[1], 0);
		} else if (pid > 0) {
			if (op->writers && !w_pid)
				w_pid = write_file_forever(op->writers);

			if (!stat_pid)
				stat_pid = stat_proc();

			progress();

			debug("forked %d\n", pid);

			/* See child_killed */
			last_pid = pid;

			/* Read the allocation status from child */
			ret = read(fds[0], &failed_alloc_pages,
				sizeof(failed_alloc_pages));

			if (ret >= 0) {
				if (failed_alloc_pages < pages) {
					debug("Parent detects failed allocation"
						" or info from child: %d, %d\n",
						failed_alloc_pages, pages);
					goto out;
				} else {
					debug("Parent detects proper allocation"
						":  %d, %d\n",
						failed_alloc_pages, pages);
				}
			} else {
				err(1, "pipe read error");
			}
			sched_yield();
		} else {
			err(1, "fork failed");
		}
	}
out:
	/* stop meminfo. collect data later in cleanup */
	kill(stat_pid, SIGUSR1);

	v2 = get_vmstat();

	/* Trigger concurrency report */
	report_conc_kpi_test(num_children,
			failed_alloc_pages, op->type, op->adj, op->writers);

	vmstat_diff(v1, v2);
	free(v1);
	free(v2);
	ckilled = 0;
	cleanup();
}

/*
 * Latency tests:
 * Latency in user terms is defined as the time taken to launch an application.
 * As far a memory is concerned this roughly translates to the time taken to
 * allocate memory by the newly launched application. lat_test measures the
 * latency by forking tasks till only one task remains. The time taken to
 * allocate memory is measured and the latency index is calculated as the time
 * taken to allocate 1MB of memory.
 *
 * The differents variants of latency test and the recommended ways of running
 * the tests are similar to that of conc_test. Plese read the comments for
 * conc_test.
 */
static void test_lat(struct conc_op *op)
{
	pid_t pid = 0;
	struct sigaction act;
	int ret;
	int num_children = 0;
	int pages;
	int fds[2];
	int failed_alloc_pages;
	double time;
	double avg_time = 0;
	int ps = getpagesize();
	pid_t w_pid = 0, stat_pid = 0;
	long *v1, *v2;

	/* The current process simulate a foreground task */
	set_oom_score_adj_self(0);

	act.sa_handler = child_killed;
	act.sa_flags = SA_RESTART;
	if (0 < sigaction(SIGCHLD, &act, NULL))
		err(1, "parent sigaction failed");

	v1 = get_vmstat();
	/*
	 * Pipe to synchronize the allocations by each child
	 * and to get alloc failure message.
	 */
	if (0 > pipe(fds))
		err(1, "pipe open failed");

	while (1) {
		num_children++;
		n_children++;
		pages = get_page_count(num_children, op->type);

		/* Moving task to background is done very late to
		 * avoid the task being killed while performing
		 * allocation.
		 */
		if (pid)
			set_oom_score_adj_pid(pid, op->adj);

		pid = fork();
		if (!pid) {
			debug("child %d (pid:%d) allocating %d pages\n",
					num_children, getpid(), pages);
			child_func(pages, fds[1], 1);
		} else if (pid > 0) {
			fd_set set;
			struct timeval timeout;

			if (op->writers && !w_pid)
				w_pid = write_file_forever(op->writers);

			if (!stat_pid)
				stat_pid = stat_proc();

			progress();

			FD_ZERO(&set);
			FD_SET(fds[0], &set);

			timeout.tv_sec = 15;
			timeout.tv_usec = 0;

			debug("forked %d\n", pid);

			/*
			 * The task forked now can be killed during the
			 * allocation. Then we will wait forever in the
			 * read. So set a timeout.
			 */
			signal(SIGCHLD, SIG_IGN);
			ret = select(fds[0] + 1, &set, NULL, NULL, &timeout);
			if (ret < 0) {
				err(1, "select failed");
			} else if (!ret) {
				debug("select timeout, last child killed: %d\n",
					n_children);
				failed_alloc_pages = 0;
				signal(SIGCHLD, SIG_DFL);
				kill(pid, SIGUSR1);
				sleep(2);
				kill(pid, SIGUSR2);
				goto out;
			}
			signal(SIGCHLD, SIG_DFL);

			/* Read the allocation status from child */
			ret = read(fds[0], &failed_alloc_pages,
				sizeof(failed_alloc_pages));

			if (ret >= 0) {
				double curr_lat = 0;

				/* Read the allocation time */
				ret = read(fds[0], &time,
					sizeof(time));

				/*
				 * Latency is calculated as time taken to
				 * allocate 1MB of memory.
				 */
				curr_lat = ((time * ((1024 * 1024) / ps)) /
						failed_alloc_pages);
				avg_time = (avg_time + curr_lat) / 2;

				if (failed_alloc_pages < pages) {
					debug("Parent detects failed allocation"
						" or last child exit: %d, %d,"
						"%f (ms), %f (ms), %f (ms)\n",
						failed_alloc_pages, pages,
						time, curr_lat, avg_time);
					goto out;
				} else {
					debug("Parent detects proper allocation"
						":  %d, %d, %f (ms), %f (ms),"
						"%f (ms)\n",
						failed_alloc_pages, pages, time,
						curr_lat, avg_time);
					failed_alloc_pages = 0;
				}
			} else {
				err(1, "pipe read error");
			}
			sched_yield();
		} else {
			err(1, "fork failed");
		}
	}
out:
	kill(stat_pid, SIGUSR1);
	v2 = get_vmstat();

	/* Trigger concurrency report */
	report_lat_kpi_test(num_children,
		failed_alloc_pages,
		op->type, avg_time, op->adj, op->writers);

	vmstat_diff(v1, v2);
	free(v1);
	free(v2);
	cleanup();
}

static void test_lat_constant(struct conc_op *op)
{
	struct timeval tv;
	double time;
	double min = DBL_MAX, max = 0, avg = 0;
	void *p;
	int ps = getpagesize();
	long *v1, *v2;

	while (op->count--) {
		v1 = get_vmstat();
		mprof_tick(&tv);
		if ((p = alloc_pages(ps, op->pages)) == NULL) {
			debug("alloc failed for %d pages", op->pages);
			continue;
		}

		time = mprof_tock(&tv);
		v2 = get_vmstat();
		munmap(p, ps * op->pages);

		time = (time * ((1024 * 1024)/ ps)) / op->pages;

		if (time < min)
			min = time;

		if (time > max)
			max = time;

		if (!avg)
			avg = time;
		else
			avg = (avg + time) / 2;

		debug("%f time for 1M alloc\n", time);
		vmstat_diff(v1, v2);
		free(v1);
		free(v2);
		sleep(op->delay);
	}

	printf("Latency test:(%d pages,%ds delay) min: %f, max: %f, avg:%f\n\n",
			op->pages, op->delay, min, max, avg);
}

/*
 * Stability tests:
 * This is a set of tests run in a loop, which can be run standalone or
 * along with a parallel monkey or stress test to figure out any kind
 * of memory stability issues.
 */
void stability_test(struct conc_op *op)
{
	while (1) {
		test_lat(op);
		sched_yield();
	}
}

static int op_conc_global_setup(
	struct alloc_profile_entry entries[] __unused)
{
	if (!vmstat_strings)
		vmstat_init();
	drop_cache();
	return 0;
}

static void op_conc_global_teardown(void)
{
	if (vmstat_strings)
		free_vmstat();
}

static int op_conc_parse(struct alloc_profile_entry *entry,
				struct line_info *li)
{
	struct conc_op *op = (struct conc_op *) entry->priv;

	STRTOL(op->type, li->words[LINE_IDX_TYPE], 0);
	STRTOL(op->adj, li->words[LINE_IDX_ADJ], 0);
	STRTOL(op->writers, li->words[LINE_IDX_WRITERS], 0);
	STRTOL(op->debug, li->words[LINE_IDX_DEBUG], 0);
	STRTOL(op->repeat, li->words[LINE_IDX_REPEAT], 0);
	STRTOL(op->pages, li->words[LINE_IDX_PAGES], 0);
	STRTOL(op->delay, li->words[LINE_IDX_DELAY], 0);
	STRTOL(op->count, li->words[LINE_IDX_COUNT], 0);

	return 0;
}

static int op_conc_run(struct alloc_profile_entry *entry)
{
	struct conc_op *op = (struct conc_op *) entry->priv;

	verbose = op->debug;
	debug("Starting %s with [%d, %d, %d, %d]\n", __func__,
			op->type, op->adj, op->writers, op->debug);
	op->repeat++;
	while (op->repeat--)
		test_conc(op);
	verbose = 0;

	return 0;
}

static int op_lat_run(struct alloc_profile_entry *entry)
{
	struct conc_op *op = (struct conc_op *) entry->priv;

	verbose = op->debug;
	if (op->type == CONSTANT) {
		debug("Starting %s with [%d, %d, %d, %d, %d]\n", __func__,
			op->debug, op->type, op->pages, op->delay, op->count);
		test_lat_constant(op);
	} else {
		debug("Starting %s with [%d, %d, %d, %d]\n", __func__,
			op->type, op->adj, op->writers, op->debug);
		op->repeat++;
		while (op->repeat--)
			test_lat(op);
	}
	verbose = 0;

	return 0;
}

static int op_stability_run(struct alloc_profile_entry *entry)
{
	struct conc_op *op = (struct conc_op *) entry->priv;

	verbose = op->debug;
	debug("Starting %s with [%d, %d, %d, %d]\n", __func__,
			op->type, op->adj, op->writers, op->debug);
	stability_test(op);
	verbose = 0;

	return 0;
}

static struct alloc_profile_ops conc_ops = {
	.parse = op_conc_parse,
	.run = op_conc_run,
	.global_setup = op_conc_global_setup,
	.global_teardown = op_conc_global_teardown,
};

static struct alloc_profile_ops lat_ops = {
	.parse = op_conc_parse,
	.run = op_lat_run,
	.global_setup = op_conc_global_setup,
	.global_teardown = op_conc_global_teardown,
};

static struct alloc_profile_ops stability_ops = {
	.parse = op_conc_parse,
	.run = op_stability_run,
	.global_setup = op_conc_global_setup,
	.global_teardown = op_conc_global_teardown,
};

ALLOC_PROFILE_OP_SIZED(&conc_ops, conc, sizeof(struct conc_op));
ALLOC_PROFILE_OP_SIZED(&lat_ops, lat, sizeof(struct conc_op));
ALLOC_PROFILE_OP_SIZED(&stability_ops, stability, sizeof(struct conc_op));
