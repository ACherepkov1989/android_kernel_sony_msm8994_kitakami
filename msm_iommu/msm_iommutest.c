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

/* MSM IOMMU tester. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/types.h>
#include <ctype.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include "iommutest.h"

#define ERR	0
#define INFO	1
int debug_level = ERR;
#define debug(level, x...)	do { (level > debug_level) ? 0 : printf(x); } \
				while (0)

/* Device path names */
#define MSM_IOMMU_TEST	"/dev/msm_iommu_test"

#define NOMINAL_TEST		0
#define ADV_TEST		1
#define STRESS_TEST		2
#define REPEAT_TEST		3

struct test_results {
	unsigned int no_tests;
	unsigned int no_skipped;
	unsigned int no_failed;
};

unsigned int TEST_TYPE = NOMINAL_TEST;
unsigned int no_repeats = 1;
unsigned int force = 0;
static unsigned int do_basic_va2pa_test;
static unsigned int cats_data_read = 0;
static char *tbu_id_cb_name;
static char iommu_to_test[NAME_LEN];
static unsigned int *mm_tbu;

#define NUM_TBUS		20
#define TBU_NAME_LENGTH		50

#define PRINT_FORMAT "%15s (%20s)"

int parse_args(int argc, char **argv)
{
	unsigned int level;

	if (argc != 7)
		return 1;

	switch (argv[1][0]) {
	case 'n':
	{
		TEST_TYPE = NOMINAL_TEST;
		break;
	}
	case 'a':
	{
		TEST_TYPE = ADV_TEST;
		break;
	}
	case 's':
	{
		TEST_TYPE = STRESS_TEST;
		break;
	}
	case 'r':
	{
		TEST_TYPE = REPEAT_TEST;
		no_repeats = atoi(argv[2]);
		break;
	}
	default:
		return -EINVAL;
	}

	level = atoi(argv[3]);

	if (level != INFO && level != ERR)
		return -EINVAL;
	debug_level = level;

	force = atoi(argv[4]);
	do_basic_va2pa_test = atoi(argv[5]);

	STRNCPY_SAFE(iommu_to_test, argv[6], NAME_LEN);
	return 0;
}

int read_bfb_settings(struct test_iommu *tst_iommu,
		      struct target_struct *target,
		      struct get_next_cb *gnc)
{
	char file_name[101];
	FILE *fp;
	int ret = 0;
	char i_name[30];
	char line[101];
	unsigned int reg;
	unsigned int value;

	if (gnc->lpae_enabled)
		snprintf(file_name, 100, "%s_lpae_bfb.txt", target->name);
	else
		snprintf(file_name, 100, "%s_bfb.txt", target->name);
	fp = fopen(file_name, "r");

	if (fp == NULL) {
		ret = -EINVAL;
		goto out;
	}
	tst_iommu->bfb_size = 0;
	while (fgets(line, 100, fp) != NULL) {
		sscanf(line, "%s %x %x", i_name, &reg, &value);
		if (strcmp(i_name, gnc->iommu_name) == 0) {
			if (tst_iommu->bfb_size < MAX_BFB_REGS) {
				unsigned int sz = tst_iommu->bfb_size;
				tst_iommu->bfb_regs[sz] = reg;
				tst_iommu->bfb_data[sz] = value;
				++tst_iommu->bfb_size;
			} else {
				debug(ERR, "Too many BFB settings\n");
				ret = -ENOMEM;
				break;
			}
		}
	}
close_fd:
	fclose(fp);
out:
	return ret;
}

int do_bfb_test(int iommu_test_fd, struct get_next_cb *gnc,
		struct target_struct *target, int *skipped)
{
	int ret = 0;
	struct test_iommu tst_iommu;
	*skipped = 0;

	if (gnc->iommu_secure && !force) {
		debug(INFO, PRINT_FORMAT ": Testing BFB: SKIPPED! (Secure)\n",
		      gnc->iommu_name, gnc->cb_name);
		*skipped = 1;
		return 0;
	}

	tst_iommu.iommu_no = gnc->iommu_no;
	tst_iommu.cb_no = gnc->cb_no;
	tst_iommu.flags = 0;

	ret = read_bfb_settings(&tst_iommu, target, gnc);
	if (ret) {
		if (ret == -ENOMEM) {
			debug(INFO, PRINT_FORMAT ": Testing BFB: FAILED! (Unit test needs update)\n",
			     gnc->iommu_name, gnc->cb_name);
		} else {
			debug(INFO, PRINT_FORMAT ": Testing BFB: SKIPPED! (Setting Missing)\n",
			     gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		}
		goto out;
	}

	ret = ioctl(iommu_test_fd, IOC_IOMMU_TEST_IOMMU_BFB, &tst_iommu);

	if (ret) {
		if (tst_iommu.ret_code == -EBUSY) {
			debug(INFO, PRINT_FORMAT ": Testing BFB: SKIPPED! (CTX Busy)\n",
			      gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		} else if (ret) {
			debug(INFO, PRINT_FORMAT ": Testing BFB: FAILED! (%d)\n",
			     gnc->iommu_name, gnc->cb_name, tst_iommu.ret_code);
		}
	} else {
		debug(INFO, PRINT_FORMAT ": Testing BFB: OK (%u regs)\n",
		      gnc->iommu_name, gnc->cb_name, tst_iommu.bfb_size);
	}

out:
	return ret;
}

int read_cats_tbu_id(struct target_struct *target)
{
	char file_name[100];
	FILE *fp;
	int ret = 0;
	char cb_name[50];
	char line[101];
	unsigned int tbu_id;
	unsigned int is_mm_tbu;

	snprintf(file_name, 100, "%s_cats.txt", target->name);
	fp = fopen(file_name, "r");

	if (fp == NULL) {
		ret = -EINVAL;
		goto out;
	}

	tbu_id_cb_name = malloc((sizeof(char)) * NUM_TBUS * TBU_NAME_LENGTH);
	if (!tbu_id_cb_name) {
		 ret = -ENOMEM;
		 goto out;
	}

	mm_tbu = malloc(sizeof(unsigned int) * NUM_TBUS);
	if (!mm_tbu) {
		ret = -ENOMEM;
		free(tbu_id_cb_name);
		goto out;
	}

	while (fgets(line, 100, fp) != NULL) {
		sscanf(line, "%x %x %s", &tbu_id, &is_mm_tbu, cb_name);

		if (tbu_id >= NUM_TBUS)
			continue;

		memcpy(&tbu_id_cb_name[tbu_id * TBU_NAME_LENGTH], cb_name,
				TBU_NAME_LENGTH);
		mm_tbu[tbu_id] = is_mm_tbu;
	}

	fclose(fp);
out:
	return ret;
}

int do_cats_test(int iommu_test_fd, struct get_next_cb *gnc, int *skipped,
		struct target_struct *target)
{
	int ret = 0;
	struct test_iommu tst_iommu;
	*skipped = 0;
	int i;

	if (!force) {
		debug(INFO, PRINT_FORMAT ": Testing CATS: SKIPPED! (Secure)\n",
		      gnc->iommu_name, gnc->cb_name);
		*skipped = 1;
		return 0;
	}

	if (!cats_data_read) {
		ret = read_cats_tbu_id(target);
		if (ret == -ENOMEM) {
			debug(INFO, PRINT_FORMAT ": Testing CATS: FAILED! (no memory)\n",
				gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			return 0;
		} else if (ret) {
			*skipped = 1;
			debug(INFO, PRINT_FORMAT ": Testing CATS: SKIPPED! (Setting Missing)\n",
				gnc->iommu_name, gnc->cb_name);
			return 0;
		}
		cats_data_read++;
	}

	for (i = 0; i < NUM_TBUS; i++) {
		if (!strcmp(&tbu_id_cb_name[i * TBU_NAME_LENGTH], gnc->cb_name)) {
			tst_iommu.cats_tbu_id = i;
			tst_iommu.is_mm_tbu = mm_tbu[i];
			break;
		} else
			tst_iommu.cats_tbu_id = -1;
	}

	tst_iommu.iommu_no = gnc->iommu_no;
	tst_iommu.cb_no = gnc->cb_no;
	tst_iommu.flags = 0;

	ret = ioctl(iommu_test_fd, IOC_IOMMU_TEST_CATS, &tst_iommu);

	if (ret) {
		if (tst_iommu.ret_code == -EBUSY) {
			debug(INFO, PRINT_FORMAT ": Testing CATS: SKIPPED! (CTX Busy)\n",
			      gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		} else if (tst_iommu.ret_code == -EINVAL) {
			debug(INFO, PRINT_FORMAT ": Testing CATS: SKIPPED! (!TBU_ID)\n",
			      gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		} else if (ret) {
			debug(INFO, PRINT_FORMAT ": Testing CATS: FAILED! (%d)\n",
			     gnc->iommu_name, gnc->cb_name, tst_iommu.ret_code);
		}
	} else {
		debug(INFO, PRINT_FORMAT ": Testing CATS: OK\n",
		      gnc->iommu_name, gnc->cb_name);
	}
	return ret;
}

int do_int_test(int iommu_test_fd, struct get_next_cb *gnc, int *skipped)
{
	int ret = 0;
	struct test_iommu tst_iommu;
	*skipped = 0;

	if (gnc->iommu_secure && !force) {
		debug(INFO, PRINT_FORMAT ": Testing Interrupt: SKIPPED! (Secure)\n",
		      gnc->iommu_name, gnc->cb_name);
		*skipped = 1;
		return 0;
	}

	tst_iommu.iommu_no = gnc->iommu_no;
	tst_iommu.cb_no = gnc->cb_no;
	tst_iommu.flags = 0;

	ret = ioctl(iommu_test_fd, IOC_IOMMU_TEST_IOMMU_INT, &tst_iommu);

	if (ret) {
		if (tst_iommu.ret_code == -EBUSY) {
			debug(INFO, PRINT_FORMAT ": Testing Interrupt: SKIPPED! (CTX Busy)\n",
			      gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		} else if (ret) {
			debug(INFO, PRINT_FORMAT ": Testing Interrupt: FAILED! (%d)\n",
			     gnc->iommu_name, gnc->cb_name, tst_iommu.ret_code);
		}
	} else {
		debug(INFO, PRINT_FORMAT ": Testing Interrupt: OK\n",
		      gnc->iommu_name, gnc->cb_name);
	}
	return ret;
}

int do_va2pa_test(int iommu_test_fd, struct get_next_cb *gnc, int *skipped)
{
	int ret = 0;
	struct test_iommu tst_iommu;
	*skipped = 0;

	tst_iommu.iommu_no = gnc->iommu_no;
	tst_iommu.cb_no = gnc->cb_no;
	tst_iommu.flags = do_basic_va2pa_test ? TEST_FLAG_BASIC : 0;

	if (gnc->iommu_secure && !force) {
		debug(INFO, PRINT_FORMAT ": Testing VA2PA: SKIPPED! (Secure)\n",
		      gnc->iommu_name, gnc->cb_name);
		*skipped = 1;
		return 0;
	}

	ret = ioctl(iommu_test_fd, IOC_IOMMU_TEST_IOMMU_VA2PA, &tst_iommu);

	if (ret) {
		if (tst_iommu.ret_code == -EBUSY) {
			debug(INFO, PRINT_FORMAT ": Testing VA2PA: SKIPPED! (CTX Busy)\n",
			      gnc->iommu_name, gnc->cb_name);
			*skipped = 1;
			ret = 0;
		} else if (ret) {
			debug(INFO, PRINT_FORMAT ": Testing VA2PA: FAILED! (%d)\n",
			     gnc->iommu_name, gnc->cb_name, tst_iommu.ret_code);
		}
	} else {
		debug(INFO, PRINT_FORMAT ": Testing VA2PA: OK\n",
		      gnc->iommu_name, gnc->cb_name);
	}

	return ret;
}

struct test_results run_nominal_tests(void)
{
	int ret = 0;
	int iommu_test_fd;
	unsigned int iommu_idx = 0;
	unsigned int cb_idx = 0;
	struct get_next_cb gnc;
	int skipped;
	struct test_results result = { 0, 0, 0 };
	struct target_struct target;
	unsigned int current_iommu = -1;

	iommu_test_fd = open(MSM_IOMMU_TEST, O_RDWR);

	if (iommu_test_fd < 0) {
		debug(ERR, "Failed to open msm iommu test device\n");
		perror("msm iommu");
		result.no_failed = 1;
		return result;
	}

	ret = ioctl(iommu_test_fd, IOC_IOMMU_GET_TARGET, &target);
	if (ret) {
		debug(ERR, "Failed to get target: %d\n", ret);
		result.no_failed = 1;
		return result;
	}

	gnc.iommu_no = iommu_idx;
	gnc.cb_no = cb_idx;
	gnc.cb_secure = 0;
	gnc.valid_cb = 0;
	gnc.valid_iommu = 0;
	gnc.lpae_enabled = 0;

	ret = ioctl(iommu_test_fd, IOC_IOMMU_GET_NXT_IOMMU_CB, &gnc);
	if (ret) {
		debug(ERR, "Failed to get IOMMU CTX: %d\n", ret);
		goto out;
	}
	debug(INFO, "Running on target: %s (%s)\n", target.name,
		gnc.lpae_enabled ? "LPAE" : "VMSA");

	while (gnc.valid_iommu) {
		if (strcmp(iommu_to_test, "all") == 0 ||
		    strcmp(iommu_to_test, gnc.iommu_name) == 0) {
			if (current_iommu != gnc.iommu_no) {
				ret = do_bfb_test(iommu_test_fd, &gnc, &target,
						  &skipped);
				if (ret)
					++result.no_failed;

				result.no_skipped += skipped;
				++result.no_tests;

				current_iommu = gnc.iommu_no;
			}
			if (gnc.valid_cb) {
				ret = do_int_test(iommu_test_fd, &gnc,
						  &skipped);

				if (ret)
					++result.no_failed;

				result.no_skipped += skipped;
				++result.no_tests;

				ret = do_va2pa_test(iommu_test_fd, &gnc,
						    &skipped);
				if (ret)
					++result.no_failed;

				result.no_skipped += skipped;
				++result.no_tests;

				ret = do_cats_test(iommu_test_fd, &gnc,
						   &skipped, &target);
				if (ret)
					++result.no_failed;

				result.no_skipped += skipped;
				++result.no_tests;

				++cb_idx;
			} else {
				++iommu_idx;
				cb_idx = 0;
			}
		} else {
			++iommu_idx;
			cb_idx = 0;
		}

		gnc.iommu_no = iommu_idx;
		gnc.cb_no = cb_idx;
		gnc.cb_secure = 0;
		gnc.valid_cb = 0;
		gnc.valid_iommu = 0;

		ret = ioctl(iommu_test_fd, IOC_IOMMU_GET_NXT_IOMMU_CB, &gnc);
		if (ret) {
			debug(ERR, "Failed to get IOMMU CTX: %d\n", ret);
			break;
		}
	}
out:
	if (tbu_id_cb_name)
		free(tbu_id_cb_name);
	if (mm_tbu)
		free(mm_tbu);
	close(iommu_test_fd);
	return result;
}

struct test_results run_adv_tests(void)
{
	return run_nominal_tests();
}

struct test_results  run_stress_tests(void)
{
	return run_nominal_tests();
}

struct test_results  run_repeat_tests(void)
{
	struct test_results result = { 0, 0, 0 };
	struct test_results final_res = { 0, 0, 0 };
	unsigned int i;

	for (i = 0; i < no_repeats; ++i) {
		result = run_nominal_tests();
		final_res.no_failed += result.no_failed;
		final_res.no_tests += result.no_tests;
		final_res.no_skipped += result.no_skipped;
	}
	return final_res;
}

int main(int argc, char **argv)
{
	unsigned int i = 0, type;
	int ret = 0;
	struct test_results results;

	if (parse_args(argc, argv)) {
		debug(ERR, "incorrect arguments passed\n");
		return -EINVAL;
	}
	/* Run tests */
	if (TEST_TYPE == NOMINAL_TEST) {
		debug(INFO, "\n\nRunning Nominal IOMMU tests :\n\n");
		results = run_nominal_tests();
		printf("Ran %u tests, %u skipped, %u failed\n",
		       results.no_tests, results.no_skipped, results.no_failed);
		if (results.no_failed) {
			debug(INFO, "Nominal tests failed\n");
			return -1;
		}
	}
	if (TEST_TYPE == ADV_TEST) {
		debug(INFO, "\n\nRunning Adversarial IOMMU tests :\n\n");
		results = run_adv_tests();
		printf("Ran %u tests, %u skipped, %u failed\n",
		       results.no_tests, results.no_skipped, results.no_failed);
		if (results.no_failed) {
			debug(INFO, "Adversarial tests failed\n");
			return ret;
		}
	}
	if (TEST_TYPE == STRESS_TEST) {
		debug(INFO, "\n\nRunning Stress IOMMU tests :\n\n");
		results = run_stress_tests();
		printf("Ran %u tests, %u skipped, %u failed\n",
		       results.no_tests, results.no_skipped, results.no_failed);
		if (results.no_failed) {
			debug(INFO, "Stress tests failed\n");
			return ret;
		}
	}
	if (TEST_TYPE == REPEAT_TEST) {
		debug(INFO, "\n\nRunning Repeat IOMMU tests :\n\n");
		results = run_repeat_tests();
		printf("Ran %u tests, %u skipped, %u failed\n",
		       results.no_tests, results.no_skipped, results.no_failed);
		if (results.no_failed) {
			debug(INFO, "Repeat tests failed\n");
			return ret;
		}
	}
	return ret;
}
