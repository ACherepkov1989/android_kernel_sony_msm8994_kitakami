/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _GLINK_TEST_COMMON_H_
#define _GLINK_TEST_COMMON_H_
#define SEQ_PRINTFV seq_printf

#include <linux/debugfs.h>
#include "glink_private.h"

extern int glink_loopback_xprt_init(void);
extern void glink_loopback_xprt_exit(void);
extern void glink_loopback_xprt_link_up(void);

#define GLINK_STATUS(seq_filep, x...) do { \
	SEQ_PRINTFV(seq_filep, x); \
	if (!(QCOM_GLINK_PERF & glink_get_debug_mask())) \
		GLINK_IPC_LOG_STR("<UT> " x); \
} while (0)

#define GLINK_STATUS_MT(seq_filep, x...) do { \
	mutex_lock(&multithread_test_mutex_lha0); \
	SEQ_PRINTFV(seq_filep, x); \
	if (!(QCOM_GLINK_PERF & glink_get_debug_mask())) \
		GLINK_IPC_LOG_STR("<UT> " x); \
	mutex_unlock(&multithread_test_mutex_lha0); \
} while (0)

/**
 * Unit test assertion for logging test cases.
 *
 * @a lval
 * @b rval
 * @cmp comparison operator
 *
 * Assertion fails if (@a cmp @b) is not true which then
 * logs the function and line number where the error occurred
 * along with the values of @a and @b.
 *
 * Variations with MT at the end, such as UT_ASSERT_INT_MT, are
 * to be used in a multithreaded environment, as the sequential
 * output file is not thread-safe.
 *
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 * @multithread_test_mutex_lha0 - mutex to lock seq file pointer
 */
#define UT_ASSERT_INT(a, cmp, b) \
	{ \
	int a_tmp = (a); \
	int b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, "%s:%d Fail: " #a "(%d) " #cmp " " #b "(%d)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		GLINK_ERR("%s:%d Fail: " #a "(%d) " #cmp " " #b "(%d)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_INT_MT(a, cmp, b) \
	{ \
	int a_tmp = (a); \
	int b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%d Fail: " #a "(%d) " #cmp " ", \
				__func__, __LINE__, a_tmp); \
		seq_printf(s, #b "(%d) thread: %d\n", \
				b_tmp, current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%d %s: " #a "(%d) " #cmp " " #b "(%d) %s: %d\n", \
				__func__, __LINE__, "Fail", \
				a_tmp, b_tmp, "thread", current->pid); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_UINT(a, cmp, b) \
	{ \
	unsigned a_tmp = (a); \
	unsigned b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, "%s:%u Fail: " #a "(%u) " #cmp " " #b "(%u)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		GLINK_ERR("%s:%u Fail: " #a "(%u) " #cmp " " #b "(%u)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_UINT_MT(a, cmp, b) \
	{ \
	unsigned a_tmp = (a); \
	unsigned b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%u Fail: " #a "(%u) " #cmp " ", \
				__func__, __LINE__, a_tmp); \
		seq_printf(s, #b "(%u) thread: %d\n", \
				b_tmp, current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%u %s: " #a "(%u) " #cmp " " #b "(%u) %s: %d\n", \
				__func__, __LINE__, "Fail", \
				a_tmp, b_tmp, "thread", current->pid); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_BOOL(a, cmp, b) \
	{ \
	bool a_tmp = (a); \
	bool b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, "%s:%u Fail: " #a "(%u) " #cmp " " #b "(%u)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		GLINK_ERR("%s:%u Fail: " #a "(%u) " #cmp " " #b "(%u)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_BOOL_MT(a, cmp, b) \
	{ \
	bool a_tmp = (a); \
	bool b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%u Fail: " #a "(%u) " #cmp " ", \
				__func__, __LINE__, a_tmp); \
		seq_printf(s, #b "(%u) thread: %d\n", \
				b_tmp, current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%u %s: " #a "(%u) " #cmp " " #b "(%u) %s: %d\n", \
				__func__, __LINE__, "Fail",\
				a_tmp, b_tmp, "thread", current->pid); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_PTR(a, cmp, b) \
	{ \
	void *a_tmp = (a); \
	void *b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, "%s:%d Fail: " #a "(%p) " #cmp " " #b "(%p)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		GLINK_ERR("%s:%d Fail: " #a "(%p) " #cmp " " #b "(%p)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_PTR_MT(a, cmp, b) \
	{ \
	void *a_tmp = (a); \
	void *b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%d Fail: " #a "(%p) " #cmp " ", \
				__func__, __LINE__, a_tmp); \
		seq_printf(s, #b "(%p) thread: %d\n", \
				b_tmp, current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%d %s: " #a "(%p) " #cmp " " #b "(%p) %s: %d\n", \
				__func__, __LINE__, "Fail", \
				a_tmp, b_tmp, "thread", current->pid); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_ERR_PTR(a) \
	{ \
	void *a_tmp = (a); \
	if (IS_ERR_OR_NULL(a_tmp)) { \
		seq_printf(s, "%s:%d Fail: " #a "(%ld)\n", \
				__func__, __LINE__, PTR_ERR(a_tmp)); \
		GLINK_ERR("%s:%d Fail: " #a "(%ld)\n", \
				__func__, __LINE__, PTR_ERR(a_tmp)); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_ERR_PTR_MT(a) \
	{ \
	void *a_tmp = (a); \
	if (IS_ERR_OR_NULL(a_tmp)) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%d Fail: " #a "(%ld) thread: %d\n", \
				__func__, __LINE__, PTR_ERR(a_tmp), \
				current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%d Fail: " #a "(%ld) thread: %d\n", \
				__func__, __LINE__, \
				PTR_ERR(a_tmp), current->pid); \
		failed = 1; \
		break; \
	} \
	}

/**
 * String compare unit test assertion for test cases.
 *
 * @str1 One string
 * @str2 Another string
 * @count: The maximum number of bytes to compare
 *
 * Assertion fails if @count bytes of @str1 is not equal to @str2.
 * In the failure case, the macro logs the funtion and line number
 * where the error occurred along with the values of @str1, @str2
 * and @count.
 *
 * UT_ASSERT_STRING_COMPARE_MT is to be used in a multi-
 * threaded environment, as the sequential output file is
 * not thread-safe.
 *
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 * @multithread_test_mutex_lha0 - mutex to lock seq file pointer
 */
#define UT_ASSERT_STRING_COMPARE(str1, str2) \
	{ \
	if (strcmp(str1, str2)) { \
		seq_printf(s, "%s:%d Fail: String compare[%s][%s]\n", \
				__func__, __LINE__, str1, str2); \
		GLINK_ERR("%s:%d Fail: String compare[%s][%s]\n", \
				__func__, __LINE__, str1, str2); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_STRING_COMPARE_MT(str1, str2) \
	{ \
	if (strcmp(str1, str2)) { \
		mutex_lock(&multithread_test_mutex_lha0); \
		seq_printf(s, "%s:%d Fail: String compare[%s][%s] %s: %d\n", \
				__func__, __LINE__, str1, str2, "thread", \
				current->pid); \
		mutex_unlock(&multithread_test_mutex_lha0); \
		GLINK_ERR("%s:%d Fail: String compare[%s][%s] thread: %d\n", \
				__func__, __LINE__, str1, str2, current->pid); \
		failed = 1; \
		break; \
	} \
	}

#endif
