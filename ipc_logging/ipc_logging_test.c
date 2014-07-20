/* ipc_logging/ipc_logging_test.c
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/seq_file.h>

#include <linux/ipc_logging.h>
#include "../../../kernel/trace/ipc_logging_private.h"

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
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 */
#define UT_ASSERT_INT(a, cmp, b) \
	{ \
	int a_tmp = (a); \
	int b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, \
			"%s:%d Fail: " #a "(%d) " #cmp " " #b "(%d)\n", \
				__func__, __LINE__, \
				a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_PTR(a, cmp, b) \
	{ \
	void *a_tmp = (a); \
	void *b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, \
			"%s:%d Fail: " #a "(%p) " #cmp " " #b "(%p)\n", \
				__func__, __LINE__, \
				a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_UINT(a, cmp, b) \
	{ \
	unsigned a_tmp = (a); \
	unsigned b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, \
			"%s:%d Fail: " #a "(%u) " #cmp " " #b "(%u)\n", \
				__func__, __LINE__, \
				a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_HEX(a, cmp, b) \
	{ \
	unsigned a_tmp = (a); \
	unsigned b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		seq_printf(s, \
			"%s:%d Fail: " #a "(%x) " #cmp " " #b "(%x)\n", \
				__func__, __LINE__, \
				a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

/**
 * In-range unit test assertion for test cases.
 *
 * @a lval
 * @minv Minimum value
 * @maxv Maximum value
 *
 * Assertion fails if @a is not on the exclusive range minv, maxv
 * ((@a < @minv) or (@a > @maxv)).  In the failure case, the macro
 * logs the function and line number where the error occurred along
 * with the values of @a and @minv, @maxv.
 *
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 */
#define UT_ASSERT_INT_IN_RANGE(a, minv, maxv) \
	{ \
	int a_tmp = (a); \
	int minv_tmp = (minv); \
	int maxv_tmp = (maxv); \
	if (((a_tmp) < (minv_tmp)) || ((a_tmp) > (maxv_tmp))) { \
		seq_printf(s, \
			"%s:%d Fail: " #a "(%d) < " #minv "(%d) or " \
				 #a "(%d) > " #maxv "(%d)\n", \
				__func__, __LINE__, \
				a_tmp, minv_tmp, a_tmp, maxv_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define MAX_MSG_DECODED_SIZE (MAX_MSG_SIZE*4)

#define TIME_SEC_OFFSET 1
#define TIME_SEC_FIELD_SIZE 7

#define TIME_NSEC_OFFSET 8
#define TIME_NSEC_FIELD_SIZE 10

#define STRING_OFFSET 19

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
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 */
#define UT_ASSERT_STRING_COMPARE(str1, str2) \
	{ \
	if (strcmp(str1, str2)) { \
		seq_printf(s, "%s:%d Fail:String compare[%s][%s]\n", \
				__func__, __LINE__, str1, str2); \
		failed = 1; \
		break; \
	} \
	}

/**
 * String to unsigned long unit test assertion for test cases.
 *
 * @str Input string
 * @base Integer base
 * @val Output unsigned long
 *
 * Assertion fails if @str is not able to convert into unsigned long.
 * In the failure case, the macro logs the function and line number
 * where the error occurred along with the value of @str.
 *
 * Assumes that the following local variables exist:
 * @s - sequential output file pointer
 * @failed - set to true if test fails
 */
#define UT_ASSERT_STRING_TO_UL(str, base, val) \
	{ \
	if (kstrtoul(str, base, &val)) { \
		seq_printf(s, \
			"%s:%d Fail: Time converstion str[%s]\n", \
				__func__, __LINE__, str); \
		failed = 1; \
		break; \
	} \
	}

#define DECODE_TIME(in_buff, val) \
do { \
	char sec[10] = {"\0"}; \
	char nsec[10] = {"\0"}; \
	char *substr = NULL; \
	unsigned long t_sec = 0; \
	unsigned long t_nsec = 0; \
	scnprintf(sec, TIME_SEC_FIELD_SIZE, in_buff + TIME_SEC_OFFSET); \
	substr = strrchr(sec, ' '); \
	if (substr) \
		substr++; \
	else \
		substr = sec; \
	UT_ASSERT_STRING_TO_UL(substr, 10, t_sec);\
	scnprintf(nsec, TIME_NSEC_FIELD_SIZE, in_buff + TIME_NSEC_OFFSET); \
	UT_ASSERT_STRING_TO_UL(nsec, 10, t_nsec);\
	do_div(t_nsec, 1000000U); \
	val = (t_sec * 1000) + t_nsec; \
} while (0)


/**
 * ipc_logging_ut_basic - Basic sanity test using local loopback.
 *
 * @s: pointer to output file
 *
 * This test simulates create, read, write and destroy of ipc_logging_context
 * when no other drivers implemented IPC_LOGGING.
 */
static void ipc_logging_ut_basic(struct seq_file *s)
{
	static void *ipc_log_test_ctxt;
	int ipc_log_test_pages = 2;
	int failed = 0;
	char *test_data = {"hello world\n"};
	char read_data[50] = {"\0"};

	char log1[30] = {"\0"};
	unsigned long mtime1 = 0;
	char log2[30] = {"\0"};
	unsigned long mtime2 = 0;
	int read_size;

	seq_printf(s, "Running %s\n", __func__);
	do {
		ipc_log_test_ctxt = ipc_log_context_create(ipc_log_test_pages,
							"ipc_logging_test", 0);
		UT_ASSERT_PTR(ipc_log_test_ctxt, !=, NULL);

		ipc_log_string(ipc_log_test_ctxt, "%s", test_data);

		read_size = ipc_log_extract(ipc_log_test_ctxt, read_data,
							MAX_MSG_DECODED_SIZE);

		scnprintf(log1, sizeof(log1), read_data + STRING_OFFSET);
		DECODE_TIME(read_data, mtime1);
		UT_ASSERT_STRING_COMPARE(log1, test_data);

		msleep(50);

		ipc_log_string(ipc_log_test_ctxt, "%s", test_data);

		read_size = ipc_log_extract(ipc_log_test_ctxt, read_data,
							MAX_MSG_DECODED_SIZE);

		scnprintf(log2, sizeof(log2), read_data + STRING_OFFSET);
		DECODE_TIME(read_data, mtime2);
		UT_ASSERT_STRING_COMPARE(log2, test_data);

		UT_ASSERT_INT_IN_RANGE((mtime2 - mtime1), 50, 65);
		seq_printf(s, "\tOK\n");
	} while (0);

	if (failed) {
		pr_err("%s: Failed\n", __func__);
		seq_printf(s, "\tFailed\n");
	}

	ipc_log_context_destroy(ipc_log_test_ctxt);
}

/**
 * ipc_logging_ut_wrap_test - Verify log wrapping.
 *
 * @s: pointer to output file
 *
 * This tests fills a log multiple times to verify that expected combinations
 * of wrapping are handled correctly.  Specifically, this test case confirms:
 *   * splitting messages across page boundaries works as expected
 *   * all combinations of read/write indexes are working as expected
 */
static void ipc_logging_ut_wrap_test(struct seq_file *s)
{
	static void *ctx;
	int failed = 0;
	char *test_data = {"hello world\n"};
	char read_data[50] = {"\0"};

	char log1[30] = {"\0"};
	unsigned long mtime1 = 0;
	char log2[30] = {"\0"};
	unsigned long mtime2 = 0;
	int read_size;

	seq_printf(s, "Running %s\n", __func__);
	do {
		ctx = ipc_log_context_create(3, "ipc_logging_test", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		ipc_log_string(ctx, "%s", test_data);

		read_size = ipc_log_extract(ctx, read_data,
							MAX_MSG_DECODED_SIZE);

		scnprintf(log1, sizeof(log1), read_data + STRING_OFFSET);
		DECODE_TIME(read_data, mtime1);
		UT_ASSERT_STRING_COMPARE(log1, test_data);

		msleep(50);

		ipc_log_string(ctx, "%s", test_data);

		read_size = ipc_log_extract(ctx, read_data,
							MAX_MSG_DECODED_SIZE);

		scnprintf(log2, sizeof(log2), read_data + STRING_OFFSET);
		DECODE_TIME(read_data, mtime2);
		UT_ASSERT_STRING_COMPARE(log2, test_data);

		UT_ASSERT_INT_IN_RANGE((mtime2 - mtime1), 50, 65);
		seq_printf(s, "\tOK\n");
	} while (0);

	if (failed) {
		pr_err("%s: Failed\n", __func__);
		seq_printf(s, "\tFailed\n");
	}

	ipc_log_context_destroy(ctx);
}

/**
 * extraction_dump - Dump unit test logs used to verify log extraction tool.
 *
 * @s: pointer to output file
 *
 * Generates several different logs that are used as part of the log extraction
 * tool unit test. Multipage tests with full pages will wrap around, so
 * the logs they create shouldn't start with "line 0". Tests that create
 * partially full pages will not wrap around, so the logs they create
 * should start with "line 0".
 *
 * Note:  This tool will crash the kernel to trigger the memory dump.
 */
static void extraction_dump(struct seq_file *s)
{
	int failed = 0;
	int n;
	void *ctx;
	char data[MAX_MSG_DECODED_SIZE];

	/*
	 * msgs_per_page - The number of TSV messages that can fit in one
	 * log page. 180 is an approximation used for this test case.
	 */
	const int msgs_per_page = 180;

	seq_printf(s, "Running %s\n", __func__);
	do {
		/* Create 1-page partially-full log */
		ctx = ipc_log_context_create(1, "ut_partial_1", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < msgs_per_page / 2; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		/* Create 1-page full log */
		ctx = ipc_log_context_create(1, "ut_full_1", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		/* Add 20 to msgs_per_page to ensure the log wraps */
		for (n = 0; n < msgs_per_page + 20; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		/* Create multi-page partially-full log */
		ctx = ipc_log_context_create(3, "ut_partial_3", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < (3 * msgs_per_page) / 2; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		/* Create multi-page full log */
		ctx = ipc_log_context_create(3, "ut_full_3", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < 3 * msgs_per_page; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		/* Create multi-page full log and empty it */
		ctx = ipc_log_context_create(2, "ut_read_extract_2", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < 2 * msgs_per_page; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		do {
			n = ipc_log_extract(ctx, data, sizeof(data));
			UT_ASSERT_INT(n, >=, 0)
		} while (n > 0);

		/*
		 * Create multi-page full log that wraps to the last page.
		 */
		ctx = ipc_log_context_create(3, "ut_full_3_overwrite", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < 8 * msgs_per_page; ++n)
			ipc_log_string(ctx, "line %d\n", n);

		/*
		 * Create multi-page full log that wraps to the last page,
		 * and then empty it
		 */
		ctx = ipc_log_context_create(3, "ut_full_3_ow_extrct", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		for (n = 0; n < 8 * msgs_per_page; ++n)
			ipc_log_string(ctx, "pre-extract %d\n", n);

		do {
			n = ipc_log_extract(ctx, data, sizeof(data));
			UT_ASSERT_INT(n, >=, 0)
		} while (n > 0);

		for (n = 0; n < 2 * msgs_per_page; ++n)
			ipc_log_string(ctx, "post-extract %d\n", n);

		if (failed)
			break;


		/* Crash system to start memory dump */
		panic("Crashing system to collect memory dump\n");
	} while (0);

	if (failed) {
		pr_err("%s: Failed\n", __func__);
		seq_printf(s, "\tFailed\n");
	}
}

/**
 * ipc_logging_ut_nd_read - Test non-destructive read support.
 *
 * @s: pointer to output file
 *
 * This test verifies non-destructive read support used by debugfs.
 */
static void ipc_logging_ut_nd_read(struct seq_file *s)
{
	const int msgs_per_page = 180;
	const int num_pages = 3;
	int failed = 0;
	int n;
	int i;
	int read_size;
	char *data;
	char expected[128];
	void *ctx = NULL;

	seq_printf(s, "Running %s\n", __func__);
	data = kzalloc(MAX_MSG_DECODED_SIZE, GFP_KERNEL);
	do {
		UT_ASSERT_PTR(data, !=, NULL);
		ctx = ipc_log_context_create(num_pages,
			"ipc_logging_ut_nd_read", 0);
		UT_ASSERT_PTR(ctx, !=, NULL);

		/* Fill log and do non-destructive read of all lines */
		for (n = 0; n < msgs_per_page * num_pages; ++n)
			ipc_log_string(ctx, "line %d", n);

		do {
			read_size = ipc_log_extract(ctx, data,
				MAX_MSG_DECODED_SIZE);
		} while (read_size > 0);

		/* Write a single line and verify only single line read */
		for (n = 1000; n < 1000 + msgs_per_page * num_pages; ++n) {
			scnprintf(expected, sizeof(expected), "line %d\n", n);
			ipc_log_string(ctx, "%s", expected);

			i = ipc_log_extract(ctx, data, MAX_MSG_DECODED_SIZE);
			UT_ASSERT_INT(0, <, i);
			UT_ASSERT_INT(STRING_OFFSET, <, strlen(data));
			UT_ASSERT_STRING_COMPARE(expected, data +
				STRING_OFFSET);

			i = ipc_log_extract(ctx, data, MAX_MSG_DECODED_SIZE);
			UT_ASSERT_INT(0, ==, i);
		}
		seq_puts(s, "\tOK\n");
	} while (0);
	kfree(data);

	if (failed) {
		pr_err("%s: Failed\n", __func__);
		seq_puts(s, "\tFailed\n");
	}

	ipc_log_context_destroy(ctx);
}

static struct dentry *dent;

static int debugfs_show(struct seq_file *s, void *data)
{
	void (*show)(struct seq_file *) = s->private;

	show(s);

	return 0;
}

static int debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_show, inode->i_private);
}

static const struct file_operations debug_ops = {
	.open = debug_open,
	.release = single_release,
	.read = seq_read,
	.llseek = seq_lseek,
};

void ipc_logging_debug_create(const char *name,
			 void (*show)(struct seq_file *))
{
	struct dentry *file;

	file = debugfs_create_file(name, 0444, dent, show, &debug_ops);
	if (!file)
		pr_err("%s: unable to create file '%s'\n", __func__, name);
}

static int __init ipc_logging_debugfs_init(void)
{
	dent = debugfs_create_dir("ipc_logging_test", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	/*
	 * Add Unit Test entries.
	 *
	 * The idea with unit tests is that you can run all of them
	 * from ADB shell by doing:
	 *  adb shell
	 *  cat ut*
	 *
	 * And if particular tests fail, you can then repeatedly run the
	 * failing tests as you debug and resolve the failing test.
	 */
	ipc_logging_debug_create("ut_basic",
			ipc_logging_ut_basic);
	ipc_logging_debug_create("ut_wrap_test",
			ipc_logging_ut_wrap_test);
	ipc_logging_debug_create("extraction_dump",
			extraction_dump);
	ipc_logging_debug_create("ut_nd_read",
			ipc_logging_ut_nd_read);

	return 0;
}

static void ipc_logging_debugfs_exit(void)
{
	if (dent != NULL)
		debugfs_remove_recursive(dent);
}

module_init(ipc_logging_debugfs_init);
module_exit(ipc_logging_debugfs_exit);
MODULE_DESCRIPTION("IPC Logging Unit Test");
MODULE_LICENSE("GPL v2");
