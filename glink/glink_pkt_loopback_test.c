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

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <getopt.h>
#include <poll.h>
#include <sys/mman.h>

#define GLINK_PKT_IOCTL_MAGIC (0xC3)

#define GLINK_PKT_IOCTL_QUEUE_RX_INTENT \
	_IOW(GLINK_PKT_IOCTL_MAGIC, 0, unsigned int)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))
#define MAX_NAME_LEN 32
#define MAX_SIZE (4 * 4096)
#define GLINK_CNTL_DEV   "/dev/glink_pkt_loopback_ctrl"
#define GLINK_DATA_DEV   "/dev/glink_pkt_loopback"

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
 * Assumes that the following local variable exist:
 * @failed - set to true if test fails
 */
#define UT_ASSERT_INT(a, cmp, b) \
	{ \
	int a_tmp = (a); \
	int b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		printf("%s:%d Fail: " #a "(%d) " #cmp " " #b "(%d)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

#define UT_ASSERT_PTR(a, cmp, b) \
	{ \
	void *a_tmp = (a); \
	void *b_tmp = (b); \
	if (!((a_tmp)cmp(b_tmp))) { \
		printf("%s:%d Fail: " #a "(%p) " #cmp " " #b "(%p)\n", \
				__func__, __LINE__, a_tmp, b_tmp); \
		failed = 1; \
		break; \
	} \
	}

/*
 * LOOPBACK COMMAND TYPE
 */
enum request_type {
	OPEN = 1,
	CLOSE,
	QUEUE_RX_INTENT_CONFIG,
	TX_CONFIG,
	RX_DONE_CONFIG,
};

/*
 * struct req_hdr - Loopback command request header
 * @req_id:	Identifier of the request command.
 * @req_type:	Type of the request command.
 * @req_size:	Size of the request command.
 */
struct req_hdr {
	uint32_t req_id;
	uint32_t req_type;
	uint32_t req_size;
};

/*
 * struct open_req - Loopback command open request
 * @delay_ms:	Time delay to process open request.
 * @name_len:	Length of the channel name.
 * @ch_name:	Channel name.
 */
struct open_req {
	uint32_t delay_ms;
	uint32_t name_len;
	char ch_name[MAX_NAME_LEN];
};

/*
 * struct close_req - Loopback command close request
 * @delay_ms:	Time delay to process close request.
 * @name_len:	Length of the channel name.
 * @ch_name:	Channel name.
 */
struct close_req {
	uint32_t delay_ms;
	uint32_t name_len;
	char ch_name[MAX_NAME_LEN];
};

/*
 * struct queue_rx_intent_config_req - queue rx intent request
 * @num_intents:	Number of intents to be queued.
 * @intent_size:	Size of the intent.
 * @random_delay:	Random delay to process the request.
 * @delay_ms:		Time delay to process close request.
 * @name_len:		Length of the channel name.
 * @ch_name:		Channel name.
 */
struct queue_rx_intent_config_req {
	uint32_t num_intents;
	uint32_t intent_size;
	uint32_t random_delay;
	uint32_t delay_ms;
	uint32_t name_len;
	char ch_name[MAX_NAME_LEN];
};

enum transform_type {
	NO_TRANSFORM = 0,
	PACKET_COUNT,
	CHECKSUM,
};

/*
 * struct tx_config_req - Tx data configuration request
 * @random_delay:	Random delay to process the request.
 * @delay_ms:		Time delay to process close request.
 * @echo_count:		Echo count for Tx data.
 * @transform_type:	Type of Tx data.
 * @name_len:		Length of the channel name.
 * @ch_name:		Channel name.
 */
struct tx_config_req {
	uint32_t random_delay;
	uint32_t delay_ms;
	uint32_t echo_count;
	uint32_t transform_type;
	uint32_t name_len;
	char ch_name[MAX_NAME_LEN];
};

/*
 * struct rx_done_config_req - Rx done data configuration request
 * @random_delay:	Random delay to process the request.
 * @delay_ms:		Time delay to process close request.
 * @name_len:		Length of the channel name.
 * @ch_name:		Channel name.
 */
struct rx_done_config_req {
	uint32_t random_delay;
	uint32_t delay_ms;
	uint32_t name_len;
	char ch_name[MAX_NAME_LEN];
};

/*
 * union req_payload - Loppback request payload
 * @open:		Open request.
 * @close:		Close request.
 * @q_rx_int_conf:	Queue rx intent config request.
 * @tx_conf:		Tx config request.
 * @rx_done_conf:	Rx donr config request.
 */
union req_payload {
	struct open_req open;
	struct close_req close;
	struct queue_rx_intent_config_req q_rx_int_conf;
	struct tx_config_req tx_conf;
	struct rx_done_config_req rx_done_conf;
};

/*
 * struct req - Loopback request structure
 * @hdr:	Loopback request header.
 * @payload:	Loopback request payload.
 */
struct req {
	struct req_hdr hdr;
	union req_payload payload;
};

/**
 * Struct resp - Loopback response structure
 * @req_id:	Loopback request identifier.
 * @req_type:	Loopback request command type.
 * @response:	Loopback command response.
 */
struct resp {
	uint32_t req_id;
	uint32_t req_type;
	uint32_t response;
};

static unsigned int request_id;

static char *data_device_nm = GLINK_DATA_DEV;
static char *cntl_device_nm = GLINK_CNTL_DEV;

static int cntl_fd;
static int data_fd;

static int iterations = 30;
static unsigned int resp_size = 12;
static const char *test_data = "Hello World";
static unsigned int test_data_len = 12;
static char rx_test_data[12];
static int pattern = 0xAA;
static const char *data_ch_name = "glink_pkt_lloop_CLNT";
static struct req pkt;
static struct resp resp_pkt;
typedef int (*test_func)();
static int basic_test(void);
static pthread_t getsigs_pthread;
static pthread_t setsigs_pthread;
static int set_sigs;
static int stop_signal_test = 0;
test_func do_test = basic_test;

/**
 * send_open_cmd() - Send open command to loopback server
 * @fd:	File descriptor of loopback server control port.
 *
 * This function is used to send the open command request
 * to Loopback server over the control port @fd.
 *
 * Return: 0 for success and -1 for failure.
 */
int send_open_cmd(int fd)
{
	int ret;
	int failed = 0;

	do {

		ret = ioctl(fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &resp_size);
		UT_ASSERT_INT(ret, ==, 0);

		/* prepare OPEN command */
		pkt.hdr.req_id = request_id++;
		pkt.hdr.req_type = OPEN;
		pkt.hdr.req_size = sizeof(struct open_req);
		pkt.payload.open.delay_ms = 0;
		strcpy(pkt.payload.open.ch_name, data_ch_name);
		pkt.payload.open.name_len = strlen(data_ch_name);;

		ret = write(fd, &pkt, sizeof(struct req));
		UT_ASSERT_INT(ret, ==, sizeof(struct req));

		ret = read(fd, &resp_pkt, sizeof(struct resp));
		UT_ASSERT_INT(ret, ==, sizeof(struct resp));
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * send_tx_config_cmd() - Send tx config command to loopback server
 * @fd:	File descriptor of loopback server control port.
 *
 * This function is used to send the tx config command request
 * to Loopback server over the control port @fd.
 *
 * Return: 0 for success and -1 for failure.
 */
int send_tx_config_cmd(int fd)
{
	int ret;
	int failed = 0;

	do {
		ret = ioctl(fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &resp_size);
		UT_ASSERT_INT(ret, ==, 0);

		/* prepare TX_CONFIG command */
		pkt.hdr.req_id = request_id++;
		pkt.hdr.req_type = TX_CONFIG;
		pkt.hdr.req_size = sizeof(struct tx_config_req);
		pkt.payload.tx_conf.random_delay = 0;
		pkt.payload.tx_conf.delay_ms = 0;
		pkt.payload.tx_conf.echo_count = 1;
		pkt.payload.tx_conf.transform_type = PACKET_COUNT;
		strcpy(pkt.payload.tx_conf.ch_name, data_ch_name);
		pkt.payload.tx_conf.name_len = strlen(data_ch_name);;

		ret = write(fd, &pkt, sizeof(struct req));
		UT_ASSERT_INT(ret, ==, sizeof(struct req));

		ret = read(fd, &resp_pkt, sizeof(struct resp));
		UT_ASSERT_INT(ret, ==, sizeof(struct resp));
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * send_rx_done_config_cmd() - Send rx done config command to loopback server
 * @fd:	File descriptor of loopback server control port.
 *
 * This function is used to send the rx done config command request
 * to Loopback server over the control port @fd.
 *
 * Return: 0 for success and -1 for failure.
 */
int send_rx_done_config_cmd(int fd)
{
	int ret;
	int failed = 0;

	do {
		ret = ioctl(fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &resp_size);
		UT_ASSERT_INT(ret, ==, 0);

		/* prepare RX_DONE_CONFIG command */
		pkt.hdr.req_id = request_id++;
		pkt.hdr.req_type = RX_DONE_CONFIG;
		pkt.hdr.req_size = sizeof(struct rx_done_config_req);
		pkt.payload.rx_done_conf.random_delay = 0;
		pkt.payload.rx_done_conf.delay_ms = 0;
		strcpy(pkt.payload.rx_done_conf.ch_name, data_ch_name);
		pkt.payload.rx_done_conf.name_len = strlen(data_ch_name);;

		ret = write(fd, &pkt, sizeof(struct req));
		UT_ASSERT_INT(ret, ==, sizeof(struct req));

		ret = read(fd, &resp_pkt, sizeof(struct resp));
		UT_ASSERT_INT(ret, ==, sizeof(struct resp));
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * send_queue_rx_intent_config_cmd() - Send queue rx intent config
 *					command to loopback server
 * @fd:	File descriptor of loopback server control port.
 *
 * This function is used to send the queue rx intent config
 * command request to Loopback server over the control port @fd.
 *
 * Return: 0 for success and -1 for failure.
 */
int send_queue_rx_intent_config_cmd(int fd, int size, int num_intents)
{
	int ret;
	int failed = 0;

	do {
		ret = ioctl(fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &resp_size);
		UT_ASSERT_INT(ret, ==, 0);

		/* prepare QUEUE_RX_INTENT_CONFIG command */
		pkt.hdr.req_id = request_id++;
		pkt.hdr.req_type = QUEUE_RX_INTENT_CONFIG;
		pkt.hdr.req_size = sizeof(struct queue_rx_intent_config_req);
		pkt.payload.q_rx_int_conf.random_delay = 0;
		pkt.payload.q_rx_int_conf.delay_ms = 0;
		pkt.payload.q_rx_int_conf.num_intents = num_intents;
		pkt.payload.q_rx_int_conf.intent_size = size;
		strcpy(pkt.payload.q_rx_int_conf.ch_name, data_ch_name);
		pkt.payload.q_rx_int_conf.name_len = strlen(data_ch_name);

		ret = write(fd, &pkt, sizeof(struct req));
		UT_ASSERT_INT(ret, ==, sizeof(struct req));

		ret = read(fd, &resp_pkt, sizeof(struct resp));
		UT_ASSERT_INT(ret, ==, sizeof(struct resp));
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * send_close_cmd() - Send close command to loopback server
 * @fd:	File descriptor of loopback server control port.
 *
 * This function is used to send the close command request
 * to Loopback server over the control port @fd.
 *
 * Return: 0 for success and -1 for failure.
 */
int send_close_cmd(int fd)
{
	int ret;
	int failed = 0;

	do {
		ret = ioctl(fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &resp_size);
		UT_ASSERT_INT(ret, ==, 0);

		/* prepare CLOSE command */
		pkt.hdr.req_id = request_id++;
		pkt.hdr.req_type = CLOSE;
		pkt.hdr.req_size = sizeof(struct close_req);
		pkt.payload.close.delay_ms = 0;
		strcpy(pkt.payload.close.ch_name, data_ch_name);
		pkt.payload.close.name_len = strlen(data_ch_name);;

		ret = write(fd, &pkt, sizeof(struct req));
		UT_ASSERT_INT(ret, ==, sizeof(struct req));

		ret = read(fd, &resp_pkt, sizeof(struct resp));
		UT_ASSERT_INT(ret, ==, sizeof(struct resp));
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * time_diff() - Find the time difference between start and end times
 * @start:	Start time.
 * @end:	End time.
 *
 * This function used to calculate the time difference between the start
 * and end times.
 *
 * Return: difference time between start and end.
 */
double time_diff(const struct timespec start, const struct timespec end)
{
	double diff_time = 0;
	struct timespec temp;
	if ((end.tv_nsec - start.tv_nsec) < 0) {
		temp.tv_sec = end.tv_sec - start.tv_sec - 1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	}
	diff_time = (double)temp.tv_sec + ((double)(temp.tv_nsec)/(double)1000000000);
	return diff_time;
}

/**
 * basic_test() - basic functions test on glink packet device
 *
 * This function used to perform the basic function like
 * open, read, write, close and ioctl on glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int basic_test(void)
{
	int ret;
	int failed = 0;
	printf("Test: Basic Test\n");
	do {
		ret = send_open_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		data_fd = open(data_device_nm, O_RDWR);
		UT_ASSERT_INT(data_fd, >, 0);

		ret = ioctl(data_fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &test_data_len);
		UT_ASSERT_INT(ret, ==, 0);

		ret = write(data_fd, test_data, test_data_len);
		UT_ASSERT_INT(ret, ==, test_data_len);

		ret = read(data_fd, rx_test_data, test_data_len);
		UT_ASSERT_INT(ret, ==, test_data_len);

		UT_ASSERT_INT(strcmp(test_data, rx_test_data), ==, 0);

		ret = send_close_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		ret = close(data_fd);
		UT_ASSERT_INT(ret, ==, 0);
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * performance_test() - performance test on glink packet device
 *
 * This function is used for performance test on glink packet device
 * by sending different sizes data for multiple iterations and calculate
 * the round trip time for each step size of data.
 *
 * Return: 0 for success and -1 for failure.
 */
static int performance_test(void)
{
	int ret;
	int size;
	int i;
	double rtt;
	double data_rate;
	struct timespec start_time;
	struct timespec end_time;
	char *tx_data = NULL;
	char *rx_data = NULL;
	struct req pkt;
	int failed = 0;
	printf("TEST: Performance Test\n");

	do {
		ret = send_open_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		data_fd = open(data_device_nm, O_RDWR);
		UT_ASSERT_INT(data_fd, >, 0);

		printf("Size\tIterations\tRTT(us)\t\tThroughput(kBps)\t\n");
		for (size = 1; size <= MAX_SIZE; size = size * 2) {
			tx_data = malloc(size);
			UT_ASSERT_PTR(NULL, !=, tx_data);
			rx_data = malloc(size);
			UT_ASSERT_PTR(NULL, !=, rx_data);
			memset(tx_data, pattern, size);
			memset(rx_data, 0x00, size);

			ret = send_queue_rx_intent_config_cmd(cntl_fd, size, iterations);
			UT_ASSERT_INT(ret, ==, 0);

			clock_gettime(CLOCK_REALTIME,&start_time);
			for (i = 0; i < iterations; i++) {
				ret = ioctl(data_fd, GLINK_PKT_IOCTL_QUEUE_RX_INTENT, &size);
				UT_ASSERT_INT(ret, ==, 0);
				ret = write(data_fd, tx_data, size);
				UT_ASSERT_INT(ret, ==, size);

				ret = read(data_fd, rx_data, size);
				UT_ASSERT_INT(ret, ==, size);
				UT_ASSERT_INT(memcmp(rx_data, tx_data, size), ==, 0);
			}
			clock_gettime(CLOCK_REALTIME, &end_time);
			rtt = time_diff(start_time, end_time);
			rtt = rtt / iterations;
			data_rate = size / rtt;
			data_rate = data_rate * 2 / 1000;
			printf("%6d\t%6d\t\t%u\t\t%u\t\t\n",
				size, iterations, (unsigned int)(rtt * 1000 * 1000), (unsigned int)data_rate);

			free(tx_data);
			free(rx_data);
		}
		ret = send_close_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		ret = close(data_fd);
		UT_ASSERT_INT(ret, ==, 0);
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * set_signals_body() - set the signal for every 1sec
 *				on glink packet device
 * @arg:	Void pointer argument passed by pthread_create().
 *
 * This function used to set the signals of the glink packet device
 * for every once second.
 *
 * Return: NILL on success or error status in failure.
 */
static void *set_signals_body(void *arg)
{
	int ret;
	int failed = 0;
	int i;

	do {
		for (i = 0; i < iterations; i++) {
			/*
			 * Glink packet driver clients have to use signals as
			 * 31:28 - Reserved for SMD RS-232 signals
			 * 27:16 - Pass through for client usage
			 * 15:0 - TICOM bits
			 *
			 * 31:28 signals are reserved hence "and with 0x0fff"
			 */
			set_sigs = (pattern + i) & 0x0fff;
			printf("Set signal [0x%x]\n", set_sigs);
			ret = ioctl(data_fd, TIOCMSET, &set_sigs);
			UT_ASSERT_INT(ret, ==, 0);
			sleep(1);
			UT_ASSERT_INT(stop_signal_test, ==, 0);
		}
	} while (0);

	if (failed) {
		stop_signal_test = 1;
		pthread_exit(&set_sigs);
	} else {
		return NULL;
	}
}

/**
 * get_signals_body() - get the signal on glink packet device
 * @arg:	Void pointer argument passed by pthread_create().
 *
 * This function used to get the signals of the glink packet device
 * when the signals are updated.
 *
 * Return: NILL on success or error status in failure.
 */
static void *get_signals_body(void *arg)
{
	int ret;
	struct pollfd fds;
	int sigs = 0;
	int i;
	int failed = 0;

	do {
		fds.fd = data_fd;
		fds.events = POLLPRI;
		fds.revents = 0;

		for (i = 0; i < iterations; i++) {
			sigs = 0;
			ret = poll(&fds, 1, -1);
			if (ret < 0 || !(fds.revents & POLLPRI))
				printf("%s: ERROR - poll returned %d; revents 0x%x\n", __func__, ret, fds.revents);

			ret = ioctl(data_fd, TIOCMGET, &sigs);
			UT_ASSERT_INT(ret, ==, 0);
			printf("Get signal [0x%x]\n", sigs);

			UT_ASSERT_INT(sigs, ==, set_sigs);
			UT_ASSERT_INT(stop_signal_test, ==, 0);
		}
	} while (0);

	if(failed) {
		stop_signal_test = 1;
		pthread_exit(&sigs);
	} else {
		return NULL;
	}
}

/**
 * signals_set_get_test() - Signal test on glink packet device
 *
 * This function used to test the signal set and get functionality
 * of the glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int signals_set_get_test(void)
{
	int ret = 0;
	int *status;
	int failed = 0;

	printf("TEST: Set and Get Signal\n");
	do {
		ret = send_open_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		data_fd = open(data_device_nm, O_RDWR);
		UT_ASSERT_INT(data_fd, >, 0);

		ret = pthread_create (&getsigs_pthread , NULL, get_signals_body, NULL);
		UT_ASSERT_INT(ret, ==, 0);
		ret = pthread_create (&setsigs_pthread , NULL, set_signals_body, NULL);
		UT_ASSERT_INT(ret, ==, 0);

		UT_ASSERT_INT(pthread_join(setsigs_pthread, (void *)&status), ==, 0);
		UT_ASSERT_PTR(status, ==, NULL);

		UT_ASSERT_INT(pthread_join(getsigs_pthread, (void *)&status), ==, 0);
		UT_ASSERT_PTR(status, ==, NULL);

		ret = send_close_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		ret = close(data_fd);
		UT_ASSERT_INT(ret, ==, 0);
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * signalbits_set_clear_test() - signal bits test on glink packet device
 *
 * This function used to test the set and clear of the signal bits on the
 * glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int signalbits_set_clear_test(void)
{
	int ret;
	int i;
	int new_sigs = 0;
	int failed = 0;
	printf("TEST: SET and CLEAR Signal BITS\n");

	do {
		ret = send_open_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		data_fd = open(data_device_nm, O_RDWR);
		UT_ASSERT_INT(data_fd, >, 0);

		ret = ioctl(data_fd, TIOCMBIS, &pattern);
		UT_ASSERT_INT(ret, ==, 0);
		ret = ioctl(data_fd, TIOCMGET, &new_sigs);
		UT_ASSERT_INT(ret, ==, 0);
		UT_ASSERT_INT((new_sigs & pattern), ==, pattern);

		ret = ioctl(data_fd, TIOCMBIC, &pattern);
		UT_ASSERT_INT(ret, ==, 0);
		ret = ioctl(data_fd, TIOCMGET, &new_sigs);
		UT_ASSERT_INT(ret, ==, 0);
		UT_ASSERT_INT((new_sigs & pattern), ==, 0);

		ret = send_close_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		ret = close(data_fd);
		UT_ASSERT_INT(ret, ==, 0);
	} while (0);

	ret = failed ? -1 : 0;
	return ret;
}

/**
 * nominal_test() - Nominal test on glink packet device
 *
 * This function used to test basic functionality of glink
 * packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int nominal_test(void)
{
	int failed = 0;

	printf("TEST: Nominal Test\n");

	do {
		UT_ASSERT_INT(basic_test(), ==, 0);
	} while (0);

	if (failed) return -1;
	else return 0;
}

/*
 * adversarial_test() - Adversarial test on glink packet device
 *
 * This function used for adversarial tests on the glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int adversarial_test(void)
{
	int failed = 0;

	printf("TEST: Adversarial Test\n");

	do {
		UT_ASSERT_INT(basic_test(), ==, 0);
		UT_ASSERT_INT(signals_set_get_test(), ==, 0);
		UT_ASSERT_INT(signalbits_set_clear_test(), ==, 0);
	} while (0);

	if (failed) return -1;
	else return 0;
}

/**
 * repeatability_test() - Repeat test on glink packet device
 *
 * This function used for repeatability test on glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int repeatability_test(void)
{
	int failed = 0;
	int i;

	if (iterations <= 30)
		printf("Use -i option for more iterations, running for default iterations 30 \n");

	printf("TEST: Repeatability Test\n");

	do {
		for (i = 0; i < iterations; i++)
			UT_ASSERT_INT(basic_test(), ==, 0);
	} while (0);

	if (failed) return -1;
	else return 0;
}

/**
 * stress_test() -  stress test on glink packet device
 *
 * This function used for stress test on glink packet device.
 *
 * Return: 0 for success and -1 for failure.
 */
static int stress_test(void)
{
	int failed = 0;
	int i;

	if (iterations <= 30)
		printf("Use -i option for more iterations, running for default iterations 30 \n");

	printf("TEST: Stress Test\n");
	do {
		for (i = 0; i < iterations; i++) {
			UT_ASSERT_INT(basic_test(), ==, 0);
			UT_ASSERT_INT(performance_test(), ==, 0);
			UT_ASSERT_INT(signals_set_get_test(), ==, 0);
			UT_ASSERT_INT(signalbits_set_clear_test(), ==, 0);
		}
	} while (0);

	if (failed) return -1;
	else return 0;
}

/**
 * start_test() - start test on glink packet device
 *
 * This function is used to setup basic communication with loopback server
 * on control port and to proceed with any testing on glink packet device.
 */
void start_test(void)
{
	int ret = 0;
	int failed = 0;

	do {
		cntl_fd = open(cntl_device_nm, O_RDWR);
		UT_ASSERT_INT(cntl_fd, >, 0);

		usleep(10);

		ret = send_tx_config_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		ret = send_rx_done_config_cmd(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);

		UT_ASSERT_INT(do_test(), ==, 0);

		ret = close(cntl_fd);
		UT_ASSERT_INT(ret, ==, 0);
		printf("\tPASS\n");
	} while (0);

	if (failed) {
		printf("\tFailed\n");
		close(data_fd);
		close(cntl_fd);
	}
}

static void usage(int ret)
{
	printf("Usage: glink_pkt_loopback_test [OPTIONS] [PARAMS] [TEST_TYPE]...\n"
			"Runs the loopback test with parameters specified\n"
			"over a GLINK PKT device.  /dev/glink_pkt_loopback will be"
			"used if a device is not specified\n"
			"\n"
			"OPTIONS can be:\n"
			"  -h, --help            print this help message and exit\n"
			"\nPARAMS can be:\n"
			"  -i, --iterations      number of test iterations (default 1)\n"
			"  -p, --pattern         pattern to be written (default 0xAA)\n"
			"  -t, --testnum	1: Normal, 2: performance\n"
			"			3:Signal set get test\n"
			"			4: signal bits set & clear test"
			"\n"
			"TEST_TYPE can be:\n"
			"  -n, --nominal         run standard functionality tests\n"
			"  -a, --adversarial     run tests that try to break the\n"
			"                          driver, not supported currently\n"
			"  -s, --stress          run tests that try to maximize the\n"
			"                          capacity of the driver,\n"
			"                          not supported currently\n"
			"  -r, --repeatability   run 10 iterations of both the\n"
			"                          nominal and adversarial tests,\n"
			"                          not supported currently\n");

	exit(ret);
}

static uint32_t parse_command(int argc, char *const argv[])
{
	int command;
	unsigned ret = 0;
	int test_num = 0;

	struct option longopts[] = {
		{"nominal", no_argument, NULL, 'n'},
		{"adversarial", no_argument, NULL, 'a'},
		{"stress", no_argument, NULL, 's'},
		{"repeatability", no_argument, NULL, 'r'},
		{"help", no_argument, NULL, 'h'},
		{"iterations", required_argument, NULL, 'i'},
		{"pattern", required_argument, NULL, 'p'},
		{"testnum", required_argument, NULL, 't'},
		{NULL, 0, NULL, 0},
	};

	while ((command = getopt_long(argc, argv, "v:nasrhcwb:p:i:d:f:t:", longopts,
					NULL)) != -1) {
		switch (command) {
		case 'n':
			do_test = nominal_test;
			break;
		case 'a':
			do_test = adversarial_test;
			break;
		case 's':
			do_test = stress_test;
			break;
		case 'r':
			do_test = repeatability_test;
			break;
		case 'i':
			iterations = atoi(optarg);
			break;
		case 'p':
			pattern = (unsigned int)atoi(optarg);
			break;
		case 'h':
			usage(0);
			break;
		case 't':
			test_num = atoi(optarg);
			if (test_num == 1)
				do_test = basic_test;
			else if (test_num == 2)
				do_test = performance_test;
			else if (test_num == 3)
				do_test = signals_set_get_test;
			else if (test_num == 4)
				do_test = signalbits_set_clear_test;
			else
				do_test = basic_test;
			break;
		default:
			usage(-1);
		}
	}

	return ret;
}

int main(int argc, char**argv)
{
	parse_command(argc, argv);
	start_test();
	return 0;
}
