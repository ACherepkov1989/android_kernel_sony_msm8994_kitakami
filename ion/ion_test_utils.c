/*
 * Copyright (c) 2012, The Linux Foundation. All rights reserved.
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

/* MSM ION test Utils. */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <linux/input.h>
#include <linux/msm_ion.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "ion_test_plan.h"
#include "ion_test_utils.h"

int debug_level = ERR;

void set_debug_level(int level)
{
	debug_level = level;
}

char *itoa(int num, char *buf, size_t len)
{
	if (snprintf(buf, len, "%d", num) == -1)
		return NULL;
	else
		return buf;
}

int sock_write(int fd, const char *buf, int size)
{
	int sent = 0, wc = 0;
	int index = 0;
	const char *data = buf;
	while (sent < size) {
		wc = write(fd, &data[sent], size - sent);
		if (wc < 0) {
			debug(INFO, "\n error in writing to socket\n");
			return -EIO;
		} else {
			sent += wc;
		}
	};
	return 0;
}

int sock_read(int fd, char *buf, int size)
{
	int rc = 0, rd = 0;
	int index = 0;
	char *data = buf;
	while (rd < size) {
		rc = read(fd, &data[rd], size - rd);
		if (rc < 0) {
			debug(INFO, "\n error in reading from socket\n");
			return -EIO;
		} else {
			rd += rc;
		}
	}
	return 0;
}
