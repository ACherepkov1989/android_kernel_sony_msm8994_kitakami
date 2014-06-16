/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#ifndef _LOOPBACK_A2_H_
#define _LOOPBACK_A2_H_

#include "../../../drivers/soc/qcom/bam_dmux_private.h"

int mock_a2_init(void);

int mock_a2_tx_list_pop(struct bam_mux_hdr *hdr, void *buff);

int mock_a2_rx_list_push(void *data, struct tx_pkt_info *pkt);

void mock_a2_clear_lists(void);

#endif /* _LOOPBACK_A2_H_ */
