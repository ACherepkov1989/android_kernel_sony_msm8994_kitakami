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

#ifndef _LOOPBACK_SMSM_H_
#define _LOOPBACK_SMSM_H_

#include <linux/types.h>

int mock_smsm_change_state(uint32_t smsm_entry,
	uint32_t clear_mask, uint32_t set_mask);

uint32_t mock_smsm_get_state(uint32_t smsm_entry);

int mock_smsm_state_cb_register(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t old_state, uint32_t new_state),
	void *data);

int mock_smsm_state_cb_deregister(uint32_t smsm_entry, uint32_t mask,
	void (*notify)(void *, uint32_t, uint32_t), void *data);

int mock_smsm_init(void);
#endif /* _LOOPBACK_SMSM_H_ */
