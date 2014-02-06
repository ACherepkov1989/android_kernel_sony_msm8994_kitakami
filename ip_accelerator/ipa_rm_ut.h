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

#ifndef _IPA_RM_UT_H_
#define _IPA_RM_UT_H_

/**
 * ipa_rm_ut - unit test module
 * Defines sanity test scenarios executed from debugfs
 * writer function defined in ipa_rm module
 */

#include <linux/msm_ipa.h>
#include <linux/ipa.h>

int build_rmnet_bridge_use_case_graph(
		int (*create_resource)(struct ipa_rm_create_params *create_params),
		int (*consumer_cb)(enum ipa_rm_event event,
				enum ipa_rm_resource_name resource_name));

int build_rmnet_bridge_use_case_dependencies(
		int (*add_dependency)(enum ipa_rm_resource_name dependant_name,
						enum ipa_rm_resource_name dependency_name));
int request_release_resource_sequence(
		int (*resource_request)(enum ipa_rm_resource_name resource_name),
		int (*resource_release)(enum ipa_rm_resource_name resource_name));

void clean_ut(void);

#endif /* _IPA_RM_UT_H_ */
