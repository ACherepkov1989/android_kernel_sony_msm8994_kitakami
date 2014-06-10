/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include "RoutingDriverWrapper.h"
#include "HeaderInsertion.h"
#include "Filtering.h"
#include "IPAFilteringTable.h"
#include "hton.h"
#include "TestsUtils.h"
#include "USBIntegrationFixture.h"
#include <unistd.h>
#include <string.h>

// TBD: check what is the header name
#define IPA_TO_WLAN_HEADER_NAME "wlan0"
#define LAN_ROUTING_TABLE_NAME "LAN_RT"
#define WAN_ROUTING_TABLE_NAME "WAN_RT"

// The parameters below should be filled by the user, in BIG ENDIAN format !
static const int SRC_MAC_ADDR = 0xAABBCC;
static const int DST_MAC_ADDR = 0xDDEEFF;

static const int WLAN_DST_IP_ADDR = 0xAABBCCDD;
static const int WLAN_DST_IP_ADDR_MASK = 0xFFFFFFFF;

class WLANIntegrationLoopbackApp: public USBIntegrationFixture {
public:
	WLANIntegrationLoopbackApp()
	{
		m_name = "WLANIntegrationLoopbackApp";
		m_description = "WLAN integration - Loopback Application";
		m_testSuiteName.push_back("WLANInteg");
		m_runInRegression = false;	// This test should not run in regression testing
		Register(*this);
	} // Constructor WLANIntegrationLoopbackApp()

	virtual bool AddRules()
	{
		LOG_MSG_STACK("Entering Function");

		if (!configure_header_insertion()) {
			LOG_MSG_ERROR("Header configuration failed");
			return false;
		}

		if (!configure_routing_tables()) {
			LOG_MSG_ERROR("Routing tables configuration failed");
			return false;
		}

		if (!configure_filtering_tables()) {
			LOG_MSG_ERROR("Filtering rules configuration failed");
			return false;
		}

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

private:
	bool configure_header_insertion()
	{
		const int WMI_HDR_DEVICE_ID_OFFSET = 10;
		const int WMI_HDR_DEVICE_ID_MASK = 0xF8;
		const int ETHERNET_HDR_SRC_ADDR_OFFSET = 12;
		const int ETHERNET_HDR_DST_ADDR_OFFSET = 18;
		const int ETHERNET_HDR_ADDR_LEN = 6;
		uint8_t device_id;

		LOG_MSG_STACK("Entering Function");

		// Get a copy of IPA --> WLAN Header, the partial header was
		// installed by the WLAN driver
		struct ipa_ioc_copy_hdr ipa_to_wlan_header;
		strncpy(ipa_to_wlan_header.name, IPA_TO_WLAN_HEADER_NAME,
			strlen(IPA_TO_WLAN_HEADER_NAME));
		if (!m_HeaderInsertion.CopyHeader(&ipa_to_wlan_header)) {
			LOG_MSG_ERROR("Failed copying header");
			return false;
		}

		// HTC header
		// length, bytes 3-4 will be filled by IPA HW

		// WMI header
		// Fill device ID in byte 5, 3 LS bits
		// Assuming device ID is 000, TBD, how does IPA know the device ID ?
		device_id = 0;
		ipa_to_wlan_header.hdr[WMI_HDR_DEVICE_ID_OFFSET] &=
			(WMI_HDR_DEVICE_ID_MASK | device_id);

		// 802.3 header (Ethernet)
		// Fill SRC and DST MAC addresses
		memcpy(&ipa_to_wlan_header.hdr[ETHERNET_HDR_SRC_ADDR_OFFSET],
		       &SRC_MAC_ADDR, ETHERNET_HDR_ADDR_LEN);

		memcpy(&ipa_to_wlan_header.hdr[ETHERNET_HDR_DST_ADDR_OFFSET],
		       &DST_MAC_ADDR, ETHERNET_HDR_ADDR_LEN);

		// LLC SNAP header
		// TBD, who should fill the packet type, and when ?

		// Commit the header back to IPA driver and HW. Flag it as non partial
		ipa_to_wlan_header.is_partial = 0;

		struct ipa_ioc_add_hdr *ipa_to_wlan_header_wrapper;
		int len = sizeof(struct ipa_ioc_add_hdr) +
			1*sizeof(struct ipa_hdr_add);
		ipa_to_wlan_header_wrapper = (struct ipa_ioc_add_hdr *) calloc(1, len);
		if (!ipa_to_wlan_header_wrapper) {
			LOG_MSG_ERROR("Memory allocation failure");
			return false;
		}
		memset(ipa_to_wlan_header_wrapper, 0, len);

		ipa_to_wlan_header_wrapper->commit = 1;
		ipa_to_wlan_header_wrapper->num_hdrs = 1;

		memcpy(&ipa_to_wlan_header_wrapper->hdr[0],
		       &ipa_to_wlan_header,
		       sizeof(ipa_to_wlan_header));

		if (!m_HeaderInsertion.AddHeader(ipa_to_wlan_header_wrapper)) {
			LOG_MSG_ERROR("Failed adding header");
			return false;
		}

		// Save the header handle for later use
		m_ipa_to_wlan_hdr_hdl = ipa_to_wlan_header_wrapper->hdr[0].hdr_hdl;

		free(ipa_to_wlan_header_wrapper);

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_routing_tables()
	{
		LOG_MSG_STACK("Entering Function");

		if (!configure_lan_routing()) {
			LOG_MSG_ERROR("Failed configuring LAN routing tables");
			return false;
		}

		if (!configure_wan_routing()) {
			LOG_MSG_ERROR("Failed configuring WAN routing tables");
			return false;
		}

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_lan_routing()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		const int NUM_RULES = 8;

		LOG_MSG_STACK("Entering Function");

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));
		if(!rt_rule) {
			LOG_MSG_ERROR("Memory allocation failure");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, LAN_ROUTING_TABLE_NAME);

		// Set the parameters for all the rules
		for (int i = 0; i < NUM_RULES; i++) {
			rt_rule->rules[i].rule.hdr_hdl = m_ipa_to_wlan_hdr_hdl;
			rt_rule->rules[i].rule.attrib.attrib_mask = IPA_FLT_TOS;
			rt_rule->rules[i].rule.attrib.u.v4.tos = i;
		}

		rt_rule->rules[0].rule.dst = IPA_CLIENT_TEST1_CONS;
		rt_rule->rules[1].rule.dst = IPA_CLIENT_TEST2_CONS;
		rt_rule->rules[2].rule.dst = IPA_CLIENT_TEST2_CONS;
		rt_rule->rules[3].rule.dst = IPA_CLIENT_TEST1_CONS;
		rt_rule->rules[4].rule.dst = IPA_CLIENT_TEST3_CONS;
		rt_rule->rules[5].rule.dst = IPA_CLIENT_TEST3_CONS;
		rt_rule->rules[6].rule.dst = IPA_CLIENT_TEST4_CONS;
		rt_rule->rules[7].rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_Routing.AddRoutingRule(rt_rule)) {
			LOG_MSG_ERROR("Routing rule addition failed!\n");
			return false;
		}

		free(rt_rule);

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_wan_routing()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		const int NUM_RULES = 1;

		LOG_MSG_STACK("Entering Function");

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));
		if(!rt_rule) {
			LOG_MSG_ERROR("Memory allocation failure");
			return false;
		}

		// An 'empty' rule means catch all
		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, WAN_ROUTING_TABLE_NAME);

		rt_rule->rules[0].rule.hdr_hdl = m_ipa_to_wlan_hdr_hdl;

		if (false == m_Routing.AddRoutingRule(rt_rule)) {
			LOG_MSG_ERROR("Routing rule addition failed!\n");
			return false;
		}

		free(rt_rule);

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_filtering_tables()
	{
		LOG_MSG_STACK("Entering Function");

		if (!configure_rx_ep_filtering_table()) {
			LOG_MSG_ERROR("Failed configuring Rx endpoint filtering rules");
			return false;
		}

		if (!configure_wlan_dl_ep_filtering_table()) {
			LOG_MSG_ERROR("Failed configuring WLAN DL endpoint filtering rules");
			return false;
		}

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_rx_ep_filtering_table()
	{
		const int NUM_RULES = 2;
		struct ipa_ioc_add_flt_rule *filtering_table;
		struct ipa_ioc_get_rt_tbl lan_routing_table;
		struct ipa_ioc_get_rt_tbl wan_routing_table;

		LOG_MSG_STACK("Entering Function");

		// Get the needed routing tables handles
		lan_routing_table.ip = IPA_IP_v4;
		strcpy (lan_routing_table.name, LAN_ROUTING_TABLE_NAME);
		if (!m_Routing.GetRoutingTable(&lan_routing_table)) {
			LOG_MSG_ERROR("Failed getting LAN_RT routing table");
			return false;
		}

		wan_routing_table.ip = IPA_IP_v4;
		strcpy (wan_routing_table.name, WAN_ROUTING_TABLE_NAME);
		if (!m_Routing.GetRoutingTable(&wan_routing_table)) {
			LOG_MSG_ERROR("Failed getting WAN_RT routing table");
			return false;
		}

		// Allocate the filtering table
		filtering_table = (struct ipa_ioc_add_flt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_flt_rule) +
			NUM_RULES*sizeof(struct ipa_flt_rule_add));
		if (!filtering_table) {
			LOG_MSG_ERROR("Memory allocation failure");
			return false;
		}

		filtering_table->commit = 1;
		filtering_table->ep = IPA_CLIENT_TEST1_PROD;
		filtering_table->global = 0;
		filtering_table->ip = IPA_IP_v4;
		filtering_table->num_rules = NUM_RULES;

		filtering_table->rules[0].rule.action = IPA_PASS_TO_ROUTING;
		filtering_table->rules[0].rule.rt_tbl_hdl = lan_routing_table.hdl;
		filtering_table->rules[0].rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		filtering_table->rules[0].rule.attrib.u.v4.dst_addr = WLAN_DST_IP_ADDR;
		filtering_table->rules[0].rule.attrib.u.v4.dst_addr_mask = WLAN_DST_IP_ADDR_MASK;

		filtering_table->rules[1].rule.action = IPA_PASS_TO_ROUTING;
		filtering_table->rules[1].rule.rt_tbl_hdl = wan_routing_table.hdl;
		filtering_table->rules[1].rule.attrib.attrib_mask = 0; // Match all

		if (!m_Filtering.AddFilteringRule(filtering_table)) {
			LOG_MSG_ERROR("Failed adding filtering table");
			return false;
		}

		free(filtering_table);

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	bool configure_wlan_dl_ep_filtering_table()
	{
		const int NUM_RULES = 1;
		struct ipa_ioc_add_flt_rule *filtering_table;
		struct ipa_ioc_get_rt_tbl lan_routing_table;

		LOG_MSG_STACK("Entering Function");

		// Get the needed routing tables handles
		lan_routing_table.ip = IPA_IP_v4;
		strcpy (lan_routing_table.name, LAN_ROUTING_TABLE_NAME);
		if (!m_Routing.GetRoutingTable(&lan_routing_table)) {
			LOG_MSG_ERROR("Failed getting LAN_RT routing table");
			return false;
		}

		// Allocate the filtering table
		filtering_table = (struct ipa_ioc_add_flt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_flt_rule) +
			NUM_RULES*sizeof(struct ipa_flt_rule_add));
		if (!filtering_table) {
			LOG_MSG_ERROR("Memory allocation failure");
			return false;
		}

		filtering_table->commit = 1;
		filtering_table->ep = IPA_CLIENT_A2_TETHERED_PROD;
		filtering_table->global = 0;
		filtering_table->ip = IPA_IP_v4;
		filtering_table->num_rules = NUM_RULES;

		filtering_table->rules[0].rule.action = IPA_PASS_TO_ROUTING;
		filtering_table->rules[0].rule.rt_tbl_hdl = lan_routing_table.hdl;
		filtering_table->rules[0].rule.attrib.attrib_mask = 0;

		if (!m_Filtering.AddFilteringRule(filtering_table)) {
			LOG_MSG_ERROR("Failed adding filtering table");
			return false;
		}

		free(filtering_table);

		LOG_MSG_STACK("Leaving Function (Returning True)");
		return true;
	}

	uint32_t m_ipa_to_wlan_hdr_hdl;
};

// This test is currently a Work in Progress
static WLANIntegrationLoopbackApp wlanIntegrationLoopbackApp;
