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

#define IPV6_ENABLED

//---------------------------------------------------------------------------------/
// Test001: USB Integration Application (Header Removal, Insertion and Loopback    /
//---------------------------------------------------------------------------------/
class USBIntegrationLoopbackApp: public USBIntegrationFixture {
public:
	USBIntegrationLoopbackApp()
	{
		m_name = "USBIntegrationLoopbackApp";
		m_testSuiteName.push_back("USBInteg");
		m_runInRegression = false; // This test should not run in regression testing
		m_description =" \
			USB integration - Loopback Application\
			Pipe Configuration: \
				PC -> USB Cable -> USB_PROD --> A2_TETHERED_CONS -->\
				PC <- USB Cable <- USB_CONS <-- A2_TETHERED_PROD <-- \
			A5 Reads Data from A2_TETHERED_CONS, switch between the SRC and DST IPs and push\
			Packets into A2_TETHERED_PROD\
		Test Sequence \
		1. Generate and commit 2 Routing Tables (IPv4+IPv6) with destination pipe A2_TETHERED_CONS\
		2. Generate and commit 2 Routing Tables (IPv4+IPv6) with destination pipe USB_CONS\
		3. Generate and commit 2 Filtering Tables (IPv4+IPv6) for Endpoint A2_TETHERED_PROD \
		4. Generate and commit 2 Filtering Tables (IPv4+IPv6) for Endpoint USB_PROD \
		5. Start The Loopback:  \
			Reade data from pipe, if no Data Available, sleep for 1 sec, and retry.\
			Read Packet from A2_TETHERED_CONS  \
			Swap the SRC and DST IP Address:  \
			Send Packet to A2_TETHERED_PROD  \
			goto 5  \
				";

//		A2_TETHERED_PROD = pipe #8
//		A2_TETHERED_CONS = pipe #9
//
//		Connection Diagram:
//
//		 __________    A2_TETHERED_CONS   __________     USB_PROD   __________
//		|          |<--------------------|          |<--------------|          |
//		|    A5    |                     |   IPA    |               |   USB    |
//		|          |   A2_TETHERED_PROD  |          |    USB_CONS   |          |
//		|__________|-------------------->|__________|-------------->|__________|

		Register(*this);
	}



	virtual bool AddRules() {
		m_eIP = IPA_IP_v4;
		const char aBypass1IPv4[20] = "USB1_PROD_LP";
		const char aBypass2IPv4[20] = "USB1_CONS_LP";
#ifdef IPV6_ENABLED
		const char aBypass1IPv6[20] = "IPv6BypassToApp";
		const char aBypass2IPv6[20] = "IPv6BypasstoUSB";
#endif
		// Routing Tables Handles
		uint32_t nRTTableHdlIPv4ToSys, nRTTableHdlIPv4ToUSB;
#ifdef IPV6_ENABLED
		uint32_t nRTTableHdlIPv6ToSys, nRTTableHdlIPv6ToUSB;
#endif
		bool bRetVal = true;
		//Filtering Tables
		IPAFilteringTable cFLTTableIPv4ToSys,cFLTTableIPv4ToUSB;
#ifdef IPV6_ENABLED
		IPAFilteringTable cFLTTableIPv6ToSys,cFLTTableIPv6ToUSB;
#endif
		struct ipa_flt_rule_add sFilterRuleEntry;
		struct ipa_ioc_get_hdr sGetIPv4Header,sGetIPv6Header;

		LOG_MSG_STACK("Entering Function");
		memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));

		// Creating First Routing Table
		if (!CreateBypassRoutingTable(&m_Routing, IPA_IP_v4, aBypass1IPv4, IPA_CLIENT_A2_TETHERED_CONS, // USB->System Pipe
			0,&nRTTableHdlIPv4ToSys)) {

			LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_INFO("IPv4 Routing Table %s was inserted.\n",aBypass1IPv4);

#ifdef IPV6_ENABLED
		if (!CreateBypassRoutingTable(&m_Routing, IPA_IP_v6, aBypass1IPv6, IPA_CLIENT_A2_TETHERED_CONS, // USB->System Pipe
			0,&nRTTableHdlIPv6ToSys)) {
			LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_INFO("IPv6 Routing Table %s was inserted.\n",aBypass1IPv6);
#endif
		// Fetch Header Insertion Entries (these were added by USB driver)
		memset(&sGetIPv4Header, 0, sizeof(sGetIPv4Header));
		strcpy(sGetIPv4Header.name, "USB_ETHER_IPV4");//IPv4 Ethertype Header
		if (!m_HeaderInsertion.GetHeaderHandle(&sGetIPv4Header))
		{
			LOG_MSG_ERROR(" Failed");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_DEBUG("Received Header1 Handle = 0x%x",sGetIPv4Header.hdl);

		if (!CreateBypassRoutingTable(&m_Routing, IPA_IP_v4, aBypass2IPv4, IPA_CLIENT_TEST_CONS,//IPA->USB
				sGetIPv4Header.hdl,&nRTTableHdlIPv4ToUSB)) {
			LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_INFO("Creation of IPv4 bypass routing tables completed successfully:\nnRTTableHdlIPv4ToSys=0x%x, \nnRTTableHdlIPv4ToUSB=0x%x,",
				nRTTableHdlIPv4ToSys, nRTTableHdlIPv4ToUSB);
#ifdef IPV6_ENABLED
		// Fetch Header Insertion Entries (these were added by USB driver)
		memset(&sGetIPv6Header, 0, sizeof(sGetIPv6Header));
		strcpy(sGetIPv6Header.name, "USB_ETHER_IPV6");//IPv6 Ethertype Header
		if (!m_HeaderInsertion.GetHeaderHandle(&sGetIPv6Header))
		{
			LOG_MSG_ERROR(" Failed");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_DEBUG("Received Header2 Handle = 0x%x",sGetIPv6Header.hdl);

		if (!CreateBypassRoutingTable(&m_Routing, IPA_IP_v6, aBypass2IPv6, IPA_CLIENT_TEST_CONS,//System->USB
				sGetIPv6Header.hdl,&nRTTableHdlIPv6ToUSB)) {
			LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
			bRetVal = false;
			goto bail;
		}
		LOG_MSG_INFO("Creation of IPv6 bypass routing tables completed successfully:\nnRTTableHdlIPv6ToSys=0x%x\nnRTTableHdlIPv6toUSB=0x%x",
				nRTTableHdlIPv6ToSys, nRTTableHdlIPv6ToUSB);
#endif

		// Creating Filtering Rules

		// Configuring Filtering Table No.1
		cFLTTableIPv4ToUSB.Init(IPA_IP_v4,IPA_CLIENT_A2_TETHERED_PROD,false,1);// IPA->System Pipe
		cFLTTableIPv4ToUSB.GeneratePresetRule(0,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nRTTableHdlIPv4ToUSB;

		if ( ((uint8_t)-1 == cFLTTableIPv4ToUSB.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFLTTableIPv4ToUSB.GetFilteringTable()) ) {
			LOG_MSG_ERROR ("Adding Rule (1) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		} else {
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFLTTableIPv4ToUSB.ReadRuleFromTable(0)->flt_rule_hdl,cFLTTableIPv4ToUSB.ReadRuleFromTable(0)->status);
		}

#ifdef IPV6_ENABLED
		// Configuring Filtering Table  No.2
		cFLTTableIPv6ToUSB.Init(IPA_IP_v6,IPA_CLIENT_A2_TETHERED_PROD,false,1); //System->USB Pipe
		cFLTTableIPv6ToUSB.GeneratePresetRule(0,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nRTTableHdlIPv6ToUSB;
		if ( ((uint8_t)-1 == cFLTTableIPv6ToUSB.AddRuleToTable(sFilterRuleEntry)) ||
		     !m_Filtering.AddFilteringRule(cFLTTableIPv6ToUSB.GetFilteringTable()) ) {
			LOG_MSG_ERROR ("Adding Rule (2) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		} else {
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFLTTableIPv6ToUSB.ReadRuleFromTable(0)->flt_rule_hdl,cFLTTableIPv6ToUSB.ReadRuleFromTable(0)->status);
		}
#endif
		// Configuring Filtering Table No.3
		cFLTTableIPv4ToSys.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1); // USB->System Pipe
		cFLTTableIPv4ToSys.GeneratePresetRule(0,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nRTTableHdlIPv4ToSys;
		if ( ((uint8_t)-1 == cFLTTableIPv4ToSys.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFLTTableIPv4ToSys.GetFilteringTable()) ) {
			LOG_MSG_ERROR ("Adding Rule (3) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		} else {
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFLTTableIPv4ToSys.ReadRuleFromTable(0)->flt_rule_hdl,cFLTTableIPv4ToSys.ReadRuleFromTable(0)->status);
		}

#ifdef IPV6_ENABLED
		// Configuring Filtering Table  No.4
		cFLTTableIPv6ToSys.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,false,1); // USB->System Pipe
		cFLTTableIPv6ToSys.GeneratePresetRule(0,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nRTTableHdlIPv6ToSys;
		if ( ((uint8_t)-1 == cFLTTableIPv6ToSys.AddRuleToTable(sFilterRuleEntry)) ||
		     !m_Filtering.AddFilteringRule(cFLTTableIPv6ToSys.GetFilteringTable()) ) {
			LOG_MSG_ERROR ("Adding Rule (4) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		} else {
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFLTTableIPv6ToSys.ReadRuleFromTable(0)->flt_rule_hdl,cFLTTableIPv6ToSys.ReadRuleFromTable(0)->status);
		}
#endif

	bail:
		LOG_MSG_STACK(
				"Leaving Function (Returning %s)", bRetVal?"True":"False");
		return bRetVal;
	} // AddRules()
};
static USBIntegrationLoopbackApp ipaUSBIntegrationLoopbackApp;

// Note that this test was not verified to work after the separtion of USBIntegrationFixture
// to a separate file.


