/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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


#include "RNDISAggregationTestFixture.h"

/////////////////////////////////////////////////////////////////////////////////

//define the static Pipes which will be used by all derived tests.
Pipe RNDISAggregationTestFixture::m_IpaToUsbPipeAgg(IPA_CLIENT_TEST2_CONS,
							 IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_UsbToIpaPipe(IPA_CLIENT_TEST_PROD,
						      IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_IpaToUsbPipe(IPA_CLIENT_TEST3_CONS,
						      IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_UsbToIpaPipeDeagg(IPA_CLIENT_TEST2_PROD,
							   IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_IpaToUsbPipeAggTime(IPA_CLIENT_TEST_CONS,
							     IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_IpaToUsbPipeAggPktLimit(IPA_CLIENT_TEST4_CONS,
								IPA_TEST_CONFIGURATION_17);
Pipe RNDISAggregationTestFixture::m_HsicToIpaPipe(IPA_CLIENT_TEST3_PROD,
							   IPA_TEST_CONFIGURATION_17);

RoutingDriverWrapper RNDISAggregationTestFixture::m_Routing;
Filtering RNDISAggregationTestFixture::m_Filtering;
HeaderInsertion RNDISAggregationTestFixture::m_HeaderInsertion;

/////////////////////////////////////////////////////////////////////////////////

RNDISAggregationTestFixture::RNDISAggregationTestFixture()
{
	m_testSuiteName.push_back("RndisAgg");
	Register(*this);
}

/////////////////////////////////////////////////////////////////////////////////

bool RNDISAggregationTestFixture::Setup()
{
	bool bRetVal = true;

	//Set the configuration to support USB->IPA and IPA->USB pipes.
	ConfigureScenario(IPA_TEST_CONFIGURATION_17);

	//Initialize the pipe for all the tests - this will open the inode which represents the pipe.
	bRetVal &= m_IpaToUsbPipeAgg.Init();
	bRetVal &= m_UsbToIpaPipe.Init();
	bRetVal &= m_HsicToIpaPipe.Init();
	bRetVal &= m_IpaToUsbPipe.Init();
	bRetVal &= m_UsbToIpaPipeDeagg.Init();
	bRetVal &= m_IpaToUsbPipeAggTime.Init();
	bRetVal &= m_IpaToUsbPipeAggPktLimit.Init();

	if (!m_Routing.DeviceNodeIsOpened()) {
		LOG_MSG_ERROR(
			"Routing block is not ready for immediate commands!\n");
		return false;
	}
	if (!m_Filtering.DeviceNodeIsOpened()) {
		LOG_MSG_ERROR(
			"Filtering block is not ready for immediate commands!\n");
		return false;
	}
	if (!m_HeaderInsertion.DeviceNodeIsOpened())
	{
		LOG_MSG_ERROR("Header Insertion block is not ready for immediate commands!\n");
		return false;
	}
	m_HeaderInsertion.Reset();// resetting this component will reset both Routing and Filtering tables.

	return bRetVal;
}

/////////////////////////////////////////////////////////////////////////////////

bool RNDISAggregationTestFixture::Teardown()
{
	//The Destroy method will close the inode.
	m_IpaToUsbPipeAgg.Destroy();
	m_UsbToIpaPipe.Destroy();
	m_HsicToIpaPipe.Destroy();
	m_IpaToUsbPipe.Destroy();
	m_UsbToIpaPipeDeagg.Destroy();
	m_IpaToUsbPipeAggTime.Destroy();
	m_IpaToUsbPipeAggPktLimit.Destroy();

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool RNDISAggregationTestFixture::Run()
{
	LOG_MSG_STACK("Entering Function");

	// Add the relevant filtering rules
	if (!AddRules()) {
		LOG_MSG_ERROR("Failed adding filtering rules.");
		return false;
	}
	if (!TestLogic()) {
		LOG_MSG_ERROR("Test failed, Input and expected output mismatch.");
		return false;
	}

	LOG_MSG_STACK("Leaving Function (Returning True)");
	return true;
} // Run()

/////////////////////////////////////////////////////////////////////////////////
bool RNDISAggregationTestFixture::AddRulesNoAgg() {
	m_eIP = IPA_IP_v4;
	const char aBypass[20] = "Bypass1";
	bool bRetVal = true;
	IPAFilteringTable cFilterTable0;
	struct ipa_flt_rule_add sFilterRuleEntry;
	uint32_t nTableHdl;


	LOG_MSG_STACK("Entering Function");
	memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));

	if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass, IPA_CLIENT_TEST3_CONS,
			0, &nTableHdl)) {
		LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
		bRetVal = false;
		goto bail;
	}


	LOG_MSG_INFO("Creation of bypass routing table completed successfully");

	// Creating Filtering Rules
	cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST_PROD, true, 1);
	LOG_MSG_INFO("Creation of filtering table completed successfully");

	// Configuring Filtering Rule No.1
	cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
	sFilterRuleEntry.at_rear = true;
	sFilterRuleEntry.flt_rule_hdl=-1; // return Value
	sFilterRuleEntry.status = -1; // return value
	sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
	sFilterRuleEntry.rule.retain_hdr = true;
	sFilterRuleEntry.rule.rt_tbl_hdl = nTableHdl; //put here the handle corresponding to Routing Rule 1
	sFilterRuleEntry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // Destination IP Based Filtering
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
	if (
			((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
			!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
	{
		LOG_MSG_ERROR ("Adding Rule (0) to Filtering block Failed.");
		bRetVal = false;
		goto bail;
	} else
	{
		LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,cFilterTable0.ReadRuleFromTable(0)->status);
	}

bail:
	LOG_MSG_STACK(
			"Leaving Function (Returning %s)", bRetVal?"True":"False");
	return bRetVal;
} // AddRules()

bool RNDISAggregationTestFixture::AddRulesDeAggEther() {
	m_eIP = IPA_IP_v4;
	const char aBypass[20] = "Bypass1";
	bool bRetVal = true;
	IPAFilteringTable cFilterTable0;
	struct ipa_flt_rule_add sFilterRuleEntry;
	uint32_t nTableHdl;


	LOG_MSG_STACK("Entering Function");
	memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));

	if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass, IPA_CLIENT_TEST3_CONS,
			0, &nTableHdl)) {
		LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
		bRetVal = false;
		goto bail;
	}


	LOG_MSG_INFO("Creation of bypass routing table completed successfully");

	// Creating Filtering Rules
	cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST2_PROD, true, 1);
	LOG_MSG_INFO("Creation of filtering table completed successfully");

	// Configuring Filtering Rule No.1
	cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
	sFilterRuleEntry.at_rear = true;
	sFilterRuleEntry.flt_rule_hdl=-1; // return Value
	sFilterRuleEntry.status = -1; // return value
	sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
	sFilterRuleEntry.rule.retain_hdr = true;
	sFilterRuleEntry.rule.rt_tbl_hdl = nTableHdl; //put here the handle corresponding to Routing Rule 1
	sFilterRuleEntry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // Destination IP Based Filtering
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
	if (
			((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
			!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
	{
		LOG_MSG_ERROR ("Adding Rule (0) to Filtering block Failed.");
		bRetVal = false;
		goto bail;
	} else
	{
		LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,cFilterTable0.ReadRuleFromTable(0)->status);
	}

bail:
	LOG_MSG_STACK(
			"Leaving Function (Returning %s)", bRetVal?"True":"False");
	return bRetVal;
} // AddRules()

bool RNDISAggregationTestFixture::AddRulesAggTimeLimit() {
	m_eIP = IPA_IP_v4;
	const char aBypass[20] = "Bypass1";
	bool bRetVal = true;
	IPAFilteringTable cFilterTable0;
	struct ipa_flt_rule_add sFilterRuleEntry;
	struct ipa_ioc_get_hdr sGetHeader;
	uint32_t nTableHdl;
	struct RndisEtherHeader rndisEtherHeader;


	LOG_MSG_STACK("Entering Function");
	memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));
	memset(&sGetHeader, 0, sizeof(sGetHeader));
	memset(&rndisEtherHeader, 0, sizeof(struct RndisEtherHeader));
	rndisEtherHeader.rndisHeader.MessageType = 0x01;
	rndisEtherHeader.rndisHeader.DataOffset = 0x24;
	memcpy(&rndisEtherHeader.etherHeader, Eth2Helper::m_ETH2_IP4_HDR, sizeof(struct ethhdr));

	// Create Header:
	// Allocate Memory, populate it, and add in to the Header Insertion.
	struct ipa_ioc_add_hdr * pHeaderDescriptor = NULL;
	pHeaderDescriptor = (struct ipa_ioc_add_hdr *) calloc(1,
		sizeof(struct ipa_ioc_add_hdr)
		+ 1 * sizeof(struct ipa_hdr_add));
	if (!pHeaderDescriptor) {
		LOG_MSG_ERROR("calloc failed to allocate pHeaderDescriptor");
		bRetVal = false;
		goto bail;
	}

	pHeaderDescriptor->commit = true;
	pHeaderDescriptor->num_hdrs = 1;
	// Adding Header No1.
	strcpy(pHeaderDescriptor->hdr[0].name, "RndisEthernet\0"); // Header's Name
	memcpy(pHeaderDescriptor->hdr[0].hdr, (void*)&rndisEtherHeader,
		sizeof(struct RndisEtherHeader)); //Header's Data
	pHeaderDescriptor->hdr[0].hdr_len    = sizeof(struct RndisEtherHeader);
	pHeaderDescriptor->hdr[0].hdr_hdl    = -1; //Return Value
	pHeaderDescriptor->hdr[0].is_partial = false;
	pHeaderDescriptor->hdr[0].status     = -1; // Return Parameter

	strcpy(sGetHeader.name, pHeaderDescriptor->hdr[0].name);


	if (!m_HeaderInsertion.AddHeader(pHeaderDescriptor))
	{
		LOG_MSG_ERROR("m_HeaderInsertion.AddHeader(pHeaderDescriptor) Failed.");
		bRetVal = false;
		goto bail;
	}

	if (!m_HeaderInsertion.GetHeaderHandle(&sGetHeader))
	{
		LOG_MSG_ERROR(" Failed");
		bRetVal = false;
		goto bail;
	}
	LOG_MSG_DEBUG("Received Header Handle = 0x%x", sGetHeader.hdl);


	if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass, IPA_CLIENT_TEST_CONS,
			sGetHeader.hdl, &nTableHdl)) {
		LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
		bRetVal = false;
		goto bail;
	}


	LOG_MSG_INFO("Creation of bypass routing table completed successfully");

	// Creating Filtering Rules
	cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST3_PROD, true, 1);
	LOG_MSG_INFO("Creation of filtering table completed successfully");

	// Configuring Filtering Rule No.1
	cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
	sFilterRuleEntry.at_rear = true;
	sFilterRuleEntry.flt_rule_hdl=-1; // return Value
	sFilterRuleEntry.status = -1; // return value
	sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
	sFilterRuleEntry.rule.rt_tbl_hdl = nTableHdl; //put here the handle corresponding to Routing Rule 1
	sFilterRuleEntry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // Destination IP Based Filtering
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
	if (
			((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
			!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
	{
		LOG_MSG_ERROR ("Adding Rule (0) to Filtering block Failed.");
		bRetVal = false;
		goto bail;
	} else
	{
		LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,cFilterTable0.ReadRuleFromTable(0)->status);
	}

bail:
	Free(pHeaderDescriptor);
	LOG_MSG_STACK(
			"Leaving Function (Returning %s)", bRetVal?"True":"False");
	return bRetVal;
} // AddRules()

bool RNDISAggregationTestFixture::AddRulesAggByteLimit() {
	m_eIP = IPA_IP_v4;
	const char aBypass[20] = "Bypass1";
	bool bRetVal = true;
	IPAFilteringTable cFilterTable0;
	struct ipa_flt_rule_add sFilterRuleEntry;
	struct ipa_ioc_get_hdr sGetHeader;
	uint32_t nTableHdl;
	struct RndisEtherHeader rndisEtherHeader;


	LOG_MSG_STACK("Entering Function");
	memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));
	memset(&sGetHeader, 0, sizeof(sGetHeader));
	memset(&rndisEtherHeader, 0, sizeof(struct RndisEtherHeader));

	rndisEtherHeader.rndisHeader.MessageType = 0x01;
	rndisEtherHeader.rndisHeader.DataOffset = 0x24;
	memcpy(&rndisEtherHeader.etherHeader, Eth2Helper::m_ETH2_IP4_HDR, sizeof(struct ethhdr));

	// Create Header:
	// Allocate Memory, populate it, and add in to the Header Insertion.
	struct ipa_ioc_add_hdr * pHeaderDescriptor = NULL;
	pHeaderDescriptor = (struct ipa_ioc_add_hdr *) calloc(1,
		sizeof(struct ipa_ioc_add_hdr)
		+ 1 * sizeof(struct ipa_hdr_add));
	if (!pHeaderDescriptor) {
		LOG_MSG_ERROR("calloc failed to allocate pHeaderDescriptor");
		bRetVal = false;
		goto bail;
	}

	pHeaderDescriptor->commit = true;
	pHeaderDescriptor->num_hdrs = 1;
	// Adding Header No1.
	strcpy(pHeaderDescriptor->hdr[0].name, "RndisEthernet\0"); // Header's Name
	memcpy(pHeaderDescriptor->hdr[0].hdr, (void*)&rndisEtherHeader,
		sizeof(struct RndisEtherHeader)); //Header's Data
	pHeaderDescriptor->hdr[0].hdr_len    = sizeof(struct RndisEtherHeader);
	pHeaderDescriptor->hdr[0].hdr_hdl    = -1; //Return Value
	pHeaderDescriptor->hdr[0].is_partial = false;
	pHeaderDescriptor->hdr[0].status     = -1; // Return Parameter

	strcpy(sGetHeader.name, pHeaderDescriptor->hdr[0].name);


	if (!m_HeaderInsertion.AddHeader(pHeaderDescriptor))
	{
		LOG_MSG_ERROR("m_HeaderInsertion.AddHeader(pHeaderDescriptor) Failed.");
		bRetVal = false;
		goto bail;
	}

	if (!m_HeaderInsertion.GetHeaderHandle(&sGetHeader))
	{
		LOG_MSG_ERROR(" Failed");
		bRetVal = false;
		goto bail;
	}
	LOG_MSG_DEBUG("Received Header Handle = 0x%x", sGetHeader.hdl);


	if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass, IPA_CLIENT_TEST2_CONS,
			sGetHeader.hdl, &nTableHdl)) {
		LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
		bRetVal = false;
		goto bail;
	}


	LOG_MSG_INFO("Creation of bypass routing table completed successfully");

	// Creating Filtering Rules
	cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST3_PROD, true, 1);
	LOG_MSG_INFO("Creation of filtering table completed successfully");

	// Configuring Filtering Rule No.1
	cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
	sFilterRuleEntry.at_rear = true;
	sFilterRuleEntry.flt_rule_hdl=-1; // return Value
	sFilterRuleEntry.status = -1; // return value
	sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
	sFilterRuleEntry.rule.rt_tbl_hdl = nTableHdl; //put here the handle corresponding to Routing Rule 1
	sFilterRuleEntry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // Destination IP Based Filtering
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
	if (
			((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
			!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
	{
		LOG_MSG_ERROR ("Adding Rule (0) to Filtering block Failed.");
		bRetVal = false;
		goto bail;
	} else
	{
		LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,cFilterTable0.ReadRuleFromTable(0)->status);
	}

bail:
	Free(pHeaderDescriptor);
	LOG_MSG_STACK(
			"Leaving Function (Returning %s)", bRetVal?"True":"False");
	return bRetVal;
} // AddRules()

bool RNDISAggregationTestFixture::AddRulesAggPacketLimit() {
	m_eIP = IPA_IP_v4;
	const char aBypass[20] = "Bypass1";
	bool bRetVal = true;
	IPAFilteringTable cFilterTable0;
	struct ipa_flt_rule_add sFilterRuleEntry;
	struct ipa_ioc_get_hdr sGetHeader;
	uint32_t nTableHdl;
	struct RndisEtherHeader rndisEtherHeader;


	LOG_MSG_STACK("Entering Function");
	memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));
	memset(&sGetHeader, 0, sizeof(sGetHeader));
	memset(&rndisEtherHeader, 0, sizeof(struct RndisEtherHeader));

	rndisEtherHeader.rndisHeader.MessageType = 0x01;
	rndisEtherHeader.rndisHeader.DataOffset = 0x24;
	memcpy(&rndisEtherHeader.etherHeader, Eth2Helper::m_ETH2_IP4_HDR, sizeof(struct ethhdr));

	// Create Header:
	// Allocate Memory, populate it, and add in to the Header Insertion.
	struct ipa_ioc_add_hdr * pHeaderDescriptor = NULL;
	pHeaderDescriptor = (struct ipa_ioc_add_hdr *) calloc(1,
		sizeof(struct ipa_ioc_add_hdr)
		+ 1 * sizeof(struct ipa_hdr_add));
	if (!pHeaderDescriptor) {
		LOG_MSG_ERROR("calloc failed to allocate pHeaderDescriptor");
		bRetVal = false;
		goto bail;
	}

	pHeaderDescriptor->commit = true;
	pHeaderDescriptor->num_hdrs = 1;
	// Adding Header No1.
	strcpy(pHeaderDescriptor->hdr[0].name, "RndisEthernet\0"); // Header's Name
	memcpy(pHeaderDescriptor->hdr[0].hdr, (void*)&rndisEtherHeader,
		sizeof(struct RndisEtherHeader)); //Header's Data
	pHeaderDescriptor->hdr[0].hdr_len    = sizeof(struct RndisEtherHeader);
	pHeaderDescriptor->hdr[0].hdr_hdl    = -1; //Return Value
	pHeaderDescriptor->hdr[0].is_partial = false;
	pHeaderDescriptor->hdr[0].status     = -1; // Return Parameter

	strcpy(sGetHeader.name, pHeaderDescriptor->hdr[0].name);


	if (!m_HeaderInsertion.AddHeader(pHeaderDescriptor))
	{
		LOG_MSG_ERROR("m_HeaderInsertion.AddHeader(pHeaderDescriptor) Failed.");
		bRetVal = false;
		goto bail;
	}

	if (!m_HeaderInsertion.GetHeaderHandle(&sGetHeader))
	{
		LOG_MSG_ERROR(" Failed");
		bRetVal = false;
		goto bail;
	}
	LOG_MSG_DEBUG("Received Header Handle = 0x%x", sGetHeader.hdl);


	if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass, IPA_CLIENT_TEST4_CONS,
		sGetHeader.hdl, &nTableHdl)) {
			LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
			bRetVal = false;
			goto bail;
	}


	LOG_MSG_INFO("Creation of bypass routing table completed successfully");

	// Creating Filtering Rules
	cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST3_PROD, true, 1);
	LOG_MSG_INFO("Creation of filtering table completed successfully");

	// Configuring Filtering Rule No.1
	cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
	sFilterRuleEntry.at_rear = true;
	sFilterRuleEntry.flt_rule_hdl=-1; // return Value
	sFilterRuleEntry.status = -1; // return value
	sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
	sFilterRuleEntry.rule.rt_tbl_hdl = nTableHdl; //put here the handle corresponding to Routing Rule 1
	sFilterRuleEntry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // Destination IP Based Filtering
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
	sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
	if (
		((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
		!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
		)
	{
		LOG_MSG_ERROR ("Adding Rule (0) to Filtering block Failed.");
		bRetVal = false;
		goto bail;
	} else
	{
		LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n", cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,cFilterTable0.ReadRuleFromTable(0)->status);
	}

bail:
	Free(pHeaderDescriptor);
	LOG_MSG_STACK(
		"Leaving Function (Returning %s)", bRetVal?"True":"False");
	return bRetVal;
} // AddRules()


