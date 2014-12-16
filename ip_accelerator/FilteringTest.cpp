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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <cstring> // for memcpy
#include "hton.h" // for htonl
#include "InterfaceAbstraction.h"
#include "Constants.h"
#include "Logger.h"
#include "TestsUtils.h"
#include "Filtering.h"
#include "RoutingDriverWrapper.h"
#include "IPAFilteringTable.h"

//TODO Add Enum for IP/TCP/UDP Fields

#define IP4_TOS_FIELD_OFFSET (1)
#define IPV4_PROTOCOL_OFFSET (9)
#define IPV6_NEXT_HDR_OFFSET (6)
#define IPV4_DST_ADDR_OFFSET (16)
#define IPV4_SRC_PORT_OFFSET (20)
#define IPV4_DST_PORT_OFFSET (20+2)
#define IPV6_SRC_PORT_OFFSET (40)
#define IPV6_DST_PORT_OFFSET (40+2)

#define DST_ADDR_LSB_OFFSET_IPV4 (19)
#define DST_ADDR_LSB_OFFSET_IPV6 (39)

#define IPV4_FRAGMENT_FLAGS_OFFSET (6)
#define IPV6_FRAGMENT_FLAGS_OFFSET (42)
#define IPV6_FRAGMENT_NEXT_HDR_OFFSET (40)


class IpaFilteringBlockTestFixture : public TestBase
{
public:

	IpaFilteringBlockTestFixture():
		m_sendSize (BUFF_MAX_SIZE),
		m_sendSize2 (BUFF_MAX_SIZE),
		m_sendSize3 (BUFF_MAX_SIZE),
		m_IpaIPType(IPA_IP_v4),
		m_extHdrType(NONE)
	{
		memset(m_sendBuffer, 0, sizeof(m_sendBuffer));	// First input file / IP packet
		memset(m_sendBuffer2, 0, sizeof(m_sendBuffer2));	// Second input file / IP packet
		memset(m_sendBuffer3, 0, sizeof(m_sendBuffer3));	// Third input file (default) / IP packet
		m_testSuiteName.push_back("Filtering");
	}

	bool Setup()
	{
		ConfigureScenario(PHASE_TWO_TEST_CONFIGURATION);

		m_producer.Open(INTERFACE0_TO_IPA_DATA_PATH, INTERFACE0_FROM_IPA_DATA_PATH);

		m_consumer.Open(INTERFACE1_TO_IPA_DATA_PATH, INTERFACE1_FROM_IPA_DATA_PATH);
		m_consumer2.Open(INTERFACE2_TO_IPA_DATA_PATH, INTERFACE2_FROM_IPA_DATA_PATH);
		m_defaultConsumer.Open(INTERFACE3_TO_IPA_DATA_PATH, INTERFACE3_FROM_IPA_DATA_PATH);

		if (!m_routing.DeviceNodeIsOpened())
		{
			printf("Routing block is not ready for immediate commands!\n");
			return false;
		}

		if (!m_filtering.DeviceNodeIsOpened())
		{
			printf("Filtering block is not ready for immediate commands!\n");
			return false;
		}
		m_routing.Reset(IPA_IP_v4); // This will issue a Reset command to the Filtering as well
		m_routing.Reset(IPA_IP_v6); // This will issue a Reset command to the Filtering as well
		return true;
	} // Setup()

	bool Teardown()
	{
		m_producer.Close();
		m_consumer.Close();
		m_consumer2.Close();
		m_defaultConsumer.Close();
		return true;
	} // Teardown()

	bool LoadFiles(enum ipa_ip_type ip)
	{
		string fileName;

		if (IPA_IP_v4 == ip) {
			fileName = "Input/IPv4_1";
		} else {
			fileName = "Input/IPv6";
		}

		if (!LoadDefaultPacket(ip, m_extHdrType, m_sendBuffer, m_sendSize)) {
			LOG_MSG_ERROR("Failed default Packet\n");
			return false;
		}
		printf ("Loaded %zu Bytes to Buffer 1\n",m_sendSize);

		if (!LoadDefaultPacket(ip, m_extHdrType, m_sendBuffer2, m_sendSize2)) {
			LOG_MSG_ERROR("Failed default Packet\n");
			return false;
		}
		printf ("Loaded %zu Bytes to Buffer 2\n",m_sendSize2);

		if (!LoadDefaultPacket(ip, m_extHdrType, m_sendBuffer3, m_sendSize3)) {
			LOG_MSG_ERROR("Failed default Packet\n");
			return false;
		}
		printf ("Loaded %zu Bytes to Buffer 3\n",m_sendSize3);

		return true;
	}

	bool ReceivePacketAndCompareFrom(InterfaceAbstraction& cons, Byte* send, size_t send_sz, InterfaceAbstraction& excp_cons)
	{
		size_t receivedSize = 0;
		bool isSuccess = true;

		// Receive results
		Byte *rxBuff1 = new Byte[0x400];

		if (NULL == rxBuff1)
		{
			printf("Memory allocation error.\n");
			return false;
		}

		receivedSize = cons.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, cons.m_fromChannelName.c_str());

		// Compare results
		isSuccess &= CompareResultVsGolden(send, send_sz, rxBuff1, receivedSize);

		char recievedBuffer[256] = {0};
		char SentBuffer[256] = {0};
//		char * p = recievedBuffer;
		size_t j;
		for(j = 0; j < m_sendSize; j++)
			sprintf(&SentBuffer[3*j], " %02X", send[j]);
		for(j = 0; j < receivedSize; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff1[j]);
		printf("Expected Value (%zu)\n%s\n, Received Value1(%zu)\n%s\n",send_sz,SentBuffer,receivedSize,recievedBuffer);

		delete[] rxBuff1;

		receivedSize = excp_cons.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, excp_cons.m_fromChannelName.c_str());

		return isSuccess;
	}

	virtual bool ReceivePacketsAndCompare()
	{
		size_t receivedSize = 0;
		size_t receivedSize2 = 0;
		size_t receivedSize3 = 0;
		bool isSuccess = true;

		// Receive results
		Byte *rxBuff1 = new Byte[0x400];
		Byte *rxBuff2 = new Byte[0x400];
		Byte *rxBuff3 = new Byte[0x400];

		if (NULL == rxBuff1 || NULL == rxBuff2 || NULL == rxBuff3)
		{
			printf("Memory allocation error.\n");
			return false;
		}

		receivedSize = m_consumer.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, m_consumer.m_fromChannelName.c_str());

		receivedSize2 = m_consumer2.ReceiveData(rxBuff2, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize2, m_consumer2.m_fromChannelName.c_str());

		receivedSize3 = m_defaultConsumer.ReceiveData(rxBuff3, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize3, m_defaultConsumer.m_fromChannelName.c_str());

		// Compare results
		if (!CompareResultVsGolden(m_sendBuffer, m_sendSize, rxBuff1, receivedSize))
		{
			printf("Comparison of Buffer0 Failed!\n");
			isSuccess = false;
		}

		char recievedBuffer[256] = {0};
		char SentBuffer[256] = {0};
//		char * p = recievedBuffer;
		size_t j;
		for(j = 0; j < m_sendSize; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer[j]);
		for(j = 0; j < receivedSize; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff1[j]);
		printf("Expected Value1 (%zu)\n%s\n, Received Value1(%zu)\n%s\n",m_sendSize,SentBuffer,receivedSize,recievedBuffer);

		for(j = 0; j < m_sendSize2; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer2[j]);
		for(j = 0; j < receivedSize2; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff2[j]);
		printf("Expected Value2 (%zu)\n%s\n, Received Value2(%zu)\n%s\n",m_sendSize2,SentBuffer,receivedSize2,recievedBuffer);

		for(j = 0; j < m_sendSize3; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer3[j]);
		for(j = 0; j < receivedSize3; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff3[j]);
		printf("Expected Value3 (%zu)\n%s\n, Received Value3(%zu)\n%s\n",m_sendSize3,SentBuffer,receivedSize3,recievedBuffer);

		isSuccess &= CompareResultVsGolden(m_sendBuffer2, m_sendSize2, rxBuff2, receivedSize2);
		isSuccess &= CompareResultVsGolden(m_sendBuffer3, m_sendSize3, rxBuff3, receivedSize3);

		delete[] rxBuff1;
		delete[] rxBuff2;
		delete[] rxBuff3;

		return isSuccess;
	}

	// This function creates three IPv4 bypass routing entries and commits them.
	bool CreateThreeIPv4BypassRoutingTables (const char * bypass0, const char * bypass1, const char * bypass2)
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		struct ipa_ioc_add_rt_rule *rt_rule0 = 0, *rt_rule1 = 0,*rt_rule2 = 0;
		struct ipa_rt_rule_add *rt_rule_entry;

		rt_rule0 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule0) {
			printf("calloc failed to allocate rt_rule0 in %s\n",__FUNCTION__);
			return false;
		}
		rt_rule1 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule1) {
			printf("calloc failed to allocate rt_rule1 in %s\n",__FUNCTION__);
			Free(rt_rule0);
			return false;
		}
		rt_rule2 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule2) {
			printf("calloc failed to allocate rt_rule2 in %s\n",__FUNCTION__);
			Free(rt_rule0);
			Free(rt_rule1);
			return false;
		}

		rt_rule0->num_rules = 1;
		rt_rule0->ip = IPA_IP_v4;
		rt_rule0->commit = true;
		strcpy (rt_rule0->rt_tbl_name, bypass0);

		rt_rule_entry = &rt_rule0->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;//Setting
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0x00000000;// All Packets will get a "Hit"
		if (false == m_routing.AddRoutingRule(rt_rule0))
		{
			printf("Routing rule addition(rt_rule0) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		rt_rule1->num_rules = 1;
		rt_rule1->ip = IPA_IP_v4;
		rt_rule1->commit = true;
		strcpy (rt_rule1->rt_tbl_name, bypass1);
		rt_rule_entry = &rt_rule1->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0x00000000;// All Packets will get a "Hit"
		if (false == m_routing.AddRoutingRule(rt_rule1))
		{
			printf("Routing rule addition(rt_rule1) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		rt_rule2->num_rules = 1;
		rt_rule2->ip = IPA_IP_v4;
		rt_rule2->commit = true;
		strcpy (rt_rule2->rt_tbl_name, bypass2);
		rt_rule_entry = &rt_rule2->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0x00000000;// All Packets will get a "Hit"
		if (false == m_routing.AddRoutingRule(rt_rule2))
		{
			printf("Routing rule addition(rt_rule2) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		Free (rt_rule2);
		Free (rt_rule1);
		Free (rt_rule0);
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}

	// This function creates three IPv6 bypass routing entries and commits them.
	bool CreateThreeIPv6BypassRoutingTables (const char * bypass0, const char * bypass1, const char * bypass2)
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		struct ipa_ioc_add_rt_rule *rt_rule0 = 0, *rt_rule1 = 0,*rt_rule2 = 0;
		struct ipa_rt_rule_add *rt_rule_entry;

		rt_rule0 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule0) {
			printf("calloc failed to allocate rt_rule0 in %s\n",__FUNCTION__);
			return false;
		}
		rt_rule1 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule1) {
			printf("calloc failed to allocate rt_rule1 in %s\n",__FUNCTION__);
			Free(rt_rule0);
			return false;
		}
		rt_rule2 = (struct ipa_ioc_add_rt_rule *)
			calloc(1,
					sizeof(struct ipa_ioc_add_rt_rule) +
						1*sizeof(struct ipa_rt_rule_add)
				);
		if(!rt_rule2) {
			printf("calloc failed to allocate rt_rule2 in %s\n",__FUNCTION__);
			Free(rt_rule0);
			Free(rt_rule1);
			return false;
		}

		rt_rule0->num_rules = 1;
		rt_rule0->ip = IPA_IP_v6;
		rt_rule0->commit = true;
		strcpy (rt_rule0->rt_tbl_name, bypass0);

		rt_rule_entry = &rt_rule0->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;//Setting
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = 0xeeff0011;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = 0x22334455;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = 0x66778899;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0x00000000;// All Packets will get a "Hit"
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		if (false == m_routing.AddRoutingRule(rt_rule0))
		{
			printf("Routing rule addition(rt_rule0) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		rt_rule1->num_rules = 1;
		rt_rule1->ip = IPA_IP_v6;
		rt_rule1->commit = true;
		strcpy (rt_rule1->rt_tbl_name, bypass1);
		rt_rule_entry = &rt_rule1->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = 0xeeff0011;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = 0x22334455;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = 0x66778899;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0x00000000;// All Packets will get a "Hit"
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		if (false == m_routing.AddRoutingRule(rt_rule1))
		{
			printf("Routing rule addition(rt_rule1) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		rt_rule2->num_rules = 1;
		rt_rule2->ip = IPA_IP_v6;
		rt_rule2->commit = true;
		strcpy (rt_rule2->rt_tbl_name, bypass2);
		rt_rule_entry = &rt_rule2->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO Header Inserion - gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0] = 0xaabbccdd;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1] = 0xeeff0011;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2] = 0x22334455;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3] = 0x66778899;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0x00000000;// All Packets will get a "Hit"
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		if (false == m_routing.AddRoutingRule(rt_rule2))
		{
			printf("Routing rule addition(rt_rule2) failed!\n");
			Free (rt_rule2);
			Free (rt_rule1);
			Free (rt_rule0);
			return false;
		}


		Free (rt_rule2);
		Free (rt_rule1);
		Free (rt_rule0);
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}

	virtual bool ModifyPackets() = 0;
	virtual bool AddRules() = 0;

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		// Add the relevant filtering rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding filtering rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(m_IpaIPType);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		res = ModifyPackets();
		if (false == res) {
			printf("Failed to modify packets.\n");
			return false;
		}

		// Send first packet
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize2);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send third packet
		isSuccess = m_producer.SendData(m_sendBuffer3, m_sendSize3);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Receive packets from the channels and compare results
		isSuccess = ReceivePacketsAndCompare();

		printf("Leaving %s, %s(), Returning %d\n",__FUNCTION__, __FILE__,isSuccess);

		return isSuccess;
	} // Run()


	~IpaFilteringBlockTestFixture()
	{
//		Free(m_sendBuffer);
//		Free(m_sendBuffer2);
//		Free(m_sendBuffer3);
		m_sendSize = 0;
		m_sendSize2 = 0;
		m_sendSize3 = 0;
	}

	static Filtering m_filtering;
	static RoutingDriverWrapper m_routing;
	InterfaceAbstraction m_producer;
	InterfaceAbstraction m_consumer;
	InterfaceAbstraction m_consumer2;
	InterfaceAbstraction m_defaultConsumer;

	static const size_t BUFF_MAX_SIZE = 1024;

	Byte m_sendBuffer[BUFF_MAX_SIZE];	// First input file / IP packet
	Byte m_sendBuffer2[BUFF_MAX_SIZE];	// Second input file / IP packet
	Byte m_sendBuffer3[BUFF_MAX_SIZE];	// Third input file (default) / IP packet
	size_t m_sendSize;
	size_t m_sendSize2;
	size_t m_sendSize3;
	enum ipa_ip_type m_IpaIPType;
	enum ipv6_ext_hdr_type m_extHdrType;

private:
};

RoutingDriverWrapper IpaFilteringBlockTestFixture::m_routing;
Filtering IpaFilteringBlockTestFixture::m_filtering;


/*---------------------------------------------------------------------------*/
/* Test001: Destination IP address and subnet mask match against LAN subnet  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest001 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest001()
	{
		m_name = "IpaFilteringBlockTest001";
		m_description =
		"Filtering block test 001 - Destination IP address and subnet mask match against LAN subnet (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules: (DST & Mask Match). \
			All DST_IP == (127.0.0.1 & 255.0.0.255)traffic goes to routing table 0 \
			All DST_IP == (192.169.1.1 & 255.0.0.255)traffic goes to routing table 1 \
			All DST_IP == (192.169.1.2 & 255.0.0.255)traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,3);
		printf("FilterTable*.Init Completed Successfully..\n");

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;

	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};

/*---------------------------------------------------------------------------*/
/* Test002: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest002 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest002()
	{
		m_name = "IpaFilteringBlockTest002";
		m_description =
		"Filtering block test 002 - Destination IP address exact match against broadcast IP address (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules (MASK = 0xFF..FF). \
			All DST_IP == 127.0.0.1 traffic goes to routing table 0 \
			All DST_IP == 192.169.1.1 traffic goes to routing table 1 \
			All DST_IP == 192.169.1.2 traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF; // Exact Match
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};



/*---------------------------------------------------------------------------*/
/* Test003: Destination UDP port exact match against DHCP port  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest003 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest003()
	{
		m_name = "IpaFilteringBlockTest003";
		m_description =
			"Filtering block test 003 - Destination UDP port exact match against DHCP port (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All DST_UDP_PORT == 546 (DHCP Client)traffic goes to routing table 0 \
			All DST_UDP_PORT == 547 (DHCP Server) traffic goes to routing table 1 \
			All DST_UDP_PORT == 500 (Non DHCP) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_PORT;
		flt_rule_entry.rule.attrib.dst_port = 546; // DHCP Client Port No 546

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 547; // DHCP Server Port No 547
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 500; // Non-DHCP Port

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Port should be switched to Network Mode.
		port = ntohs(546);//DHCP Client Port
		memcpy (&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(547);//DHCP Server Port
		memcpy (&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(500);//Non - DHCP Port
		memcpy (&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		return true;
	}// ModifyPacktes ()
};

/*------------------------------------------------------------------------------*/
/* Test004: Firewall filtering rules based on source and destination port ranges  */
/*------------------------------------------------------------------------------*/
class IpaFilteringBlockTest004 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest004()
	{
		m_name = "IpaFilteringBlockTest004";
		m_description =
		"Filtering block test 004 - Firewall filtering rules based on source and destination port ranges (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All (5 >= SRC_PORT_RANGE >= 15) & (50 >= DST_PORT_RANGE >= 150) traffic goes to routing table 0 \
			All (15 >= SRC_PORT_RANGE >= 25) & (150 >= DST_PORT_RANGE >= 250) traffic goes to routing table 1 \
			All (25 >= SRC_PORT_RANGE >= 35) & (250 >= DST_PORT_RANGE >= 350) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT_RANGE | IPA_FLT_DST_PORT_RANGE;
		//TODO: Fix from here.....
		flt_rule_entry.rule.attrib.src_port_lo =5;
		flt_rule_entry.rule.attrib.src_port_hi =15;
		flt_rule_entry.rule.attrib.dst_port_lo =50;
		flt_rule_entry.rule.attrib.dst_port_hi =150;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 15;
		flt_rule_entry.rule.attrib.src_port_hi = 25;
		flt_rule_entry.rule.attrib.dst_port_lo = 150;
		flt_rule_entry.rule.attrib.dst_port_hi = 250;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 25;
		flt_rule_entry.rule.attrib.src_port_hi = 35;
		flt_rule_entry.rule.attrib.dst_port_lo = 250;
		flt_rule_entry.rule.attrib.dst_port_hi = 350;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;

		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		port = htons(10);
		memcpy(&m_sendBuffer[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(100);
		memcpy(&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(20);
		memcpy(&m_sendBuffer2[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(200);
		memcpy(&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(30);
		memcpy(&m_sendBuffer3[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(300);
		memcpy(&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};
/*---------------------------------------------------------------------------*/
/* Test005: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest005 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest005()
	{
		m_name = "IpaFilteringBlockTest005";
		m_description =
		"Filtering block test 005 - Filtering Based on Protocol type (TCP/UDP/ICMP) (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All UDP traffic goes to routing table 0 \
			All TCP traffic goes to routing table 1 \
			All ICMP traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_PROTOCOL;
		flt_rule_entry.rule.attrib.u.v4.protocol = 17; // Filter only UDP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 6; // Filter only TCP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 1; // Filter only ICMP Packets.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		m_sendBuffer[IPV4_PROTOCOL_OFFSET] = 0x11;// UDP 0x11 = 17
		m_sendBuffer2[IPV4_PROTOCOL_OFFSET] = 0x06;// TCP 0x06 = 6
		m_sendBuffer3[IPV4_PROTOCOL_OFFSET] = 0x01;// ICMP 0x01 = 1
		return true;
	}// ModifyPacktes ()
};


/*---------------------------------------------------------------------------*/
/* Test006: Destination IP address and subnet mask match against LAN subnet  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest006 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest006()
	{
		m_name = "IpaFilteringBlockTest006";
		m_description =
		"Filtering block test 006 - Destination IP address and subnet mask match against LAN subnet (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules: (DST & Mask Match). \
			All DST_IP == (127.0.0.1 & 255.0.0.255)traffic goes to routing table 0 \
			All DST_IP == (192.169.1.1 & 255.0.0.255)traffic goes to routing table 1 \
			All DST_IP == (192.169.1.2 & 255.0.0.255)traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		printf("FilterTable*.Init Completed Successfully..\n");

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
		return true;
	}// ModifyPacktes ()
};

/*---------------------------------------------------------------------------*/
/* Test007: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest007 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest007()
	{
		m_name = "IpaFilteringBlockTest007";
		m_description =
		"Filtering block test 007 - Destination IP address exact match against broadcast IP address (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules (MASK = 0xFF..FF). \
			All DST_IP == 127.0.0.1 traffic goes to routing table 0 \
			All DST_IP == 192.169.1.1 traffic goes to routing table 1 \
			All DST_IP == 192.169.1.2 traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		//		Test Description:
		//		1. Generate and commit three routing tables.
		//			Each table will contain a single "bypass" rule (all data goes to output pipe 0, 1 and 2(accordingly))
		//		2. Generate and commit three filtering rules (each in different Filtering Table)
		//			All Filter DST_IP == 127.0.0.1 traffic goes to routing table 1
		//			All Filter DST_IP == 192.168.1.1 traffic goes to routing table 2
		//			All Filter DST_IP == 192.168.1.2 traffic goes to routing table 3

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF; // Exact Match
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};



/*---------------------------------------------------------------------------*/
/* Test008: Destination UDP port exact match against DHCP port  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest008 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest008()
	{
		m_name = "IpaFilteringBlockTest008";
		m_description =
		"Filtering block test 008 - Destination UDP port exact match against DHCP port (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All DST_UDP_PORT == 546 (DHCP Client)traffic goes to routing table 0 \
			All DST_UDP_PORT == 547 (DHCP Server) traffic goes to routing table 1 \
			All DST_UDP_PORT == 500 (Non DHCP) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit three routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0, 1  and 2 (accordingly))
//		2. Generate and commit Three filtering rules.
//			All DEST_IP == 127.0.0.1 traffic goes to routing table 0
//			All DEST_IP == 192.169.1.1 traffic goes to routing table 1
//			Non Matching traffic goes to routing table 3

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_PORT;
		flt_rule_entry.rule.attrib.dst_port = 546; // DHCP Client Port No 546

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 547; // DHCP Server Port No 547
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 500; // Non-DHCP Port

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Port should be switched to Network Mode.
		port = ntohs(546);//DHCP Client Port
		memcpy (&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(547);//DHCP Server Port
		memcpy (&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(500);//Non - DHCP Port
		memcpy (&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};

/*------------------------------------------------------------------------------*/
/* Test009: Firewall filtering rules based on source and destination port ranges  */
/*------------------------------------------------------------------------------*/
class IpaFilteringBlockTest009 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest009()
	{
		m_name = "IpaFilteringBlockTest009";
		m_description =
		"Filtering block test 009 - Firewall filtering rules based on source and destination port ranges (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All (5 >= SRC_PORT_RANGE >= 15) & (50 >= DST_PORT_RANGE >= 150) traffic goes to routing table 0 \
			All (15 >= SRC_PORT_RANGE >= 25) & (150 >= DST_PORT_RANGE >= 250) traffic goes to routing table 1 \
			All (25 >= SRC_PORT_RANGE >= 35) & (250 >= DST_PORT_RANGE >= 350) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT_RANGE | IPA_FLT_DST_PORT_RANGE;
		//TODO: Fix from here.....
		flt_rule_entry.rule.attrib.src_port_lo =5;
		flt_rule_entry.rule.attrib.src_port_hi =15;
		flt_rule_entry.rule.attrib.dst_port_lo =50;
		flt_rule_entry.rule.attrib.dst_port_hi =150;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 15;
		flt_rule_entry.rule.attrib.src_port_hi = 25;
		flt_rule_entry.rule.attrib.dst_port_lo = 150;
		flt_rule_entry.rule.attrib.dst_port_hi = 250;

		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 25;
		flt_rule_entry.rule.attrib.src_port_hi = 35;
		flt_rule_entry.rule.attrib.dst_port_lo = 250;
		flt_rule_entry.rule.attrib.dst_port_hi = 350;

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;

		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		port = htons(10);
		memcpy(&m_sendBuffer[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(100);
		memcpy(&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(20);
		memcpy(&m_sendBuffer2[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(200);
		memcpy(&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(30);
		memcpy(&m_sendBuffer3[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(300);
		memcpy(&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};
/*---------------------------------------------------------------------------*/
/* Test010: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest010 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest010()
	{
		m_name = "IpaFilteringBlockTest010";
		m_description =
		"Filtering block test 010 - Filtering Based on Protocol type (TCP/UDP/ICMP) (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All UDP traffic goes to routing table 0 \
			All TCP traffic goes to routing table 1 \
			All ICMP traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,true,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_PROTOCOL;
		flt_rule_entry.rule.attrib.u.v4.protocol = 17; // Filter only UDP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 6; // Filter only TCP Packets.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 1; // Filter only ICMP Packets.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		m_sendBuffer[IPV4_PROTOCOL_OFFSET] = 0x11;// UDP 0x11 = 17
		m_sendBuffer2[IPV4_PROTOCOL_OFFSET] = 0x06;// TCP 0x06 = 6
		m_sendBuffer3[IPV4_PROTOCOL_OFFSET] = 0x01;// ICMP 0x01 = 1
		return true;
	}// ModifyPacktes ()
};


/*---------------------------------------------------------------------------*/
/* Test021: Destination IP address and subnet mask match against LAN subnet  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest021 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest021()
	{
		m_name = "IpaFilteringBlockTest021";
		m_description =
		"Filtering block test 021 - Destination IP address and subnet mask match against LAN subnet (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules: (DST & Mask Match). \
			All DST_IP == (127.0.0.1 & 255.0.0.255)traffic goes to routing table 0 \
			All DST_IP == (192.169.1.1 & 255.0.0.255)traffic goes to routing table 1 \
			All DST_IP == (192.169.1.2 & 255.0.0.255)traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);
		printf("FilterTable*.Init Completed Successfully..\n");

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;

	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};

/*---------------------------------------------------------------------------*/
/* Test022: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest022 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest022()
	{
		m_name = "IpaFilteringBlockTest022";
		m_description =
		"Filtering block test 022 - Destination IP address exact match against broadcast IP address (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules (MASK = 0xFF..FF). \
			All DST_IP == 127.0.0.1 traffic goes to routing table 0 \
			All DST_IP == 192.169.1.1 traffic goes to routing table 1 \
			All DST_IP == 192.169.1.2 traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF; // Exact Match
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};



/*---------------------------------------------------------------------------*/
/* Test023: Destination UDP port exact match against DHCP port 				 */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest023 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest023()
	{
		m_name = "IpaFilteringBlockTest023";
		m_description =
		"Filtering block test 023 - Destination UDP port exact match against DHCP port (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All DST_UDP_PORT == 546 (DHCP Client)traffic goes to routing table 0 \
			All DST_UDP_PORT == 547 (DHCP Server) traffic goes to routing table 1 \
			All DST_UDP_PORT == 500 (Non DHCP) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_PORT;
		flt_rule_entry.rule.attrib.dst_port = 546; // DHCP Client Port No 546

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 547; // DHCP Server Port No 547
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 500; // Non-DHCP Port

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Port should be switched to Network Mode.
		port = ntohs(546);//DHCP Client Port
		memcpy (&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(547);//DHCP Server Port
		memcpy (&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(500);//Non - DHCP Port
		memcpy (&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};

/*------------------------------------------------------------------------------*/
/* Test004: Firewall filtering rules based on source and destination port ranges*/
/*------------------------------------------------------------------------------*/
class IpaFilteringBlockTest024 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest024()
	{
		m_name = "IpaFilteringBlockTest024";
		m_description =
		"Filtering block test 024 - Firewall filtering rules based on source and destination port ranges (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All (5 >= SRC_PORT_RANGE >= 15) & (50 >= DST_PORT_RANGE >= 150) traffic goes to routing table 0 \
			All (15 >= SRC_PORT_RANGE >= 25) & (150 >= DST_PORT_RANGE >= 250) traffic goes to routing table 1 \
			All (25 >= SRC_PORT_RANGE >= 35) & (250 >= DST_PORT_RANGE >= 350) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT_RANGE | IPA_FLT_DST_PORT_RANGE;
		//TODO: Fix from here.....
		flt_rule_entry.rule.attrib.src_port_lo =5;
		flt_rule_entry.rule.attrib.src_port_hi =15;
		flt_rule_entry.rule.attrib.dst_port_lo =50;
		flt_rule_entry.rule.attrib.dst_port_hi =150;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 15;
		flt_rule_entry.rule.attrib.src_port_hi = 25;
		flt_rule_entry.rule.attrib.dst_port_lo = 150;
		flt_rule_entry.rule.attrib.dst_port_hi = 250;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 25;
		flt_rule_entry.rule.attrib.src_port_hi = 35;
		flt_rule_entry.rule.attrib.dst_port_lo = 250;
		flt_rule_entry.rule.attrib.dst_port_hi = 350;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;

		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		port = htons(10);
		memcpy(&m_sendBuffer[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(100);
		memcpy(&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(20);
		memcpy(&m_sendBuffer2[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(200);
		memcpy(&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(30);
		memcpy(&m_sendBuffer3[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(300);
		memcpy(&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};
/*---------------------------------------------------------------------------*/
/* Test005: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest025 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest025()
	{
		m_name = "IpaFilteringBlockTest025";
		m_description =
				"Filtering block test 025 - Filtering Based on Protocol type (TCP/UDP/ICMP) (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All UDP traffic goes to routing table 0 \
			All TCP traffic goes to routing table 1 \
			All ICMP traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_PROTOCOL;
		flt_rule_entry.rule.attrib.u.v4.protocol = 17; // Filter only UDP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 6; // Filter only TCP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 1; // Filter only ICMP Packets.

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		m_sendBuffer[IPV4_PROTOCOL_OFFSET] = 0x11;// UDP 0x11 = 17
		m_sendBuffer2[IPV4_PROTOCOL_OFFSET] = 0x06;// TCP 0x06 = 6
		m_sendBuffer3[IPV4_PROTOCOL_OFFSET] = 0x01;// ICMP 0x01 = 1
		return true;
	}// ModifyPacktes ()
};


/*---------------------------------------------------------------------------*/
/* Test006: Destination IP address and subnet mask match against LAN subnet  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest026 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest026()
	{
		m_name = "IpaFilteringBlockTest026";
		m_description =
				"Filtering block test 026 - Destination IP address and subnet mask match against LAN subnet (End-Point specific Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules: (DST & Mask Match). \
			All DST_IP == (127.0.0.1 & 255.0.0.255)traffic goes to routing table 0 \
			All DST_IP == (192.169.1.1 & 255.0.0.255)traffic goes to routing table 1 \
			All DST_IP == (192.169.1.2 & 255.0.0.255)traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		printf("FilterTable*.Init Completed Successfully..\n");

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFF0000FF; // Mask
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2 // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		// TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};

/*---------------------------------------------------------------------------*/
/* Test007: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest027 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest027()
	{
		m_name = "IpaFilteringBlockTest027";
		m_description =
		"Filtering block test 027 - Destination IP address exact match against broadcast IP address (End-Point specific Filtering Table, each rule is added in a Insert using a dedicated single commit) \
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules (MASK = 0xFF..FF). \
			All DST_IP == 127.0.0.1 traffic goes to routing table 0 \
			All DST_IP == 192.169.1.1 traffic goes to routing table 1 \
			All DST_IP == 192.169.1.2 traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		//		Test Description:
		//		1. Generate and commit three routing tables.
		//			Each table will contain a single "bypass" rule (all data goes to output pipe 0, 1 and 2(accordingly))
		//		2. Generate and commit three filtering rules (each in different Filtering Table)
		//			All Filter DST_IP == 127.0.0.1 traffic goes to routing table 1
		//			All Filter DST_IP == 192.168.1.1 traffic goes to routing table 2
		//			All Filter DST_IP == 192.168.1.2 traffic goes to routing table 3

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF; // Exact Match
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0x7F000001; // Filter DST_IP == 127.0.0.1.
		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		int address;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		address = ntohl(0x7F000001);//127.0.0.1
		memcpy(&m_sendBuffer[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80101);//192.168.1.1
		memcpy(&m_sendBuffer2[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));
		address = ntohl(0xC0A80102);//192.168.1.2
		memcpy(&m_sendBuffer3[IPV4_DST_ADDR_OFFSET], &address, sizeof(address));

		return true;
	}// ModifyPacktes ()
};



/*---------------------------------------------------------------------------*/
/* Test008: Destination UDP port exact match against DHCP port  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest028 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest028()
	{
		m_name = "IpaFilteringBlockTest028";
		m_description =
		"Filtering block test 028 - Destination UDP port exact match against DHCP port (End-Point specific Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All DST_UDP_PORT == 546 (DHCP Client)traffic goes to routing table 0 \
			All DST_UDP_PORT == 547 (DHCP Server) traffic goes to routing table 1 \
			All DST_UDP_PORT == 500 (Non DHCP) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit three routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0, 1  and 2 (accordingly))
//		2. Generate and commit Three filtering rules.
//			All DEST_IP == 127.0.0.1 traffic goes to routing table 0
//			All DEST_IP == 192.169.1.1 traffic goes to routing table 1
//			Non Matching traffic goes to routing table 3

		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_PORT;
		flt_rule_entry.rule.attrib.dst_port = 546; // DHCP Client Port No 546

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 547; // DHCP Server Port No 547
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.dst_port = 500; // Non-DHCP Port

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Port should be switched to Network Mode.
		port = ntohs(546);//DHCP Client Port
		memcpy (&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(547);//DHCP Server Port
		memcpy (&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = ntohs(500);//Non - DHCP Port
		memcpy (&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};

/*------------------------------------------------------------------------------*/
/* Test009: Firewall filtering rules based on source and destination port ranges  */
/*------------------------------------------------------------------------------*/
class IpaFilteringBlockTest029 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest029()
	{
		m_name = "IpaFilteringBlockTest029";
		m_description =
		"Filtering block test 029 - Firewall filtering rules based on source and destination port ranges (End-Point specific Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit Three filtering rules . \
			All (5 >= SRC_PORT_RANGE >= 15) & (50 >= DST_PORT_RANGE >= 150) traffic goes to routing table 0 \
			All (15 >= SRC_PORT_RANGE >= 25) & (150 >= DST_PORT_RANGE >= 250) traffic goes to routing table 1 \
			All (25 >= SRC_PORT_RANGE >= 35) & (250 >= DST_PORT_RANGE >= 350) traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT_RANGE | IPA_FLT_DST_PORT_RANGE;
		//TODO: Fix from here.....
		flt_rule_entry.rule.attrib.src_port_lo =5;
		flt_rule_entry.rule.attrib.src_port_hi =15;
		flt_rule_entry.rule.attrib.dst_port_lo =50;
		flt_rule_entry.rule.attrib.dst_port_hi =150;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 15;
		flt_rule_entry.rule.attrib.src_port_hi = 25;
		flt_rule_entry.rule.attrib.dst_port_lo = 150;
		flt_rule_entry.rule.attrib.dst_port_hi = 250;

		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.src_port_lo = 25;
		flt_rule_entry.rule.attrib.src_port_hi = 35;
		flt_rule_entry.rule.attrib.dst_port_lo = 250;
		flt_rule_entry.rule.attrib.dst_port_hi = 350;

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;

		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		port = htons(10);
		memcpy(&m_sendBuffer[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(100);
		memcpy(&m_sendBuffer[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(20);
		memcpy(&m_sendBuffer2[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(200);
		memcpy(&m_sendBuffer2[IPV4_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(30);
		memcpy(&m_sendBuffer3[IPV4_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(300);
		memcpy(&m_sendBuffer3[IPV4_DST_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};
/*---------------------------------------------------------------------------*/
/* Test010: Destination IP address exact match against broadcast IP address  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest030 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest030()
	{
		m_name = "IpaFilteringBlockTest030";
		m_description =
		"Filtering block test 030 - Filtering Based on Protocol type (TCP/UDP/ICMP) (End-Point specific Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All UDP traffic goes to routing table 0 \
			All TCP traffic goes to routing table 1 \
			All ICMP traffic goes to routing table 2";
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
//		Test Description:
//		1. Generate and commit two routing tables.
//			Each table will contain a single "bypass" rule (all data goes to output pipe 0 and 1 (accordingly))
//		2. Generate and commit two filtering rules.
//			All UDP traffic goes to routing table 1
//			All TCP traffic goes to routing table 2
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable1.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);
		FilterTable2.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_PROTOCOL;
		flt_rule_entry.rule.attrib.u.v4.protocol = 17; // Filter only UDP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 6; // Filter only TCP Packets.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v4.protocol = 1; // Filter only ICMP Packets.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		m_sendBuffer[IPV4_PROTOCOL_OFFSET] = 0x11;// UDP 0x11 = 17
		m_sendBuffer2[IPV4_PROTOCOL_OFFSET] = 0x06;// TCP 0x06 = 6
		m_sendBuffer3[IPV4_PROTOCOL_OFFSET] = 0x01;// ICMP 0x01 = 1
		return true;
	}// ModifyPacktes ()
};
/*-------------------------------------------------------------------------------------*/
/* Test031: Filtering Based on fragment extension, End-Point specific Filtering Table  */
/*-------------------------------------------------------------------------------------*/
class IpaFilteringBlockTest031 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest031()
	{
		m_name = "IpaFilteringBlockTest031";
		m_description =
				"Filtering block test 031 - Filtering Based on fragment extension(End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit 2 filtering rules: \
			All fragmented packets goes to routing table 0: \
			Packets with MF flag set & \
			Packets with MF flag set to zero and fragment offset field nonzero \
			All other packets(non fragmented) goes to routing table 1: \
			Packets with MF flag set to zero and fragment offset field zero goes to routing table 1";
		Register(*this);
	}

	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv4BypassRoutingTables(bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v4;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v4;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v4;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v4,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_FRAGMENT;
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		flt_rule_entry.rule.attrib.u.v4.dst_addr = 0xaabbccdd;
		flt_rule_entry.rule.attrib.u.v4.dst_addr_mask = 0x00000000;// All Packets will get a "Hit"

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		m_sendBuffer[IPV4_FRAGMENT_FLAGS_OFFSET] = 0x20;//MF=1
		m_sendBuffer[IPV4_FRAGMENT_FLAGS_OFFSET+1] = 0x0;//MF=1
		m_sendBuffer2[IPV4_FRAGMENT_FLAGS_OFFSET] = 0x0;//MF=0 && frag_off =126
		m_sendBuffer2[IPV4_FRAGMENT_FLAGS_OFFSET+1] = 0x7E;//MF=0 && frag_off =126
		m_sendBuffer3[IPV4_FRAGMENT_FLAGS_OFFSET] = 0x0;// MF=0 && frag_off =0
		m_sendBuffer3[IPV4_FRAGMENT_FLAGS_OFFSET+1] = 0x0;// MF=0 && frag_off =0
		return true;
	}// ModifyPacktes ()

	virtual bool ReceivePacketsAndCompare()
	{
		size_t receivedSize = 0;
		size_t receivedSize2 = 0;
		size_t receivedSize3 = 0;
		bool isSuccess = true;

		// Receive results
		Byte *rxBuff1 = new Byte[0x400];
		Byte *rxBuff2 = new Byte[0x400];
		Byte *rxBuff3 = new Byte[0x400];

		if (NULL == rxBuff1 || NULL == rxBuff2 || NULL == rxBuff3)
		{
			printf("Memory allocation error.\n");
			return false;
		}

		receivedSize = m_consumer.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, m_consumer.m_fromChannelName.c_str());

		receivedSize2 = m_consumer.ReceiveData(rxBuff2, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize2, m_consumer.m_fromChannelName.c_str());

		receivedSize3 = m_consumer2.ReceiveData(rxBuff3, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize3, m_consumer2.m_fromChannelName.c_str());

		// Compare results
		if (!CompareResultVsGolden(m_sendBuffer, m_sendSize, rxBuff1, receivedSize))
		{
			printf("Comparison of Buffer0 Failed!\n");
			isSuccess = false;
		}

		char recievedBuffer[256] = {0};
		char SentBuffer[256] = {0};
//		char * p = recievedBuffer;
		size_t j;
		for(j = 0; j < m_sendSize; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer[j]);
		for(j = 0; j < receivedSize; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff1[j]);
		printf("Expected Value1 (%zu)\n%s\n, Received Value1(%zu)\n%s\n",m_sendSize,SentBuffer,receivedSize,recievedBuffer);

		for(j = 0; j < m_sendSize2; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer2[j]);
		for(j = 0; j < receivedSize2; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff2[j]);
		printf("Expected Value2 (%zu)\n%s\n, Received Value2(%zu)\n%s\n",m_sendSize2,SentBuffer,receivedSize2,recievedBuffer);

		for(j = 0; j < m_sendSize3; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer3[j]);
		for(j = 0; j < receivedSize3; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff3[j]);
		printf("Expected Value3 (%zu)\n%s\n, Received Value3(%zu)\n%s\n",m_sendSize3,SentBuffer,receivedSize3,recievedBuffer);

		isSuccess &= CompareResultVsGolden(m_sendBuffer2, m_sendSize2, rxBuff2, receivedSize2);
		isSuccess &= CompareResultVsGolden(m_sendBuffer3, m_sendSize3, rxBuff3, receivedSize3);

		delete[] rxBuff1;
		delete[] rxBuff2;
		delete[] rxBuff3;

		return isSuccess;
	}
};

/*---------------------------------------------------------------------------------------------*/
/* Test050: Destination IPv6 address and Subnet Mask exact match against broadcast IP address  */
/*---------------------------------------------------------------------------------------------*/
class IpaFilteringBlockTest050 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest050()
	{
		m_name = "IpaFilteringBlockTest050";
		m_description =
		"Filtering block test 050 - Destination IPv6 address and Mask exact match against broadcast IP address (Global Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All DST_IPv6 == 0x...AA traffic goes to routing table 1 \
			All DST_IPv6 == 0x...BB traffic goes to routing table 2 \
			All DST_IPv6 == 0x...CC traffic goes to routing table 3";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv6BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v6;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v6;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v6;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,true,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements

		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x000000FF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0]		 = 0XFF020000; // Filter DST_IP
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1]		 = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2]      = 0x11223344;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677AA;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677BB;
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677CC;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV6] = 0xAA;
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV6] = 0xBB;
		m_sendBuffer3[DST_ADDR_LSB_OFFSET_IPV6] = 0xCC;
		return true;
	}// ModifyPacktes ()
};


/*---------------------------------------------------------------------------------------------*/
/* Test051: Destination IPv6 address and Subnet Mask exact match against broadcast IP address  */
/*---------------------------------------------------------------------------------------------*/
class IpaFilteringBlockTest051 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest051()
	{
		m_name = "IpaFilteringBlockTest051";
		m_description =
		"Filtering block test 051 - Destination IPv6 address and Mask exact match against broadcast IP address (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All DST_IPv6 == 0x...AA traffic goes to routing table 1 \
			All DST_IPv6 == 0x...BB traffic goes to routing table 2 \
			All DST_IPv6 == 0x...CC traffic goes to routing table 3";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv6BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v6;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v6;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v6;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR; // TODO: Fix this, doesn't match the Rule's Requirements

		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x000000FF;// Exact Match
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0]      = 0XFF020000; // Filter DST_IP
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2]      = 0x11223344;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677AA;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677BB;
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3]      = 0X556677CC;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		// TODO: Add verification that we access only allocated addresses
		// TODO: Fix this, doesn't match the Rule's Requirements
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV6] = 0xAA;
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV6] = 0xBB;
		m_sendBuffer3[DST_ADDR_LSB_OFFSET_IPV6] = 0xCC;
		return true;
	}// ModifyPacktes ()
};

/*---------------------------------------------------------------------------*/
/* Test052: IPv6 Filtering Based on Protocol type (TCP/UDP/ICMP)  */
/*---------------------------------------------------------------------------*/
class IpaFilteringBlockTest052 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest052()
	{
		m_name = "IpaFilteringBlockTest052";
		m_description =
		"Filtering block test 052 - Filtering Based on Protocol type (TCP/UDP/ICMP) (Global Filtering Table, each rule is added in a Insert using a dedicated single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules: (DST & Mask Match). \
			All UDP traffic goes to routing table 0 \
			All TCP traffic goes to routing table 1 \
			All ICMP traffic goes to routing table 2";
		m_IpaIPType = IPA_IP_v6;
		m_extHdrType = FRAGMENT;
		m_minIPAHwType = IPA_HW_v2_5;
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv6BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v6;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v6;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v6;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0,FilterTable1,FilterTable2;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable1.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,true,1);
		FilterTable2.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,true,1);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_NEXT_HDR;
		flt_rule_entry.rule.attrib.u.v6.next_hdr = 17; // Filter only UDP Packets.
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v6.next_hdr = 6; // Filter only TCP Packets.
		if (
				((uint8_t)-1 == FilterTable1.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable1.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable1.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable1.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.u.v6.next_hdr = 1; // Filter only ICMP Packets.

		if (
				((uint8_t)-1 == FilterTable2.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable2.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable2.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable2.ReadRuleFromTable(0)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		m_sendBuffer[IPV6_NEXT_HDR_OFFSET] = 0x2C;//FRAGMENT HEADER(44)
		m_sendBuffer[IPV6_FRAGMENT_NEXT_HDR_OFFSET] = 0x11;// UDP 0x11 = 17
		m_sendBuffer2[IPV6_FRAGMENT_NEXT_HDR_OFFSET] = 0x06;// TCP 0x06 = 6
		m_sendBuffer3[IPV6_FRAGMENT_NEXT_HDR_OFFSET] = 0x01;// ICMP 0x01 = 1
		return true;
	}// ModifyPacktes ()
};
/*-------------------------------------------------------------------------------------*/
/* Test053: Filtering Based on fragment extension, End-Point specific Filtering Table  */
/*-------------------------------------------------------------------------------------*/
class IpaFilteringBlockTest053 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest053()
	{
		m_name = "IpaFilteringBlockTest053";
		m_description =
				"Filtering block test 053 - Filtering Based on fragment extension(End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit 2 filtering rules: \
			All fragmented packets goes to routing table 0: \
			Packets with MF flag set & \
			Packets with MF flag set to zero and fragment offset field nonzero \
			All other packets(non fragmented) goes to routing table 1: \
			Packets with MF flag set to zero and fragment offset field zero goes to routing table 1";
		m_IpaIPType = IPA_IP_v6;
		m_extHdrType = FRAGMENT;
		m_minIPAHwType = IPA_HW_v2_5;
		Register(*this);
	}

	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv6BypassRoutingTables(bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v6;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v6;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v6;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_FRAGMENT;
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[0] = 0xaabbccdd;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[1] = 0xeeff0011;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[2] = 0x22334455;
		flt_rule_entry.rule.attrib.u.v6.dst_addr[3] = 0x66778899;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[0] = 0x00000000;// All Packets will get a "Hit"
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[1] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		flt_rule_entry.rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;
		if (
			((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
			!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}
		m_sendBuffer[IPV6_NEXT_HDR_OFFSET] = 0x2C;//FRAGMENT HEADER(44)
		m_sendBuffer[IPV6_FRAGMENT_FLAGS_OFFSET] = 0x00;//MF=1
		m_sendBuffer[IPV6_FRAGMENT_FLAGS_OFFSET+1] = 0x1;//MF=1
		m_sendBuffer2[IPV6_FRAGMENT_FLAGS_OFFSET] = 0x3;//MF=0 && frag_off =126
		m_sendBuffer2[IPV6_FRAGMENT_FLAGS_OFFSET+1] = 0xF0;//MF=0 && frag_off =126
		m_sendBuffer3[IPV6_FRAGMENT_FLAGS_OFFSET] = 0x0;// MF=0 && frag_off =0
		m_sendBuffer3[IPV6_FRAGMENT_FLAGS_OFFSET+1] = 0x0;// MF=0 && frag_off =0
		return true;
	}// ModifyPacktes ()

	virtual bool ReceivePacketsAndCompare()
	{
		size_t receivedSize = 0;
		size_t receivedSize2 = 0;
		size_t receivedSize3 = 0;
		bool isSuccess = true;

		// Receive results
		Byte *rxBuff1 = new Byte[0x400];
		Byte *rxBuff2 = new Byte[0x400];
		Byte *rxBuff3 = new Byte[0x400];

		if (NULL == rxBuff1 || NULL == rxBuff2 || NULL == rxBuff3)
		{
			printf("Memory allocation error.\n");
			return false;
		}

		receivedSize = m_consumer.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, m_consumer.m_fromChannelName.c_str());

		receivedSize2 = m_consumer.ReceiveData(rxBuff2, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize2, m_consumer.m_fromChannelName.c_str());

		receivedSize3 = m_consumer2.ReceiveData(rxBuff3, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize3, m_consumer2.m_fromChannelName.c_str());

		// Compare results
		if (!CompareResultVsGolden(m_sendBuffer, m_sendSize, rxBuff1, receivedSize))
		{
			printf("Comparison of Buffer0 Failed!\n");
			isSuccess = false;
		}

		char recievedBuffer[256] = {0};
		char SentBuffer[256] = {0};
//		char * p = recievedBuffer;
		size_t j;
		for(j = 0; j < m_sendSize; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer[j]);
		for(j = 0; j < receivedSize; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff1[j]);
		printf("Expected Value1 (%zu)\n%s\n, Received Value1(%zu)\n%s\n",m_sendSize,SentBuffer,receivedSize,recievedBuffer);

		for(j = 0; j < m_sendSize2; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer2[j]);
		for(j = 0; j < receivedSize2; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff2[j]);
		printf("Expected Value2 (%zu)\n%s\n, Received Value2(%zu)\n%s\n",m_sendSize2,SentBuffer,receivedSize2,recievedBuffer);

		for(j = 0; j < m_sendSize3; j++)
			sprintf(&SentBuffer[3*j], " %02X", m_sendBuffer3[j]);
		for(j = 0; j < receivedSize3; j++)
//			recievedBuffer += sprintf(recievedBuffer, "%02X", rxBuff1[i]);
			sprintf(&recievedBuffer[3*j], " %02X", rxBuff3[j]);
		printf("Expected Value3 (%zu)\n%s\n, Received Value3(%zu)\n%s\n",m_sendSize3,SentBuffer,receivedSize3,recievedBuffer);

		isSuccess &= CompareResultVsGolden(m_sendBuffer2, m_sendSize2, rxBuff2, receivedSize2);
		isSuccess &= CompareResultVsGolden(m_sendBuffer3, m_sendSize3, rxBuff3, receivedSize3);

		delete[] rxBuff1;
		delete[] rxBuff2;
		delete[] rxBuff3;

		return isSuccess;
	}
};

/*----------------------------------------------------------------------------------------------*/
/* Test054: IPV6 filtering based on based on source and destination port  			*/
/*----------------------------------------------------------------------------------------------*/
class IpaFilteringBlockTest054 : public IpaFilteringBlockTestFixture
{
public:
	IpaFilteringBlockTest054()
	{
		m_name = "IpaFilteringBlockTest054";
		m_description =
		"Filtering block test 054 - IPV6 filtering rules based on source and destination port (End-Point specific Filtering Table, Insert all rules in a single commit)\
		1. Generate and commit three routing tables. \
			Each table contains a single \"bypass\" rule (all data goes to output pipe 0, 1  and 2 (accordingly)) \
		2. Generate and commit three filtering rules:\
			All (SRC_PORT = 1000) traffic goes to routing table 0 \
			All (DST_PORT = 100) traffic goes to routing table 1 \
			All (5 >= SRC_PORT_RANGE >= 15) traffic goes to routing table 2";
		m_IpaIPType = IPA_IP_v6;
		m_minIPAHwType = IPA_HW_v2_5;
		Register(*this);
	}


	virtual bool AddRules()
	{
		printf("Entering %s, %s()\n",__FUNCTION__, __FILE__);
		const char bypass0[20] = "Bypass0";
		const char bypass1[20] = "Bypass1";
		const char bypass2[20] = "Bypass2";
		struct ipa_ioc_get_rt_tbl routing_table0,routing_table1,routing_table2;

		if (!CreateThreeIPv6BypassRoutingTables (bypass0,bypass1,bypass2))
		{
			printf("CreateThreeBypassRoutingTables Failed\n");
			return false;
		}

		printf("CreateThreeBypassRoutingTables completed successfully\n");
		routing_table0.ip = IPA_IP_v6;
		strcpy (routing_table0.name,bypass0);
		if (!m_routing.GetRoutingTable(&routing_table0))
		{
			printf("m_routing.GetRoutingTable(&routing_table0=0x%p) Failed.\n",&routing_table0);
			return false;
		}
		routing_table1.ip = IPA_IP_v6;
		strcpy (routing_table1.name,bypass1);
		if (!m_routing.GetRoutingTable(&routing_table1))
		{
			printf("m_routing.GetRoutingTable(&routing_table1=0x%p) Failed.\n",&routing_table1);
			return false;
		}

		routing_table2.ip = IPA_IP_v6;
		strcpy (routing_table2.name,bypass2);
		if (!m_routing.GetRoutingTable(&routing_table2))
		{
			printf("m_routing.GetRoutingTable(&routing_table2=0x%p) Failed.\n",&routing_table2);
			return false;
		}

		IPAFilteringTable FilterTable0;
		struct ipa_flt_rule_add flt_rule_entry;
		FilterTable0.Init(IPA_IP_v6,IPA_CLIENT_TEST_PROD,false,3);

		// Configuring Filtering Rule No.0
		FilterTable0.GeneratePresetRule(1,flt_rule_entry);
		flt_rule_entry.at_rear = true;
		flt_rule_entry.flt_rule_hdl=-1; // return Value
		flt_rule_entry.status = -1; // return value
		flt_rule_entry.rule.action=IPA_PASS_TO_ROUTING;
		flt_rule_entry.rule.rt_tbl_hdl=routing_table0.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT;
		flt_rule_entry.rule.attrib.src_port = 1000;

		printf ("flt_rule_entry was set successfully, preparing for insertion....\n");
		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
				)
		{
			printf ("%s::Error Adding RuleTable(0) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.1
		flt_rule_entry.rule.rt_tbl_hdl=routing_table1.hdl; //put here the handle corresponding to Routing Rule 1
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_DST_PORT;
		flt_rule_entry.rule.attrib.dst_port = 100;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(1) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.2
		flt_rule_entry.rule.rt_tbl_hdl=routing_table2.hdl; //put here the handle corresponding to Routing Rule 2
		flt_rule_entry.rule.attrib.attrib_mask = IPA_FLT_SRC_PORT_RANGE;
		flt_rule_entry.rule.attrib.src_port_lo = 5;
		flt_rule_entry.rule.attrib.src_port_hi = 15;

		if (
				((uint8_t)-1 == FilterTable0.AddRuleToTable(flt_rule_entry)) ||
				!m_filtering.AddFilteringRule(FilterTable0.GetFilteringTable())
			)
		{
			printf ("%s::Error Adding RuleTable(2) to Filtering, aborting...\n",__FUNCTION__);
			return false;
		} else
		{
			printf("flt rule hdl0=0x%x, status=0x%x\n", FilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,FilterTable0.ReadRuleFromTable(2)->status);
		}
		printf("Leaving %s, %s()\n",__FUNCTION__, __FILE__);
		return true;
	}// AddRules()

	virtual bool ModifyPackets()
	{
		unsigned short port;
		if (
				(NULL == m_sendBuffer) ||
				(NULL == m_sendBuffer2) ||
				(NULL == m_sendBuffer3)
			)
		{
			printf ("Error : %s was called with NULL Buffers\n",__FUNCTION__);
			return false;
		}

		port = htons(1000);
		memcpy(&m_sendBuffer[IPV6_SRC_PORT_OFFSET], &port, sizeof(port));
		port = htons(100);
		memcpy(&m_sendBuffer2[IPV6_DST_PORT_OFFSET], &port, sizeof(port));
		port = htons(10);
		memcpy(&m_sendBuffer3[IPV6_SRC_PORT_OFFSET], &port, sizeof(port));

		return true;
	}// ModifyPacktes ()
};


static class IpaFilteringBlockTest001 ipaFilteringBlockTest001;//Global Filtering Test
static class IpaFilteringBlockTest002 ipaFilteringBlockTest002;//Global Filtering Test
static class IpaFilteringBlockTest003 ipaFilteringBlockTest003;//Global Filtering Test
static class IpaFilteringBlockTest004 ipaFilteringBlockTest004;//Global Filtering Test
static class IpaFilteringBlockTest005 ipaFilteringBlockTest005;//Global Filtering Test
static class IpaFilteringBlockTest006 ipaFilteringBlockTest006;//Global Filtering Test
static class IpaFilteringBlockTest007 ipaFilteringBlockTest007;//Global Filtering Test
static class IpaFilteringBlockTest008 ipaFilteringBlockTest008;//Global Filtering Test
static class IpaFilteringBlockTest009 ipaFilteringBlockTest009;//Global Filtering Test
static class IpaFilteringBlockTest010 ipaFilteringBlockTest010;//Global Filtering Test

static class IpaFilteringBlockTest021 ipaFilteringBlockTest021;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest022 ipaFilteringBlockTest022;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest023 ipaFilteringBlockTest023;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest024 ipaFilteringBlockTest024;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest025 ipaFilteringBlockTest025;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest026 ipaFilteringBlockTest026;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest027 ipaFilteringBlockTest027;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest028 ipaFilteringBlockTest028;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest029 ipaFilteringBlockTest029;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest030 ipaFilteringBlockTest030;//Global Filtering Test, End point Specific Filtering Table
static class IpaFilteringBlockTest031 ipaFilteringBlockTest031;//Global Filtering Test, End point Specific Filtering Table

static class IpaFilteringBlockTest050 ipaFilteringBlockTest050;// IPv6 Test, Global Filtering Table
static class IpaFilteringBlockTest051 ipaFilteringBlockTest051;// IPv6 Test, End point Specific Filtering Table
static class IpaFilteringBlockTest052 ipaFilteringBlockTest052;// IPv6 Test, Global Filtering Table
static class IpaFilteringBlockTest053 ipaFilteringBlockTest053;// IPv6 Test, End point Specific Filtering Table
static class IpaFilteringBlockTest054 ipaFilteringBlockTest054;// IPv6 Test, End point Specific Filtering Table

