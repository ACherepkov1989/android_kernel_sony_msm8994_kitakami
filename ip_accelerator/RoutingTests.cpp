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

#include "InterfaceAbstraction.h"
#include "Constants.h"
#include "Logger.h"
#include "TestsUtils.h"
#include "linux/msm_ipa.h"
#include "RoutingDriverWrapper.h"
#include "Filtering.h"
#include "IPAFilteringTable.h"

#define TOS_FIELD_OFFSET (1)
#define DST_ADDR_LSB_OFFSET_IPV4 (19)
#define DST_ADDR_MSB_OFFSET_IPV6 (24)
#define DST_ADDR_LSB_OFFSET_IPV6 (39)
#define TRAFFIC_CLASS_MSB_OFFSET_IPV6 (0)
#define TRAFFIC_CLASS_LSB_OFFSET_IPV6 (1)
#define FLOW_CLASS_MSB_OFFSET_IPV6 (1)
#define FLOW_CLASS_MB_OFFSET_IPV6 (2)
#define FLOW_CLASS_LSB_OFFSET_IPV6 (3)


class IpaRoutingBlockTestFixture:public TestBase
{
public:
	IpaRoutingBlockTestFixture():
		m_sendSize (BUFF_MAX_SIZE),
		m_sendSize2 (BUFF_MAX_SIZE),
		m_sendSize3 (BUFF_MAX_SIZE),
		m_IpaIPType(IPA_IP_v4)
	{
		memset(m_sendBuffer, 0, sizeof(m_sendBuffer));
		memset(m_sendBuffer2, 0, sizeof(m_sendBuffer2));
		memset(m_sendBuffer3, 0, sizeof(m_sendBuffer3));
		m_testSuiteName.push_back("Routing");
	}

	bool Setup()
	{
		ConfigureScenario(PHASE_TWO_TEST_CONFIGURATION);

		m_producer.Open(INTERFACE0_TO_IPA_DATA_PATH, INTERFACE0_FROM_IPA_DATA_PATH);

		m_consumer.Open(INTERFACE1_TO_IPA_DATA_PATH, INTERFACE1_FROM_IPA_DATA_PATH);
		m_consumer2.Open(INTERFACE2_TO_IPA_DATA_PATH, INTERFACE2_FROM_IPA_DATA_PATH);
		m_defaultConsumer.Open(INTERFACE3_TO_IPA_DATA_PATH, INTERFACE3_FROM_IPA_DATA_PATH);

		if (!m_routing.DeviceNodeIsOpened()) {
			printf("Routing block is not ready for immediate commands!\n");
			return false;
		}

		if (!m_filtering.DeviceNodeIsOpened()) {
			printf("Filtering block is not ready for immediate commands!\n");
			return false;
		}
		m_routing.Reset(IPA_IP_v4);
		m_routing.Reset(IPA_IP_v6);

		return true;
	} /* Setup()*/

	bool Teardown()
	{
		if (!m_routing.DeviceNodeIsOpened()) {
			printf("Routing block is not ready for immediate commands!\n");
			return false;
		}
		if (!m_filtering.DeviceNodeIsOpened()) {
			printf("Filtering block is not ready for immediate commands!\n");
			return false;
		}

		m_producer.Close();
		m_consumer.Close();
		m_consumer2.Close();
		m_defaultConsumer.Close();
		return true;
	} /* Teardown() */

	bool LoadFiles(enum ipa_ip_type ip)
	{
		string fileName;

		if (IPA_IP_v4 == ip) {
			fileName = "Input/IPv4_1";
		} else {
			fileName = "Input/IPv6";
		}

		if (!LoadDefaultPacket(ip, m_sendBuffer, m_sendSize)) {
			LOG_MSG_ERROR("Failed loading default Packet");
			return false;
		}

		if (!LoadDefaultPacket(ip, m_sendBuffer2, m_sendSize2)) {
			LOG_MSG_ERROR("Failed loading default Packet");
			return false;
		}

		if (!LoadDefaultPacket(ip, m_sendBuffer3, m_sendSize3)) {
			LOG_MSG_ERROR("Failed loading default Packet");
			return false;
		}

		return true;
	}

	bool ReceivePacketAndCompareFrom(InterfaceAbstraction& cons, Byte* send, size_t send_sz, InterfaceAbstraction& excp_cons)
	{
		size_t receivedSize = 0;
		bool isSuccess = true;

		/* Receive results*/
		Byte *rxBuff1 = new Byte[0x400];

		if (NULL == rxBuff1)
		{
			printf("Memory allocation error.\n");
			return false;
		}

		receivedSize = cons.ReceiveData(rxBuff1, 0x400);
		printf("Received %zu bytes on %s.\n", receivedSize, cons.m_fromChannelName.c_str());

		// Compare results
		isSuccess &= CompareResultVsGolden(send,  send_sz,  rxBuff1, receivedSize);

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

	bool ReceivePacketsAndCompare()
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

		/* Compare results */
		isSuccess &= CompareResultVsGolden(m_sendBuffer,  m_sendSize,  rxBuff1, receivedSize);
		isSuccess &= CompareResultVsGolden(m_sendBuffer2, m_sendSize2, rxBuff2, receivedSize2);
		isSuccess &= CompareResultVsGolden(m_sendBuffer3, m_sendSize3, rxBuff3, receivedSize3);

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

		delete[] rxBuff1;
		delete[] rxBuff2;
		delete[] rxBuff3;

		return isSuccess;
	}

	~IpaRoutingBlockTestFixture()
	{
//		free(m_sendBuffer);
//		free(m_sendBuffer2);
//		free(m_sendBuffer3);
		m_sendSize = 0;
		m_sendSize2 = 0;
		m_sendSize3 = 0;
	}

	void InitFilteringBlock()
	{
		IPAFilteringTable fltTable;
		struct ipa_ioc_get_rt_tbl st_rt_tbl;
		struct ipa_flt_rule_add flt_rule_entry;

		memset(&st_rt_tbl, 0, sizeof(st_rt_tbl));
		memset(&flt_rule_entry, 0, sizeof(flt_rule_entry));
		strcpy(st_rt_tbl.name, "LAN");
		st_rt_tbl.ip = m_IpaIPType;
		fltTable.Init(m_IpaIPType, IPA_CLIENT_TEST_PROD, true, 1);
		m_routing.GetRoutingTable(&st_rt_tbl);
		flt_rule_entry.rule.rt_tbl_hdl = st_rt_tbl.hdl;
		fltTable.AddRuleToTable(flt_rule_entry);
		m_filtering.AddFilteringRule(fltTable.GetFilteringTable());
	}

	static RoutingDriverWrapper m_routing;
	static Filtering m_filtering;

	static const size_t BUFF_MAX_SIZE = 1024;

	InterfaceAbstraction m_producer;
	InterfaceAbstraction m_consumer;
	InterfaceAbstraction m_consumer2;
	InterfaceAbstraction m_defaultConsumer;
	Byte m_sendBuffer[BUFF_MAX_SIZE];	// First input file / IP packet
	Byte m_sendBuffer2[BUFF_MAX_SIZE];	// Second input file / IP packet
	Byte m_sendBuffer3[BUFF_MAX_SIZE];	// Third input file (default) / IP packet
	size_t m_sendSize;
	size_t m_sendSize2;
	size_t m_sendSize3;
	enum ipa_ip_type m_IpaIPType;


private:
};

RoutingDriverWrapper IpaRoutingBlockTestFixture::m_routing;
Filtering IpaRoutingBlockTestFixture::m_filtering;

/*---------------------------------------------------------------------------*/
/* Test1: Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest1 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest1()
	{
		m_name = "IpaRoutingBlockTest1";
		m_description =" \
		Routing block test 001 - Destination address exact match\1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All DST_IP == (192.169.2.170 & 255.255.255.255)traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All DST_IP == (192.168.2.255 & 255.255.255.255)traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v4;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v4);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV4] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV4] = 0xAA;
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A802FF;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A802AA;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test2: Tests routing by destination address with a subnet (mask) */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest2 : IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest2()
	{
		m_name = "IpaRoutingBlockTest2";
		m_description =" \
		Routing block test 002 - Destination address subnet match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All DST_IP == (192.169.170.0 & 255.255.255.0)traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All DST_IP == (192.168.255.0 & 255.255.255.0)traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v4;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		printf("ENTRY: IpaRoutingBlockTest2::Run()\n");

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v4);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[18] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[18] = 0xAA;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		printf("ENTRY: IpaRoutingBlockTest2::AddRules()\n");

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A8FF00;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFF00;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A8AA00;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFF00;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		printf("Before calling m_routing.AddRoutingRule()\n");
		printf("m_routing = %p\n", &m_routing);

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};


/*---------------------------------------------------------------------------*/
/* Test3: Tests routing by TOS (Type Of Service) */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest3 : IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest3()
	{
		m_name = "IpaRoutingBlockTest3";
		m_description = " \
		Routing block test 003 - TOS exact match\
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All TOS == 0xFF traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All TOS == 0xAA traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v4;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v4);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[TOS_FIELD_OFFSET] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[TOS_FIELD_OFFSET] = 0xAA;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if (!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl;
		// gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TOS;
		rt_rule_entry->rule.attrib.u.v4.tos = 0xFF;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TOS;
		rt_rule_entry->rule.attrib.u.v4.tos = 0xAA;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test4: Destination address exact match and TOS exact match */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest4 : IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest4()
	{
		m_name = "IpaRoutingBlockTest4";
		m_description =" \
		Routing block test 004 - Destination address and TOS exact match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All DST_IP == (192.169.2.170 & 255.255.255.255) and TOS == 0xFF traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All DST_IP == (192.168.2.255 & 255.255.255.255) and TOS == 0xAA traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v4;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v4);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[TOS_FIELD_OFFSET] = 0xFF;
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV4] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[TOS_FIELD_OFFSET] = 0xAA;
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV4] = 0xAA;
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v4;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TOS | IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.tos = 0xFF;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A802FF;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;


		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TOS | IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v4.tos = 0xAA;
		rt_rule_entry->rule.attrib.u.v4.dst_addr      = 0xC0A802AA;
		rt_rule_entry->rule.attrib.u.v4.dst_addr_mask = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test1: IPv6 - Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest5 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest5()
	{
		m_name = "IpaRoutingBlockTest5";
		m_description =" \
		Routing block test 005 - IPv6 Destination address exact match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All DST_IP ==	0XFF020000 \
							0x00000000 \
							0x00000000 \
							0X000000FF \
		traffic goes to pipe IPA_CLIENT_TEST2_CONS \
		All DST_IP ==	0XFF020000 \
						0x00000000 \
						0x00000000 \
						0X000000FF \
		traffic goes to pipe IPA_CLIENT_TEST3_CONS\
		All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v6);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV6] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV6] = 0xAA;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v6;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0XFF020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X000000FF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0XFF020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X000000AA;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test5: IPv6 - Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest006 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest006()
	{
		m_name = "IpaRoutingBlockTest006";
		m_description =" \
		Routing block test 006 - IPv6 Destination address Subnet match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match 0xFFFFFFFF,0xFFFFFFFF,0x00000000,0x0000000). \
			All DST_IP ==	0X11020000 \
							0x00000000 \
							0x00000000 \
							0X0000000C \
		traffic goes to pipe IPA_CLIENT_TEST2_CONS \
		All DST_IP ==	0X22020000 \
						0x00000000 \
						0x00000000 \
						0X0000000C \
		traffic goes to pipe IPA_CLIENT_TEST3_CONS\
		All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v6);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[DST_ADDR_MSB_OFFSET_IPV6] = 0x11;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[DST_ADDR_MSB_OFFSET_IPV6] = 0x22;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v6;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO: Header Insertion gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0X11020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X0000000C;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO: Header Insertion gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0X22020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X0000000C;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0x00000000;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test1: IPv6 - Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest007 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest007()
	{
		m_name = "IpaRoutingBlockTest007";
		m_description = " \
		Routing block test 007 - IPv6 Exact Traffic Class Match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All Traffic Class == 0xAA traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All Traffic Class == 0xBB traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v6);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[TRAFFIC_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer[TRAFFIC_CLASS_MSB_OFFSET_IPV6] |= 0x0A;
		m_sendBuffer[TRAFFIC_CLASS_LSB_OFFSET_IPV6] &= 0x0F;
		m_sendBuffer[TRAFFIC_CLASS_LSB_OFFSET_IPV6] |= 0xA0;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[TRAFFIC_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer2[TRAFFIC_CLASS_MSB_OFFSET_IPV6] |= 0x0B;
		m_sendBuffer2[TRAFFIC_CLASS_LSB_OFFSET_IPV6] &= 0x0F;
		m_sendBuffer2[TRAFFIC_CLASS_LSB_OFFSET_IPV6] |= 0xB0;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v6;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
		//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO: Header Insertion gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TC;
		rt_rule_entry->rule.attrib.u.v6.tc = 0xAA;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // TODO: Header Insertion gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_TC;
		rt_rule_entry->rule.attrib.u.v6.tc = 0xBB;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test1: IPv6 - Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest008 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest008()
	{
		m_name = "IpaRoutingBlockTest008";
		m_description = " \
		Routing block test 008 - IPv6 Destination address exact match and Traffic Class Match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All Traffic Class == 0xAA & IPv6 DST Addr 0xFF020000...00AA traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All Traffic Class == 0xBB & IPv6 DST Addr 0xFF020000...00BB traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v6);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[TRAFFIC_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer[TRAFFIC_CLASS_MSB_OFFSET_IPV6] |= 0x0A;
		m_sendBuffer[TRAFFIC_CLASS_LSB_OFFSET_IPV6] &= 0x0F;
		m_sendBuffer[TRAFFIC_CLASS_LSB_OFFSET_IPV6] |= 0xA0;
		m_sendBuffer[DST_ADDR_LSB_OFFSET_IPV6] = 0xFF;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[TRAFFIC_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer2[TRAFFIC_CLASS_MSB_OFFSET_IPV6] |= 0x0B;
		m_sendBuffer2[TRAFFIC_CLASS_LSB_OFFSET_IPV6] &= 0x0F;
		m_sendBuffer2[TRAFFIC_CLASS_LSB_OFFSET_IPV6] |= 0xB0;
		m_sendBuffer2[DST_ADDR_LSB_OFFSET_IPV6] = 0xAA;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v6;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR | IPA_FLT_TC;
		rt_rule_entry->rule.attrib.u.v6.tc = 0xAA;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0XFF020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X000000FF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_DST_ADDR | IPA_FLT_TC;
		rt_rule_entry->rule.attrib.u.v6.tc = 0xBB;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[0]      = 0XFF020000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[1]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[2]      = 0x00000000;
		rt_rule_entry->rule.attrib.u.v6.dst_addr[3]      = 0X000000AA;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[0] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[1] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[2] = 0xFFFFFFFF;
		rt_rule_entry->rule.attrib.u.v6.dst_addr_mask[3] = 0xFFFFFFFF;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

/*---------------------------------------------------------------------------*/
/* Test1: IPv6 - Tests routing by destination address */
/*---------------------------------------------------------------------------*/
class IpaRoutingBlockTest009 : public IpaRoutingBlockTestFixture
{
public:
	IpaRoutingBlockTest009()
	{
		m_name = "IpaRoutingBlockTest009";
		m_description = " \
		Routing block test 009 - IPv6 Exact Flow Label Match \
		1. Generate and commit a single routing tables. \
		2. Generate and commit Three routing rules: (DST & Mask Match). \
			All Flow Label == 0xABCDE traffic goes to pipe IPA_CLIENT_TEST2_CONS \
			All Flow Label == 0x12345 traffic goes to pipe IPA_CLIENT_TEST3_CONS\
			All other traffic goes to pipe IPA_CLIENT_TEST4_CONS";
		m_IpaIPType = IPA_IP_v6;
		Register(*this);
	}

	bool Run()
	{
		bool res = false;
		bool isSuccess = false;

		// Add the relevant routing rules
		res = AddRules();
		if (false == res) {
			printf("Failed adding routing rules.\n");
			return false;
		}

		// Load input data (IP packet) from file
		res = LoadFiles(IPA_IP_v6);
		if (false == res) {
			printf("Failed loading files.\n");
			return false;
		}

		// Send first packet
		m_sendBuffer[FLOW_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer[FLOW_CLASS_MSB_OFFSET_IPV6] |= 0x0A;
		m_sendBuffer[FLOW_CLASS_MB_OFFSET_IPV6] = 0xBC;
		m_sendBuffer[FLOW_CLASS_LSB_OFFSET_IPV6] = 0xDE;
		isSuccess = m_producer.SendData(m_sendBuffer, m_sendSize);
		if (false == isSuccess)
		{
			printf("SendData failure.\n");
			return false;
		}

		// Send second packet
		m_sendBuffer2[FLOW_CLASS_MSB_OFFSET_IPV6] &= 0xF0;
		m_sendBuffer2[FLOW_CLASS_MSB_OFFSET_IPV6] |= 0x01;
		m_sendBuffer2[FLOW_CLASS_MB_OFFSET_IPV6] = 0x23;
		m_sendBuffer2[FLOW_CLASS_LSB_OFFSET_IPV6] = 0x45;
		isSuccess = m_producer.SendData(m_sendBuffer2, m_sendSize);
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

		return isSuccess;
	} // Run()

	bool AddRules()
	{
		struct ipa_ioc_add_rt_rule *rt_rule;
		struct ipa_rt_rule_add *rt_rule_entry;
		const int NUM_RULES = 3;

		rt_rule = (struct ipa_ioc_add_rt_rule *)
			calloc(1, sizeof(struct ipa_ioc_add_rt_rule) +
			       NUM_RULES*sizeof(struct ipa_rt_rule_add));

		if(!rt_rule) {
			printf("fail\n");
			return false;
		}

		rt_rule->commit = 1;
		rt_rule->num_rules = NUM_RULES;
		rt_rule->ip = IPA_IP_v6;
		strcpy(rt_rule->rt_tbl_name, "LAN");

		rt_rule_entry = &rt_rule->rules[0];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST2_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_FLOW_LABEL;
		rt_rule_entry->rule.attrib.u.v6.flow_label = 0xABCDE;

		rt_rule_entry = &rt_rule->rules[1];
		rt_rule_entry->at_rear = 0;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST3_CONS;
//		rt_rule_entry->rule.hdr_hdl = hdr_entry->hdr_hdl; // gidons, there is no support for header insertion / removal yet.
		rt_rule_entry->rule.attrib.attrib_mask = IPA_FLT_FLOW_LABEL;
		rt_rule_entry->rule.attrib.u.v6.flow_label = 0x12345;

		rt_rule_entry = &rt_rule->rules[2];
		rt_rule_entry->at_rear = 1;
		rt_rule_entry->rule.dst = IPA_CLIENT_TEST4_CONS;

		if (false == m_routing.AddRoutingRule(rt_rule))
		{
			printf("Routing rule addition failed!\n");
			return false;
		}

		printf("rt rule hdl1=%x\n", rt_rule_entry->rt_rule_hdl);

		free(rt_rule);

		InitFilteringBlock();

		return true;
	}
};

static class IpaRoutingBlockTest1 ipaRoutingBlockTest1;
static class IpaRoutingBlockTest2 ipaRoutingBlockTest2;
static class IpaRoutingBlockTest3 ipaRoutingBlockTest3;
static class IpaRoutingBlockTest4 ipaRoutingBlockTest4;
static class IpaRoutingBlockTest5 ipaRoutingBlockTest5;
static class IpaRoutingBlockTest006 ipaRoutingBlockTest006;
static class IpaRoutingBlockTest007 ipaRoutingBlockTest007;
static class IpaRoutingBlockTest008 ipaRoutingBlockTest008;
static class IpaRoutingBlockTest009 ipaRoutingBlockTest009;


