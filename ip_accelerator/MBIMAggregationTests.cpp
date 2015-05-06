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
#include "hton.h" // for htonl
#include "MBIMAggregationTestFixture.h"
#include "MBIMAggregationTestFixtureConf10.h"
#include "MBIMAggregationTestFixtureConf11.h"
#include "MBIMAggregationTestFixtureConf12.h"
#include "Constants.h"
#include "TestsUtils.h"
#include "linux/msm_ipa.h"

#define AGGREGATION_LOOP 4
#define IPV4_DST_ADDR_OFFSET (16)

enum ipa_elan_version {
	ELAN1,
	ELAN2,
};

/////////////////////////////////////////////////////////////////////////////////
//							MBIM Aggregation scenarios                         //
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationScenarios {
public:
	//MBIM Aggregation test - sends 5 packets and receives 1 aggregated packet
	static bool MBIMAggregationTest(enum ipa_elan_version elan, Pipe* input,
			Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Deaggregation test - sends an aggregated packet made from 5 packets
	//and receives 5 packets
	static bool MBIMDeaggregationTest(enum ipa_elan_version elan, Pipe* input,
			Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Deaggregation one packet test - sends an aggregated packet made from
	//1 packet and receives 1 packet
	static bool MBIMDeaggregationOnePacketTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Deaggregation and Aggregation test - sends an aggregated packet made
	//from 5 packets and receives the same aggregated packet
	static bool MBIMDeaggregationAndAggregationTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM multiple Deaggregation and Aggregation test - sends 5 aggregated
	//packets each one made of 1 packet and receives an aggregated packet made
	//of the 5 packets
	static bool MBIMMultipleDeaggregationAndAggregationTest(
			enum ipa_elan_version elan, Pipe* input, Pipe* output,
			enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation Loop test - sends 5 packets and expects to receive 1
	//aggregated packet a few times
	static bool MBIMAggregationLoopTest(enum ipa_elan_version elan, Pipe* input,
			Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation time limit test - sends 1 small packet smaller than the
	//byte limit and receives 1 aggregated packet
	static bool MBIMAggregationTimeLimitTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation byte limit test - sends 2 packets that together are
	//larger than the byte limit
	static bool MBIMAggregationByteLimitTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Deaggregation multiple NDP test - sends an aggregated packet made
	//from 5 packets and 2 NDPs and receives 5 packets
	static bool MBIMDeaggregationMultipleNDPTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation 2 pipes test - sends 3 packets from one pipe and an
	//aggregated packet made of 2 packets from another pipe and receives 1
	//aggregated packet made of all 5 packets
	static bool MBIMAggregation2PipesTest(enum ipa_elan_version elan,
			Pipe* input1, Pipe* input2, Pipe* output, enum ipa_ip_type m_eIP,
			bool isQcncm);
	//MBIM Aggregation time limit loop test - sends 5 small packet smaller than
	//the byte limit and receives 5 aggregated packet
	static bool MBIMAggregationTimeLimitLoopTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation 0 limits test - sends 5 packets and expects to get each
	//packet back aggregated (both size and time limits are 0)
	static bool MBIMAggregation0LimitsTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation multiple packets test - sends 9 packets with same stream
	//ID and receives 1 aggregated packet with 2 NDPs
	static bool MBIMAggregationMultiplePacketsTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation different stream IDs test - sends 5 packets with
	//different stream IDs and receives 1 aggregated packet made of 5 NDPs
	static bool MBIMAggregationDifferentStreamIdsTest(enum ipa_elan_version elan,
			Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm);
	//MBIM Aggregation no interleaving stream IDs test - sends 5 packets with
	//interleaving stream IDs (0, 1, 0, 1, 0) and receives 1 aggregated packet
	//made of 5 NDPs
	static bool MBIMAggregationNoInterleavingStreamIdsTest(
			enum ipa_elan_version elan, Pipe* input, Pipe* output,
			enum ipa_ip_type m_eIP, bool isQcncm);

private:
	//This method will deaggregate an aggregated packet and compare the packets
	//to the expected packets
	static bool DeaggragateAndComparePackets(
			Byte pAggregatedPacket[MAX_PACKET_SIZE],
			Byte pExpectedPackets[NUM_PACKETS][MAX_PACKET_SIZE],
			int pPacketsSizes[NUM_PACKETS], int nNumPackets,
			int nAggregatedPacketSize, bool isQcncm);
	//This method will aggregate packets
	static void AggregatePackets(
			Byte pAggregatedPacket[MAX_PACKET_SIZE]/*ouput*/,
			Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE],
			int pPacketsSizes[NUM_PACKETS], int nNumPackets,
			int nAggregatedPacketSize, bool isQcncm);
	//This method will aggregate packets and take into consideration their
	//stream id to seperate them into different NDPs
	static void AggregatePacketsWithStreamId(
			Byte pAggregatedPacket[MAX_PACKET_SIZE]/*ouput*/,
			Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE],
			int pPacketsSizes[NUM_PACKETS], int nNumPackets,
			int nAggregatedPacketSize, Byte pPacketsStreamId[NUM_PACKETS]);
	//This method will deaggregate an aggregated packet made of one packet and
	//compare the packet to the expected packet
	static bool DeaggragateAndCompareOnePacket(
			Byte pAggregatedPacket[MAX_PACKET_SIZE],
			Byte pExpectedPacket[MAX_PACKET_SIZE], int nPacketsSize,
			int nAggregatedPacketSize, bool isQcncm);
	//This method will deaggregate an aggregated packet and compare the packets
	//to the expected packets
	static bool DeaggragateAndComparePacketsWithStreamId(
			Byte pAggregatedPacket[MAX_PACKET_SIZE],
			Byte pExpectedPackets[][MAX_PACKET_SIZE], int pPacketsSizes[],
			int nNumPackets, int nAggregatedPacketSize,
			Byte pPacketsStreamId[NUM_PACKETS]);
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationTest(enum ipa_elan_version elan,
		Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated
	//packet minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
		{
			pPacketsSizes[i]++;
		}
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
			{
				pPackets[i][j] = i;
			}
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	//send the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
					pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be there)"
			"\n", MAX_PACKET_SIZE);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}
	//deaggregating the aggregated packet
	return DeaggragateAndComparePackets(pReceivedPacket, pPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMDeaggregationTest(enum ipa_elan_version elan,
		Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm)
{
	bool bTestResult = true;
	//The packets that the aggregated packet will be made of
	Byte pExpectedPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that the aggregated packet will be made of
	int pPacketsSizes[NUM_PACKETS];
	//Buffers for the packets that will be received
	Byte pReceivedPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//Total size of all the packets that the aggregated packet will be made of
	//(this is the max size of the aggregated packet
	//minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[MAX_PACKET_SIZE] = {0};

	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
		{
			pPacketsSizes[i]++;
		}
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
			{
				pExpectedPackets[i][j] = i;
			}
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pExpectedPackets[i],
					pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pExpectedPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pExpectedPackets[i][size] = i;
				size++;
			}
		}
	}

	//initializing the aggregated packet
	AggregatePackets(pAggregatedPacket, pExpectedPackets, pPacketsSizes,
			NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);

	//send the aggregated packet
	LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes)\n",
			sizeof(pAggregatedPacket));
	int nBytesSent = input->Send(pAggregatedPacket, sizeof(pAggregatedPacket));
	if (sizeof(pAggregatedPacket) != nBytesSent)
	{
		LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes) "
				"failed!\n", sizeof(pAggregatedPacket));
		return false;
	}

	//receive the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Reading packet %d from the USB pipe(%d bytes should be "
				"there)\n", i, pPacketsSizes[i]);
		int nBytesReceived = output->Receive(pReceivedPackets[i],
				pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving packet %d from the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			print_buff(pReceivedPackets[i], nBytesReceived);
			return false;
		}
	}

	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	for (int i = 0; i < NUM_PACKETS; i++)
		bTestResult &= !memcmp(pExpectedPackets[i], pReceivedPackets[i],
				pPacketsSizes[i]);

	return bTestResult;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMDeaggregationOnePacketTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	bool bTestResult = true;
	//The packets that the aggregated packet will be made of
	Byte pExpectedPackets[1][MAX_PACKET_SIZE];
	//The real sizes of the packets that the aggregated packet will be made of
	int pPacketsSizes[1] = {100};
	//Buffers for the packets that will be received
	Byte pReceivedPackets[1][MAX_PACKET_SIZE];
	//Total size of the aggregated packet
	//(this is the max size of the aggregated packet
	//minus the size of the header and the NDP)
	int nTotalAggregatedPacketSize = 100 + 12 + 16;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[MAX_PACKET_SIZE] = {0};

	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[1];

	//initialize the packet
	if (ELAN1 == elan)
	{
		for (int j = 0; j < pPacketsSizes[0]; j++)
			pExpectedPackets[0][j] = 0;
	}
	else if (ELAN2 == elan)
	{
		// Load input data (IP packet) from file
		pIpPacketsSizes[0] = 100;
		if (!LoadDefaultPacket(m_eIP, pExpectedPackets[0], pIpPacketsSizes[0]))
		{
			LOG_MSG_ERROR("Failed default Packet");
			return false;
		}
		nIPv4DSTAddr = ntohl(0x7F000001);
		memcpy (&pExpectedPackets[0][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
				sizeof(nIPv4DSTAddr));
		int size = pIpPacketsSizes[0];
		while (size < pPacketsSizes[0])
		{
			pExpectedPackets[0][size] = 0;
			size++;
		}
	}

	//initializing the aggregated packet
	AggregatePackets(pAggregatedPacket, pExpectedPackets, pPacketsSizes, 1,
			nTotalAggregatedPacketSize, isQcncm);

	//send the aggregated packet
	LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes)\n",
			nTotalAggregatedPacketSize);
	int nBytesSent = input->Send(pAggregatedPacket, nTotalAggregatedPacketSize);
	if (nTotalAggregatedPacketSize != nBytesSent)
	{
		LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes) "
				"failed!\n", nTotalAggregatedPacketSize);
		return false;
	}

	//receive the packet
	for (int i = 0; i < 1; i++)
	{
		LOG_MSG_DEBUG("Reading packet %d from the USB pipe(%d bytes should be "
				"there)\n", i, pPacketsSizes[i]);
		int nBytesReceived = output->Receive(pReceivedPackets[i],
				pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving packet %d from the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			print_buff(pReceivedPackets[i], nBytesReceived);
			return false;
		}
	}

	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	for (int i = 0; i < 1; i++)
		bTestResult &= !memcmp(pExpectedPackets[i], pReceivedPackets[i],
				pPacketsSizes[i]);

	return bTestResult;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMDeaggregationAndAggregationTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that the aggregated packet will be made of
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that the aggregated packet will be made of
	int pPacketsSizes[NUM_PACKETS];
	//Buffers for the packets that will be received
	Byte pReceivedPacket[MAX_PACKET_SIZE];
	//Total size of all the packets that the aggregated packet will be made of
	//(this is the max size of the aggregated packet
	//minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[MAX_PACKET_SIZE] = {0};
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	//initializing the aggregated packet
	AggregatePackets(pAggregatedPacket, pPackets, pPacketsSizes, NUM_PACKETS,
			MAX_PACKET_SIZE, isQcncm);

	//send the aggregated packet
	LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes)\n",
			MAX_PACKET_SIZE);
	int nBytesSent = input->Send(pAggregatedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesSent)
	{
		LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		return false;
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading aggregated packet from the USB pipe(%d bytes should "
			"be there)\n", MAX_PACKET_SIZE);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		LOG_MSG_DEBUG("Received %d bytes\n", nBytesReceived);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}


	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	return DeaggragateAndComparePackets(pReceivedPacket, pPackets, pPacketsSizes,
			NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMMultipleDeaggregationAndAggregationTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that the aggregated packets will be made of
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that the aggregated packet will be made of
	int pPacketsSizes[NUM_PACKETS];
	//Buffers for the packets that will be received
	Byte pReceivedPacket[MAX_PACKET_SIZE];
	//Total size of all the packets that the aggregated packet will be made of
	//(this is the max size of the aggregated packet
	//minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[NUM_PACKETS][MAX_PACKET_SIZE];
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	//initializing the aggregated packets
	for (int i = 0; i < NUM_PACKETS; i++)
		AggregatePackets(pAggregatedPacket[i], &pPackets[i], &pPacketsSizes[i],
				1, pPacketsSizes[i] + 12 + 16, isQcncm);

	//send the aggregated packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending aggregated packet %d into the USB pipe(%d "
				"bytes)\n", i, pPacketsSizes[i] + 12 + 16);
		int nBytesSent = input->Send(pAggregatedPacket[i],
				pPacketsSizes[i] + 12 + 16);
		if (pPacketsSizes[i] + 12 + 16 != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending aggregated packet %d into the USB pipe(%d "
					"bytes) failed!\n", i, pPacketsSizes[i] + 12 + 16);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading aggregated packet from the USB pipe(%d bytes should "
			"be there)\n", MAX_PACKET_SIZE);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		LOG_MSG_DEBUG("Received %d bytes\n", nBytesReceived);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}


	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	return DeaggragateAndComparePackets(pReceivedPacket, pPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationLoopTest(enum ipa_elan_version elan,
		Pipe* input, Pipe* output, enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated
	//packet minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	int num_iters = AGGREGATION_LOOP - 1;
	if (ELAN1 == elan)
		num_iters = AGGREGATION_LOOP + 1;
	for (int j = 0; j < num_iters; j++)
	{
		//send the packets
		for (int i = 0; i < NUM_PACKETS; i++)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
					pPacketsSizes[i]);
			int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
			if (pPacketsSizes[i] != nBytesSent)
			{
				LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
						"failed!\n", i, pPacketsSizes[i]);
				return false;
			}
		}

		memset(pReceivedPacket, 0, sizeof(pReceivedPacket));
		//receive the aggregated packet
		LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
				"there)\n", MAX_PACKET_SIZE);
		int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
		if (MAX_PACKET_SIZE != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d "
					"bytes) failed!\n", MAX_PACKET_SIZE);
			print_buff(pReceivedPacket, nBytesReceived);
			return false;
		}

		LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
		if (false == DeaggragateAndComparePackets(pReceivedPacket, pPackets,
				pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, isQcncm))
		{
			LOG_MSG_DEBUG("Comparing aggregated packet failed!\n");
			return false;
		}

	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationTimeLimitTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[1][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[1] = {0};
	//Buffer for the packet that will be received
	Byte pReceivedPacket[MAX_PACKET_SIZE] = {0};
	//Size of aggregated packet
	int nTotalPacketsSize = 24;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[1];

	//initialize the packets
	for (int i = 0; i < 1 ; i++)
	{
		pPacketsSizes[i] = 52 + 4*i;
		nTotalPacketsSize += pPacketsSizes[i] + 4; //size of the packet + 4 bytes for index and length
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}
	int nAllPacketsSizes = 0;
	for (int i = 0; i < 1; i++)
		nAllPacketsSizes += pPacketsSizes[i];
	while (0 != nAllPacketsSizes % 4)
	{
		nAllPacketsSizes++;
		nTotalPacketsSize++;  //zero padding for NDP offset to be 4x
	}

	//send the packets
	for (int i = 0; i < 1; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", nTotalPacketsSize);
	int nBytesReceived = output->Receive(pReceivedPacket, nTotalPacketsSize);
	if (nTotalPacketsSize != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", nTotalPacketsSize);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}

	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	if (false == DeaggragateAndComparePackets(pReceivedPacket, pPackets,
			pPacketsSizes, 1, nTotalPacketsSize, isQcncm))
	{
		LOG_MSG_DEBUG("Comparing aggregated packet failed!\n");
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationByteLimitTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[2][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[2];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE] = {0};
	//Size of aggregated packet
	int nTotalPacketsSize = 24;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[2];

	//initialize the packets
	for (int i = 0; i < 2; i++)
	{
		pPacketsSizes[i] = (MAX_PACKET_SIZE / 2) + 10;
		nTotalPacketsSize += pPacketsSizes[i] + 4;
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}


	//send the packets
	for (int i = 0; i < 2; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", nTotalPacketsSize);
	int nBytesReceived = output->Receive(pReceivedPacket, nTotalPacketsSize);
	if (nTotalPacketsSize != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", nTotalPacketsSize);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}

	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	if (false == DeaggragateAndComparePackets(pReceivedPacket, pPackets,
			pPacketsSizes, 2, nTotalPacketsSize, isQcncm))
	{
		LOG_MSG_DEBUG("Comparing aggregated packet failed!\n");
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMDeaggregationMultipleNDPTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	bool bTestResult = true;
	//The packets that the aggregated packet will be made of
	Byte pExpectedPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that the aggregated packet will be made of
	int pPacketsSizes[NUM_PACKETS];
	//Buffers for the packets that will be received
	Byte pReceivedPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//Total size of all the packets that the aggregated packet will be made of
	//(this is the max size of the aggregated packet
	//minus the size of the header and the 2 NDPs)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 36;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[MAX_PACKET_SIZE] = {0};
	//The stream Id byte for every packet - this will determine on which NDP the
	//packet will appear
	Byte pPacketsStreamId[NUM_PACKETS] = {0};
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else {
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
			pPacketsSizes[i] += (pPacketsSizes[i] % 4 == 0 ? 0 :
				4 - pPacketsSizes[i] % 4);
		}
		nTotalPacketsSize -= pPacketsSizes[i];
		pPacketsStreamId[i] = i < 3 ? 0 : 1;
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pExpectedPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pExpectedPackets[i],
					pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pExpectedPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pExpectedPackets[i][size] = i;
				size++;
			}
		}
	}

	//initializing the aggregated packet
	AggregatePacketsWithStreamId(pAggregatedPacket, pExpectedPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, pPacketsStreamId);

	//send the aggregated packet
	LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes)\n",
			sizeof(pAggregatedPacket));
	int nBytesSent = input->Send(pAggregatedPacket, sizeof(pAggregatedPacket));
	if (sizeof(pAggregatedPacket) != nBytesSent)
	{
		LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes) "
				"failed!\n", sizeof(pAggregatedPacket));
		return false;
	}

	//receive the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Reading packet %d from the USB pipe(%d bytes should be "
				"there)\n", i, pPacketsSizes[i]);
		int nBytesReceived = output->Receive(pReceivedPackets[i],
				pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving packet %d from the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			print_buff(pReceivedPackets[i], nBytesReceived);
			return false;
		}
	}

	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	for (int i = 0; i < NUM_PACKETS; i++)
		bTestResult &= !memcmp(pExpectedPackets[i], pReceivedPackets[i],
				pPacketsSizes[i]);

	return bTestResult;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregation2PipesTest(enum ipa_elan_version elan,
		Pipe* input1, Pipe* input2, Pipe* output, enum ipa_ip_type m_eIP,
		bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated
	//packet minus the size of the header and the NDP)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * NUM_PACKETS) - 24;
	//The aggregated packet that will be sent
	Byte pAggregatedPacket[2][MAX_PACKET_SIZE];
	//The size of the sent aggregated packet
	int nAggregatedPacketSize[2] = {0};
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	if (ELAN1 == elan)
	{
		for (int i = 0; i < 2; i++)
		{
			nAggregatedPacketSize[i] += pPacketsSizes[i]; //adding the packet
			nAggregatedPacketSize[i] += 12;  //adding the header
			nAggregatedPacketSize[i] += 16; //adding the NDP
			//initializing the aggregated packet
			AggregatePackets(pAggregatedPacket[i], &pPackets[i],
					&pPacketsSizes[i], 1, nAggregatedPacketSize[i], isQcncm);
		}
	}
	else if (ELAN2 == elan)
	{
		nAggregatedPacketSize[0] += pPacketsSizes[0] + pPacketsSizes[1]; //adding the packets
		nAggregatedPacketSize[0] += 12;  //adding the header
		nAggregatedPacketSize[0] += 12 + 4*2; //adding the NDP
		//initializing the aggregated packet
		AggregatePackets(pAggregatedPacket[0], pPackets, pPacketsSizes, 2,
				nAggregatedPacketSize[0], isQcncm);
	}

	//send the aggregated packet
	if (ELAN1 == elan)
	{
		for (int i = 0; i < 2; i++)
		{
			LOG_MSG_DEBUG("Sending aggregated packet %d into the USB pipe(%d "
					"bytes)\n", i, nAggregatedPacketSize[i]);
			int nBytesSent = input1->Send(pAggregatedPacket[i],
					nAggregatedPacketSize[i]);
			if (nAggregatedPacketSize[i] != nBytesSent)
			{
				LOG_MSG_DEBUG("Sending aggregated packet %d into the USB pipe(%d "
						"bytes) failed!\n", i, nAggregatedPacketSize[i]);
				return false;
			}
		}
	}
	else if (ELAN2 == elan)
	{
		LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d "
				"bytes)\n", nAggregatedPacketSize[0]);
		int nBytesSent = input1->Send(pAggregatedPacket[0],
				nAggregatedPacketSize[0]);
		if (nAggregatedPacketSize[0] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending aggregated packet into the USB pipe(%d bytes) "
					"failed!\n", nAggregatedPacketSize[0]);
			return false;
		}
	}

	//send the packets
	for (int i = 2; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input2->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", MAX_PACKET_SIZE);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}

	//deaggregating the aggregated packet
	return DeaggragateAndComparePackets(pReceivedPacket, pPackets, pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationTimeLimitLoopTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[1][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[1] = {0};
	//Buffer for the packet that will be received
	Byte pReceivedPacket[MAX_PACKET_SIZE] = {0};
	//Size of aggregated packet
	int nTotalPacketsSize = 24;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < 1 ; i++)
	{
		pPacketsSizes[i] = 52 + 4*i;
		nTotalPacketsSize += pPacketsSizes[i] + 4; //size of the packet + 4 bytes for index and length
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}
	int nAllPacketsSizes = 0;
	for (int i = 0; i < 1; i++)
		nAllPacketsSizes += pPacketsSizes[i];
	while (0 != nAllPacketsSizes % 4)
	{
		nAllPacketsSizes++;
		nTotalPacketsSize++;  //zero padding for NDP offset to be 4x
	}

	for (int k = 0; k < AGGREGATION_LOOP; k++)
	{
		//send the packets
		for (int i = 0; i < 1; i++)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
					pPacketsSizes[i]);
			int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
			if (pPacketsSizes[i] != nBytesSent)
			{
				LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
						"failed!\n", i, pPacketsSizes[i]);
				return false;
			}
		}

		//receive the aggregated packet
		LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
				"there)\n", nTotalPacketsSize);
		int nBytesReceived = output->Receive(pReceivedPacket,
				nTotalPacketsSize);
		if (nTotalPacketsSize != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d "
					"bytes) failed!\n", nTotalPacketsSize);
			print_buff(pReceivedPacket, nBytesReceived);
			return false;
		}

		//comparing the received packet to the aggregated packet
		LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
		if (false == DeaggragateAndComparePackets(pReceivedPacket, pPackets,
				pPacketsSizes, 1, nTotalPacketsSize, isQcncm))
		{
			LOG_MSG_DEBUG("Comparing aggregated packet failed!\n");
			return false;
		}
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregation0LimitsTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The expected aggregated packets sizes
	int pAggragatedPacketsSizes[NUM_PACKETS] = {0};
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS ; i++)
	{
		pPacketsSizes[i] = 52 + 4*i;
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	//calculate aggregated packets sizes
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		pAggragatedPacketsSizes[i] += pPacketsSizes[i];
		while (0 != pAggragatedPacketsSizes[i] % 4)
			pAggragatedPacketsSizes[i]++;  //zero padding for NDP offset to be 4x
		pAggragatedPacketsSizes[i] += 28;  //header + NDP
	}

	//send the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Reading packet %d from the USB pipe(%d bytes should be "
				"there)\n", i, pAggragatedPacketsSizes[i]);
		int nBytesReceived = output->Receive(pReceivedPackets[i],
				pAggragatedPacketsSizes[i]);
		if (pAggragatedPacketsSizes[i] != nBytesReceived)
		{
			LOG_MSG_DEBUG("Receiving aggregated packet %d from the USB pipe(%d "
					"bytes) failed!\n", i, pAggragatedPacketsSizes[i]);
			print_buff(pReceivedPackets[i], nBytesReceived);
			return false;
		}
	}


	//comparing the received packet to the aggregated packet
	LOG_MSG_DEBUG("Checking sent.vs.received packet\n");
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		if (false == DeaggragateAndCompareOnePacket(pReceivedPackets[i],
				pPackets[i], pPacketsSizes[i], pAggragatedPacketsSizes[i],
				isQcncm))
		{
			LOG_MSG_DEBUG("Comparing aggregated packet %d failed!\n", i);
			return false;
		}
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationMultiplePacketsTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[MAX_PACKETS_IN_NDP + 1][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[MAX_PACKETS_IN_NDP + 1];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated packet
	//minus the size of the header and the 2 NDPs)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (4 * (MAX_PACKETS_IN_NDP + 1)) - 36;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[MAX_PACKETS_IN_NDP + 1];
	//initialize the packets
	for (int i = 0; i < MAX_PACKETS_IN_NDP + 1; i++)
	{
		if (MAX_PACKETS_IN_NDP == i)
			pPacketsSizes[i] = nTotalPacketsSize;
		else
		{
			pPacketsSizes[i] = nTotalPacketsSize / (MAX_PACKETS_IN_NDP + 1);
			pPacketsSizes[i] += (pPacketsSizes[i] % 4 == 0 ? 0 :
				4 - pPacketsSizes[i] % 4);
		}
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			nIPv4DSTAddr = ntohl(0x7F000001);
			memcpy (&pPackets[i][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
					sizeof(nIPv4DSTAddr));
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	//send the packets
	for (int i = 0; i < MAX_PACKETS_IN_NDP + 1; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
					pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);

		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", MAX_PACKET_SIZE);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE);
	if (MAX_PACKET_SIZE != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE);
		print_buff(pReceivedPacket, nBytesReceived);
		return false;
	}

	//deaggregating the aggregated packet
	return DeaggragateAndComparePackets(pReceivedPacket, pPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE, isQcncm);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationDifferentStreamIdsTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated
	//packet minus the size of the header and the NDPs)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (16 * NUM_PACKETS) - 12;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];
	Byte pPacketsStreamId[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		pPacketsStreamId[i] = i;
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize + 12;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	nIPv4DSTAddr = ntohl(0x7F000001);
	memcpy (&pPackets[0][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80101);
	memcpy (&pPackets[1][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80102);
	memcpy (&pPackets[2][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80103);
	memcpy (&pPackets[3][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80104);
	memcpy (&pPackets[4][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));

	//send the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", MAX_PACKET_SIZE + 12);
	int nBytesReceived = output->Receive(pReceivedPacket, MAX_PACKET_SIZE + 12);
	if (MAX_PACKET_SIZE + 12 != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes) "
				"failed!\n", MAX_PACKET_SIZE + 12);
		print_buff(pReceivedPacket, nBytesReceived + 12);
		return false;
	}

	//deaggregating the aggregated packet
	return DeaggragateAndComparePacketsWithStreamId(pReceivedPacket, pPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE + 12, pPacketsStreamId);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::MBIMAggregationNoInterleavingStreamIdsTest(
		enum ipa_elan_version elan, Pipe* input, Pipe* output,
		enum ipa_ip_type m_eIP, bool isQcncm)
{
	//The packets that will be sent
	Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE];
	//The real sizes of the packets that will be sent
	int pPacketsSizes[NUM_PACKETS];
	//Buffer for the packet that will be received
	Byte pReceivedPacket[2*MAX_PACKET_SIZE];
	//Total size of all sent packets (this is the max size of the aggregated packet
	//minus the size of the header and the NDPs)
	int nTotalPacketsSize = MAX_PACKET_SIZE - (16 * NUM_PACKETS) - 12;
	uint32_t nIPv4DSTAddr;
	size_t pIpPacketsSizes[NUM_PACKETS];
	Byte pPacketsStreamId[NUM_PACKETS];

	//initialize the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		pPacketsStreamId[i] = i % 2;
		if (NUM_PACKETS - 1 == i)
			pPacketsSizes[i] = nTotalPacketsSize + 12;
		else
			pPacketsSizes[i] = nTotalPacketsSize / NUM_PACKETS;
		while (0 != pPacketsSizes[i] % 4)
			pPacketsSizes[i]++;
		nTotalPacketsSize -= pPacketsSizes[i];
		if (ELAN1 == elan)
		{
			for (int j = 0; j < pPacketsSizes[i]; j++)
				pPackets[i][j] = i;
		}
		else if (ELAN2 == elan)
		{
			// Load input data (IP packet) from file
			pIpPacketsSizes[i] = MAX_PACKET_SIZE;
			if (!LoadDefaultPacket(m_eIP, pPackets[i], pIpPacketsSizes[i]))
			{
				LOG_MSG_ERROR("Failed default Packet");
				return false;
			}
			int size = pIpPacketsSizes[i];
			while (size < pPacketsSizes[i])
			{
				pPackets[i][size] = i;
				size++;
			}
		}
	}

	nIPv4DSTAddr = ntohl(0x7F000001);
	memcpy (&pPackets[0][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80101);
	memcpy (&pPackets[1][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0x7F000001);
	memcpy (&pPackets[2][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0xC0A80101);
	memcpy (&pPackets[3][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));
	nIPv4DSTAddr = ntohl(0x7F000001);
	memcpy (&pPackets[4][IPV4_DST_ADDR_OFFSET],&nIPv4DSTAddr,
			sizeof(nIPv4DSTAddr));

	//send the packets
	for (int i = 0; i < NUM_PACKETS; i++)
	{
		LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes)\n", i,
				pPacketsSizes[i]);
		int nBytesSent = input->Send(pPackets[i], pPacketsSizes[i]);
		if (pPacketsSizes[i] != nBytesSent)
		{
			LOG_MSG_DEBUG("Sending packet %d into the USB pipe(%d bytes) "
					"failed!\n", i, pPacketsSizes[i]);
			return false;
		}
	}

	//receive the aggregated packet
	LOG_MSG_DEBUG("Reading packet from the USB pipe(%d bytes should be "
			"there)\n", MAX_PACKET_SIZE + 12);
	int nBytesReceived = output->Receive(pReceivedPacket,
			MAX_PACKET_SIZE + 12);
	if (MAX_PACKET_SIZE + 12 != nBytesReceived)
	{
		LOG_MSG_DEBUG("Receiving aggregated packet from the USB pipe(%d bytes)"
				" failed!\n", MAX_PACKET_SIZE + 12);
		print_buff(pReceivedPacket, nBytesReceived + 12);
		return false;
	}

	//deaggregating the aggregated packet
	return DeaggragateAndComparePacketsWithStreamId(pReceivedPacket, pPackets,
			pPacketsSizes, NUM_PACKETS, MAX_PACKET_SIZE + 12, pPacketsStreamId);
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::DeaggragateAndComparePackets(
		Byte pAggregatedPacket[MAX_PACKET_SIZE],
		Byte pExpectedPackets[NUM_PACKETS][MAX_PACKET_SIZE],
		int pPacketsSizes[NUM_PACKETS], int nNumPackets, int nAggregatedPacketSize,
		bool isQcncm)
{
	int nPacketNum = 0;
	int i = 0;
	int nNdpStart = 0;
	Byte pNdpIndex[2] = {0};
	Byte pNdpLen[2] = {0};
	if (0x4e != pAggregatedPacket[i] || 0x43 != pAggregatedPacket[i+1] ||
			0x4d != pAggregatedPacket[i+2]|| 0x48 != pAggregatedPacket[i+3])
	{
		LOG_MSG_DEBUG("Error: Wrong NTH16 signature: 0x%02x 0x%02x 0x%02x "
				"0x%02x(should be 0x4e, 0x43, 0x4d, 0x48)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1],
				pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
		return false;
	}
	i += 4;
	if (0x0c != pAggregatedPacket[i] || 0x00 != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong header length: 0x%02x 0x%02x(should be 0x0c, "
				"0x00)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1]);
		return false;
	}
	i += 4;  //ignoring sequence number
	if ((nAggregatedPacketSize & 0x00FF) != pAggregatedPacket[i] ||
			(nAggregatedPacketSize >> 8) != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong aggregated packet length: 0x%02x 0x%02x"
				"(should be 0x%02x, 0x%02x)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1],
				nAggregatedPacketSize & 0x00FF, nAggregatedPacketSize >> 8);
		return false;
	}
	i += 2;
	pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
	pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
	//reading the NDP
	while (0x00 != pNdpIndex[0] || 0x00 != pNdpIndex[1])
	{
		i = pNdpIndex[0] + 256*pNdpIndex[1];  //NDP should begin here
		nNdpStart = i;
		if (!isQcncm)
		{
			if (0x49 != pAggregatedPacket[i] || 0x50 != pAggregatedPacket[i+1] ||
					0x53 != pAggregatedPacket[i+2] || 0x00 != pAggregatedPacket[i+3])
			{
				LOG_MSG_DEBUG("Error: Wrong NDP16 signature: 0x%02x 0x%02x "
						"0x%02x 0x%02x(should be 0x49, 0x50, 0x53, 0x00)\n",
						pAggregatedPacket[i], pAggregatedPacket[i+1],
						pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
				return false;
			}
		}
		else
		{
			if (0x44 != pAggregatedPacket[i] || 0x4e != pAggregatedPacket[i+1] ||
					0x51 != pAggregatedPacket[i+2] || 0x50 != pAggregatedPacket[i+3])
			{
				LOG_MSG_DEBUG("Error: Wrong QNDP signature: 0x%02x 0x%02x "
						"0x%02x 0x%02x(should be 0x44, 0x4e, 0x51, 0x50)\n",
						pAggregatedPacket[i], pAggregatedPacket[i+1],
						pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
				return false;
			}
		}
		i += 4;
		pNdpLen[0] = pAggregatedPacket[i];  //least significant byte
		pNdpLen[1] = pAggregatedPacket[i+1];  //most significant byte
		if (0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2] ||
				0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] -1])
		{
			LOG_MSG_DEBUG("Error: Wrong end of NDP: 0x%02x 0x%02x(should be 0x00,"
					" 0x00)\n",
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2],
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 1]);
			return false;
		}
		i += 2;
		pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
		pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
		i += 2;
		while (i <= nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2)
		{ //going over all the datagrams in this NDP
			Byte pDatagramIndex[2] = {0};
			Byte pDatagramLen[2] = {0};
			int packetIndex = 0;
			pDatagramIndex[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramIndex[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			if (0x00 == pDatagramIndex[0] && 0x00 == pDatagramIndex[1])
				break;  //zero padding after all datagrams
			if (nPacketNum >= nNumPackets)
			{
				LOG_MSG_DEBUG("Error: wrong number of packets: %d(should be %d)\n",
						nPacketNum, nNumPackets);
				return false;
			}
			pDatagramLen[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramLen[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			packetIndex = pDatagramIndex[0] + 256*pDatagramIndex[1];
			if (pDatagramLen[0] + 256*pDatagramLen[1] != pPacketsSizes[nPacketNum])
			{
				LOG_MSG_DEBUG("Error: Wrong packet %d length: 0x%02x 0x%02x"
						"(should be %d)\n", nPacketNum, pDatagramLen[0],
						pDatagramLen[1], pPacketsSizes[nPacketNum]);
				return false;
			}
			if (0 != memcmp(pExpectedPackets[nPacketNum],
					&pAggregatedPacket[packetIndex], pPacketsSizes[nPacketNum]))
			{
				LOG_MSG_DEBUG("Error: Comparison of packet %d failed!\n",
						nPacketNum);
				return false;
			}
			nPacketNum++;
		}
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

void MBIMAggregationScenarios::AggregatePackets(
		Byte pAggregatedPacket[MAX_PACKET_SIZE]/*ouput*/,
		Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE],
		int pPacketsSizes[NUM_PACKETS], int nNumPackets,
		int nAggregatedPacketSize, bool isQcncm)
{
	int i = 0;
	int pDatagramIndexes[NUM_PACKETS] = {0};
	int nNdpIndex = 0;
	int nNdpLen = 0;
	//NTH16 signature
	pAggregatedPacket[i] = 0x4e;
	pAggregatedPacket[i+1] = 0x43;
	pAggregatedPacket[i+2] = 0x4d;
	pAggregatedPacket[i+3] = 0x48;
	i += 4;
	//header length
	pAggregatedPacket[i] = 0x0c;
	pAggregatedPacket[i+1] = 0x00;
	i += 2;
	//sequence number
	pAggregatedPacket[i] = 0x00;
	pAggregatedPacket[i+1] = 0x00;
	i += 2;
	//aggregated packet length
	pAggregatedPacket[i] = nAggregatedPacketSize & 0x00FF;
	pAggregatedPacket[i+1] = nAggregatedPacketSize >> 8;
	i += 2;
	//NDP index
	for (int j = 0; j < nNumPackets; j++)
		nNdpIndex += pPacketsSizes[j];
	nNdpIndex += i + 2;
	while (0 != nNdpIndex % 4)
		nNdpIndex++;
	pAggregatedPacket[i] = nNdpIndex & 0x00FF;
	pAggregatedPacket[i+1] = nNdpIndex >> 8;
	i += 2;
	//packets
	for (int j = 0; j < nNumPackets; j++)
	{
		pDatagramIndexes[j] = i;
		for (int k = 0; k < pPacketsSizes[j]; k++)
		{
			pAggregatedPacket[i] = pPackets[j][k];
			i++;
		}
	}
	while (i < nNdpIndex)
	{
		pAggregatedPacket[i] = 0x00;
		i++;
	}
	if (!isQcncm)
	{
		//NDP16 signature
		pAggregatedPacket[i] = 0x49;
		pAggregatedPacket[i+1] = 0x50;
		pAggregatedPacket[i+2] = 0x53;
		pAggregatedPacket[i+3] = 0x00;
	}
	else
	{
		//QNDP signature
		pAggregatedPacket[i] = 0x44;
		pAggregatedPacket[i+1] = 0x4e;
		pAggregatedPacket[i+2] = 0x51;
		pAggregatedPacket[i+3] = 0x50;
	}
	i += 4;
	//NDP length
	nNdpLen = 4*nNumPackets + 8 + 2;
	while (nNdpLen % 4 != 0)
		nNdpLen += 2;
	pAggregatedPacket[i] = nNdpLen & 0x00FF;
	pAggregatedPacket[i+1] = nNdpLen >> 8;
	i += 2;
	//next NDP
	pAggregatedPacket[i] = 0x00;
	pAggregatedPacket[i+1] = 0x00;
	i += 2;
	for (int j = 0; j < nNumPackets; j++)
	{
		//datagram index
		pAggregatedPacket[i] = pDatagramIndexes[j] & 0x00FF;
		pAggregatedPacket[i+1] = pDatagramIndexes[j] >> 8;
		i += 2;
		//datagram length
		pAggregatedPacket[i] = pPacketsSizes[j] & 0x00FF;
		pAggregatedPacket[i+1] = pPacketsSizes[j] >> 8;
		i += 2;
	}
	//zeros in the end of NDP
	while (i < nAggregatedPacketSize)
	{
		pAggregatedPacket[i] = 0x00;
		i++;
	}
}

/////////////////////////////////////////////////////////////////////////////////

void MBIMAggregationScenarios::AggregatePacketsWithStreamId(
		Byte pAggregatedPacket[MAX_PACKET_SIZE]/*ouput*/,
		Byte pPackets[NUM_PACKETS][MAX_PACKET_SIZE],
		int pPacketsSizes[NUM_PACKETS], int nNumPackets, int nAggregatedPacketSize,
		Byte pPacketsStreamId[NUM_PACKETS])
{
	int i = 0;
	int n = 0;
	int pDatagramIndexes[NUM_PACKETS] = {0};
	int nNdpIndex[NUM_PACKETS] = {0};
	int nNdpLen = 0;
	Byte currStreamId = pPacketsStreamId[0];
	int nNdpFirstPacket[NUM_PACKETS] = {0};
	int nNdpAfterLastPacket[NUM_PACKETS] = {0};
	int nNumNDPs = 0;
	for (n = 0; n < nNumPackets; n++)
	{
		if (currStreamId != pPacketsStreamId[n])
		{
			nNdpAfterLastPacket[nNumNDPs] = n;
			nNumNDPs++;
			nNdpFirstPacket[nNumNDPs] = n;
			currStreamId = pPacketsStreamId[n];
		}
	}
	nNdpAfterLastPacket[nNumNDPs] = n;
	nNumNDPs++;
	//calculate NDP indexes
	nNdpIndex[0] += 12;  //adding the header
	for (int j = 0; j < nNumNDPs; j++)
	{
		for (n = nNdpFirstPacket[j]; n < nNdpAfterLastPacket[j]; n++)
			nNdpIndex[j] += pPacketsSizes[n];  //adding the packets
		while (0 != nNdpIndex[j] % 4)
			nNdpIndex[j]++;
		if (j < nNumNDPs - 1)
			nNdpIndex[j+1] += nNdpIndex[j] + 12 + 4*(nNdpAfterLastPacket[j] -
					nNdpFirstPacket[j]);  //adding the location after the current NDP to the next NDP
	}
	//start building the aggregated packet
	//NTH16 signature
	pAggregatedPacket[i] = 0x4e;
	pAggregatedPacket[i+1] = 0x43;
	pAggregatedPacket[i+2] = 0x4d;
	pAggregatedPacket[i+3] = 0x48;
	i += 4;
	//header length
	pAggregatedPacket[i] = 0x0c;
	pAggregatedPacket[i+1] = 0x00;
	i += 2;
	//sequence number
	pAggregatedPacket[i] = 0x00;
	pAggregatedPacket[i+1] = 0x00;
	i += 2;
	//aggregated packet length
	pAggregatedPacket[i] = nAggregatedPacketSize & 0x00FF;
	pAggregatedPacket[i+1] = nAggregatedPacketSize >> 8;;
	i += 2;
	//first NDP index
	pAggregatedPacket[i] = nNdpIndex[0] & 0x00FF;
	pAggregatedPacket[i+1] = nNdpIndex[0] >> 8;
	i += 2;
	for (n = 0; n < nNumNDPs; n++)
	{
		//packets
		for (int j = nNdpFirstPacket[n]; j < nNdpAfterLastPacket[n]; j++)
		{
			pDatagramIndexes[j] = i;
			for (int k = 0; k < pPacketsSizes[j]; k++)
			{
				pAggregatedPacket[i] = pPackets[j][k];
				i++;
			}
		}
		while (i < nNdpIndex[n])
		{
			pAggregatedPacket[i] = 0x00;
			i++;
		}
		//NDP signature
		pAggregatedPacket[i] = 0x49;
		pAggregatedPacket[i+1] = 0x50;
		pAggregatedPacket[i+2] = 0x53;
		pAggregatedPacket[i+3] = pPacketsStreamId[nNdpFirstPacket[n]];
		i += 4;
		//NDP length
		nNdpLen = 4*(nNdpAfterLastPacket[n] - nNdpFirstPacket[n]) + 8 + 2;
		while (nNdpLen % 4 != 0)
			nNdpLen += 2;
		pAggregatedPacket[i] = nNdpLen & 0x00FF;
		pAggregatedPacket[i+1] = nNdpLen >> 8;
		i += 2;
		//next NDP
		pAggregatedPacket[i] = nNdpIndex[n+1] & 0x00FF;
		pAggregatedPacket[i+1] = nNdpIndex[n+1] >> 8;
		i += 2;
		for (int j = nNdpFirstPacket[n]; j < nNdpAfterLastPacket[n]; j++)
		{
			//datagram index
			pAggregatedPacket[i] = pDatagramIndexes[j] & 0x00FF;
			pAggregatedPacket[i+1] = pDatagramIndexes[j] >> 8;
			i += 2;
			//datagram length
			pAggregatedPacket[i] = pPacketsSizes[j] & 0x00FF;
			pAggregatedPacket[i+1] = pPacketsSizes[j] >> 8;
			i += 2;
		}
		//zeros in the end of NDP
		while (i < nNdpIndex[n] + nNdpLen)
		{
			pAggregatedPacket[i] = 0x00;
			i++;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::DeaggragateAndCompareOnePacket(
		Byte pAggregatedPacket[MAX_PACKET_SIZE],
		Byte pExpectedPacket[MAX_PACKET_SIZE], int nPacketsSize,
		int nAggregatedPacketSize, bool isQcncm)
{
	int nPacketNum = 0;
	int i = 0;
	int nNdpStart = 0;
	Byte pNdpIndex[2] = {0};
	Byte pNdpLen[2] = {0};
	if (0x4e != pAggregatedPacket[i] || 0x43 != pAggregatedPacket[i+1] ||
			0x4d != pAggregatedPacket[i+2]|| 0x48 != pAggregatedPacket[i+3])
	{
		LOG_MSG_DEBUG("Error: Wrong NTH16 signature: 0x%02x 0x%02x 0x%02x "
				"0x%02x(should be 0x4e, 0x43, 0x4d, 0x48)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1],
				pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
		return false;
	}
	i += 4;
	if (0x0c != pAggregatedPacket[i] || 0x00 != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong header length: 0x%02x 0x%02x(should be 0x0c,"
				" 0x00)\n", pAggregatedPacket[i], pAggregatedPacket[i+1]);
		return false;
	}
	i += 4;  //ignoring sequence number
	if ((nAggregatedPacketSize & 0x00FF) != pAggregatedPacket[i] ||
			(nAggregatedPacketSize >> 8) != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong aggregated packet length: 0x%02x 0x%02x"
				"(should be 0x%02x, 0x%02x)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1],
				nAggregatedPacketSize & 0x00FF, nAggregatedPacketSize >> 8);
		return false;
	}
	i += 2;
	pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
	pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
	//reading the NDP
	while (0x00 != pNdpIndex[0] || 0x00 != pNdpIndex[1])
	{
		i = pNdpIndex[0] + 256*pNdpIndex[1];  //NDP should begin here
		nNdpStart = i;
		if (!isQcncm)
		{
			if (0x49 != pAggregatedPacket[i] || 0x50 != pAggregatedPacket[i+1] ||
					0x53 != pAggregatedPacket[i+2] || 0x00 != pAggregatedPacket[i+3])
			{
				LOG_MSG_DEBUG("Error: Wrong NDP16 signature: 0x%02x 0x%02x "
						"0x%02x 0x%02x(should be 0x49, 0x50, 0x53, 0x00)\n",
						pAggregatedPacket[i], pAggregatedPacket[i+1],
						pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
				return false;
			}
		}
		else
		{
			if (0x44 != pAggregatedPacket[i] || 0x4e != pAggregatedPacket[i+1] ||
					0x51 != pAggregatedPacket[i+2] || 0x50 != pAggregatedPacket[i+3])
			{
				LOG_MSG_DEBUG("Error: Wrong QNDP signature: 0x%02x 0x%02x "
						"0x%02x 0x%02x(should be 0x44, 0x4e, 0x51, 0x50)\n",
						pAggregatedPacket[i], pAggregatedPacket[i+1],
						pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
				return false;
			}
		}
		i += 4;
		pNdpLen[0] = pAggregatedPacket[i];  //least significant byte
		pNdpLen[1] = pAggregatedPacket[i+1];  //most significant byte
		if (0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2] ||
				0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] -1])
		{
			LOG_MSG_DEBUG("Error: Wrong end of NDP: 0x%02x 0x%02x(should be "
					"0x00, 0x00)\n",
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2],
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 1]);
			return false;
		}
		i += 2;
		pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
		pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
		i += 2;
		while (i <= nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2)
		{ //going over all the datagrams in this NDP
			Byte pDatagramIndex[2] = {0};
			Byte pDatagramLen[2] = {0};
			int packetIndex = 0;
			pDatagramIndex[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramIndex[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			if (0x00 == pDatagramIndex[0] && 0x00 == pDatagramIndex[1])
				break;  //zero padding after all datagrams
			if (nPacketNum > 1)
			{
				LOG_MSG_DEBUG("Error: wrong number of packets: %d(should be %d)\n",
						nPacketNum, 1);
				return false;
			}
			pDatagramLen[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramLen[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			packetIndex = pDatagramIndex[0] + 256*pDatagramIndex[1];
			if (pDatagramLen[0] + 256*pDatagramLen[1] != nPacketsSize)
			{
				LOG_MSG_DEBUG("Error: Wrong packet %d length: 0x%02x 0x%02x"
						"(should be %d)\n", nPacketNum, pDatagramLen[0],
						pDatagramLen[1], nPacketsSize);
				return false;
			}
			if (0 != memcmp(pExpectedPacket, &pAggregatedPacket[packetIndex],
					nPacketsSize))
			{
				LOG_MSG_DEBUG("Error: Comparison of packet %d failed!\n",
						nPacketNum);
				return false;
			}
			nPacketNum++;
		}
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationScenarios::DeaggragateAndComparePacketsWithStreamId(
		Byte pAggregatedPacket[MAX_PACKET_SIZE],
		Byte pExpectedPackets[][MAX_PACKET_SIZE], int pPacketsSizes[],
		int nNumPackets, int nAggregatedPacketSize,
		Byte pPacketsStreamId[NUM_PACKETS])
{
	int nPacketNum = 0;
	int i = 0;
	int nNdpStart = 0;
	Byte pNdpIndex[2] = {0};
	Byte pNdpLen[2] = {0};
	if (0x4e != pAggregatedPacket[i] || 0x43 != pAggregatedPacket[i+1] ||
			0x4d != pAggregatedPacket[i+2]|| 0x48 != pAggregatedPacket[i+3])
	{
		LOG_MSG_DEBUG("Error: Wrong NTH16 signature: 0x%02x 0x%02x 0x%02x "
				"0x%02x(should be 0x4e, 0x43, 0x4d, 0x48)\n",
				pAggregatedPacket[i], pAggregatedPacket[i+1],
				pAggregatedPacket[i+2], pAggregatedPacket[i+3]);
		return false;
	}
	i += 4;
	if (0x0c != pAggregatedPacket[i] || 0x00 != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong header length: 0x%02x 0x%02x(should be "
				"0x0c, 0x00)\n",pAggregatedPacket[i], pAggregatedPacket[i+1]);
		return false;
	}
	i += 4;  //ignoring sequence number
	if ((nAggregatedPacketSize & 0x00FF) != pAggregatedPacket[i] ||
			(nAggregatedPacketSize >> 8) != pAggregatedPacket[i+1])
	{
		LOG_MSG_DEBUG("Error: Wrong aggregated packet length: 0x%02x 0x%02x"
				"(should be 0x%02x, 0x%02x)\n", pAggregatedPacket[i],
				pAggregatedPacket[i+1], nAggregatedPacketSize & 0x00FF,
				nAggregatedPacketSize >> 8);
		return false;
	}
	i += 2;
	pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
	pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
	//reading the NDP
	while (0x00 != pNdpIndex[0] || 0x00 != pNdpIndex[1])
	{
		i = pNdpIndex[0] + 256*pNdpIndex[1];  //NDP should begin here
		nNdpStart = i;
		if (0x49 != pAggregatedPacket[i] || 0x50 != pAggregatedPacket[i+1] ||
				0x53 != pAggregatedPacket[i+2])
		{
			LOG_MSG_DEBUG("Error: Wrong NDP16 signature: 0x%02x 0x%02x 0x%02x"
					"(should be 0x49, 0x50, 0x53)\n", pAggregatedPacket[i],
					pAggregatedPacket[i+1], pAggregatedPacket[i+2]);
			return false;
		}
		if (pPacketsStreamId[nPacketNum] != pAggregatedPacket[i+3])
		{
			LOG_MSG_DEBUG("Error: Wrong NDP stream id: 0x%02x(should be 0x%02x)\n",
					pAggregatedPacket[i+3], pPacketsStreamId[nPacketNum]);
			return false;
		}
		i += 4;
		pNdpLen[0] = pAggregatedPacket[i];  //least significant byte
		pNdpLen[1] = pAggregatedPacket[i+1];  //most significant byte
		if (0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2] ||
				0x00 != pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] -1])
		{
			LOG_MSG_DEBUG("Error: Wrong end of NDP: 0x%02x 0x%02x(should be 0x00, "
					"0x00)\n",
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2],
					pAggregatedPacket[nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 1]);
			return false;
		}
		i += 2;
		pNdpIndex[0] = pAggregatedPacket[i];  //least significant byte
		pNdpIndex[1] = pAggregatedPacket[i+1];  //most significant byte
		i += 2;
		while (i <= nNdpStart + pNdpLen[0] + 256*pNdpLen[1] - 2)
		{ //going over all the datagrams in this NDP
			Byte pDatagramIndex[2] = {0};
			Byte pDatagramLen[2] = {0};
			int packetIndex = 0;
			pDatagramIndex[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramIndex[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			if (0x00 == pDatagramIndex[0] && 0x00 == pDatagramIndex[1])
				break;  //zero padding after all datagrams
			if (nPacketNum >= nNumPackets)
			{
				LOG_MSG_DEBUG("Error: wrong number of packets: %d(should be %d)\n",
						nPacketNum, nNumPackets);
				return false;
			}
			pDatagramLen[0] = pAggregatedPacket[i];  //least significant byte
			pDatagramLen[1] = pAggregatedPacket[i+1];  //most significant byte
			i += 2;
			packetIndex = pDatagramIndex[0] + 256*pDatagramIndex[1];
			if (pDatagramLen[0] + 256*pDatagramLen[1] != (int)pPacketsSizes[nPacketNum])
			{
				LOG_MSG_DEBUG("Error: Wrong packet %d length: 0x%02x 0x%02x"
						"(should be %d)\n", nPacketNum, pDatagramLen[0],
						pDatagramLen[1], pPacketsSizes[nPacketNum]);
				return false;
			}
			if (0 != memcmp(pExpectedPackets[nPacketNum],
					&pAggregatedPacket[packetIndex], pPacketsSizes[nPacketNum]))
			{
				LOG_MSG_DEBUG("Error: Comparison of packet %d failed!\n",
						nPacketNum);
				return false;
			}
			nPacketNum++;
		}
	}

	return true;
}


/////////////////////////////////////////////////////////////////////////////////
//	            Configuration 9 tests - DMA mode for Elan1                     //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregationTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregationTest()
	{
		m_name = "DmaModeMBIMAggregationTest";
		m_description = "MBIM Aggregation test - sends 5 packets and receives 1 "
				"aggregated packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregationTest(ELAN1,
				&m_UsbNoAggToIpaPipeAgg, &m_IpaToUsbPipeAggr, IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMDeaggregationOnePacketTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMDeaggregationOnePacketTest()
	{
		m_name = "DmaModeMBIMDeaggregationOnePacketTest";
		m_description = "MBIM Deaggregation one packet test - sends an aggregated packet "
				"made of 1 packet and receives 1 packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationOnePacketTest(ELAN1,
				&m_UsbDeaggToIpaPipeNoAgg, &m_IpaToUsbPipeNoAgg, IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMMultipleDeaggregationAndAggregationTest:
	public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMMultipleDeaggregationAndAggregationTest()
	{
		m_name = "DmaModeMBIMMultipleDeaggregationAndAggregationTest";
		m_description = "MBIM Multiple Deaggregation and Aggregation test - sends 5 aggregated "
				"packets each one made of 1 packet and receives an aggregated packet made of the"
				"5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMMultipleDeaggregationAndAggregationTest(
				ELAN1, &m_UsbDeaggToIpaPipeAgg, &m_IpaToUsbPipeAggr, IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregationLoopTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregationLoopTest()
	{
		m_name = "DmaModeMBIMggregationLoopTest";
		m_description = "MBIM Aggregation Loop test - sends 5 packets and expects to"
				"receives 1 aggregated packet a few times";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregationLoopTest(ELAN1,
				&m_UsbNoAggToIpaPipeAgg, &m_IpaToUsbPipeAggr, IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregationTimeLimitTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregationTimeLimitTest()
	{
		m_name = "DmaModeMBIMAggregationTimeLimitTest";
		m_description = "MBIM Aggregation time limit test - sends 1 small packet "
				"smaller than the byte limit and receives 1 aggregated packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitTest(ELAN1,
				&m_UsbNoAggToIpaPipeAggTime, &m_IpaToUsbPipeAggTime, IPA_IP_v4,
				false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregationByteLimitTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregationByteLimitTest()
	{
		m_name = "DmaModeMBIMAggregationByteLimitTest";
		m_description = "MBIM Aggregation byte limit test - sends 2 packets that together "
				"are larger than the byte limit ";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregationByteLimitTest(ELAN1,
				&m_UsbNoAggToIpaPipeAgg, &m_IpaToUsbPipeAggr, IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregation2PipesTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregation2PipesTest()
	{
		m_name = "DmaModeMBIMAggregation2PipesTest";
		m_description = "MBIM Aggregation 2 pipes test - sends 3 packets from one pipe"
				"and 2 aggregated packets made of 1 packet from another pipe and "
				"receives 1 aggregated packet made of all 5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregation2PipesTest(ELAN1,
				&m_UsbDeaggToIpaPipeAgg, &m_UsbNoAggToIpaPipeAgg, &m_IpaToUsbPipeAggr,
				IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregationTimeLimitLoopTest: public MBIMAggregationTestFixture {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregationTimeLimitLoopTest()
	{
		m_name = "DmaModeMBIMAggregationTimeLimitLoopTest";
		m_description = "MBIM Aggregation time limit loop test - sends 5 small packet "
				"smaller than the byte limit and receives 5 aggregated packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitLoopTest(ELAN1,
				&m_UsbNoAggToIpaPipeAggTime, &m_IpaToUsbPipeAggTime, IPA_IP_v4,
				false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
//			     Configuration 10 tests - DMA mode for Elan1                   //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class DmaModeMBIMAggregation0LimitsTest: public MBIMAggregationTestFixtureConf10 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	DmaModeMBIMAggregation0LimitsTest()
	{
		m_name = "DmaModeMBIMAggregation0LimitsTest";
		m_description = "MBIM Aggregation 0 limits test - sends 5 packets and expects"
				"to get each packet back aggregated (both size and time limits are 0)";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	bool Run()
	{
		return MBIMAggregationScenarios::MBIMAggregation0LimitsTest(ELAN1,
				&m_UsbToIpaPipeAggZeroLimits, &m_IpaToUsbPipeAggZeroLimits,
				IPA_IP_v4, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
//				    Configuration 11 tests - Elan2 and later                   //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationTest()
	{
		m_name = "MBIMAggregationTest";
		m_description = "MBIM Aggregation test - sends 5 packets and receives 1 "
				"aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTest(ELAN2, &m_UsbToIpaPipe,
				&m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMDeaggregationTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMDeaggregationTest()
	{
		m_name = "MBIMDeaggregationTest";
		m_description = "MBIM Deaggregation test - sends an aggregated packet made from"
				"5 packets and receives 5 packets";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

class MBIMDeaggregationOnePacketTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMDeaggregationOnePacketTest()
	{
		m_name = "MBIMDeaggregationOnePacketTest";
		m_description = "MBIM Deaggregation one packet test - sends an aggregated packet made"
				"of 1 packet and receives 1 packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationOnePacketTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


class MBIMDeaggregationAndAggregationTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMDeaggregationAndAggregationTest()
	{
		m_name = "MBIMDeaggregationAndAggregationTest";
		m_description = "MBIM Deaggregation and Aggregation test - sends an aggregated "
				"packet made from 5 packets and receives the same aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationAndAggregationTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


class MBIMMultipleDeaggregationAndAggregationTest:
	public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMMultipleDeaggregationAndAggregationTest()
	{
		m_name = "MBIMMultipleDeaggregationAndAggregationTest";
		m_description = "MBIM Multiple Deaggregation and Aggregation test - sends 5 aggregated "
				"packets each one made of 1 packet and receives an aggregated packet made of the"
				"5 packets";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMMultipleDeaggregationAndAggregationTest(
				ELAN2, &m_UsbToIpaPipeDeagg, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationLoopTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationLoopTest()
	{
		m_name = "MBIMggregationLoopTest";
		m_description = "MBIM Aggregation Loop test - sends 5 packets and expects to"
				"receives 1 aggregated packet a few times";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationLoopTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationTimeLimitTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationTimeLimitTest()
	{
		m_name = "MBIMAggregationTimeLimitTest";
		m_description = "MBIM Aggregation time limit test - sends 1 small packet "
				"smaller than the byte limit and receives 1 aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregationTime();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAggTime, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationByteLimitTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationByteLimitTest()
	{
		m_name = "MBIMAggregationByteLimitTest";
		m_description = "MBIM Aggregation byte limit test - sends 2 packets that together "
				"are larger than the byte limit ";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationByteLimitTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMDeaggregationMultipleNDPTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMDeaggregationMultipleNDPTest()
	{
		m_name = "MBIMDeaggregationMultipleNDPTest";
		m_description = "MBIM Deaggregation multiple NDP test - sends an aggregated"
				"packet made from 5 packets and 2 NDPs and receives 5 packets";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationMultipleNDPTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregation2PipesTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregation2PipesTest()
	{
		m_name = "MBIMAggregation2PipesTest";
		m_description = "MBIM Aggregation 2 pipes test - sends 3 packets from one pipe"
				"and an aggregated packet made of 2 packets from another pipe and "
				"receives 1 aggregated packet made of all 5 packets";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregation2PipesTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationTimeLimitLoopTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationTimeLimitLoopTest()
	{
		m_name = "MBIMAggregationTimeLimitLoopTest";
		m_description = "MBIM Aggregation time limit loop test - sends 5 small packet "
				"smaller than the byte limit and receives 5 aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregationTime();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitLoopTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAggTime, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationMultiplePacketsTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationMultiplePacketsTest()
	{
		m_name = "MBIMAggregationMultiplePacketsTest";
		m_description = "MBIM Aggregation multiple packets test - sends 9 packets "
				"with same stream ID and receives 1 aggregated packet with 2 NDPs";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationMultiplePacketsTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregation0LimitsTest: public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregation0LimitsTest()
	{
		m_name = "MBIMAggregation0LimitsTest";
		m_description = "MBIM Aggregation 0 limits test - sends 5 packets and expects"
				"to get each packet back aggregated (both size and time limits are 0)";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation0Limits();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregation0LimitsTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg0Limits, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationDifferentStreamIdsTest:
	public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationDifferentStreamIdsTest()
	{
		m_name = "MBIMAggregationDifferentStreamIdsTest";
		m_description = "MBIM Aggregation different stream IDs test - sends 5 packets"
				"with different stream IDs and receives 1 aggregated packet made of 5"
				"NDPs";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		m_eIP = IPA_IP_v4;
		const char aBypass[NUM_PACKETS][20] = {{"Bypass1"}, {"Bypass2"}, {"Bypass3"},
				{"Bypass4"}, {"Bypass5"}};
		uint32_t nTableHdl[NUM_PACKETS];
		bool bRetVal = true;
		IPAFilteringTable cFilterTable0;
		struct ipa_flt_rule_add sFilterRuleEntry;
		struct ipa_ioc_get_hdr sGetHeader[NUM_PACKETS];
		uint8_t aHeadertoAdd[NUM_PACKETS];

		for (int i = 0; i < NUM_PACKETS; i++)
			aHeadertoAdd[i] = (uint8_t)i;

		LOG_MSG_STACK("Entering Function");
		memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));
		for (int i = 0; i < NUM_PACKETS; i++)
			memset(&sGetHeader[i], 0, sizeof(sGetHeader[i]));
		// Create Header:
		// Allocate Memory, populate it, and add in to the Header Insertion.
		struct ipa_ioc_add_hdr * pHeaderDescriptor = NULL;
		pHeaderDescriptor = (struct ipa_ioc_add_hdr *) calloc(1,
				sizeof(struct ipa_ioc_add_hdr)
						+ NUM_PACKETS * sizeof(struct ipa_hdr_add));
		if (!pHeaderDescriptor)
		{
			LOG_MSG_ERROR("calloc failed to allocate pHeaderDescriptor");
			bRetVal = false;
			goto bail;
		}

		pHeaderDescriptor->commit = true;
		pHeaderDescriptor->num_hdrs = NUM_PACKETS;
		// Adding Header No1.
		strcpy(pHeaderDescriptor->hdr[0].name, "StreamId0\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[0].hdr, (void*)&aHeadertoAdd[0],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[0].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[0].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[0].is_partial = false;
		pHeaderDescriptor->hdr[0].status     = -1; // Return Parameter

		// Adding Header No2.
		strcpy(pHeaderDescriptor->hdr[1].name, "StreamId1\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[1].hdr, (void*)&aHeadertoAdd[1],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[1].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[1].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[1].is_partial = false;
		pHeaderDescriptor->hdr[1].status     = -1; // Return Parameter

		// Adding Header No3.
		strcpy(pHeaderDescriptor->hdr[2].name, "StreamId2\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[2].hdr, (void*)&aHeadertoAdd[2],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[2].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[2].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[2].is_partial = false;
		pHeaderDescriptor->hdr[2].status     = -1; // Return Parameter

		// Adding Header No4.
		strcpy(pHeaderDescriptor->hdr[3].name, "StreamId3\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[3].hdr, (void*)&aHeadertoAdd[3],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[3].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[3].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[3].is_partial = false;
		pHeaderDescriptor->hdr[3].status     = -1; // Return Parameter

		// Adding Header No5.
		strcpy(pHeaderDescriptor->hdr[4].name, "StreamId4\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[4].hdr, (void*)&aHeadertoAdd[4],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[4].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[4].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[4].is_partial = false;
		pHeaderDescriptor->hdr[4].status     = -1; // Return Parameter

		for (int i = 0; i < NUM_PACKETS; i++)
			strcpy(sGetHeader[i].name, pHeaderDescriptor->hdr[i].name);


		if (!m_HeaderInsertion.AddHeader(pHeaderDescriptor))
		{
			LOG_MSG_ERROR("m_HeaderInsertion.AddHeader(pHeaderDescriptor) Failed.");
			bRetVal = false;
			goto bail;
		}
		for (int i = 0; i < NUM_PACKETS; i++)
		{
			if (!m_HeaderInsertion.GetHeaderHandle(&sGetHeader[i]))
			{
				LOG_MSG_ERROR(" Failed");
				bRetVal = false;
				goto bail;
			}
			LOG_MSG_DEBUG("Received Header %d Handle = 0x%x", i, sGetHeader[i].hdl);
		}

		for (int i = 0; i < NUM_PACKETS; i++)
		{
			if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass[i],
					IPA_CLIENT_TEST2_CONS, sGetHeader[i].hdl,&nTableHdl[i]))
			{
				LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
				bRetVal = false;
				goto bail;
			}
		}

		LOG_MSG_INFO("Creation of 5 bypass routing tables completed successfully");

		// Creating Filtering Rules
		cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST_PROD, true, NUM_PACKETS);
		LOG_MSG_INFO("Creation of filtering table completed successfully");

		// Configuring Filtering Rule No.1
		cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[0]; //put here the handle corresponding to Routing Rule 1
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
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return Value
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[1]; //put here the handle corresponding to Routing Rule 2
		sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
		{
			LOG_MSG_ERROR ("Adding Rule(1) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(1)->status);
		}

		// Configuring Filtering Rule No.3
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[2]; //put here the handle corresponding to Routing Rule 2
		sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0xC0A80102; // Filter DST_IP == 192.168.1.2.

		if (
				((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
		{
			LOG_MSG_ERROR ("Adding Rule(2) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(2)->status);
		}

		// Configuring Filtering Rule No.4
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[3]; //put here the handle corresponding to Routing Rule 2
		sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0xC0A80103; // Filter DST_IP == 192.168.1.3.

		if (
				((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
		{
			LOG_MSG_ERROR ("Adding Rule(3) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(2)->status);
		}

		// Configuring Filtering Rule No.5
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[4]; //put here the handle corresponding to Routing Rule 2
		sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0xC0A80104; // Filter DST_IP == 192.168.1.4.

		if (
				((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
		{
			LOG_MSG_ERROR ("Adding Rule(4) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(2)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(2)->status);
		}

	bail:
		Free(pHeaderDescriptor);
		LOG_MSG_STACK(
				"Leaving Function (Returning %s)", bRetVal?"True":"False");
		return bRetVal;
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationDifferentStreamIdsTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class MBIMAggregationNoInterleavingStreamIdsTest:
	public MBIMAggregationTestFixtureConf11 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	MBIMAggregationNoInterleavingStreamIdsTest()
	{
		m_name = "MBIMAggregationNoInterleavingStreamIdsTest";
		m_description = "MBIM Aggregation no interleaving stream IDs test - sends 5 packets"
				"with interleaving stream IDs (0, 1, 0, 1, 0) and receives 1 aggregated "
				"packet made of 5 NDPs";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		m_eIP = IPA_IP_v4;
		const char aBypass[2][20] = {{"Bypass1"}, {"Bypass2"}};
		uint32_t nTableHdl[2];
		bool bRetVal = true;
		IPAFilteringTable cFilterTable0;
		struct ipa_flt_rule_add sFilterRuleEntry;
		struct ipa_ioc_get_hdr sGetHeader[2];
		uint8_t aHeadertoAdd[2];

		for (int i = 0; i < 2; i++)
			aHeadertoAdd[i] = (uint8_t)i;

		LOG_MSG_STACK("Entering Function");
		memset(&sFilterRuleEntry, 0, sizeof(sFilterRuleEntry));
		for (int i = 0; i < 2; i++)
			memset(&sGetHeader[i], 0, sizeof(sGetHeader[i]));
		// Create Header:
		// Allocate Memory, populate it, and add in to the Header Insertion.
		struct ipa_ioc_add_hdr * pHeaderDescriptor = NULL;
		pHeaderDescriptor = (struct ipa_ioc_add_hdr *) calloc(1,
				sizeof(struct ipa_ioc_add_hdr)
						+ 2 * sizeof(struct ipa_hdr_add));
		if (!pHeaderDescriptor)
		{
			LOG_MSG_ERROR("calloc failed to allocate pHeaderDescriptor");
			bRetVal = false;
			goto bail;
		}

		pHeaderDescriptor->commit = true;
		pHeaderDescriptor->num_hdrs = 2;
		// Adding Header No1.
		strcpy(pHeaderDescriptor->hdr[0].name, "StreamId0\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[0].hdr, (void*)&aHeadertoAdd[0],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[0].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[0].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[0].is_partial = false;
		pHeaderDescriptor->hdr[0].status     = -1; // Return Parameter

		// Adding Header No2.
		strcpy(pHeaderDescriptor->hdr[1].name, "StreamId1\0"); // Header's Name
		memcpy(pHeaderDescriptor->hdr[1].hdr, (void*)&aHeadertoAdd[1],
				sizeof(uint8_t)); //Header's Data
		pHeaderDescriptor->hdr[1].hdr_len    = sizeof(uint8_t);
		pHeaderDescriptor->hdr[1].hdr_hdl    = -1; //Return Value
		pHeaderDescriptor->hdr[1].is_partial = false;
		pHeaderDescriptor->hdr[1].status     = -1; // Return Parameter

		for (int i = 0; i < 2; i++)
			strcpy(sGetHeader[i].name, pHeaderDescriptor->hdr[i].name);


		if (!m_HeaderInsertion.AddHeader(pHeaderDescriptor))
		{
			LOG_MSG_ERROR("m_HeaderInsertion.AddHeader(pHeaderDescriptor) Failed.");
			bRetVal = false;
			goto bail;
		}
		for (int i = 0; i < 2; i++)
		{
			if (!m_HeaderInsertion.GetHeaderHandle(&sGetHeader[i]))
			{
				LOG_MSG_ERROR(" Failed");
				bRetVal = false;
				goto bail;
			}
			LOG_MSG_DEBUG("Received Header %d Handle = 0x%x", i, sGetHeader[i].hdl);
		}

		for (int i = 0; i < 2; i++)
		{
			if (!CreateBypassRoutingTable(&m_Routing, m_eIP, aBypass[i],
					IPA_CLIENT_TEST2_CONS, sGetHeader[i].hdl,&nTableHdl[i]))
			{
				LOG_MSG_ERROR("CreateBypassRoutingTable Failed\n");
				bRetVal = false;
				goto bail;
			}
		}

		LOG_MSG_INFO("Creation of 2 bypass routing tables completed successfully");

		// Creating Filtering Rules
		cFilterTable0.Init(m_eIP,IPA_CLIENT_TEST_PROD, true, 2);
		LOG_MSG_INFO("Creation of filtering table completed successfully");

		// Configuring Filtering Rule No.1
		cFilterTable0.GeneratePresetRule(1,sFilterRuleEntry);
		sFilterRuleEntry.at_rear = true;
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return value
		sFilterRuleEntry.rule.action=IPA_PASS_TO_ROUTING;
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[0]; //put here the handle corresponding to Routing Rule 1
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
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(0)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(0)->status);
		}

		// Configuring Filtering Rule No.2
		sFilterRuleEntry.flt_rule_hdl=-1; // return Value
		sFilterRuleEntry.status = -1; // return Value
		sFilterRuleEntry.rule.rt_tbl_hdl=nTableHdl[1]; //put here the handle corresponding to Routing Rule 2
		sFilterRuleEntry.rule.attrib.u.v4.dst_addr = 0xC0A80101; // Filter DST_IP == 192.168.1.1.
		if (
				((uint8_t)-1 == cFilterTable0.AddRuleToTable(sFilterRuleEntry)) ||
				!m_Filtering.AddFilteringRule(cFilterTable0.GetFilteringTable())
			)
		{
			LOG_MSG_ERROR ("Adding Rule(1) to Filtering block Failed.");
			bRetVal = false;
			goto bail;
		}
		else
		{
			LOG_MSG_DEBUG("flt rule hdl0=0x%x, status=0x%x\n",
					cFilterTable0.ReadRuleFromTable(1)->flt_rule_hdl,
					cFilterTable0.ReadRuleFromTable(1)->status);
		}

	bail:
		Free(pHeaderDescriptor);
		LOG_MSG_STACK(
				"Leaving Function (Returning %s)", bRetVal?"True":"False");
		return bRetVal;
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationNoInterleavingStreamIdsTest(
				ELAN2, &m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, false);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
//		      Configuration 12 tests - Qcncm for Elan2 and later               //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationTest()
	{
		m_name = "QcncmMBIMAggregationTest";
		m_description = "MBIM Aggregation test - sends 5 packets and receives 1 "
				"aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTest(ELAN2, &m_UsbToIpaPipe,
				&m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class  QcncmMBIMDeaggregationTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	 QcncmMBIMDeaggregationTest()
	 {
		m_name = "QcncmMBIMDeaggregationTest";
		m_description = "MBIM Deaggregation test - sends an aggregated packet made from"
				"5 packets and receives 5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

class QcncmMBIMDeaggregationOnePacketTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMDeaggregationOnePacketTest()
	{
		m_name = "QcncmMBIMDeaggregationOnePacketTest";
		m_description = "MBIM Deaggregation one packet test - sends an aggregated packet made"
				"of 1 packet and receives 1 packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationOnePacketTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


class QcncmMBIMDeaggregationAndAggregationTest:
	public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMDeaggregationAndAggregationTest()
	{
		m_name = "QcncmMBIMDeaggregationAndAggregationTest";
		m_description = "MBIM Deaggregation and Aggregation test - sends an aggregated "
				"packet made from 5 packets and receives the same aggregated packet";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationAndAggregationTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


class QcncmMBIMMultipleDeaggregationAndAggregationTest:
	public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMMultipleDeaggregationAndAggregationTest()
	{
		m_name = "QcncmMBIMMultipleDeaggregationAndAggregationTest";
		m_description = "MBIM Multiple Deaggregation and Aggregation test - sends 5 aggregated "
				"packets each one made of 1 packet and receives an aggregated packet made of the"
				"5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMMultipleDeaggregationAndAggregationTest(
				ELAN2, &m_UsbToIpaPipeDeagg, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////

};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationLoopTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationLoopTest()
	{
		m_name = "QcncmMBIMAggregationLoopTest";
		m_description = "MBIM Aggregation Loop test - sends 5 packets and expects to"
				"receives 1 aggregated packet a few times";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationLoopTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMDeaggregationMultipleNDPTest:
	public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMDeaggregationMultipleNDPTest()
	{
		m_name = "QcncmMBIMDeaggregationMultipleNDPTest";
		m_description = "MBIM Deaggregation multiple NDP test - sends an aggregated"
				"packet made from 5 packets and 2 NDPs and receives 5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRulesDeaggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMDeaggregationMultipleNDPTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_IpaToUsbPipe, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationMultiplePacketsTest:
	public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationMultiplePacketsTest()
	{
		m_name = "QcncmMBIMAggregationMultiplePacketsTest";
		m_description = "MBIM Aggregation multiple packets test - sends 9 packets "
				"with same stream ID and receives 1 aggregated packet with 2 NDPs";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationMultiplePacketsTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregation2PipesTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregation2PipesTest()
	{
		m_name = "QcncmMBIMAggregation2PipesTest";
		m_description = "MBIM Aggregation 2 pipes test - sends 3 packets from one pipe"
				"and an aggregated packet made of 2 packets from another pipe and "
				"receives 1 aggregated packet made of all 5 packets";
		m_minIPAHwType = IPA_HW_v1_0;
		m_maxIPAHwType = IPA_HW_v2_1;
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregation2PipesTest(ELAN2,
				&m_UsbToIpaPipeDeagg, &m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationTimeLimitTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationTimeLimitTest()
	{
		m_name = "QcncmMBIMAggregationTimeLimitTest";
		m_description = "MBIM Aggregation time limit test - sends 1 small packet "
				"smaller than the byte limit and receives 1 aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregationTime();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAggTime, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationByteLimitTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationByteLimitTest()
	{
		m_name = "QcncmMBIMAggregationByteLimitTest";
		m_description = "MBIM Aggregation byte limit test - sends 2 packets that together "
				"are larger than the byte limit ";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationByteLimitTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregationTimeLimitLoopTest:
	public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregationTimeLimitLoopTest()
	{
		m_name = "QcncmMBIMAggregationTimeLimitLoopTest";
		m_description = "MBIM Aggregation time limit loop test - sends 5 small packet "
				"smaller than the byte limit and receives 5 aggregated packet";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregationTime();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregationTimeLimitLoopTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAggTime, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

class QcncmMBIMAggregation0LimitsTest: public MBIMAggregationTestFixtureConf12 {
public:

	/////////////////////////////////////////////////////////////////////////////////

	QcncmMBIMAggregation0LimitsTest()
	{
		m_name = "QcncmMBIMAggregation0LimitsTest";
		m_description = "MBIM Aggregation 0 limits test - sends 5 packets and expects"
				"to get each packet back aggregated (both size and time limits are 0)";
	}

	/////////////////////////////////////////////////////////////////////////////////

	virtual bool AddRules()
	{
		return AddRules1HeaderAggregation0Limits();
	} // AddRules()

	/////////////////////////////////////////////////////////////////////////////////

	bool TestLogic()
	{
		return MBIMAggregationScenarios::MBIMAggregation0LimitsTest(ELAN2,
				&m_UsbToIpaPipe, &m_IpaToUsbPipeAgg0Limits, m_eIP, true);
	}

	/////////////////////////////////////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

//Those tests should be run with configuration number 9 for Elan1
//Please look at the Fixture for more configurations update.
static DmaModeMBIMAggregationTest dmaModeMBIMAggregationTest;
static DmaModeMBIMDeaggregationOnePacketTest dmaModeMBIMDeaggregationOnePacketTest;
static DmaModeMBIMMultipleDeaggregationAndAggregationTest
			dmaModeMBIMMultipleDeaggregationAndAggregationTest;
static DmaModeMBIMAggregationLoopTest dmaModeMBIMAggregationLoopTest;
static DmaModeMBIMAggregationTimeLimitTest dmaModeMBIMAggregationTimeLimitTest;
static DmaModeMBIMAggregation2PipesTest dmaModeMBIMAggregation2PipesTest;
static DmaModeMBIMAggregationByteLimitTest dmaModeMBIMAggregationByteLimitTest;
static DmaModeMBIMAggregationTimeLimitLoopTest
			dmaModeMBIMAggregationTimeLimitLoopTest;

//This tests should be run with configuration number 10 for Elan1
static DmaModeMBIMAggregation0LimitsTest dmaModeMBIMAggregation0LimitsTest;

//Those tests should be run with configuration number 11 for Elan2 and later
static MBIMAggregationTest mbimAggregationTest;
static MBIMDeaggregationTest mbimDeaggregationTest;
static MBIMDeaggregationOnePacketTest mbimDeaggregationOnePacketTest;
static MBIMDeaggregationAndAggregationTest mbimDeaggregationAndAggregationTest;
static MBIMMultipleDeaggregationAndAggregationTest
		mbimMultipleDeaggregationAndAggregationTest;
static MBIMAggregationLoopTest mbimAggregationLoopTest;
static MBIMDeaggregationMultipleNDPTest mbimDeaggregationMultipleNDPTest;
static MBIMAggregationMultiplePacketsTest mbimAggregationMultiplePacketsTest;
static MBIMAggregation2PipesTest mbimAggregation2PipesTest;
static MBIMAggregationNoInterleavingStreamIdsTest
		mbimAggregationNoInterleavingStreamIdsTest;
static MBIMAggregationDifferentStreamIdsTest mbimAggregationDifferentStreamIdsTest;
static MBIMAggregationTimeLimitTest mbimAggregationTimeLimitTest;
static MBIMAggregationByteLimitTest mbimAggregationByteLimitTest;
static MBIMAggregationTimeLimitLoopTest mbimAggregationTimeLimitLoopTest;
static MBIMAggregation0LimitsTest mbimAggregation0LimitsTest;

//Those tests should be run with configuration number 12 for qcncm on Elan2 and later
static QcncmMBIMAggregationTest qcncmMBIMAggregationTest;
static QcncmMBIMDeaggregationTest qcncmMBIMDeaggregationTest;
static QcncmMBIMDeaggregationOnePacketTest qcncmMBIMDeaggregationOnePacketTest;
static QcncmMBIMDeaggregationAndAggregationTest
		qcncmMBIMDeaggregationAndAggregationTest;
static QcncmMBIMMultipleDeaggregationAndAggregationTest
		qcncmMBIMMultipleDeaggregationAndAggregationTest;
static QcncmMBIMAggregationLoopTest qcncmMBIMAggregationLoopTest;
static QcncmMBIMDeaggregationMultipleNDPTest qcncmMBIMDeaggregationMultipleNDPTest;
static QcncmMBIMAggregationMultiplePacketsTest
		qcncmMBIMAggregationMultiplePacketsTest;
static QcncmMBIMAggregation2PipesTest qcncmMBIMAggregation2PipesTest;
static QcncmMBIMAggregationTimeLimitTest qcncmMBIMAggregationTimeLimitTest;
static QcncmMBIMAggregationByteLimitTest qcncmMBIMAggregationByteLimitTest;
static QcncmMBIMAggregationTimeLimitLoopTest qcncmMBIMAggregationTimeLimitLoopTest;
static QcncmMBIMAggregation0LimitsTest qcncmMBIMAggregation0LimitsTest;

/////////////////////////////////////////////////////////////////////////////////
//                                  EOF                                      ////
/////////////////////////////////////////////////////////////////////////////////
