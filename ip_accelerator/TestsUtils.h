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

#ifndef __TESTS_UTILS__H__
#define __TESTS_UTILS__H__

#include <stdarg.h>
#include <vector>
#include <string>
#include <linux/if_ether.h>
#include "TestBase.h"
#include "Constants.h"
#include "RoutingDriverWrapper.h"
#include "InterfaceAbstraction.h"
#include "Logger.h"
#include "Constants.h"

using namespace std;
#define TEST_APP_VERSION "2.00"

#define Free(x) do { if (x) {free(x); x = NULL; } } while (0)

#define ETH2_DST_ADDR_OFFSET (0)
#define ETH2_SRC_ADDR_OFFSET (ETH_ALEN)
#define ETH2_ETH_TYPE_OFFSET (ETH2_SRC_ADDR_OFFSET + ETH_ALEN)
#define ETH2_ETH_TYPE_LEN (2)
#define ETH2_PAYLOAD_OFFSET (ETH2_ETH_TYPE_OFFSET + ETH2_ETH_TYPE_LEN)

#define WLAN_HDR_SIZE (4)
#define RNDIS_HDR_SIZE (44)

// 26 ROME WLAN Frame =
// 4	ROME WLAN header +
// 14	IEEE 802.3 +
// 8	802.2 LLC/SNAP
#define _802_3_HDR_SIZE (26)

// [WLAN][ETH2] header
#define WLAN_ETH2_HDR_SIZE (WLAN_HDR_SIZE + ETH_HLEN)

// [RNDIS][ETH2] header
#define RNDIS_ETH2_HDR_SIZE (RNDIS_HDR_SIZE + ETH_HLEN)
#define IP4_PACKET_SIZE (70) // Arbitrary number

// OFFSET = sizeof(struct rndis_pkt_hdr) - RNDIS_HDR_OFST(data_ofst)
#define RNDIS_DATA_OFFSET (36)

// [WLAN][802.3] header
#define WLAN_802_3_HDR_SIZE (WLAN_HDR_SIZE + _802_3_HDR_SIZE)

enum msgType {
	ERROR = 0,
	DEBUG,
	INFO,
	STACK
};

/**
	@brief
	Do not Use this function. Use MACROs instead.

	@details
	Do not Use this function.
	Instead use the MACROs: LOG_MSG_ERROR, LOG_MSG_INFO & LOG_MSG_DEBUG
	*/
void __log_msg(enum msgType,
		const char *filename,
		int line, const char *function,
		const char *format, ...);

#define LOG_MSG_ERROR(...) \
__log_msg(ERROR, __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_MSG_DEBUG(...) \
__log_msg(DEBUG, __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_MSG_INFO(...) \
__log_msg(INFO, __FILE__, __LINE__, __func__, __VA_ARGS__)
#define LOG_MSG_STACK(...) \
__log_msg(STACK, __FILE__, __LINE__, __func__, __VA_ARGS__)

/*#define LOG_MSG_ERROR(x...)
__log_msg(ERROR, __FILE__, __LINE__, __func__, x)
#define LOG_MSG_DEBUG(x...)
__log_msg(DEBUG, __FILE__, __LINE__, __func__, x)
#define LOG_MSG_INFO(x...)
__log_msg(INFO, __FILE__, __LINE__, __func__, x)
#define LOG_MSG_STACK(x...)
__log_msg(STACK, __FILE__, __LINE__, __func__, x)*/

/**
	@brief
	Function loads a default IPv4 / IPv6 Packet

	@param [in] eIP - Type of Packet to load (IPA_IP_v4 / IPA_IP_v6)
	@param [in] pBuffer - pointer to the destination buffer
	@param [in,out] nMaxSize - The size of the buffer.
	Upon function return,
	the total number of bytes copied will be stored in this parameter.
	@return boolean indicating whether the
	operation completed successfully or not.

	@details
	Function loads a default IPv4 / IPv6 packet into pBuffer.
	*/
bool LoadDefaultPacket(
	enum ipa_ip_type eIP,
	uint8_t *pBuffer,
	size_t &nMaxSize);

bool LoadDefaultEth2Packet(
	enum ipa_ip_type eIP,
	uint8_t *pBuffer,
	size_t &nMaxSize);

bool LoadDefaultWLANEth2Packet(
	enum ipa_ip_type eIP,
	uint8_t *pBuffer,
	size_t &nMaxSize);

bool LoadDefaultWLAN802_32Packet(
	enum ipa_ip_type eIP,
	uint8_t *pBuffer,
	size_t &nMaxSize);

/**
	@brief
	Function loads a default IPv4 / IPv6 Packet

	@param [in] eIP - Type of Packet to load (IPA_IP_v4 / IPA_IP_v6)
	@param [in] extHdrType - Type of IPV6 extension header(FRAGMENT / NONE)
	@param [in] pBuffer - pointer to the destination buffer
	@param [in,out] nMaxSize - The size of the buffer.
	Upon function return,
	the total number of bytes copied will be stored in this parameter.
	@return boolean indicating whether the
	operation completed successfully or not.

	@details
	Function loads a default IPv4 / IPv6 packet into pBuffer.
	*/
bool LoadDefaultPacket(
		enum ipa_ip_type eIP,
		enum ipv6_ext_hdr_type extHdrType,
		uint8_t *pBuffer,
		size_t &nMaxSize);
/**
	@brief
	Function Sends a Packet, Receive a packet
	and compares the received result with an expected buffer

	@param [in] pSink - Destination to which a packet will be sent.
	@param [in] pSendBuffer -
	Pointer to a buffer containing the packet that will be sent.
	@param [in] nSendBuffSize - The size of the data in the packet.
	@param [in] pSource - Source from which a packet will be received.
	@param [in] pExpectedBuffer - Pointer a
	buffer containing the expected packet (from the receiver)
	@param [in] nExpectedBuffSize - The size of
	valid data within pExpectedBuffer.
	@return Boolean indicating whether the operation
	completed successfully and the buffers matching or not.

	@details
	Function sends a packet to pSink, and receives a packet from pSource.
	The packet received from pSource
	is compared to the expected data from pExpectedBuffer.
	If ExpectData is identical to the
	received data, the function returns TRUE.
	*/
bool SendReceiveAndCompare(
		InterfaceAbstraction * pSink,
		uint8_t *pSendBuffer,
		size_t nSendBuffSize,
		InterfaceAbstraction * pSource,
		uint8_t *pExpectedBuffer,
		size_t nExpectedBuffSize);

/**
	@brief
	This function creates a bypass rule within a table in the Routing block

	@param [in] pRouting - pointer to the Routing Class
	@param [in] eIP - Type of Packet to load (IPA_IP_v4 / IPA_IP_v6)
	@param [in] pTableName - pointer to the Table's Name.
	@param [in] eRuleDestination - destination of the bypass rule.
	@param [in] uHeaderHandle -
	handle to the Header that should be Added (0 should be used as default).
	@param [out] pTableHdl -
	pointer to the table Handle (Can be Null)
	@return boolean indicating whether
	the operation completed successfully or not.

	@details
	This function creates bypass rule within a table in the Routing block.
	*/
bool CreateBypassRoutingTable(
		RoutingDriverWrapper * pRouting,
		enum ipa_ip_type eIP,
		const char *pTableName,
		enum ipa_client_type eRuleDestination,
		uint32_t uHeaderHandle,
		uint32_t *pTableHdl);

/**
	@brief
		Configures the sytem to one of the pre-determined
		configurations.

		@param [in] testConfiguration - Configuration number
		@param [in] params - additional parameters
	@return void

	@details
		Writes the configuration index to /dev/ipa_test. In case
		the system has already been configured, returns.
*/
void ConfigureScenario(int testConfiguration);
void ConfigureScenario(int testConfiguration, const char *params);


int ConfigureSystem(int testConfiguration, int fd);
int ConfigureSystem(int testConfiguration, int fd, const char *params);

/**
	@brief
		Compares to data buffers.

		@param [in] goldenBuffer - Pointer to the first data
			buffer
		@param [in] goldenSize - First data buffer size
		@param [in] receivedBuffer - Pointer to the second data
			buffer
		@param [in] receivedSize - Second data buffer size
		@return True - the buffers are identical. False
			otherwise.

		@details
	In case the sizes are differnt, false is returned.
*/
bool CompareResultVsGolden(
		unsigned char *goldenBuffer,
		unsigned int goldenSize,
		unsigned char *receivedBuffer,
		unsigned int receivedSize);

/**
	@brief
		Loads a file to memory

		@param [in] fileFullPath
		@param [inout] sizeLoaded - returns the number of bytes
			which were read from the file
	@return Address of the loaded data buffer

		@details
		Allocates memory by itself, user should free the memory

*/
unsigned char *LoadFileToMemory(
		const string & fileFullPath,
		unsigned int *sizeLoaded);

/**
		@brief
		Checks whether a file exists on disk

		@param [in] filename
	@return True if the file exists, false otherwise.

		@details
*/
bool file_exists(const char *filename);

/**
		@brief
		Prints a data buffer.
		@param [in] data - Pointer to the data
		@param [in] size - How many bytes to print
	@return void

		@details
*/
void print_buff(void *data, size_t size);

void add_buff(uint8_t *data, size_t size, uint8_t val);

class Eth2Helper {
public:
	static const Byte m_ETH2_IP4_HDR[ETH_HLEN];

	static bool LoadEth2IP4Header(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadEth2IP6Header(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadEth2IP4Packet(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadEth2IP6Packet(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);
};

class WlanHelper {
public:
	static const Byte m_WLAN_HDR[WLAN_HDR_SIZE];

	static bool LoadWlanHeader(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadWlanEth2IP4Header(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadWlanEth2IP6Header(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadWlanEth2IP4Packet(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);

	static bool LoadWlanEth2IP4PacketByLength(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t len,
		uint8_t padValue);

	static bool LoadWlanEth2IP6Packet(
		uint8_t *pBuffer,
		size_t bufferSize,
		size_t *pLen);
};

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(1)     /* set alignment to 1 byte boundary */
struct RndisHeader {
	uint32_t MessageType;
	uint32_t MessageLength;
	uint32_t DataOffset;
	uint32_t DataLength;
	uint32_t OOBDataOffset;
	uint32_t OOBDataLength;
	uint32_t OOBNumber;
	uint32_t PacketInfoOffset;
	uint32_t PacketInfoLength;
	uint64_t Reserved;
};

struct RndisEtherHeader {
	struct RndisHeader rndisHeader;
	struct ethhdr etherHeader;
};
#pragma pack(pop)   /* restore original alignment from stack */

class RNDISAggregationHelper {
public:
	static const size_t RNDIS_AGGREGATION_BYTE_LIMIT = 1024;

	static bool LoadRNDISHeader(
		uint8_t *pBuffer,
		size_t bufferSize,
		uint32_t messageLength,
		size_t *pLen);

	static bool LoadRNDISEth2IP4Header(
		uint8_t *pBuffer,
		size_t bufferSize,
		uint32_t messageLength,
		size_t *pLen);

	static bool LoadRNDISPacket(
		enum ipa_ip_type eIP,
		uint8_t *pBuffer,
		size_t &nMaxSize);

	static bool LoadEtherPacket(
		enum ipa_ip_type eIP,
		uint8_t *pBuffer,
		size_t &nMaxSize);

	static bool ComparePackets(
		Byte *pPacket1,
		int pPacket1Size,
		Byte *pPacket2,
		int pPacket2Size);

	static bool CompareEthervsRNDISPacket(
		Byte *pIPPacket,
		size_t ipPacketSize,
		Byte *pRNDISPacket,
		size_t rndisPacketSize);

	static bool CompareIPvsRNDISPacket(
		Byte *pIPPacket,
		int ipPacketSize,
		Byte *pRNDISPacket,
		size_t rndisPacketSize);
};

#endif
