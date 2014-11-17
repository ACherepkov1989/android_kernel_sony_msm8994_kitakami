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

#include "USBIntegrationFixture.h"
#include "Constants.h"
#include "TestsUtils.h"
#include <time.h>
#include <string.h>

#define IPV4_SRC_ADDR_OFFSET (12)
#define IPV4_DST_ADDR_OFFSET (16)

USBIntegrationFixture::USBIntegrationFixture() : m_uBufferSize(0)
{
	memset(m_aBuffer, 0, sizeof(m_aBuffer));
}


bool USBIntegrationFixture::Setup()
{
	LOG_MSG_STACK("Entering Function");

	ConfigureScenario(PHASE_SIX_TEST_CONFIGURATION);

	m_Producer.Open(INTERFACE0_TO_IPA_DATA_PATH,
			INTERFACE0_FROM_IPA_DATA_PATH);
	m_Consumer1.Open(INTERFACE1_TO_IPA_DATA_PATH,
			INTERFACE1_FROM_IPA_DATA_PATH);

	if (!m_Routing.DeviceNodeIsOpened()) {
		LOG_MSG_ERROR("Routing block is not ready for immediate commands!\n");
		return false;
	}
	if (!m_Filtering.DeviceNodeIsOpened()) {
		LOG_MSG_ERROR("Filtering block is not ready for immediate commands!\n");
		return false;
	}
	if (!m_HeaderInsertion.DeviceNodeIsOpened())
	{
		LOG_MSG_ERROR("Header Insertion block is not ready for immediate commands!\n");
		return false;
	}
	m_Routing.Reset(IPA_IP_v4);
	m_Routing.Reset(IPA_IP_v6);

	LOG_MSG_STACK("Leaving Function (Returning True)");
	return true;
} // Setup()

bool USBIntegrationFixture::Run() {
	m_uBufferSize = BUFF_MAX_SIZE;
	LOG_MSG_STACK("Entering Function");

	// Add the relevant filtering rules
	if (!AddRules()) {
		LOG_MSG_ERROR("Failed adding Filtering / Routing rules.");
		return false;
	}

	LOG_MSG_INFO("Please configure the network interfaces and press any key...\n");
	getchar();

	// Load input data (IP packet) from file
	if (!LoadDefaultPacket(m_eIP, m_aBuffer, m_uBufferSize)) {
		LOG_MSG_ERROR("Failed default Packet");
		return false;
	}

	if (!TestLogic()) {
		LOG_MSG_ERROR("Test failed, Input and expected output mismatch.");
		return false;
	}

	LOG_MSG_STACK("Leaving Function (Returning True)");
	return true;

} // Run()

bool USBIntegrationFixture::TestLogic()
{
	bool bRetVal = true;

	LOG_MSG_STACK("Entering Function");
	LOG_MSG_INFO("Please Push packets and press any key to continue...\n");
	getchar();

	do
	{
		do // Wait for data to arrive in USB1_PROD pipe
		{
			LOG_MSG_DEBUG("Waiting for Packet (1 s)");
			// Sleep for 100 msec
			struct timespec time;
			time.tv_sec = 0;
			time.tv_nsec = 100e6;
			nanosleep(&time, NULL);
			m_nBufSize = m_Consumer1.ReceiveData(m_aBuffer,BUFF_MAX_SIZE);
			LOG_MSG_INFO("Packet Recieved (%d)Bytes!",m_nBufSize);
		} while (0 == m_nBufSize);

		if (!SwapIPv4Addresses(m_aBuffer,m_nBufSize))
		{
			LOG_MSG_ERROR("Function SwapIPAddresses Failed... Aborting...");
			bRetVal = false;
			return bRetVal;
		}

		// Send the data
		m_Producer.SendData(m_aBuffer,m_nBufSize);
		LOG_MSG_INFO("Packet Sent (%d)Bytes",m_nBufSize);
	} while(1);

	LOG_MSG_STACK("Leaving Function (Returning %s)",bRetVal?"True":"False");
	return bRetVal;
}

bool USBIntegrationFixture::SwapIPv4Addresses(Byte *buf, size_t size)
{
	Byte aSrcIP[4],aDstIP[4];

	if (size < 20) {// Minimal Size that includes first 20 Bytes
		LOG_MSG_ERROR("Function SwapIPAddresses Failed... Aborting...");
		return false;
	}
	memcpy(aSrcIP, (buf + IPV4_SRC_ADDR_OFFSET), sizeof(aSrcIP));
	memcpy(aDstIP, (buf + IPV4_DST_ADDR_OFFSET), sizeof(aDstIP));
	LOG_MSG_INFO("Replacing Source aSrcIP=%d.%d.%d.%d with Destination aDstIP=%d.%d.%d.%d",
			aSrcIP[0],aSrcIP[1],aSrcIP[2],aSrcIP[3],
			aDstIP[0],aDstIP[1],aDstIP[2],aDstIP[3]);
	memcpy((buf + IPV4_SRC_ADDR_OFFSET), aDstIP, sizeof(aDstIP));
	memcpy((buf + IPV4_DST_ADDR_OFFSET), aSrcIP, sizeof(aSrcIP));
	return true;
}

bool USBIntegrationFixture::Teardown() {
	m_Routing.Reset(IPA_IP_v4);
	m_Routing.Reset(IPA_IP_v6);
	m_Producer.Close();
	m_Consumer1.Close();
	return true;
} // Teardown()

USBIntegrationFixture::~USBIntegrationFixture()
{
}

RoutingDriverWrapper USBIntegrationFixture::m_Routing;
Filtering USBIntegrationFixture::m_Filtering;
HeaderInsertion USBIntegrationFixture::m_HeaderInsertion;
