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
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include "TestsUtils.h"
#include "InterfaceAbstraction.h"
#include "Constants.h"
#include "Pipe.h"

///////////////////////////////////////////////////////////////////////////////

extern Logger g_Logger;

static uint8_t IPv4Packet[] = {
		0x45, 0x00, 0x00, 0x2e,
		0x00, 0x00, 0x40, 0x00,
		0xff, 0x06, 0xf5, 0xfd,// Protocol = 06 (TCP)
		0xc0, 0xa8, 0x02, 0x13,// IPv4 SRC Addr 192.168.2.19
		0xc0, 0xa8, 0x02, 0x68,// IPv4 DST Addr 192.168.2.104
		0x04, 0x57, 0x08, 0xae,
		0x00, 0x00, 0x30, 0x34,
		0x00, 0x00, 0x00, 0x50,
		0x50, 0xc1, 0x40, 0x00,
		0xab, 0xc9, 0x00, 0x00,
		0x00, 0xaa, 0xaa, 0xaa,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb, 0xbb, 0xbb,
		0xbb, 0xbb
};

static uint8_t IPv6Packet[] = {
		0x60, 0x00, 0x00, 0x00,
		0x00, 0x1c, 0x06, 0x01, // Protocol = 6 (TCP)
		0xfe, 0x80, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0xd9, 0xf9, 0xce, 0x5e,
		0x02, 0xec, 0x32, 0x99,
		0xff, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0c,
		0x12, 0x34, 0x12, 0x34, // port src = 0x1234 dest = 0x1234
		0x00, 0x14, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0xda, 0x7a, 0xda, 0x7a // payload
};

static uint8_t IPv6PacketFragExtHdr[] = {
		0x60, 0x00, 0x00, 0x00,
		0x00, 0x0c, 0x2C, 0x01, // Next header = FRAGMENT HEADER(44)
		0xfe, 0x80, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0xd9, 0xf9, 0xce, 0x5e,
		0x02, 0xec, 0x32, 0x99,
		0xff, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0c,
		0x06, 0x00, 0x00, 0x00, // fragment header, Protocol = 6 (TCP)
		0x00, 0x00, 0x00, 0x00,
		0x12, 0x34, 0x12, 0x34, // port src = 0x1234 dest = 0x1234
		0x00, 0x14, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		0xda, 0x7a, 0xda, 0x7a  // payload
};



bool LoadDefaultPacket(enum ipa_ip_type eIP, enum ipv6_ext_hdr_type extHdrType, uint8_t *pBuffer, size_t &nMaxSize)
{
	if (IPA_IP_v4 == eIP) {
		if (nMaxSize < sizeof(IPv4Packet))
		{
			LOG_MSG_ERROR("Buffer is smaller than %d, no Data was copied.",sizeof(IPv4Packet));
			return false;
		}
		memcpy(pBuffer,IPv4Packet, sizeof(IPv4Packet));
		nMaxSize = sizeof(IPv4Packet);
		return true;
	} else {
		if (extHdrType == FRAGMENT)
		{
			if (nMaxSize < sizeof(IPv6PacketFragExtHdr))
			{
				LOG_MSG_ERROR("Buffer is smaller than %d, no Data was copied.",sizeof(IPv6PacketFragExtHdr));
				return false;
			}
			memcpy(pBuffer,IPv6PacketFragExtHdr, sizeof(IPv6PacketFragExtHdr));
			nMaxSize = sizeof(IPv6PacketFragExtHdr);

		}
		else
		{
			if (nMaxSize < sizeof(IPv6Packet))
			{
				LOG_MSG_ERROR("Buffer is smaller than %d, no Data was copied.",sizeof(IPv6Packet));
				return false;
			}
			memcpy(pBuffer,IPv6Packet, sizeof(IPv6Packet));
			nMaxSize = sizeof(IPv6Packet);
		}
		return true;
	}
}

bool LoadDefaultPacket(enum ipa_ip_type eIP, uint8_t *pBuffer, size_t &nMaxSize)
{
	return LoadDefaultPacket(eIP, NONE, pBuffer, nMaxSize);
}

bool SendReceiveAndCompare(InterfaceAbstraction *pSink, uint8_t* pSendBuffer, size_t nSendBuffSize,
		InterfaceAbstraction *pSource, uint8_t* pExpectedBuffer, size_t nExpectedBuffSize)
{
	LOG_MSG_STACK("Entering Function");
	bool bRetVal = true;
	uint8_t * pRxBuff = new uint8_t[2*(nExpectedBuffSize+1)];
	size_t nReceivedBuffSize = 0;
	size_t j;

	// Send buffer to pSink
	bRetVal = pSink->SendData((Byte *) pSendBuffer, nSendBuffSize);
	if (!bRetVal)
	{
		LOG_MSG_ERROR("SendData (pOutputBuffer=0x%p) failed",pSendBuffer);
		goto bail;
	}

	// Receive buffer from pSource
	if (NULL == pRxBuff)
	{
		LOG_MSG_ERROR("Failed to allocated pRxBuff[%d]",2*(nExpectedBuffSize+1));
		goto bail;
	}
	nReceivedBuffSize = pSource->ReceiveData(pRxBuff, 2*(nExpectedBuffSize+1)); // We cannot overflow here.
	LOG_MSG_INFO("Received %d bytes on %s.", nReceivedBuffSize, pSource->m_fromChannelName.c_str());
	if (0 > nReceivedBuffSize)
	{
		bRetVal = false;
		goto bail;
	}

	{// Logging Expected and Received buffers
		char aExpectedBufferStr[3*nExpectedBuffSize+1];
		char aRecievedBufferStr[3*nReceivedBuffSize+1];
		memset(aExpectedBufferStr,0,3*nExpectedBuffSize+1);
		memset(aRecievedBufferStr,0,3*nReceivedBuffSize+1);

		for(j = 0; j < nExpectedBuffSize; j++)
			sprintf(&aExpectedBufferStr[3*j], " %02X", pExpectedBuffer[j]);
		for(j = 0; j < nReceivedBuffSize; j++)
			sprintf(&aRecievedBufferStr[3*j], " %02X", pRxBuff[j]);
		LOG_MSG_INFO("\nExpected Value (%d)\n%s\n, Received Value(%d)\n%s\n",nExpectedBuffSize,aExpectedBufferStr,nReceivedBuffSize,aRecievedBufferStr);
	}

	//Comparing Expected and received sizes
	if (nExpectedBuffSize != nReceivedBuffSize)
	{
		LOG_MSG_INFO("Buffers' Size differ: expected(%d), Received(%d)",nExpectedBuffSize,nReceivedBuffSize);
		bRetVal = false;
		goto bail;
	}

	bRetVal = !memcmp((void*)pRxBuff, (void*)pExpectedBuffer, nExpectedBuffSize);
	LOG_MSG_INFO("Buffers %s ",bRetVal?"MATCH":"MISMATCH");


	LOG_MSG_INFO("Verify that pipe is Empty");
	nReceivedBuffSize = pSource->ReceiveData(pRxBuff, 2*(nExpectedBuffSize+1)); // We cannot overflow here.
	while (nReceivedBuffSize){
		char aRecievedBufferStr[3*nReceivedBuffSize+1];
		bRetVal = false;
		LOG_MSG_ERROR("More Data in Pipe!\nReceived %d bytes on %s.", nReceivedBuffSize, pSource->m_fromChannelName.c_str());
		memset(aRecievedBufferStr,0,3*nReceivedBuffSize+1);
		for(j = 0; j < nReceivedBuffSize; j++) {
			sprintf(&aRecievedBufferStr[3*j], " %02X", pRxBuff[j]);
		}
		LOG_MSG_ERROR("\nReceived Value(%d)\n%s\n",nReceivedBuffSize,aRecievedBufferStr);
		nReceivedBuffSize = pSource->ReceiveData(pRxBuff, 2*(nExpectedBuffSize+1)); // We cannot overflow here.
	}

bail:
	delete (pRxBuff);
	LOG_MSG_STACK("Leaving Function (Returning %s)",bRetVal?"True":"False");
	return bRetVal;
}

bool CreateBypassRoutingTable (RoutingDriverWrapper * pRouting,enum ipa_ip_type eIP,
		const char * pTableName, enum ipa_client_type eRuleDestination,
		uint32_t uHeaderHandle, uint32_t * pTableHdl)
{
	bool bRetVal = true;
	struct ipa_ioc_add_rt_rule *pRoutingRule = NULL;
	struct ipa_rt_rule_add *pRoutingRuleEntry = NULL;
	struct ipa_ioc_get_rt_tbl sRoutingTable;

	LOG_MSG_STACK("Entering Function");
	memset(&sRoutingTable,0,sizeof(sRoutingTable));
	pRoutingRule = (struct ipa_ioc_add_rt_rule *)
		calloc(1,
				sizeof(struct ipa_ioc_add_rt_rule) +
		       1*sizeof(struct ipa_rt_rule_add)
			);
	if(!pRoutingRule) {
		LOG_MSG_ERROR("calloc failed to allocate pRoutingRule");
		bRetVal = false;
		goto bail;
	}

	pRoutingRule->num_rules = 1;
	pRoutingRule->ip = ((IPA_IP_v4 == eIP)? IPA_IP_v4 : IPA_IP_v6);
	pRoutingRule->commit = true;
	strcpy (pRoutingRule->rt_tbl_name, pTableName);

	pRoutingRuleEntry = &(pRoutingRule->rules[0]);
	pRoutingRuleEntry->at_rear = 1;
	pRoutingRuleEntry->rule.dst = eRuleDestination;// Setting Rule's Destination Pipe
	pRoutingRuleEntry->rule.hdr_hdl = uHeaderHandle; // Header handle
	pRoutingRuleEntry->rule.attrib.attrib_mask = 0;// All Packets will get a "Hit"
	if (false == pRouting->AddRoutingRule(pRoutingRule))
	{
		LOG_MSG_ERROR("Routing rule addition(pRoutingRule) failed!");
		bRetVal = false;
		goto bail;
	}
	if (!pRoutingRuleEntry->rt_rule_hdl)
	{
		LOG_MSG_ERROR("pRoutingRuleEntry->rt_rule_hdl == 0, Routing rule addition(pRoutingRule) failed!");
		bRetVal = false;
		goto bail;
	}
	LOG_MSG_INFO("pRoutingRuleEntry->rt_rule_hdl == 0x%x added to Table %s",pRoutingRuleEntry->rt_rule_hdl,pTableName);
	sRoutingTable.ip = eIP;
	strcpy(sRoutingTable.name, pTableName);
	if (!pRouting->GetRoutingTable(&sRoutingTable)) {
		LOG_MSG_ERROR(
				"m_routing.GetRoutingTable(&sRoutingTable=0x%p) Failed.", &sRoutingTable);
		bRetVal = false;
		goto bail;
	}
	if(NULL != pTableHdl){
		(* pTableHdl ) = sRoutingTable.hdl;
		LOG_MSG_DEBUG("Table Handle =0x%x will be returned.",(*pTableHdl));
	}

bail:
	Free (pRoutingRule);
	LOG_MSG_STACK("Leaving Function (Returning %s)",bRetVal?"True":"False");
	return bRetVal;
}

//Don't use these methods directly. use MACROs instead
void __log_msg(enum msgType logType, const char* filename, int line, const char* function, const char* format, ... )
{
	va_list args;
	switch (logType) {
	case ERROR:
		fprintf( stderr, "ERROR!");
		break;
	case DEBUG:
		fprintf( stderr, "DEBUG:");
		break;
	case INFO:
		fprintf( stderr, "INFO :");
		break;
	case STACK:
		fprintf( stderr, "STACK:");
		break;
	default:
		fprintf( stderr, "BUG!!!");
		break;
	}
	fprintf( stderr, " [%s:%d, %s()] ",filename,line,function);
	va_start( args, format );
	vfprintf( stderr, format, args );
	va_end( args );
	fprintf( stderr, "\n" );
}

bool file_exists(const char* filename)
{
	return (access(filename, F_OK) == 0);
}

int ConfigureSystem(int testConfiguration, int fd)
{
	return ConfigureSystem(testConfiguration, fd, NULL);
}

int ConfigureSystem(int testConfiguration, int fd, const char* params)
{
	char testConfigurationStr[10];
	int ret;
	char *pSendBuffer;
	char str[10];
	int bufferLen;

	if (params)
		bufferLen = strlen(params) + 10;
	else
		bufferLen = 10;

	pSendBuffer = new char[bufferLen];
	if (NULL == pSendBuffer)
	{
		LOG_MSG_ERROR("Failed to allocated pSendBuffer[%d]", bufferLen);
		return -1;
	}

	if (params)
		snprintf(pSendBuffer, bufferLen, "%d %s", testConfiguration, params);
	else
		snprintf(pSendBuffer, bufferLen, "%d", testConfiguration);

	ret = write(fd, pSendBuffer, sizeof(pSendBuffer) );
	if (ret < 0) {
		g_Logger.AddMessage(LOG_ERROR ,"%s Write operation failed.\n", __FUNCTION__);
		goto bail;
	}

	// Wait until the system is fully configured

	// Convert testConfiguration to string
	snprintf(testConfigurationStr, 10, "%d", testConfiguration);

	// Read the configuration index from the device node
	ret = read(fd, str, sizeof(str));
	if (ret < 0) {
		g_Logger.AddMessage(LOG_ERROR ,"%s Read operation failed.\n", __FUNCTION__);
		goto bail;
	}

	while ( strcmp(str, testConfigurationStr) ) {
		// Sleep for 5 msec
		struct timespec time;
		time.tv_sec = 0;
		time.tv_nsec = 50e6;
		nanosleep(&time, NULL);
		ret = read(fd, str, sizeof(str));
		if (ret < 0) {
			g_Logger.AddMessage(LOG_ERROR ,"%s Read operation failed.\n", __FUNCTION__);
			goto bail;
		}
	}
bail:
	delete(pSendBuffer);
	return ret;
}

void ConfigureScenario(int testConfiguration)
{
	ConfigureScenario(testConfiguration, NULL);
}

void ConfigureScenario(int testConfiguration, const char* params)
{
	int fd, ret;
	char str[10];
	int currentConf;

	// Open /dev/ipa_test device node. This will allow to configure the system
	// and read its current configuration.
	fd = open(CONFIGURATION_NODE_PATH, O_RDWR);
	if (fd < 0) {
		g_Logger.AddMessage(LOG_ERROR ,"%s Could not open configuration device node.\n", __FUNCTION__);
		exit(0);
	}

	// Read the current configuration.
	ret = read(fd, str, sizeof(str));
	if (ret < 0) {
		g_Logger.AddMessage(LOG_ERROR ,"%s Read operation failed.\n", __FUNCTION__);
		return;
	}
	currentConf = atoi(str);

	// Do not reconfigure to the same configuration
	if (currentConf == testConfiguration) {
		g_Logger.AddMessage(LOG_DEVELOPMENT,"%s System was already configured as required(%d)\n",
			__FUNCTION__, currentConf);
		return;
	}

	/* in case the system is not "clean"*/
	if (-1 != currentConf) {
		g_Logger.AddMessage(LOG_DEVELOPMENT,"%s System has other configuration (%d) - cleanup\n", __FUNCTION__, currentConf);
		ret = ConfigureSystem(-1, fd);
		if (ret < 0) {
			g_Logger.AddMessage(LOG_ERROR ,"%s Configure operation failed.\n",
					__FUNCTION__);
			return;
		}
	}

	// Start system configuration.
	g_Logger.AddMessage(LOG_DEVELOPMENT,"%s Setting system to the required configuration (%d)\n", __FUNCTION__, testConfiguration);

	ret = ConfigureSystem(testConfiguration, fd, params);
	if (ret < 0) {
		g_Logger.AddMessage(LOG_ERROR ,"%s configure operation failed.\n",
				__FUNCTION__);
		return;
	}

	system("mdev -s");

	close(fd);
}//func

bool CompareResultVsGolden(Byte *goldenBuffer,   unsigned int goldenSize,
			   Byte *receivedBuffer, unsigned int receivedSize)
{
	if (receivedSize != goldenSize) {
		g_Logger.AddMessage(LOG_VERBOSE , "%s File sizes are different.\n", __FUNCTION__);
		return false;
	}
	return !memcmp((void*)receivedBuffer, (void*)goldenBuffer, goldenSize);
}

Byte *LoadFileToMemory(const string &name, unsigned int *sizeLoaded)
{
	FILE *file;
	Byte *buffer;
	size_t fileLen;

	// Open file
	file = fopen(name.c_str(), "rb");
	if (!file) {
		g_Logger.AddMessage(LOG_ERROR , "Unable to open file %s\n", name.c_str());
		return NULL;
	}

	// Get file length
	fseek(file, 0, SEEK_END);
	fileLen = ftell(file);
	fseek(file, 0, SEEK_SET);

	// Allocate memory
	buffer=(Byte*)malloc(fileLen+1);
	if (!buffer) {
		fprintf(stderr, "Memory error!\n");
		fclose(file);
		return NULL;
	}

	// Read file contents into buffer
	*sizeLoaded = fread(buffer, 1, fileLen, file);
	fclose(file);

	return buffer;
}

void print_buff(void *data, size_t size)
{
	uint8_t bytes_in_line = 16;
	uint i, j, num_lines;
	char str[1024], tmp[4];

	num_lines = size / bytes_in_line;
	if (size % bytes_in_line > 0)
		num_lines++;

	printf("Printing buffer at address 0x%p, size = %zu: \n", data, size);
	for (i = 0 ; i < num_lines; i++) {
		strncpy(str, "\0", sizeof(str));
		for (j = 0; (j < bytes_in_line) && ((i * bytes_in_line + j) < size); j++) {
			snprintf(tmp, sizeof(tmp), "%02x ",
				 ((unsigned char*)data)[i * bytes_in_line + j]);
			strncat(str, tmp, sizeof(str) - strlen(str) - 1);
		}
		printf("%s\n", str);
	}
}

