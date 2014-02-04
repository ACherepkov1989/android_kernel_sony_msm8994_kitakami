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

#ifndef _USB_INTEGRATION_FIXTURE_H_
#define _USB_INTEGRATION_FIXTURE_H_

#include "TestBase.h"
#include "RoutingDriverWrapper.h"
#include "Filtering.h"
#include "HeaderInsertion.h"
#include "InterfaceAbstraction.h"

class USBIntegrationFixture:public TestBase {
public:

	USBIntegrationFixture();

	virtual bool AddRules() = 0;

	bool TestLogic();

	bool Setup();
	bool Run();
	bool Teardown();

	~USBIntegrationFixture();

	static RoutingDriverWrapper m_Routing;
	static Filtering m_Filtering;
	static HeaderInsertion m_HeaderInsertion;
	InterfaceAbstraction m_Producer;
	InterfaceAbstraction m_Consumer1;

protected:
	static const size_t BUFF_MAX_SIZE = 1500;
	static const uint8_t MAX_HEADER_SIZE = 64;
	/* 64Bytes - Max Header Length */
	enum ipa_ip_type m_eIP;
	uint8_t m_aBuffer[BUFF_MAX_SIZE];
	/* Input file \ IP packet */
	size_t m_uBufferSize;
	size_t m_nBufSize;

private:
	bool SwapIPv4Addresses(Byte *buf, size_t size);
};

#endif
