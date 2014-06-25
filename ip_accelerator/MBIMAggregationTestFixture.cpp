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

#include "MBIMAggregationTestFixture.h"

/////////////////////////////////////////////////////////////////////////////////

//define the static Pipes which will be used by all derived tests.
Pipe MBIMAggregationTestFixture::m_IpaToUsbPipeNoAgg(IPA_CLIENT_TEST3_CONS,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_IpaToUsbPipeAggr(IPA_CLIENT_TEST_CONS,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_UsbNoAggToIpaPipeAgg(IPA_CLIENT_TEST3_CONS,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_UsbDeaggToIpaPipeNoAgg(IPA_CLIENT_TEST_PROD,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_UsbDeaggToIpaPipeAgg(IPA_CLIENT_TEST2_CONS,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_IpaToUsbPipeAggTime(IPA_CLIENT_TEST2_CONS,
		IPA_TEST_CONFIGURATION_9);
Pipe MBIMAggregationTestFixture::m_UsbNoAggToIpaPipeAggTime(IPA_CLIENT_TEST4_PROD,
		IPA_TEST_CONFIGURATION_9);

/////////////////////////////////////////////////////////////////////////////////

MBIMAggregationTestFixture::MBIMAggregationTestFixture()
{
	m_testSuiteName.push_back("DmaMbim16Agg");
	this->m_runInRegression = false;
	Register(*this);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationTestFixture::Setup()
{
	bool bRetVal = true;

	//Set the configuration to support USB->IPA and IPA->USB pipes.
	ConfigureScenario(9);

	//Initialize the pipe for all the tests - this will open the inode which represents the pipe.
	bRetVal &= m_IpaToUsbPipeNoAgg.Init();
	bRetVal &= m_IpaToUsbPipeAggr.Init();
	bRetVal &= m_UsbNoAggToIpaPipeAgg.Init();
	bRetVal &= m_UsbDeaggToIpaPipeNoAgg.Init();
	bRetVal &= m_UsbDeaggToIpaPipeAgg.Init();
	bRetVal &= m_IpaToUsbPipeAggTime.Init();
	bRetVal &= m_UsbNoAggToIpaPipeAggTime.Init();

	return bRetVal;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationTestFixture::Teardown()
{
	//The Destroy method will close the inode.
	m_IpaToUsbPipeNoAgg.Destroy();
	m_IpaToUsbPipeAggr.Destroy();
	m_UsbNoAggToIpaPipeAgg.Destroy();
	m_UsbDeaggToIpaPipeNoAgg.Destroy();
	m_UsbDeaggToIpaPipeAgg.Destroy();
	m_IpaToUsbPipeAggTime.Destroy();
	m_UsbNoAggToIpaPipeAggTime.Destroy();

	return true;
}

