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

#include "MBIMAggregationTestFixtureConf10.h"

/////////////////////////////////////////////////////////////////////////////////

//define the static Pipes which will be used by all derived tests.
Pipe MBIMAggregationTestFixtureConf10::m_IpaToUsbPipeAggZeroLimits(IPA_CLIENT_TEST_CONS,
		IPA_TEST_CONFIGURATION_10);
Pipe MBIMAggregationTestFixtureConf10::m_UsbToIpaPipeAggZeroLimits(IPA_CLIENT_TEST_PROD,
		IPA_TEST_CONFIGURATION_10);

/////////////////////////////////////////////////////////////////////////////////

MBIMAggregationTestFixtureConf10::MBIMAggregationTestFixtureConf10()
{
	m_testSuiteName.push_back("DmaMbim16Agg0");
	Register(*this);
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationTestFixtureConf10::Setup()
{
	bool bRetVal = true;

	//Set the configuration to support USB->IPA and IPA->USB pipes.
	ConfigureScenario(10);

	//Initialize the pipe for all the tests - this will open the inode which represents the pipe.
	bRetVal &= m_IpaToUsbPipeAggZeroLimits.Init();
	bRetVal &= m_UsbToIpaPipeAggZeroLimits.Init();

	return bRetVal;
}

/////////////////////////////////////////////////////////////////////////////////

bool MBIMAggregationTestFixtureConf10::Teardown()
{
	//The Destroy method will close the inode.
	m_IpaToUsbPipeAggZeroLimits.Destroy();
	m_UsbToIpaPipeAggZeroLimits.Destroy();

	return true;
}
