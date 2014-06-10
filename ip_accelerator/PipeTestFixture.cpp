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

#include "PipeTestFixture.h"

/*define the static Pipes which will be used by all derived tests.*/
Pipe PipeTestFixture::m_IpaToUsbPipe(IPA_CLIENT_TEST_CONS, IPA_TEST_CONFIFURATION_1);
Pipe PipeTestFixture::m_UsbToIpaPipe(IPA_CLIENT_TEST_PROD, IPA_TEST_CONFIFURATION_1);

PipeTestFixture::PipeTestFixture()
{
	m_testSuiteName.push_back("Pipes");
	Register(*this);
}

bool PipeTestFixture::Setup()
{
	bool bRetVal = true;

	/*Set the configuration to support USB->IPA and IPA->USB pipes.*/
	ConfigureScenario(1);

	/*Initialize the pipe for all the tests -
	 * this will open the inode which represents the pipe.
	 */
	bRetVal &= m_IpaToUsbPipe.Init();
	bRetVal &= m_UsbToIpaPipe.Init();
	return bRetVal;
}

bool PipeTestFixture::Teardown()
{
	/*The Destroy method will close the inode.*/
	m_IpaToUsbPipe.Destroy();
	m_UsbToIpaPipe.Destroy();
	return true;
}
