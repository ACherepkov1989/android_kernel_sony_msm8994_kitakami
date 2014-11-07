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

#ifndef _TEST_MANAGER_H_
#define _TEST_MANAGER_H_

#include "TestBase.h"
#include <vector>
#include <string>
#include "linux/msm_ipa.h"

using namespace std;


class TestManager /* Singleton */
{
public:
	static TestManager *GetInstance();
	~TestManager();
	void Register(TestBase & test);
	bool Setup();
	/* This is the place to put initialization
	 * for the whole test framework
	 */
	bool Run(vector <string> testSuiteList,
		 vector <string> testNameList);
	/* This function will run all the tests in the system */
	bool Teardown();
	/* This is the place to put tear-down for the whole test framework */
	vector < TestBase * > m_testList;
	/* Holds pointers to all of the tests in the system */

private:
	TestManager();
	TestManager(TestManager const &);
	TestManager & operator = (TestManager const &);
	void PrintSeparator(size_t len);
	void PrintRegisteredTests();
	void BuildRegressionTestSuite();
	void GetIPAHwType();

	static TestManager *m_instance;

	size_t m_numTestsRun;
	size_t m_numTestsFailed;
	enum ipa_hw_type m_IPAHwType;

	vector < string > m_failedTestsNames;
};

#endif
