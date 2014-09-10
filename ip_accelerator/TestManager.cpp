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
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include "TestManager.h"
#include "TestsUtils.h"
#include <fcntl.h>
#include <unistd.h>
#include "ipa_test_module.h"
#include <sys/ioctl.h>

using namespace std;

/* Global static pointer used to ensure a single instance of the class.*/
TestManager* TestManager::m_instance = NULL;

TestManager::TestManager()
{
	m_testList.clear();
	m_failedTestsNames.clear();
	m_numTestsFailed = 0;
	m_numTestsRun = 0;
	GetIPAHwType();
}

////////////////////////////////////////////////////////////////////////////////////////////

TestManager::~TestManager()
{
	m_testList.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////

TestManager* TestManager::GetInstance()
{
	if (!m_instance)   // Only allow one instance of class to be generated.
		m_instance = new TestManager;

	return m_instance;
}

////////////////////////////////////////////////////////////////////////////////////////////

void TestManager::Register(TestBase &test)
{
	m_testList.push_back(&test);
}

////////////////////////////////////////////////////////////////////////////////////////////

bool TestManager::Run(vector<string> testSuiteList, vector<string> testNameList)
{
	TestBase *test = NULL;
	bool pass = true;
	vector<string>::iterator testIter;
	vector<string>::iterator testSuiteIter;
	bool runTest = false;

	if (m_testList.size() == 0)
		return false;

	/*PrintRegisteredTests();*/

	for (unsigned int i = 0 ; i < m_testList.size() ; i++ , runTest = false) {
		pass = true;
		test = m_testList[i];

		// Run only tests from the list of test suites which is stated in the command
		// line. In case the list is empty, run all tests.
		if (testSuiteList.size() > 0) {
			for (unsigned int j = 0; j < test->m_testSuiteName.size(); j++) {
				testSuiteIter = find(testSuiteList.begin(), testSuiteList.end(), test->m_testSuiteName[j]);
				if (testSuiteIter != testSuiteList.end()) {
					runTest = true;
				}
			}
		}

		// We also support test by name
		if (testNameList.size() > 0) {
			testIter = find(testNameList.begin(), testNameList.end(), test->m_name);
			if (testIter != testNameList.end())
				runTest = true;
		}

		// Run the test only if it's applicable to the current IPA HW type / version
		if (runTest) {
			if (!(m_IPAHwType >= test->m_minIPAHwType && m_IPAHwType <= test->m_maxIPAHwType))
				runTest = false;
		}

		if (!runTest)
			continue;

		printf("\n\nExecuting test %s\n", test->m_name.c_str());

		printf("Setup()\n");
		pass &= test->Setup();

		//In case the test's setup did not go well it will be a bad idea to try and run it.
		if (true == pass)
		{
			printf("Run()\n");
			pass &= test->Run();
		}

		printf("Teardown()\n");
		pass &= test->Teardown();

		if (pass)
		{
			m_numTestsRun++;
			PrintSeparator(test->m_name.size());
			printf("Test %s PASSED !\n", test->m_name.c_str());
			PrintSeparator(test->m_name.size());
		}
		else
		{
			m_numTestsRun++;
			m_numTestsFailed++;
			m_failedTestsNames.push_back(test->m_name);
			PrintSeparator(test->m_name.size());
			printf("Test %s FAILED !\n", test->m_name.c_str());
			PrintSeparator(test->m_name.size());
		}
	} // for

	// Print summary
	printf("\n\n");
	printf("==================== RESULTS SUMMARY ========================\n");
	printf("%zu tests were run, %zu failed.\n", m_numTestsRun, m_numTestsFailed);
	if (0 != m_numTestsFailed) {
		printf("Failed tests list:\n");
		for (size_t i = 0; i < m_numTestsFailed; i++) {
			printf("        %s\n", m_failedTestsNames[i].c_str());
			m_failedTestsNames.pop_back();
		}
	}
	printf("=============================================================\n");

	return pass;
}

////////////////////////////////////////////////////////////////////////////////////////////

void TestManager::PrintSeparator(size_t len)
{
	string separator;

	for (size_t i = 0; i < len + 15; i++) {
		separator += "-";
	}

	printf("%s\n", separator.c_str());
}

////////////////////////////////////////////////////////////////////////////////////////////

TestManager::TestManager(TestManager const&)
{

}

////////////////////////////////////////////////////////////////////////////////////////////

TestManager& TestManager::operator=(TestManager const&)
{
	return *m_instance;
}

////////////////////////////////////////////////////////////////////////////////////////////

void TestManager::PrintRegisteredTests()
{
	printf("Test list: (%zu registered)\n", m_testList.size());
	for (unsigned int i = 0; i < m_testList.size(); i++) {
		printf("%d) name = %s, suite name = %s, regression = %d\n", i, m_testList[i]->m_name.c_str(),
		       m_testList[i]->m_testSuiteName[0].c_str(), m_testList[i]->m_runInRegression);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////

void TestManager::GetIPAHwType()
{
	int fd;

	// Open ipa_test device node
	fd = open("/dev/ipa_test" , O_RDONLY);
	if (0 == fd) {
		printf("Failed opening %s.\n", "/dev/ipa_test");
		m_IPAHwType = IPA_HW_None;
	}

	m_IPAHwType = (enum ipa_hw_type)ioctl(fd, IPA_TEST_IOC_GET_HW_TYPE);
	if (-1 == m_IPAHwType) {
		printf("%s(), IPA_TEST_IOC_GET_HW_TYPE ioctl failed\n", __FUNCTION__);
		m_IPAHwType = IPA_HW_None;
	}

	printf("%s(), IPA HW type (version) = %d\n", __FUNCTION__, m_IPAHwType);
	close(fd);
}



////////////////////////////////////////////////////////////////////////////////////////////
