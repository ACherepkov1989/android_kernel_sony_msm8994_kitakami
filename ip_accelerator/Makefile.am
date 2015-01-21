KERNEL_FLAGS ?= ARCH=arm

EXTRA_CFLAGS	 = -DDEBUG
AM_CXXFLAGS = -Wall -Wundef -Wno-trigraphs -Werror
#AM_CXXFLAGS += -I./../../../../../../../../kernel/include
EXTRA_CFLAGS	+= -DIPA_ON_R3PC

module = ipa_test_module.ko
kmake  = $(MAKE) $(KERNEL_FLAGS) -C $(KERNEL_DIR) M=$(CURDIR)


$(module):
	$(kmake) modules

all-local: $(module)

install-exec-local: $(module)
	$(kmake) INSTALL_MOD_PATH=$(DESTDIR)$(prefix)/modules modules_install

clean-local:
	-test ! -d "$(KERNEL_DIR)" || $(kmake) clean


ip_acceleratordir            = $(prefix)/ip_accelerator
ip_accelerator_PROGRAMS      = ip_accelerator
dist_ip_accelerator_SCRIPTS  = run.sh
dist_ip_accelerator_SCRIPTS  += test_env_setup.sh
ip_accelerator_SOURCES =\
		TestManager.cpp \
		TestBase.cpp \
		InterfaceAbstraction.cpp \
		Pipe.cpp \
		PipeTestFixture.cpp \
		PipeTests.cpp \
		TLPAggregationTestFixture.cpp \
		TLPAggregationTests.cpp \
		MBIMAggregationTestFixture.cpp \
		MBIMAggregationTestFixtureConf10.cpp \
		MBIMAggregationTestFixtureConf11.cpp \
		MBIMAggregationTestFixtureConf12.cpp \
		MBIMAggregationTests.cpp \
		Logger.cpp \
		RoutingDriverWrapper.cpp \
		RoutingTests.cpp \
		IPAFilteringTable.cpp \
		Filtering.cpp \
		FilteringTest.cpp \
		HeaderInsertion.cpp \
		HeaderInsertionTests.cpp \
		TestsUtils.cpp \
		HeaderRemovalTestFixture.cpp \
		HeaderRemovalTests.cpp \
		IPv4Packet.cpp \
		RNDISAggregationTestFixture.cpp \
		RNDISAggregationTests.cpp \
		DataPathTestFixture.cpp \
		DataPathTests.cpp \
		IPAInterruptsTestFixture.cpp \
		IPAInterruptsTests.cpp \
		HeaderProcessingContextTestFixture.cpp \
		HeaderProcessingContextTests.cpp \
		FilteringEthernetBridgingTestFixture.cpp \
		FilteringEthernetBridgingTests.cpp \
		main.cpp
