ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(TARGET_ARCH),arm)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := ipa_test_module.ko
LOCAL_MODULE_TAGS := eng
LOCAL_CFLAGS += -DIPA_ON_R3PC=1


# include $(TOP)/build/dlkm/AndroidKernelModule.mk
# For APQ8064
include $(TOP)/device/qcom/common/dlkm/AndroidKernelModule.mk
#KBUILD_OPTIONS      := -DIPA_ON_R3PC=1
#LOCAL_CFLAGS      := -DIPA_ON_R3PC=1


include $(CLEAR_VARS)
LOCAL_C_INCLUDES := external/connectivity/stlport/stlport
LOCAL_CFLAGS += -include system/core/include/arch/linux-arm/AndroidConfig.h
LOCAL_CFLAGS += -I$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
# For APQ8064
LOCAL_CFLAGS += -I$(TOP)/kernel/include
LOCAL_CFLAGS += -DIPA_ON_R3PC=1
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE      := ipa
LOCAL_SRC_FILES   := \
		main.cpp \
		TestManager.cpp \
		TestBase.cpp \
		InterfaceAbstraction.cpp \
		TestsUtils.cpp \
		Logger.cpp \
		RoutingDriverWrapper.cpp \
		RoutingTests.cpp \
		IPAFilteringTable.cpp \
		Filtering.cpp \
		FilteringTest.cpp \
		HeaderInsertion.cpp \
		HeaderInsertionTests.cpp \
		Pipe.cpp \
		PipeTestFixture.cpp \
		PipeTests.cpp \
		HeaderRemovalTestFixture.cpp \
		HeaderRemovalTests.cpp \
		USBIntegrationFixture.cpp \
		USBIntegration.cpp \
		WLANIntegrationTests.cpp \
		IPv4Packet.cpp \
		MBIMAggregationTestFixture.cpp \
		MBIMAggregationTestFixtureConf10.cpp \
		MBIMAggregationTestFixtureConf11.cpp \
		MBIMAggregationTestFixtureConf12.cpp \
		MBIMAggregationTests.cpp \
		TLPAggregationTestFixture.cpp \
		TLPAggregationTests.cpp \
		RNDISAggregationTestFixture.cpp \
		RNDISAggregationTests.cpp

#LOCAL_SHARED_LIBRARIES := \
#    libutils \
#    libcutils \
#    liblog \
#    libc \
#    libdl \

LOCAL_MODULE_TAGS := eng
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_EXECUTABLE)

endif
endif


