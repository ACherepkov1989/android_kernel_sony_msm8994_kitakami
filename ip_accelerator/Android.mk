ifeq ($(call is-vendor-board-platform,QCOM),true)
ifneq (, $(filter aarch64 arm, $(TARGET_ARCH)))

DLKM_DIR := device/qcom/common/dlkm
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := ipa_test_module.ko
LOCAL_MODULE_TAGS  := debug
#LOCAL_CFLAGS += -DIPA_ON_R3PC=1
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_C_INCLUDES := external/connectivity/stlport/stlport
ifeq ($(TARGET_ARCH),aarch64)
LOCAL_CFLAGS += -include build/core/combo/include/arch/linux-aarch64/AndroidConfig.h
else
LOCAL_CFLAGS += -include build/core/combo/include/arch/linux-arm/AndroidConfig.h
endif
LOCAL_CFLAGS += -I$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES :=   external/stlport/stlport bionic/ bionic/libstdc++/include
# For APQ8064
LOCAL_CFLAGS += -I$(TOP)/kernel/include
#LOCAL_CFLAGS += -DIPA_ON_R3PC=1
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_MODULE      := ip_accelerator
LOCAL_SRC_FILES   := \
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
		main.cpp

LOCAL_SHARED_LIBRARIES := \
		libstlport \

LOCAL_MODULE_TAGS := debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests/ip_accelerator
include $(BUILD_EXECUTABLE)

define ADD_TEST

include $(CLEAR_VARS)
LOCAL_MODULE       := $1
LOCAL_SRC_FILES    := $1
LOCAL_MODULE_CLASS := ip_accelerator
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/ip_accelerator
include $(BUILD_PREBUILT)

endef

IP_ACCELERATOR_FILE_LIST := README.txt run.sh test_env_setup.sh
$(foreach TEST,$(IP_ACCELERATOR_FILE_LIST),$(eval $(call ADD_TEST,$(TEST))))

endif # $(TARGET_ARCH)
endif


