ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))
ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))
ifneq ($(wildcard kernel/include/soc/qcom/glink.h),)
LOCAL_PATH := $(call my-dir)

# the dlkm
DLKM_DIR   := device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_MODULE      := glink_test.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE := glink_ktest.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := glink_ktest.sh
LOCAL_MODULE_TAGS := optional debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif
endif
endif
