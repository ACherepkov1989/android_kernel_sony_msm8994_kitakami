ifeq ($(TARGET_BOARD_PLATFORM),msm8960)

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_COPY_HEADERS_TO   := kernel-tests/sps
LOCAL_COPY_HEADERS      := msm_sps_test.h
include $(BUILD_COPY_HEADERS)

LOCAL_MODULE      := msm_sps_test_module.ko
LOCAL_MODULE_TAGS := eng
include $(TOP)/device/qcom/common/dlkm/AndroidKernelModule.mk

endif
