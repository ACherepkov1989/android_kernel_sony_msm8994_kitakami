ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(TARGET_ARCH),arm)

DLKM_DIR   := device/qcom/common/dlkm
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := msm_ion_test_module.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

endif
endif
