ifeq ($(call is-vendor-board-platform,QCOM),true)

DLKM_DIR   := build/dlkm
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := msm_watchdog_test_module.ko
LOCAL_MODULE_TAGS := eng
include $(DLKM_DIR)/AndroidKernelModule.mk

endif
