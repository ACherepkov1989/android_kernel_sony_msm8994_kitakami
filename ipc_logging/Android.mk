ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))
ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))
LOCAL_PATH := $(call my-dir)
commonSources :=

#the DLKM
DLKM_DIR   := device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_MODULE      := ipc_logging_test.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

# Test scripts
include $(CLEAR_VARS)
LOCAL_MODULE := ipc_logging_test.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := ipc_logging_test.sh
LOCAL_MODULE_TAGS := debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif
endif
