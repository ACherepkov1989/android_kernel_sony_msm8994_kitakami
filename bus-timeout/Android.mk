ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))

DLKM_DIR   := device/qcom/common/dlkm
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := bus_timeout_mod.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE       := bus_timeout_test.sh
LOCAL_SRC_FILES    := bus_timeout_test.sh
LOCAL_MODULE_CLASS := EXECUTABLE
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/bus-timeout
include $(BUILD_PREBUILT)

endif
