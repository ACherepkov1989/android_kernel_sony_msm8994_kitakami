ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))
ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))

DLKM_DIR   := device/qcom/common/dlkm
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE      := cs_profile_mod.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

include $(CLEAR_VARS)
LOCAL_MODULE       := cs_profile.sh
LOCAL_SRC_FILES    := cs_profile.sh
LOCAL_MODULE_CLASS := EXECUTABLE
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/coresight/core/profile
include $(BUILD_PREBUILT)
endif
endif
