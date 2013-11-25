BOARD_PLATFORM_LIST := msm8974
BOARD_PLATFORM_LIST += msm8226
BOARD_PLATFORM_LIST += msm8610
BOARD_PLATFORM_LIST += msm9625

ifneq (,$(filter $(BOARD_PLATFORM_LIST),$(TARGET_BOARD_PLATFORM)))

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
