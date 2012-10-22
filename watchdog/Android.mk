ifeq ($(call is-board-platform,msm8974),true)

LOCAL_PATH := $(call my-dir)

# the dlkm
include $(CLEAR_VARS)
DLKM_DIR		:= device/qcom/common/dlkm
LOCAL_MODULE		:= msm_watchdog_test_module.ko
LOCAL_MODULE_TAGS	:= debug
include $(DLKM_DIR)/AndroidKernelModule.mk

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE       := msm_watchdog_test.sh
LOCAL_SRC_FILES    := msm_watchdog_test.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif

