BOARD_PLATFORM_LIST := msm8974
BOARD_PLATFORM_LIST += msm8226
BOARD_PLATFORM_LIST += msm8610
BOARD_PLATFORM_LIST += apq8084
BOARD_PLATFORM_LIST += mpq8092
BOARD_PLATFORM_LIST += msm8916

ifneq (,$(filter $(BOARD_PLATFORM_LIST),$(TARGET_BOARD_PLATFORM)))

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

