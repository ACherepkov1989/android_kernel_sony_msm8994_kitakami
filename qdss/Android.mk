BOARD_PLATFORM_LIST := msm8974
BOARD_PLATFORM_LIST += msm8226
BOARD_PLATFORM_LIST += msm8610
BOARD_PLATFORM_LIST += apq8084

ifneq (,$(filter $(BOARD_PLATFORM_LIST),$(TARGET_BOARD_PLATFORM)))

LOCAL_PATH := $(call my-dir)

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE       := stm_test.sh
LOCAL_SRC_FILES    := stm_test.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/coresight/stm-trace-marker
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := run.sh
LOCAL_SRC_FILES    := run.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/coresight/stm-trace-marker
include $(BUILD_PREBUILT)

endif
