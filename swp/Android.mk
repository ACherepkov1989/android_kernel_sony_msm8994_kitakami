ifneq (,$(filter arm arm64 aarch64, $(TARGET_ARCH)))

LOCAL_PATH := $(call my-dir)
commonSources :=

include $(CLEAR_VARS)
LOCAL_ARM_MODE := arm
LOCAL_MODULE := swp_test
LOCAL_MODULE_STEM_32 := swp_test
LOCAL_SRC_FILES += $(commonSources) swp_test.c
LOCAL_MODULE_TAGS := optional debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
LOCAL_MODULE_OWNER := qti
LOCAL_MULTILIB := 32
include $(BUILD_EXECUTABLE)

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE := swp_test.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := swp_test.sh
LOCAL_MODULE_TAGS := optional debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif
