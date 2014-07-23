ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))

LOCAL_PATH := $(call my-dir)
commonSources :=

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm
LOCAL_MODULE := swp_test
LOCAL_SRC_FILES += $(commonSources) swp_test.c
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
LOCAL_MODULE_OWNER := qti
include $(BUILD_EXECUTABLE)

endif
