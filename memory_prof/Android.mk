ifeq ($(call is-vendor-board-platform,QCOM),true)
ifeq ($(TARGET_ARCH),arm)
LOCAL_PATH := $(call my-dir)

# the dlkm
DLKM_DIR   := device/qcom/common/dlkm

include $(CLEAR_VARS)
LOCAL_MODULE      := the_memory_prof_module.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

# the userspace test program
include $(CLEAR_VARS)
LOCAL_MODULE := memory_prof
LOCAL_SRC_FILES += memory_prof.c
LOCAL_C_INCLUDES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SHARED_LIBRARIES := \
        libc \
        libcutils \
        libutils
LOCAL_MODULE_TAGS := optional debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_EXECUTABLE)

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE := memory_prof.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_SRC_FILES := memory_prof.sh
LOCAL_MODULE_TAGS := optional debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif
endif
