ifneq (,$(filter $(QCOM_BOARD_PLATFORMS),$(TARGET_BOARD_PLATFORM)))
ifneq (,$(filter arm aarch64 arm64, $(TARGET_ARCH)))

DLKM_DIR := device/qcom/common/dlkm
LOCAL_PATH := $(call my-dir)

# the dlkm
include $(CLEAR_VARS)
LOCAL_SRC_FILES += msm_ocmem_test_module.c compat_msm_ocmem_test_module.c
LOCAL_MODULE      := msm_ocmem_test_mod.ko
LOCAL_MODULE_TAGS := debug
include $(DLKM_DIR)/AndroidKernelModule.mk

# the userspace test program
include $(CLEAR_VARS)
LOCAL_MODULE      := ocmem_test
LOCAL_SRC_FILES   := ocmem_test.c
LOCAL_MODULE_TAGS := debug
LOCAL_MODULE_PATH := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_EXECUTABLE)

# the test script
include $(CLEAR_VARS)
LOCAL_MODULE       := ocmem_test.sh
LOCAL_SRC_FILES    := ocmem_test.sh
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests
include $(BUILD_PREBUILT)

endif
endif
