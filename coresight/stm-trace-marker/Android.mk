LOCAL_PATH := $(call my-dir)

define ADD_TEST

include $(CLEAR_VARS)
LOCAL_MODULE       := $1
LOCAL_SRC_FILES    := $1
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_TAGS  := debug
LOCAL_MODULE_PATH  := $(TARGET_OUT_DATA)/kernel-tests/coresight/core/stm-trace-marker
include $(BUILD_PREBUILT)

endef

TEST_LIST := stm_test.sh README.txt
$(foreach TEST,$(TEST_LIST),$(eval $(call ADD_TEST,$(TEST))))


$(shell mkdir -p $(LOCAL_MODULE_PATH))
$(shell cp $(LOCAL_PATH)/run.sh $(LOCAL_MODULE_PATH)/run.sh)
