LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

thermald_src_path := ./src
thermald_src_files := \
		$(thermald_src_path)/android_main.cpp

include external/stlport/libstlport.mk

LOCAL_C_INCLUDES += $(LOCAL_PATH) $(thermald_src_path) \
			external/icu/icu4c/source/common \
			external/libxml2/include \
			system/core/include/ \
			hardware/include \
			hardware/intel/power/include

LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS := -fpermissive -DTDRUNDIR='"/data/thermal-daemon"' -DTDCONFDIR='"/system/etc/thermal-daemon"'
LOCAL_STATIC_LIBRARIES := libxml2
LOCAL_SHARED_LIBRARIES := liblog libcutils libdl libstlport libicuuc libicui18n
LOCAL_PRELINK_MODULE := false
LOCAL_SRC_FILES := $(thermald_src_files)
LOCAL_MODULE := thermal-daemon
include $(BUILD_EXECUTABLE)
