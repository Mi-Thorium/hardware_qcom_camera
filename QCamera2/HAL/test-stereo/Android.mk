ifneq (,$(filter $(TARGET_ARCH), arm arm64))

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
        src/camera_memory.cpp \
        src/camera_parameters.cpp \
        src/qcamera2.cpp

LOCAL_CFLAGS := -Wno-unused-parameter -std=c++11 -fexceptions

LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/inc \
        $(LOCAL_PATH)/src \
        $(LOCAL_PATH)/../QCamera2/HAL \
        $(LOCAL_PATH)/../QCamera2/util \
        $(LOCAL_PATH)/../QCamera2/stack/common \
        $(LOCAL_PATH)/../mm-image-codec/qomx_core \
        $(LOCAL_PATH)/../mm-image-codec/qexif \
        $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_SHARED_LIBRARIES := libdl libcamera_metadata libcamera_client libutils liblog libcutils
LOCAL_COPY_HEADERS := inc/camera.h inc/camera_parameters.h

LOCAL_COPY_HEADERS_TO := libcamera/

LOCAL_32_BIT_ONLY := $(BOARD_QTI_CAMERA_32BIT_ONLY)

LOCAL_MODULE := libqcamera
LOCAL_CLANG := false
LOCAL_MODULE_TAGS := optional
LOCAL_VENDOR_MODULE := true


include $(BUILD_SHARED_LIBRARY)

$(local-intermediates-dir)/src/camera_parameters.o: PRIVATE_CFLAGS += -std=c++11

## build camera-test ##

include $(CLEAR_VARS)

LOCAL_32_BIT_ONLY := $(BOARD_QTI_CAMERA_32BIT_ONLY)

LOCAL_MODULE := camera-test-stereo
LOCAL_VENDOR_MODULE := true

LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_C_INCLUDES += $(LOCAL_PATH)/src
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../QCamera2/HAL

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
LOCAL_C_INCLUDES += $(kernel_includes)

LOCAL_SRC_FILES := test/camera_test.cpp

LOCAL_SHARED_LIBRARIES := libqcamera libcamera_metadata libcamera_client

include $(BUILD_EXECUTABLE)
endif
