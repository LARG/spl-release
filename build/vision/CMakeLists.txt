CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
#cmake_minimum_required(VERSION 3.16)

PROJECT(vision)

INCLUDE(../common.cmake)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_INSTALL_RPATH "/home/nao/lib/i386-linux-gnu/")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

#add_subdirectory(
#  "/media/sf_scratch/tf_new/0112/tensorflow/tensorflow/lite"
#  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite")

set(TF_SOURCE_DIR "/media/sf_scratch/tf_new/0112/tensorflow/tensorflow")
set(TFLITE_SOURCE_DIR "/media/sf_scratch/tf_new/0112/tensorflow/tensorflow/lite")

list(APPEND TFLITE_BENCHMARK1_SRCS
  ${TF_SOURCE_DIR}/core/util/stats_calculator.cc
  ${TFLITE_SOURCE_DIR}/profiling/memory_info.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summarizer.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summary_formatter.cc
  ${TFLITE_SOURCE_DIR}/profiling/time.cc
  ${TFLITE_SOURCE_DIR}/tools/command_line_flags.cc
  ${TFLITE_SOURCE_DIR}/tools/delegates/default_execution_provider.cc
  ${TFLITE_SOURCE_DIR}/tools/evaluation/utils.cc
  ${TFLITE_SOURCE_DIR}/tools/optimize/sparsity/format_converter.cc
  ${TFLITE_SOURCE_DIR}/tools/tool_params.cc
)

IF(NOT TOOL_BUILD)
  set(CMAKE_CXX_FLAGS "-m32 -std=c++14 -march=silvermont -O3")
  set(CMAKE_C_FLAGS "-m32 -march=silvermont -O3")
ELSE()
  set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14 -O3")
ENDIF()

FILE(GLOB_RECURSE SRCS ${INTERFACE_DIR}/shared_vision/*.cpp)
SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)

FIND_PACKAGE(ALSA REQUIRED)

ADD_EXECUTABLE(vision ${SRCS})# ${TFLITE_SOURCE_DIR})
IF(TOOL_BUILD AND ${CAFFE_BUILD})
  TARGET_LINK_LIBRARIES(vision ${CAFFE_LIBS})
ELSE()
  TARGET_LINK_LIBRARIES(vision ${LIBPYTHONSWIG} ${LIBCORE} ${LIBNAO} ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so ${CVDIR}/libopencv_videoio.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so ${CVDIR}/libopencv_ml.so)
ENDIF()
TARGET_LINK_LIBRARIES(vision ${LINK_LIBS} ${ALGLIB} ${LIBYAML-CPP} ${LIBYUVIEW} ${LIBFFT} ${LIBFFTW3F} ${ALSA_LIBRARIES} -lutil -ldl -lrt -lz )

IF(TOOL_BUILD)
  TARGET_LINK_LIBRARIES(vision ${DETECT_LIB_DIR}/libdetect_lib_tool.so)
ELSE()
  TARGET_LINK_LIBRARIES(vision ${DETECT_LIB_DIR}/libdetect_lib.so)
ENDIF()

#TARGET_LINK_LIBRARIES(vision tensorflow-lite)
target_include_directories(vision
  PUBLIC
#    ${TFLITE_INCLUDE_DIRS}
#    ${TFLITE_SOURCE_DIR}
    ${DETECT_LIB_INCLUDE}
)
INCLUDE_DIRECTORIES(${ALSA_INCLUDE_DIRS} ${DETECT_LIB_INCLUDE})
