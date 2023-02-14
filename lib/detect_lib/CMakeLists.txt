#add_subdirectory(
#  "../tf_new/0104/tensorflow/tensorflow/lite"
#  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite")

cmake_minimum_required(VERSION 3.16)
#project(minimal C CXX)
project(minimal C CXX)

IF(TOOL_BUILD)
  set(CMAKE_CXX_FLAGS "-std=c++14 -march=silvermont -O3 -fPIC")
  set(CMAKE_C_FLAGS "-march=silvermont -O3 -fPIC")
  add_definitions(-DTOOL)
ELSE()
  set(CMAKE_CXX_FLAGS "-m32 -std=c++14 -march=silvermont -O3 -fPIC")
  set(CMAKE_C_FLAGS "-m32 -march=silvermont -O3 -fPIC")
ENDIF()


# set(CMAKE_CXX_FLAGS "-march=silvermont -std=c++14 -O3")
# set(CMAKE_C_FLAGS "-march=silvermont -O3")
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)


set(TF_SOURCE_DIR "./tensorflow/tensorflow")
set(TFLITE_SOURCE_DIR ${TF_SOURCE_DIR}/lite/)

add_subdirectory(
  ${TFLITE_SOURCE_DIR}
  "${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite")

IF(NOT TOOL_BUILD)
  SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)
ENDIF()
SET(CVDIR_INCLUDE $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/include/)

# SET(CVDIR ./deps/opencv)
# SET(CVDIR_INCLUDE ./deps/opencv/include)

#populate_source_vars(${TFLITE_SOURCE_DIR}/tools/benchmark
#  TFLITE_BENCHMARK1_SRCS
#  FILTER "(_test|_plus_flex_main|_performance_options.*)\\.cc$"
#)

list(APPEND TFLITE_BENCHMARK1_SRCS
  ${TF_SOURCE_DIR}/core/util/stats_calculator.cc
  ${TFLITE_SOURCE_DIR}/profiling/memory_info.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summarizer.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summary_formatter.cc
  ${TFLITE_SOURCE_DIR}/profiling/time.cc
  ${TFLITE_SOURCE_DIR}/tools/command_line_flags.cc
  ${TFLITE_SOURCE_DIR}/tools/delegates/default_execution_provider.cc
  ${TFLITE_SOURCE_DIR}/tools/evaluation/utils.cc
  #${TFLITE_SOURCE_DIR}/tools/optimize/sparsity/format_converter.cc
  ${TFLITE_SOURCE_DIR}/tools/tool_params.cc
)

message(STATUS "sources: ${TFLITE_BENCHMARK1_SRCS}")

#add_executable(benchmark_model1
#  ${TFLITE_BENCHMARK1_SRCS}
#)
#set_target_properties(benchmark_model1 PROPERTIES LINKER_LANGUAGE CXX)
#target_compile_options(benchmark_model1
#  PRIVATE
#    ${TFLITE_BENCHMARK_CC_OPTIONS}
#)
#target_link_libraries(benchmark_model1 tensorflow-lite ${CMAKE_DL_LIBS}
#    ${TFLITE_BENCHMARK_LIBS}
#)


#add_executable(minimal ${TFLITE_SOURCE_DIR}/examples/minimal/minimal.cc)
# add_executable(minimal minimal.cc ${TFLITE_BENCHMARK1_SRCS})
# #target_compile_options(minimal
# #  PRIVATE
# #    ${TFLITE_BENCHMARK_CC_OPTIONS}
# #)
# #target_include_directories(minimal
# #  PUBLIC
# #    ${TFLITE_INCLUDE_DIRS}
# #)
# target_link_libraries(minimal tensorflow-lite ${CMAKE_DL_LIBS})


# populate_source_vars("./label_image/"
#   LABELIMG_SOURCES
#   FILTER "(_test|_plus_flex_main|_performance_options.*)\\.cc$"
# )

# #target_compile_options(tensorflow-lite
# #  PUBLIC
# #    -m32 -march=silvermont
# #    ${TFLITE_BENCHMARK_CC_OPTIONS}
# #)

# #target_compile_options(benchmark_model
# #  PUBLIC
# #    -m32 -march=silvermont
# #    ${TFLITE_BENCHMARK_CC_OPTIONS}
# #)

# message(STATUS "sources: ${LABELIMG_SOURCES}")

# add_executable(labelimg label_image/label_image.cc;label_image/label_image.h;label_image/bitmap_helpers.cc;label_image/bitmap_helpers.h;label_image/bitmap_helpers_impl.h;label_image/get_top_n_impl.h;label_image/get_top_n.h ${TFLITE_BENCHMARK1_SRCS})
# # add_executable(labelimg label_image/bitmap_helpers.cc;label_image/bitmap_helpers.h;label_image/bitmap_helpers_impl.h)

# #target_compile_options(labelimg
# #  PUBLIC
# #    -m32 -march=silvermont
# #    ${TFLITE_BENCHMARK_CC_OPTIONS}
# #)

# target_link_libraries(labelimg tensorflow-lite ${CMAKE_DL_LIBS})


# populate_source_vars("./detection/"
#   DETECT_SOURCES
#   FILTER "(detector|_plus_flex_main|_performance_options.*)\\.cc$"
# )

# message(STATUS "sources: ${DETECT_SOURCES}")

# add_executable(detect ./detection/detect.cc ${TFLITE_BENCHMARK1_SRCS})

# target_include_directories(detect
#   PUBLIC
#     ${TFLITE_INCLUDE_DIRS}
#     ${CVDIR_INCLUDE}
# )

# target_link_libraries(detect tensorflow-lite ${CMAKE_DL_LIBS}
# ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so
# ${CVDIR}/libopencv_videoio.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so
# ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so
# ${CVDIR}/libopencv_ml.so)


# add_executable(detect_yuv ./detection/detect_yuv.cc ${TFLITE_BENCHMARK1_SRCS})

# target_include_directories(detect_yuv
#   PUBLIC
#     ${TFLITE_INCLUDE_DIRS}
#     ${CVDIR_INCLUDE}
# )

# target_link_libraries(detect_yuv tensorflow-lite ${CMAKE_DL_LIBS}
# ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so
# ${CVDIR}/libopencv_videoio.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so
# ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so
# ${CVDIR}/libopencv_ml.so)


IF(NOT TOOL_BUILD)
  SET(CV_LINK_LIBS ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so
      ${CVDIR}/libopencv_videoio.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so
      ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so
      ${CVDIR}/libopencv_ml.so
     )
#ELSE()
#  SET(CV_LINK_LIBS -lopencv_core -lopencv_highgui -lopencv_imgcodecs
#       -lopencv_features2d
#      -lopencv_calib3d -lopencv_imgproc -lopencv_objdetect
#      -lopencv_ml)
ENDIF()


add_library(detect_lib SHARED EXCLUDE_FROM_ALL
src/detect_lib.cc
${TFLITE_BENCHMARK1_SRCS}
)
target_include_directories(detect_lib
  PUBLIC
    ${TFLITE_INCLUDE_DIRS}
    ${CVDIR_INCLUDE}
)
target_link_libraries(detect_lib tensorflow-lite ${CMAKE_DL_LIBS}
${CV_LINK_LIBS}
)


add_executable(test_detect_lib src/test_detect_lib.cc)
target_include_directories(test_detect_lib
  PUBLIC
    ${TFLITE_INCLUDE_DIRS}
    ${CVDIR_INCLUDE}
    $ENV{NAO_HOME}/lib/yaml-cpp/include
)
target_link_libraries(test_detect_lib detect_lib ${CMAKE_DL_LIBS}
${CV_LINK_LIBS}
$ENV{NAO_HOME}/lib/yaml-cpp/build/libyaml-cpp.a
)
