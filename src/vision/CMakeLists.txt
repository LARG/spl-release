cmake_minimum_required(VERSION 3.16)
project(vision)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_definitions(TFLITE_ENABLE_GPU=ON)
set(TFLITE_ENABLE_GPU ON)

set(AMENT_CMAKE_UNINSTALL_TARGET OFF)

# ROS and other general dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Codebase ROS packages
find_package(common REQUIRED)

set(VISION_SOURCES
VisionNode.cpp
VisionStage.cpp
main.cpp

image_sources/ImageSource.cpp
image_sources/CameraCapture.cpp
image_sources/StaticImage.cpp
image_sources/ColorSegmenter.cpp
image_sources/ExternalSource.cpp

object_detectors/ObjectDetector.cpp
object_detectors/TFLite/TFLiteDetector.cpp

line_detectors/LineDetector.cpp
line_detectors/HoughLineDetector/HoughLineDetector.cpp
)


set(TF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/tensorflow/tensorflow)
set(TF_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/tensorflow/)
set(TFLITE_SOURCE_DIR ${TF_SOURCE_DIR}/lite/)
add_subdirectory(${TFLITE_SOURCE_DIR}
${CMAKE_CURRENT_BINARY_DIR}/tensorflow-lite)


list(APPEND TFLITE_BENCHMARK1_SRCS
  ${TF_SOURCE_DIR}/core/util/stats_calculator.cc
  ${TFLITE_SOURCE_DIR}/profiling/memory_info.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summarizer.cc
  ${TFLITE_SOURCE_DIR}/profiling/profile_summary_formatter.cc
  ${TFLITE_SOURCE_DIR}/profiling/time.cc
  ${TFLITE_SOURCE_DIR}/tools/command_line_flags.cc
  ${TFLITE_SOURCE_DIR}/tools/delegates/default_execution_provider.cc
  ${TFLITE_SOURCE_DIR}/tools/delegates/delegate_provider.cc
  ${TFLITE_SOURCE_DIR}/tools/evaluation/utils.cc
  ${TFLITE_SOURCE_DIR}/tools/tool_params.cc
)


add_executable(vision_node
${VISION_SOURCES}
${TFLITE_BENCHMARK1_SRCS}
)

ament_target_dependencies(vision_node rclcpp std_msgs geometry_msgs common)

TARGET_LINK_LIBRARIES(vision_node 
-lopencv_imgproc -lopencv_core -lopencv_videoio -lopencv_imgcodecs -lopencv_highgui
jsoncpp_lib
tensorflow-lite
${CMAKE_DL_LIBS}
)

target_include_directories(vision_node
PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/util
    ${TF_INCLUDE_DIR}
)

install(TARGETS
vision_node
  DESTINATION lib/vision/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

