cmake_minimum_required(VERSION 3.5)
project(keyframe_motions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)
find_package(nao_command_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(keyframe_motion_msg REQUIRED)

ament_export_include_directories(include)

# Build walk_node

add_library(sensorvalues src/Sensors.cpp)
ament_target_dependencies(sensorvalues
  std_msgs
  nao_sensor_msgs
  nao_command_msgs)


add_library(bodyv6 src/bodyV6.cpp)
ament_target_dependencies(bodyv6)


add_library(getup src/GetupGenerator.cpp)
ament_target_dependencies(getup
  std_msgs
  nao_sensor_msgs
  nao_command_msgs
  keyframe_motion_msg
  )

add_executable(keyframe_motion_node src/keyframe_motion_node.cpp)
target_link_libraries(keyframe_motion_node getup sensorvalues)
ament_target_dependencies(keyframe_motion_node
  rclcpp)

# Install
install(TARGETS
  keyframe_motion_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS getup
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
