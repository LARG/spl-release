cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nao_sensor_msgs REQUIRED)
find_package(common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB_RECURSE SOURCES "src/*.cpp")

list(REMOVE_ITEM SOURCES "${CMAKE_SOURCE_DIR}/src/field_lines_visualization.cpp")

add_executable(localization_node ${SOURCES})

add_executable(field_lines_visualization "${CMAKE_SOURCE_DIR}/src/field_lines_visualization.cpp")
ament_target_dependencies(localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nao_sensor_msgs
  visualization_msgs
  common
  Eigen3
  angles)

ament_target_dependencies(
  field_lines_visualization
  rclcpp
  nao_sensor_msgs
  common
  visualization_msgs
  Eigen3
)

target_link_libraries(localization_node
  Eigen3::Eigen)

target_link_libraries(field_lines_visualization
  Eigen3::Eigen)

install(TARGETS
  localization_node
  field_lines_visualization
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
