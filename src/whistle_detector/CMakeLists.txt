cmake_minimum_required(VERSION 3.8)
project(whistle_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(WHISTLE_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/src/alsarecorder.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/fourier_transform.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rectangular_smooth.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/sine_generator.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/wavfile.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/whistle_classifier.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/whistle_detector.cpp"
)


add_executable(whistle_detector ${WHISTLE_SRC})
ament_target_dependencies(whistle_detector rclcpp std_msgs Boost Eigen3 rosidl_default_runtime)
target_link_libraries(whistle_detector asound fftw3f)


target_include_directories(whistle_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(whistle_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS whistle_detector
  DESTINATION lib/${PROJECT_NAME})



set(WHISTLE_DEBUGGER_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/alsarecorder.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/fourier_transform.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/rectangular_smooth.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/sine_generator.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/wavfile.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/whistle_classifier.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/whistle_debugger.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/whistle.cpp"
)
add_executable(whistledebug ${WHISTLE_DEBUGGER_SRC})
ament_target_dependencies(whistledebug rclcpp std_msgs Boost rosidl_default_runtime)
target_link_libraries(whistledebug asound fftw3f)

target_include_directories(whistledebug PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(whistledebug PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS whistledebug
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()









