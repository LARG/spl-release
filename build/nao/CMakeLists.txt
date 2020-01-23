cmake_minimum_required(VERSION 2.8)

project(naointerface)

include($ENV{NAO_HOME}/build/common.cmake)

find_package(opencv COMPONENTS core highgui)

file(GLOB_RECURSE SRCS 
  ${INTERFACE_DIR}/nao/*.cpp
  ${INTERFACE_DIR}/nao/*.h
)

ADD_LIBRARY(naointerface SHARED ${SRCS})
# TARGET_LINK_LIBRARIES(naointerface core ${LIBYUVIEW} _pythonswig_module opencv2_core opencv2_highgui)
TARGET_LINK_LIBRARIES(naointerface ${LIBCORE} ${LIBYUVIEW} ${PYTHONSWIG})
