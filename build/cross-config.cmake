# CMake Toolchain File
#
# Taken from HTWK's LolaConnector code
# github.com/NaoHTWK/LolaConnector
#
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR i686)

set(CMAKE_SYSROOT "$ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux")

set(tools "$ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux")
set(CMAKE_C_COMPILER "${tools}/i686-sbr-linux-gcc")
set(CMAKE_CXX_COMPILER "${tools}/i686-sbr-linux-g++")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
