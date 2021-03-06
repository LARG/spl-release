PROJECT(yuview)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

ADD_DEFINITIONS(-DHOME_DIRECTORY="$ENV{HOME}")
SET(NAO_HOME $ENV{NAO_HOME})

IF(DEFINED CROSS_COMPILE)
  SET(tools "${NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux")
  SET(CMAKE_C_COMPILER  "${tools}/i686-sbr-linux-gcc")
  SET(CMAKE_CXX_COMPILER  "${tools}/i686-sbr-linux-g++")
  INCLUDE_DIRECTORIES(${NAO_HOME}/naoqi/crosstoolchain/yocto-sdk/sysroots/x86_64-naoqisdk-linux/lib)
  INCLUDE_DIRECTORIES(${NAO_HOME}/naoqi/include)
  INCLUDE_DIRECTORIES(${NAO_HOME}/lib/flatbuffers/include)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")
  SET(CMAKE_SYSROOT "${NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${CMAKE_SYSROOT}")
  #SET(CMAKE_CXX_LINKER_FLAGS "-Wl,-rpath -Wl,${NAO_HOME}/naoqi/crosstoolchain/atom/cross")
  LIST(APPEND CMAKE_PREFIX_PATH ${CMAKE_SYSROOT}/lib ${CMAKE_SYSROOT}/usr/lib ${NAO_HOME}/naoqi/link_libs)
ELSE()
  SET(CMAKE_C_COMPILER gcc-5)
  SET(CMAKE_CXX_COMPILER g++-5)
ENDIF()

SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(FlatBuffers REQUIRED)
FIND_PACKAGE(Boost REQUIRED program_options filesystem system)
SET(INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/include")
SET(SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
SET(SCHEMA_DIR "${CMAKE_CURRENT_LIST_DIR}/schema")
FILE(GLOB_RECURSE SCHEMA_DEFINITIONS "${SCHEMA_DIR}/*.fbs")

INCLUDE_DIRECTORIES("${INCLUDE_DIR}")
FILE(GLOB_RECURSE CPP_SOURCES src/*.cpp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")

BUILD_FLATBUFFERS("${SCHEMA_DEFINITIONS}" "${FLATBUFFERS_INCLUDE_DIRS}" generated_schemas "${GENERATED_SCHEMA_FILES}" "${INCLUDE_DIR}/schema" "" "")

IF(DEFINED CROSS_COMPILE)
  ADD_LIBRARY(yuview ${CPP_SOURCES})
  ADD_DEPENDENCIES(yuview generated_schemas)
ELSE()
  ADD_DEFINITIONS(-DALLOW_RESIZE)
  ADD_DEFINITIONS(-DENABLE_GUI)
  ADD_LIBRARY(yuview ${CPP_SOURCES})
  ADD_EXECUTABLE(yuview-bin ${CPP_SOURCES})
  SET_TARGET_PROPERTIES(yuview-bin PROPERTIES OUTPUT_NAME yuview)
  TARGET_LINK_LIBRARIES(yuview-bin ${OPENCV2_LIBS} ${LIBFLATBUFFERS} ${Boost_LIBRARIES})
  ADD_DEPENDENCIES(yuview generated_schemas)
  ADD_DEPENDENCIES(yuview-bin generated_schemas)
ENDIF()
