cmake_minimum_required(VERSION 2.8)
#set(CMAKE_VERBOSE_MAKEFILE ON)

IF(ALLOW_DEBUG_LOG)
  ADD_DEFINITIONS(-DALLOW_DEBUG_LOG)
ENDIF()

SET(BUILD_TYPE "release")

if(OPTIMIZE)
  ADD_DEFINITIONS(-DOPTIMIZE)
    SET(BUILD_TYPE "optimized")
ENDIF()

# some base variables
SET(NAO_HOME $ENV{NAO_HOME})
IF(NOT(TOOL_BUILD))
  INCLUDE_DIRECTORIES(${NAO_HOME}/naoqi/crosstoolchain/atom/cross/i686-aldebaran-linux-gnu/include)
ENDIF(NOT(TOOL_BUILD))
SET(SRC_DIR ${NAO_HOME}/core)
SET(INTERFACE_DIR ${NAO_HOME}/interfaces)


# Include generated schema files
SET(SCHEMA_GEN_DIR "${NAO_HOME}/build/build/schema")
IF(NOT(${DEFINED_SCHEMA_DIR}))
  SET(DEFINED_SCHEMA_DIR True)
  ADD_DEFINITIONS(-DSCHEMA_DIR="${SCHEMA_GEN_DIR}")
ENDIF(NOT(${DEFINED_SCHEMA_DIR}))
SET(SCHEMA_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/schema")
FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/schema")
IF(NOT EXISTS ${SCHEMA_INCLUDE_DIR}/gen)
  EXECUTE_PROCESS(COMMAND ln -sf "${SCHEMA_GEN_DIR}" "${SCHEMA_INCLUDE_DIR}/gen")
ENDIF(NOT EXISTS ${SCHEMA_INCLUDE_DIR}/gen)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

SET(BHWALK2011_DIR ${NAO_HOME}/core/motion/bhwalk2011)
SET(LIBBHWALK2011 ${CMAKE_CURRENT_BINARY_DIR}/../../bhwalk2011/build-atom/bhwalk2011/libbhwalk2011.a)

SET(BHWALK2013_DIR ${NAO_HOME}/core/motion/bhwalk2013)
SET(LIBBHWALK2013 ${CMAKE_CURRENT_BINARY_DIR}/../../bhwalk2013/build-atom/bhwalk2013/libbhwalk2013.a)
SET(LIBSNAPPY ${BHWALK2013_DIR}/Util/snappy/lib/linux_x86/libsnappy.a)

# Adding this for the runswift walk - Josiah
SET(RSWALK2014_DIR ${NAO_HOME}/core/motion/rswalk2014)
SET(LIBRSWALK2014 ${NAO_HOME}/build/build/${BUILD_TYPE}/rswalk2014/rswalk2014/librswalk2014.a)

SET(WALK_LIBS ${LIBRSWALK2014})
SET(WALK_DIRS ${BHWALK2011_DIR} ${BHWALK2013_DIR} ${RSWALK2014_DIR})

FIND_PACKAGE(Boost REQUIRED thread system program_options)
SET(LINK_LIBS ${Boost_LIBRARIES})
SET(NAOQI_INCLUDES ${NAO_HOME}/naoqi/includes)
SET(EIGEN_DIR ${NAOQI_INCLUDES}/eigen3)
find_package(ALSA REQUIRED)

IF(TOOL_BUILD)
    SET(LIBYAML-CPP ${NAO_HOME}/lib/yaml-cpp/libyaml-cpp.a)
ELSE(TOOL_BUILD)
    SET(LIBYAML-CPP ${NAO_HOME}/lib/yaml-cpp/build/libyaml-cpp.a)
ENDIF(TOOL_BUILD)

SET(YAML_INCLUDE ${NAO_HOME}/lib/yaml-cpp/include)

SET(LIBYUVIEW ${NAO_HOME}/lib/yuview/build/cross/libyuview.a)
IF(TOOL_BUILD)
  SET(LIBYUVIEW ${NAO_HOME}/lib/yuview/libyuview.a)
ENDIF(TOOL_BUILD)
SET(YUVIEW_INCLUDE ${NAO_HOME}/lib/yuview/include)
SET(FLATBUFFERS_INCLUDE ${NAO_HOME}/lib/flatbuffers/include)
SET(LIBFLATBUFFERS ${NAO_HOME}/lib/flatbuffers/libflatbuffers.a)
SET(ALGLIB_SRC ${NAO_HOME}/lib/alglib)
SET(ALGLIB ${ALGLIB_SRC}/libalglib.a)

SET(NAOQI_ROOT ${NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux)
SET(NAOQI_CROSS_LIB ${NAOQI_ROOT}/usr/lib)
SET(NAOQI_LIB ${NAOQI_CROSS_LIB})
SET(PYTHON_INCLUDE ${NAOQI_ROOT}/usr/include/python2.7)
SET(LIBCORE ${NAO_HOME}/build/build/${BUILD_TYPE}/core/libcore.a)
SET(LIBPYTHONSWIG ${NAO_HOME}/build/build/${BUILD_TYPE}/pythonswig/lib_pythonswig_module.so)
SET(LIBNAO ${NAO_HOME}/build/build/${BUILD_TYPE}/nao/libnaointerface.so)


SET(FFT_INCLUDE ${NAO_HOME}/lib/fft/include)
SET(LIBFFT ${NAO_HOME}/lib/fft/libfftw3.a)
SET(LIBFFTW3F ${NAO_HOME}/lib/fft/libfftw3f.a)


SET(CAFFE_BUILD True)
MESSAGE("Building with Caffe: " ${CAFFE_BUILD})
IF(${CAFFE_BUILD})
  INCLUDE($ENV{NAO_HOME}/build/caffe.cmake)
ENDIF(${CAFFE_BUILD})
#ADD_DEFINITIONS(-DCPU_ONLY) # Disable GPU for nao caffe builds
#ADD_DEFINITIONS(-DUSE_OPENCV) # Allow OpenCV support in caffe
#SET(CAFFE_INCLUDE ${NAO_HOME}/lib/caffe/include ${NAO_HOME}/lib/caffe/build/src)
#IF(TOOL_BUILD)
#  SET(CAFFE_LIBS -L${NAO_HOME}/lib/caffe/build/lib -lhdf5_serial_hl -lhdf5_serial)
#ELSE(TOOL_BUILD)
#  SET(CAFFE_LIBS -L${NAO_HOME}/naoqi/link_libs -lhdf5_hl -lhdf5)
#ENDIF(TOOL_BUILD)
#SET(CAFFE_LIBS ${CAFFE_LIBS}
#  -lcaffe
#  -lglog
#  -lhdf5_cpp
#  -lhdf5_hl_cpp
#  -lcblas
#  -lgflags
#  -lprotobuf
#  -lleveldb
#  -lunwind
#  -latlas
#  -lcblas
#  -llmdb
#  -lsnappy
#  -llzma
#)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmax-errors=1")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-cpp")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lrt")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lz")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=overloaded-virtual") #TODO: enable this and remove MemoryBlock::operator=
INCLUDE_DIRECTORIES(${EIGEN_DIR} ${FLATBUFFERS_INCLUDE} ${YUVIEW_INCLUDE} ${YAML_INCLUDE} ${SRC_DIR} ${NAO_HOME}/build/include)
IF(NOT(TOOL_BUILD))
    INCLUDE_DIRECTORIES(${PYTHON_INCLUDE} ${FFT_INCLUDE} ${NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32/sbr-linux/usr/include)
ENDIF(NOT(TOOL_BUILD))

MESSAGE("C Compiler: " ${CMAKE_C_COMPILER})
MESSAGE("C++ Compiler: " ${CMAKE_CXX_COMPILER})

IF(NOT EXISTS ${ALGLIB})
  EXECUTE_PROCESS(COMMAND ${ALGLIB_SRC}/compile)
ENDIF()
SET(USER_DEFINE "USER_$ENV{USER}")
ADD_DEFINITIONS(-D"${USER_DEFINE}")
# IF(NOT(${BUILT_YUVIEW}))
#   SET(BUILT_YUVIEW True)
#   EXECUTE_PROCESS(COMMAND ${NAO_HOME}/lib/yuview/compile)
# ENDIF(NOT(${BUILT_YUVIEW}))
