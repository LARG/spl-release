CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(vision)

INCLUDE(../common.cmake)

FILE(GLOB_RECURSE SRCS ${INTERFACE_DIR}/shared_vision/*.cpp)
SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)

FIND_PACKAGE(ALSA REQUIRED)

ADD_EXECUTABLE(vision ${SRCS})
IF(TOOL_BUILD AND ${CAFFE_BUILD})
  TARGET_LINK_LIBRARIES(vision ${CAFFE_LIBS})
ELSE()
  TARGET_LINK_LIBRARIES(vision ${LIBPYTHONSWIG} ${LIBCORE} ${LIBNAO} ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so ${CVDIR}/libopencv_videoio.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so ${CVDIR}/libopencv_ml.so)
ENDIF()
TARGET_LINK_LIBRARIES(vision ${LINK_LIBS} ${ALGLIB} ${LIBYAML-CPP} ${LIBYUVIEW} ${LIBFFT} ${LIBFFTW3F} ${ALSA_LIBRARIES} -lutil -ldl -lrt -lz)
INCLUDE_DIRECTORIES(${ALSA_INCLUDE_DIRS})
