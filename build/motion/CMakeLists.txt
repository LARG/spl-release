CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(motion)

INCLUDE(../common.cmake)

FILE(GLOB_RECURSE SRCS ${INTERFACE_DIR}/shared_motion/*.cpp)

FIND_PACKAGE(ALSA REQUIRED)

ADD_EXECUTABLE(motion ${SRCS})
TARGET_LINK_LIBRARIES(motion ${LIBCORE} ${LIBNAO} ${ALGLIB} ${LIBRSWALK2014} ${LIBYUVIEW} ${LIBYAML-CPP} ${LIBFFT} ${LIBFFTW3F} ${LINK_LIBS} ${ALSA_LIBRARIES} -lutil -ldl -lrt -lz)
INCLUDE_DIRECTORIES(${ALSA_INCLUDE_DIRS})                  
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
IF(NOT(TOOL_BUILD))
    SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)
    TARGET_LINK_LIBRARIES(motion ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so ${CVDIR}/libopencv_ml.so ${CVDIR}/libopencv_imgcodecs.so ${LIBYUVIEW})
ENDIF(NOT(TOOL_BUILD))
