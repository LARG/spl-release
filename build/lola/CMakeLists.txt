cmake_minimum_required(VERSION 2.8)

project(lola)

include(../common.cmake)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE SRCS 
  ${INTERFACE_DIR}/lola/*.cpp
  ${INTERFACE_DIR}/lola/*.h
)


add_executable(lola ${SRCS})
SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)
# target_link_libraries(lola ${LIBCORE} ${LINK_LIBS} $ENV{NAO_HOME}/lib/msgpack ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so ${LIBYUVIEW} ${ALSA_LIBRARIES})

TARGET_LINK_LIBRARIES(lola ${LIBCORE} ${LIBNAO} ${ALGLIB} ${LIBRSWALK2014} $ENV{NAO_HOME}/lib/msgpack ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_imgcodecs.so ${LIBYUVIEW} ${ALSA_LIBRARIES} ${LIBYAML-CPP} ${LIBFFT} ${LIBFFTW3F} ${LINK_LIBS} -lutil -ldl -lrt -lz -lespeak)
INCLUDE_DIRECTORIES(${ALSA_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
IF(NOT(TOOL_BUILD))
    SET(CVDIR $ENV{NAO_HOME}/naoqi/crosstoolchain/atom/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib)
    TARGET_LINK_LIBRARIES(lola ${CVDIR}/libopencv_core.so ${CVDIR}/libopencv_highgui.so ${CVDIR}/libopencv_flann.so ${CVDIR}/libopencv_features2d.so ${CVDIR}/libopencv_calib3d.so ${CVDIR}/libopencv_imgproc.so ${CVDIR}/libopencv_objdetect.so ${CVDIR}/libopencv_ml.so ${CVDIR}/libopencv_imgcodecs.so ${LIBYUVIEW})
ENDIF(NOT(TOOL_BUILD))

