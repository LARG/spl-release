SET(TRAINER_DIR ${NAO_HOME}/tools/trainers)
FILE(GLOB_RECURSE trainer_SOURCES ${TRAINER_DIR}/src/*.cpp)
INCLUDE_DIRECTORIES(${CAFFE_INCLUDE} ${TRAINER_DIR}/include)

FILE(GLOB trainers ${TRAINER_DIR}/main/*.cpp)
FOREACH(trainer_main ${trainers})
  GET_FILENAME_COMPONENT(trainer ${trainer_main} NAME_WE)
  SET(trainer "${trainer}_trainer")
  ADD_EXECUTABLE(${trainer} ${trainer_SOURCES} ${trainer_main})
  TARGET_LINK_LIBRARIES(${trainer} ${CAFFE_LIBS} ${QT_LIBRARIES} ${FFTW_LIBRARIES} ${PNG_LIBRARIES} core rswalk2014 _pythonswig_module ${LIBFLATBUFFERS}x64 ${LIBYAML-CPP}x64 ${ALGLIB}x64 ${LIBYUVIEW}x64 ${OPENCV2_LIBS} ${Boost_LIBRARIES} rt pthread -lglog)
  EXECUTE_PROCESS(COMMAND ln -sf "${CMAKE_CURRENT_BINARY_DIR}/${trainer}" "${CMAKE_CURRENT_LIST_DIR}/${trainer}")
ENDFOREACH(trainer_main ${trainers})