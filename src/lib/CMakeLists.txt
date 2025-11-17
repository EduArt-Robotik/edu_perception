## Build Library
### Collecting Source Files
set(SOURCES_EDU_PERCEPTION
  qr_code_detector.cpp
  pose_estimation.cpp
  transform.cpp
)

if(TARGET depthai::core)
  set(SOURCES_EDU_PERCEPTION
    ${SOURCES_EDU_PERCEPTION}
    stereo_inference.cpp
    h264_decoder.cpp
  )
endif()

### Collecting Libs
set(LIBS_EDU_PERCEPTION
  Eigen3::Eigen
  ${OpenCV_LIBS}
  zbar
)

if (TARGET depthai::core)
  set(LIBS_EDU_PERCEPTION
    ${LIBS_EDU_PERCEPTION}
    depthai::core
    avcodec
    avformat
    swscale
    avutil
  )
endif()

### Creating Library
add_library(edu_perception
  ${SOURCES_EDU_PERCEPTION}
)
target_link_libraries(edu_perception
  ${LIBS_EDU_PERCEPTION}
)
target_include_directories(eduart_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(eduart_perception
  geometry_msgs
)
