## Build Library
add_library(edu_perception
  stereo_inference.cpp
  qr_code_detector.cpp
  pose_estimation.cpp
  transform.cpp
  h264_decoder.cpp
)
target_link_libraries(edu_perception
  Eigen3::Eigen
  depthai::core
  ${OpenCV_LIBS}
  zbar
  avcodec
  avformat
  swscale
  avutil
)
target_include_directories(edu_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(edu_perception
  geometry_msgs
)
