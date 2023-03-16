## Build Library
add_library(edu_perception
  stereo_inference.cpp
)
target_link_libraries(edu_perception
  Eigen3::Eigen
  depthai::core  
)
target_include_directories(edu_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(edu_perception

)
