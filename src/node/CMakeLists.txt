## Build Nodes
### Qr Detection and Pose Estimation
add_executable(qr_detection_and_pose_estimation
  qr_detection_and_pose_estimation_node.cpp
)
target_include_directories(qr_detection_and_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(qr_detection_and_pose_estimation
  depthai::core
  ${OpenCV_LIBS}
  edu_perception
)
ament_target_dependencies(qr_detection_and_pose_estimation
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "cv_bridge"
  "tf2_ros"
)

### Oak D Camera Node
add_executable(oak_d_camera
  oak_d_camera_node.cpp
)
target_include_directories(oak_d_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(oak_d_camera
  depthai::core
  ${OpenCV_LIBS}
)
ament_target_dependencies(oak_d_camera
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
)