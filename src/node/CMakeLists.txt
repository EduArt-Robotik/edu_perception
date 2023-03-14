## Build Nodes
### Qr Detection and Pose Estimation
add_executable(qr_detection_and_pose_estimation
  qr_detection_and_pose_estimation_node.cpp
)
target_link_libraries(qr_detection_and_pose_estimation
  depthai::core
  ${OpenCV_LIBS}
)
ament_target_dependencies(qr_detection_and_pose_estimation
  "rclcpp"
  "geometry_msgs"
  "cv_bridge"
)
