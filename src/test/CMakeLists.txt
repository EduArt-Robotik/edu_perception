ament_add_gtest(test_pose_estimation
  unittest_maker_pose_to_world_pose.cpp
)
target_include_directories(test_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(test_pose_estimation
  Eigen3::Eigen
)
