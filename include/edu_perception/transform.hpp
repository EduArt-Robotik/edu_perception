/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace eduart {
namespace perception {

// Note: this function transform backwards at the moment!
geometry_msgs::msg::Pose transform_pose(
  const geometry_msgs::msg::Pose& pose_in, const geometry_msgs::msg::Transform& transform);

} // end namespace perception
} // end namespace eduart
