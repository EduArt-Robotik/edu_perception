/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/publisher.hpp>

namespace eduart {
namespace perception {

class QrDetectionAndPoseEstimation : public rclcpp::Node
{
public:
  struct Parameter {

  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter = Parameter{});

  QrDetectionAndPoseEstimation();
  ~QrDetectionAndPoseEstimation() override;

private:
  Parameter _parameter;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
};

} // end namespace perception
} // end namespace eduart
