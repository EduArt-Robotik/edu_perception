/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/node.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <memory>

namespace eduart {
namespace perception {

class AprilTagDraw : public rclcpp::Node
{
public:
  struct Parameter {

  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  AprilTagDraw();
  ~AprilTagDraw() override;

private:
  void callbackSynchedDetection(
    std::shared_ptr<const sensor_msgs::msg::Image> image,
    std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> detection);

  const Parameter _parameter;

  // std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> _sub_image;
  // std::shared_ptr<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>> _sub_apriltag_detection;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub_image;

  // time synched subscriptions
  using Synchronizer = message_filters::TimeSynchronizer<sensor_msgs::msg::Image, apriltag_msgs::msg::AprilTagDetectionArray>;

  message_filters::Subscriber<sensor_msgs::msg::Image> _sub_image;
  message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray> _sub_apriltag_detection;
  std::shared_ptr<Synchronizer> _synchronizer;
};

} // end namespace perception
} // end namespace eduart
