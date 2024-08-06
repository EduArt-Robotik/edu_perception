/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <opencv2/core/mat.hpp>

#include <memory>
#include <cstddef>

namespace eduart {
namespace perception {

class AprilTagPoseEstimation : public rclcpp::Node
{
public:
  struct Parameter {
    std::map<std::size_t, float> marker_size;
    struct {
      float position = 0.2;
      float orientation = 10.0f * M_PI / 180.0f; // 10°
    } std_dev;
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  AprilTagPoseEstimation();
  ~AprilTagPoseEstimation() override;

private:
  void callbackDetection(std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> msg);
  void callbackCameraInfo(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg);

  const Parameter _parameter;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> _camera_info;
  cv::Mat _distortion_coefficient;
  cv::Mat _camera_matrix;
  std::map<std::size_t, cv::Mat> _marker_objet_point;

  std::shared_ptr<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>> _sub_detection;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> _sub_camera_info;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> _pub_pose;
};

} // end namespace perception
} // end namespace eduart
