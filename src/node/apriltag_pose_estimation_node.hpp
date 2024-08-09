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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
      float orientation = 45.0f * M_PI / 180.0f; // 45°
    } std_dev;

    bool transform_into_world = true;
    std::string world_frame_id = "map";
    std::string maker_frame_id_prefix = "apriltag_";
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  AprilTagPoseEstimation();
  ~AprilTagPoseEstimation() override;

private:
  void callbackDetection(std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> msg);
  void callbackCameraInfo(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg);
  void transformIntoWorld(geometry_msgs::msg::PoseWithCovarianceStamped& pose, const std::size_t marker_id);

  const Parameter _parameter;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> _camera_info;
  cv::Mat _distortion_coefficient;
  cv::Mat _camera_matrix;
  std::map<std::size_t, cv::Mat> _marker_objet_point;

  std::shared_ptr<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>> _sub_detection;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> _sub_camera_info;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> _pub_pose;

  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
};

} // end namespace perception
} // end namespace eduart
