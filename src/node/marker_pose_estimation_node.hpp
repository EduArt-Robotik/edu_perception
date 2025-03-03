/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>
#include <zbar_ros_interfaces/msg/symbol.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core/mat.hpp>

#include <memory>
#include <cstddef>
#include <unordered_map>

namespace eduart {
namespace perception {

class MarkerPoseEstimation : public rclcpp::Node
{
public:
  struct Parameter {
    std::unordered_map<std::string, float> marker_size;
    std::unordered_map<std::string, std::string> frame_id;
    struct {
      float position = 0.5;
      float orientation = 45.0f * M_PI / 180.0f; // 45°
    } std_dev;

    bool transform_into_world = false;
    bool invert_pose = true;
    std::string world_frame_id = "map";
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  MarkerPoseEstimation();
  ~MarkerPoseEstimation() override;

private:
  void callbackApriltagDetection(std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> msg);
  void callbackQrCodeDetection(std::shared_ptr<const zbar_ros_interfaces::msg::Symbol> msg);
  void callbackCameraInfo(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg);

  geometry_msgs::msg::Pose estimatePose(const std::vector<cv::Point2d>& marker_corners, const cv::Mat& object_points);
  void transformPoseIntoWorld(geometry_msgs::msg::PoseWithCovarianceStamped& pose, const std::string marker_id);

  const Parameter _parameter;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> _camera_info;
  cv::Mat _distortion_coefficient;
  cv::Mat _camera_matrix;
  std::map<std::string, cv::Mat> _marker_objet_point;

  std::shared_ptr<rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>> _sub_apriltag_detection;
  std::shared_ptr<rclcpp::Subscription<zbar_ros_interfaces::msg::Symbol>> _sub_qr_code_detection;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>> _sub_camera_info;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> _pub_pose;

  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
};

} // end namespace perception
} // end namespace eduart
