#include "apriltag_pose_estimation_node.hpp"

#include <opencv2/calib3d.hpp>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <memory>
#include <cmath>

namespace eduart {
namespace perception {

static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
  const Eigen::Vector3d e_x = Eigen::Vector3d::UnitX();
  const Eigen::Quaterniond R(q.w, q.x, q.y, q.z);
  Eigen::Vector3d v = R * e_x;
  v.y() = 0.0;

  return e_x.dot(v);
}

AprilTagPoseEstimation::Parameter AprilTagPoseEstimation::get_parameter(
  rclcpp::Node& ros_node, const Parameter& default_parameter)
{
  Parameter parameter;

  ros_node.declare_parameter<std::vector<int>>("marker.id");
  ros_node.declare_parameter<std::vector<float>>("marker.size");
  ros_node.declare_parameter<float>("std_dev.position", default_parameter.std_dev.position);
  ros_node.declare_parameter<float>("std_dev.orientation", default_parameter.std_dev.orientation);

  const auto id = ros_node.get_parameter("marker.id").as_integer_array();
  const auto size = ros_node.get_parameter("marker.size").as_double_array();

  if (id.size() != size.size()) {
    throw std::invalid_argument("AprilTagPoseEstimation: the ids and sizes must have same size!");
  }

  for (std::size_t i = 0; i < id.size(); ++i) {
    parameter.marker_size[id[i]] = size[i];
  }

  parameter.std_dev.position = ros_node.get_parameter("std_dev.position").as_double();
  parameter.std_dev.orientation = ros_node.get_parameter("std_dev.orientation").as_double();

  return parameter;
}

AprilTagPoseEstimation::AprilTagPoseEstimation()
  : rclcpp::Node("apriltag_pose_estimation")
  , _parameter(get_parameter(*this, _parameter))
  , _camera_matrix(3, 3, CV_64FC1, cv::Scalar(0.0))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer))
{
  // calculate required marker object data
  for (const auto& marker_size_entry : _parameter.marker_size) {
    const auto marker_size = marker_size_entry.second;
    const auto marker_id = marker_size_entry.first;

    _marker_objet_point[marker_id] = cv::Mat(4, 1, CV_32FC3);
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size / 2.0f,  marker_size / 2.0f, 0.0f);
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[1] = cv::Vec3f( marker_size / 2.0f,  marker_size / 2.0f, 0.0f);
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[2] = cv::Vec3f( marker_size / 2.0f, -marker_size / 2.0f, 0.0f);
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size / 2.0f, -marker_size / 2.0f, 0.0f);
  }

  // bring up ROS communication
  _sub_detection = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "detection",
    rclcpp::QoS(5).best_effort(),
    std::bind(&AprilTagPoseEstimation::callbackDetection, this, std::placeholders::_1)
  );
  _sub_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::QoS(2).reliable(),
    std::bind(&AprilTagPoseEstimation::callbackCameraInfo, this, std::placeholders::_1)
  );
  _pub_pose = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "estimated_pose", rclcpp::QoS(100).reliable()
  );
}

AprilTagPoseEstimation::~AprilTagPoseEstimation()
{

}

void AprilTagPoseEstimation::callbackDetection(std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> msg)
{
  if (_camera_info == nullptr) {
    RCLCPP_INFO(get_logger(), "no camera info retrieved yet. Do nothing...");
    return;
  }

  // prepare message as far as possible
  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  // covariances
  pose.pose.covariance[0]  = _parameter.std_dev.position * _parameter.std_dev.position;
  pose.pose.covariance[7]  = _parameter.std_dev.position * _parameter.std_dev.position;
  pose.pose.covariance[14] = _parameter.std_dev.position * _parameter.std_dev.position;
  pose.pose.covariance[21] = _parameter.std_dev.orientation * _parameter.std_dev.orientation;
  pose.pose.covariance[28] = _parameter.std_dev.orientation * _parameter.std_dev.orientation;
  pose.pose.covariance[35] = _parameter.std_dev.orientation * _parameter.std_dev.orientation;

  for (const auto& detection : msg->detections) {
    try {
      // estimate marker pose
      const auto marker_id = detection.id;
      const auto& object_points = _marker_objet_point.at(marker_id);

      std::vector<cv::Point2d> marker_corners = {
        cv::Point2d(detection.corners[0].x, detection.corners[0].y),
        cv::Point2d(detection.corners[1].x, detection.corners[1].y),
        cv::Point2d(detection.corners[2].x, detection.corners[2].y),
        cv::Point2d(detection.corners[3].x, detection.corners[3].y)
      };

      // std::cout << "marker id = " << static_cast<int>(marker_id) << std::endl;
      // std::cout << "object points:\n" << object_points << std::endl;
      // std::cout << "marker corners:\n";
      // for (const auto& point : marker_corners) std::cout << point << ", ";
      // std::cout << std::endl;
      // std::cout << "camera matrix:\n" << _camera_matrix << std::endl;
      // std::cout << "distortion:\n" << _distortion_coefficient << std::endl;

      cv::Vec3d rotation, translation;
      cv::solvePnP(
        object_points, marker_corners, _camera_matrix, _distortion_coefficient,
        rotation, translation, false, cv::SOLVEPNP_IPPE_SQUARE
      );

      // publishing result
      // header
      // apriltag was seen by the camera --> frame id of camera
      pose.header.frame_id = _camera_info->header.frame_id;
      // apriltag was detected at message time --> stamp from message
      pose.header.stamp = msg->header.stamp;

      // position
      // switch axis to transform into robot coordinate system (x in front)
      pose.pose.pose.position.x =  translation[2];
      pose.pose.pose.position.y = -translation[0];
      pose.pose.pose.position.z =  translation[1];

      // orientation
      // switch axis to transform into robot coordinate system (x in front)
      Eigen::Quaterniond rotation_q = 
        Eigen::AngleAxisd( rotation[2], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-rotation[0], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd( rotation[1], Eigen::Vector3d::UnitZ());
      pose.pose.pose.orientation.w = rotation_q.w();
      pose.pose.pose.orientation.x = rotation_q.x();
      pose.pose.pose.orientation.y = rotation_q.y();
      pose.pose.pose.orientation.z = rotation_q.z();

      if (_parameter.transform_into_world) {
        transformIntoWorld(pose, marker_id);
      }
      pose.header.frame_id = _camera_info->header.frame_id; 
      _pub_pose->publish(pose);
    }
    catch (std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "exception thrown during pose estimation. what = %s", ex.what());
    }
  }
}

void AprilTagPoseEstimation::callbackCameraInfo(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg)
{
  if (_camera_info == nullptr) {
    RCLCPP_INFO(get_logger(), "first camera info retrieved.");
  }

  _camera_info = msg;
  _distortion_coefficient = cv::Mat(msg->d, true);

  for (std::size_t i = 0; i < 9; ++i) {
    _camera_matrix.at<double>(i / 3, i % 3) = _camera_info->k[i];
  }
}

void AprilTagPoseEstimation::transformIntoWorld(
  geometry_msgs::msg::PoseWithCovarianceStamped& pose, const std::size_t marker_id)
{
  try {
    // add transformation to world to the pose
    const auto transform = _tf_buffer->lookupTransform(
      _parameter.world_frame_id,
      _parameter.maker_frame_id_prefix + std::to_string(marker_id),
      pose.header.stamp
    );
    tf2::doTransform(pose, pose, transform);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception thrown during transform coordinates into world. what = %s", ex.what());
  }
  // const Eigen::Quaterniond q_180 =
  //   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
  //   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
  //   Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
  // const Eigen::Quaterniond q_180(0.0, 0.0, 0.0, 1.0);
  const Eigen::Quaterniond q_180 =
    Eigen::AngleAxisd( 0.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd( 0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond q(
    pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  const Eigen::Vector3d p_r(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  // const Eigen::Quaterniond q_inv = q_180 * q;
  const Eigen::Quaterniond q_w = q_180 * q;

  const Eigen::Vector3d p_w = q * p_r;

  // std::cout << "yaw(q) = " << quaternion_to_yaw(pose.pose.pose.orientation) << std::endl;

  pose.pose.pose.position.x = p_w.x();
  pose.pose.pose.position.y = p_w.y();
  pose.pose.pose.position.z = p_w.z();

  pose.pose.pose.orientation.w = q_w.w();
  pose.pose.pose.orientation.x = q_w.x();
  pose.pose.pose.orientation.y = q_w.y();
  pose.pose.pose.orientation.z = q_w.z();

  // std::cout << "yaw(q_inv) = " << quaternion_to_yaw(pose.pose.pose.orientation) << std::endl;
}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::perception::AprilTagPoseEstimation>());
  rclcpp::shutdown();

  return 0;
}
