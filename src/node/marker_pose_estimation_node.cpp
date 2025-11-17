#include "marker_pose_estimation_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "zbar_ros_interfaces/msg/symbol.hpp"

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

static void invert_pose(geometry_msgs::msg::Pose& pose)
{
  const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d p_r(pose.position.x, pose.position.y, pose.position.z);

  // transform into object coordinates by inverting pose
  // note: actually a q.inverse() was expected to meet following equation:
  //       T^-1 = (T * R)^-1 = R^-1 * T^-1
  //       but somehow the orientation is already inverted. Maybe because of the marker's orientation is already inverted
  //       by the constructed maker corner points (3D).
  const Eigen::Vector3d p_w = q * (p_r * -1.0);

  pose.position.x = p_w.x();
  pose.position.y = p_w.y();
  pose.position.z = p_w.z();

  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
}

// static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q)
// {
//   const Eigen::Vector3d e_x = Eigen::Vector3d::UnitX();
//   const Eigen::Quaterniond R(q.w, q.x, q.y, q.z);
//   Eigen::Vector3d v = R * e_x;
//   v.y() = 0.0;

//   return e_x.dot(v);
// }

MarkerPoseEstimation::Parameter MarkerPoseEstimation::get_parameter(
  rclcpp::Node& ros_node, const Parameter& default_parameter)
{
  Parameter parameter;

  ros_node.declare_parameter<std::vector<std::string>>("marker.id", std::vector<std::string>());
  ros_node.declare_parameter<std::vector<float>>("marker.size", std::vector<float>());
  ros_node.declare_parameter<std::vector<std::string>>("marker.frame_id", std::vector<std::string>());
  ros_node.declare_parameter<float>("std_dev.position", default_parameter.std_dev.position);
  ros_node.declare_parameter<float>("std_dev.orientation", default_parameter.std_dev.orientation);
  ros_node.declare_parameter<bool>("transform_into_world", default_parameter.transform_into_world);
  ros_node.declare_parameter<bool>("invert_pose", default_parameter.invert_pose);
  ros_node.declare_parameter<std::string>("world_frame_id", default_parameter.world_frame_id);

  const auto ids = ros_node.get_parameter("marker.id").as_string_array();
  const auto sizes = ros_node.get_parameter("marker.size").as_double_array();
  const auto frame_id = ros_node.get_parameter("marker.frame_id").as_string_array();

  if (ids.size() != sizes.size() || ids.size() != frame_id.size()) {
    throw std::invalid_argument("MarkerPoseEstimation: the ids, sizes and frame_id must have same size!");
  }

  for (std::size_t i = 0; i < ids.size(); ++i) {
    parameter.marker_size[ids[i]] = sizes[i];
    parameter.frame_id[ids[i]] = frame_id[i];
  }

  parameter.std_dev.position = ros_node.get_parameter("std_dev.position").as_double();
  parameter.std_dev.orientation = ros_node.get_parameter("std_dev.orientation").as_double();
  parameter.transform_into_world = ros_node.get_parameter("transform_into_world").as_bool();
  parameter.invert_pose = ros_node.get_parameter("invert_pose").as_bool();
  parameter.world_frame_id = ros_node.get_parameter("world_frame_id").as_string();

  return parameter;
}

MarkerPoseEstimation::MarkerPoseEstimation()
  : rclcpp::Node("marker_pose_estimation")
  , _parameter(get_parameter(*this, _parameter))
  , _camera_matrix(3, 3, CV_64FC1, cv::Scalar(0.0))
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_unique<tf2_ros::TransformListener>(*_tf_buffer))
{
  // calculate required marker object data
  for (const auto& [marker_id, marker_size] : _parameter.marker_size) {
    _marker_objet_point[marker_id] = cv::Mat(4, 1, CV_32FC3);
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-marker_size / 2.0f,  marker_size / 2.0f, 0.0f); // bottom left
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[1] = cv::Vec3f( marker_size / 2.0f,  marker_size / 2.0f, 0.0f); // bottom right
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[2] = cv::Vec3f( marker_size / 2.0f, -marker_size / 2.0f, 0.0f); // top right
    _marker_objet_point[marker_id].ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-marker_size / 2.0f, -marker_size / 2.0f, 0.0f); // top left
  }

  // bring up ROS communication
  _sub_apriltag_detection = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
    "apriltag_detection",
    rclcpp::QoS(5).best_effort(),
    std::bind(&MarkerPoseEstimation::callbackApriltagDetection, this, std::placeholders::_1)
  );
  _sub_qr_code_detection = create_subscription<zbar_ros_interfaces::msg::Symbol>(
    "qr_code_detection",
    rclcpp::QoS(5).best_effort(),
    std::bind(&MarkerPoseEstimation::callbackQrCodeDetection, this, std::placeholders::_1)
  );
  _sub_camera_info = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::QoS(2).reliable(),
    std::bind(&MarkerPoseEstimation::callbackCameraInfo, this, std::placeholders::_1)
  );
  _pub_pose = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "estimated_pose", rclcpp::QoS(100).reliable()
  );
}

MarkerPoseEstimation::~MarkerPoseEstimation()
{

}

// \todo changed a lot for qr code input. Seems I did some mistakes last time. Proof if implementation is still correct for apriltag!
void MarkerPoseEstimation::callbackApriltagDetection(std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> msg)
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
      const auto marker_id = std::to_string(detection.id);
      std::vector<cv::Point2d> marker_corners = {
        cv::Point2d(detection.corners[0].x, detection.corners[0].y),
        cv::Point2d(detection.corners[1].x, detection.corners[1].y),
        cv::Point2d(detection.corners[2].x, detection.corners[2].y),
        cv::Point2d(detection.corners[3].x, detection.corners[3].y)
      };

      // publishing result
      // header
      // apriltag was seen by the camera --> frame id of camera
      pose.header.frame_id = _camera_info->header.frame_id;
      // apriltag was detected at message time --> stamp from message
      pose.header.stamp = msg->header.stamp;
      // estimate object pose
      pose.pose.pose = estimatePose(marker_corners, _marker_objet_point.at(marker_id));

      if (_parameter.transform_into_world) {
        transformPoseIntoWorld(pose, _parameter.frame_id.at(marker_id));
      }
      _pub_pose->publish(pose);
    }
    catch (std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "exception thrown during pose estimation. what = %s", ex.what());
    }
  }
}

void MarkerPoseEstimation::callbackQrCodeDetection(std::shared_ptr<const zbar_ros_interfaces::msg::Symbol> msg)
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
  
  try {
    const auto marker_id = msg->data;
    std::vector<cv::Point2d> marker_corners = {
      cv::Point2d(msg->points[1].x, msg->points[1].y), // bottom left
      cv::Point2d(msg->points[2].x, msg->points[2].y), // bottom right
      cv::Point2d(msg->points[3].x, msg->points[3].y), // top right
      cv::Point2d(msg->points[0].x, msg->points[0].y)  // top left
    };
   
    // publishing result
    // header
    // apriltag was seen by the camera --> frame id of camera
    pose.header.frame_id = _camera_info->header.frame_id;
    // apriltag was detected at message time --> stamp from message, but there isn't a header...
    pose.header.stamp = get_clock()->now();
    // estimate object pose
    pose.pose.pose = estimatePose(marker_corners, _marker_objet_point.at(marker_id));

    if (_parameter.transform_into_world) {
      transformPoseIntoWorld(pose, _parameter.frame_id.at(marker_id));
    }
    pose.header.frame_id = _camera_info->header.frame_id; 
    _pub_pose->publish(pose);    
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception thrown during pose estimation. what = %s", ex.what());
  }  
}

void MarkerPoseEstimation::callbackCameraInfo(std::shared_ptr<const sensor_msgs::msg::CameraInfo> msg)
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

geometry_msgs::msg::Pose MarkerPoseEstimation::estimatePose(const std::vector<cv::Point2d>& marker_corners, const cv::Mat& object_points)
{
  // estimate object pose
  cv::Vec3d rotation, translation;
  geometry_msgs::msg::Pose pose;

  cv::solvePnP(
    object_points, marker_corners, _camera_matrix, _distortion_coefficient,
    rotation, translation, false, cv::SOLVEPNP_IPPE_SQUARE
  );

  // position
  // switch axis to transform into robot coordinate system (x in front)
  pose.position.x =  translation[2];
  pose.position.y = -translation[0];
  pose.position.z = -translation[1];

  // orientation
  // switch axis to transform into robot coordinate system (x in front)
  Eigen::Quaterniond rotation_q = 
    Eigen::AngleAxisd( rotation[2], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-rotation[0], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd( rotation[1], Eigen::Vector3d::UnitZ());
  pose.orientation.w = rotation_q.w();
  pose.orientation.x = rotation_q.x();
  pose.orientation.y = rotation_q.y();
  pose.orientation.z = rotation_q.z();

  // invert pose if wanted. For example if you want to measured robots pose and not qr codes pose.
  if (_parameter.invert_pose) {
    invert_pose(pose);
  }

  return pose;
}

void MarkerPoseEstimation::transformPoseIntoWorld(
  geometry_msgs::msg::PoseWithCovarianceStamped& pose, const std::string marker_frame_id)
{
  // transform from marker frame into world frame
  try {
    // add transformation to world to the pose
    const auto transform = _tf_buffer->lookupTransform(
      _parameter.world_frame_id,
      // _parameter.apriltag_frame_id_prefix + marker_id,
      marker_frame_id,
      pose.header.stamp
    );
    tf2::doTransform(pose, pose, transform);
  }
  catch (std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "exception thrown during transform coordinates into world. what = %s", ex.what());
  }
}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::perception::MarkerPoseEstimation>());
  rclcpp::shutdown();

  return 0;
}
