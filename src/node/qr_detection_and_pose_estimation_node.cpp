#include "qr_detection_and_pose_estimation.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace eduart {
namespace perception {

QrDetectionAndPoseEstimation::Parameter QrDetectionAndPoseEstimation::get_parameter(
  rclcpp::Node &ros_node, const Parameter& default_parameter)
{
  (void)ros_node;
  (void)default_parameter;
  Parameter parameter;

  return parameter;
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
{

}

QrDetectionAndPoseEstimation::~QrDetectionAndPoseEstimation()
{

}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::perception::QrDetectionAndPoseEstimation>());
  rclcpp::shutdown();

  return 0;
}
