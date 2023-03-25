#include "edu_perception/stereo/pose_estimation.hpp"

#include <Eigen/Geometry>

namespace eduart {
namespace perception {
namespace stereo {

geometry_msgs::msg::Pose estimate_pose_of_qr_code(
  const StereoInference& inference, const QrCode& left_qr_code, const QrCode& right_qr_code)
{
  // Matching points are: point[0] --> point[1] == x axis
  //                      point[0] --> point[2] == y axis
  const Eigen::Vector3f origin = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[0].x, left_qr_code.point[0].y),
    Eigen::Vector2i(right_qr_code.point[0].x, right_qr_code.point[0].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f end_point_x_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[1].x, left_qr_code.point[1].y),
    Eigen::Vector2i(right_qr_code.point[1].x, right_qr_code.point[1].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f end_point_y_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[2].x, left_qr_code.point[2].y),
    Eigen::Vector2i(right_qr_code.point[2].x, right_qr_code.point[2].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f middle_point = (end_point_x_axis + end_point_y_axis) * 0.5;

  // Based on estimated coordinate axes build up pose of qr code.
  const Eigen::Vector3f x_axis = end_point_x_axis - origin;
  const Eigen::Vector3f y_axis = end_point_y_axis - origin;
  const Eigen::Vector3f z_axis = x_axis.cross(y_axis);

  // Use C_camera = R * C_qr_code --> C_camera^-1 * C_qr_code = R
  const Eigen::DiagonalMatrix<float, 3> camera_coordinate_sysytem(1.0f, 1.0f, 1.0f);
  const auto camera_coordinate_sysytem_inv = camera_coordinate_sysytem.inverse();

  Eigen::Matrix3f qr_code_coordinate_system;
  qr_code_coordinate_system.col(0) = x_axis.normalized();
  qr_code_coordinate_system.col(1) = y_axis.normalized();
  qr_code_coordinate_system.col(2) = z_axis.normalized();

  const Eigen::Matrix3f R = qr_code_coordinate_system * camera_coordinate_sysytem_inv;
  const Eigen::Quaternionf orientation(R);

  // Return result in ROS format.
  geometry_msgs::msg::Pose pose;

  pose.position.x = middle_point.x();
  pose.position.y = middle_point.y();
  pose.position.z = middle_point.z();

  pose.orientation.w = orientation.w();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();

  return pose;
}

} // end namespace stereo
} // end namespace perception
} // end namespace eduart
