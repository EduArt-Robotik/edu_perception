#include "edu_perception/transform.hpp"

#include <Eigen/Geometry>

namespace eduart {
namespace perception {

geometry_msgs::msg::Pose transform_pose(
  const geometry_msgs::msg::Pose& pose_in, const geometry_msgs::msg::Transform& transform)
{
  const Eigen::Quaternionf rot_transform(
    transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  const Eigen::Quaternionf rot_pose(
    pose_in.orientation.w, pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z);
  const Eigen::Quaternionf new_rot = rot_pose * rot_transform.inverse();
  const Eigen::Vector3f translation(
    new_rot * Eigen::Vector3f(-transform.translation.x, -transform.translation.y, -transform.translation.z));

  geometry_msgs::msg::Pose pose_out;
  pose_out.position.x = pose_in.position.x + translation.x();
  pose_out.position.y = pose_in.position.y + translation.y();
  pose_out.position.z = pose_in.position.z + translation.z();
  pose_out.orientation.w = new_rot.w();
  pose_out.orientation.x = new_rot.x();
  pose_out.orientation.y = new_rot.y();
  pose_out.orientation.z = new_rot.z();

  return pose_out;
}

} // end namespace perception
} // end namespace eduart
