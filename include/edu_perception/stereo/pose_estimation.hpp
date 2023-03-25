/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_perception/qr_code.hpp"
#include "stereo_inference.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace eduart {
namespace perception {
namespace stereo {

geometry_msgs::msg::Pose estimate_pose_of_qr_code(
  const StereoInference& inference, const QrCode& left_qr_code, const QrCode& right_qr_code);

} // end namespace stereo
} // end namespace perception
} // end namespace eduart
