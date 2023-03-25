#include "edu_perception/stereo_inference.hpp"
#include "edu_perception/angle.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>
#include <depthai/depthai.hpp>

namespace eduart {
namespace perception {

StereoInference::StereoInference(std::shared_ptr<dai::Device> device, const Parameter parameter)
  : _parameter(parameter)
  , _device(device)
{
  const auto calibration_data = _device->readCalibration();
  const float base_line = calibration_data.getBaselineDistance();
  
  // Original mono frames shape. Note: both camera frames must have the same size.
  _horizontal_fov = Angle::createFromDegree(calibration_data.getFov(dai::CameraBoardSocket::LEFT));
  const float focal_length = getFocalLengthPixel(_parameter.origin_width, _horizontal_fov);
  _disparity_scale_factor = base_line * focal_length;

  // Cropped frame shape.
  _resize_factor = static_cast<double>(_parameter.origin_height) / static_cast<double>(_parameter.height);
}

Eigen::Vector3f StereoInference::estimateSpatial(const Eigen::Vector2i coord_a, const Eigen::Vector2i coord_b) const
{
  const std::size_t disparity = calculateDistance(coord_a, coord_b);
  const float depth = calculateDepth(disparity);
  
  return calculateSpatial(coord_a, depth);
}

float StereoInference::getFocalLengthPixel(const std::size_t pixel_width, const Angle horizontal_fov) const
{
  return pixel_width * 0.5f / std::tan(horizontal_fov * 0.5);
}

float StereoInference::calculateDepth(const std::size_t disparity_pixel) const
{
  return _disparity_scale_factor / disparity_pixel;
}

int StereoInference::calculateDistance(const Eigen::Vector2i coord_a, const Eigen::Vector2i coord_b) const
{
  return (coord_a - coord_b).norm();
}

Angle StereoInference::calculateAngle(const int offset) const
{
  return std::atan(
    std::tan(_horizontal_fov * 0.5) * static_cast<float>(offset) / (static_cast<float>(_parameter.origin_width) * 0.5f)
  );
}

Eigen::Vector3f StereoInference::calculateSpatial(const Eigen::Vector2i coord, const float depth) const
{
  const Eigen::Vector2i middle_position(_parameter.width / 2, _parameter.height / 2);
  const Eigen::Vector2i bb_position = coord - middle_position;
  const Angle angle_x = calculateAngle(bb_position.x());
  const Angle angle_y = calculateAngle(bb_position.y());

  const Eigen::Vector3f spatial(depth, depth * std::tan(angle_x), -depth * std::tan(angle_y));

  return spatial * 0.01; // convert in meter
}

} // end namespace perception
} // end namespace eduart
