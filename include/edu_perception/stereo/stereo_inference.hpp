/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_perception/angle.hpp"

#include <cstddef>
#include <memory>

#include <Eigen/Core>

#include <opencv2/core.hpp>

// Forward Declarations
namespace dai {
class Device;
} // end namespace

namespace eduart {
namespace perception {
namespace stereo {

class StereoInference
{
public:
  struct Parameter {
    std::size_t width = 0;
    std::size_t height = 0;
    std::size_t origin_width = 0;
    std::size_t origin_height = 0;
  };

  StereoInference(std::shared_ptr<dai::Device> device, const Parameter parameter);
  virtual ~StereoInference() = default;

  Eigen::Vector3f estimateSpatial(const Eigen::Vector2i coord_a, const Eigen::Vector2i coord_b) const;

protected:
  float getFocalLengthPixel(const std::size_t pixel_width, const Angle horizontal_fov) const;
  float calculateDepth(const std::size_t disparity_pixel) const;
  int calculateDistance(const Eigen::Vector2i coord_a, const Eigen::Vector2i coord_b) const;
  Angle calculateAngle(const int offset) const;
  Eigen::Vector3f calculateSpatial(const Eigen::Vector2i coord, const float depth) const;

private:
  const Parameter _parameter;
  std::shared_ptr<dai::Device> _device;

  cv::Mat _projection_left;
  cv::Mat _projection_right;

  Angle0To2Pi _horizontal_fov;
  float _disparity_scale_factor;
  float _resize_factor;
}; 

} // end namespace stereo
} // end namespace perception
} // end namespace eduart
