#include "edu_perception/stereo/stereo_inference.hpp"
#include "edu_perception/angle.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>
#include <depthai/depthai.hpp>
#include <opencv2/calib3d.hpp>

namespace eduart {
namespace perception {
namespace stereo {

static cv::Mat convert_to_mat(const std::vector<std::vector<float>>& in)
{
  if (in.empty()) {
    return { };
  }

  cv::Mat out(in.size(), in[0].size(), CV_32F);

  for (std::size_t row = 0; row < in.size(); ++row) {
    for (std::size_t col = 0; col < in[0].size(); ++col) {
      out.at<float>(row, col) = in[row][col];
    }
  }

  return out;
}

static cv::Mat extract_rotation(const cv::Mat& transform)
{
  cv::Mat rotation(3, 3, CV_32F);

  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      rotation.at<float>(row, col) = transform.at<float>(row, col);
    }
  }

  return rotation;
}

static cv::Mat extract_translation(const cv::Mat& transform)
{
  cv::Mat translation(3, 1, CV_32F);

  translation.at<float>(0, 0) = transform.at<float>(0, 3);
  translation.at<float>(1, 0) = transform.at<float>(1, 3);
  translation.at<float>(2, 0) = transform.at<float>(2, 3);

  return translation;
}

StereoInference::StereoInference(std::shared_ptr<dai::Device> device, const Parameter parameter)
  : _parameter(parameter)
  , _device(device)
{
  const auto calibration_data = _device->readCalibration();
  const cv::Mat intrinsic_left = convert_to_mat(calibration_data.getCameraIntrinsics(
    dai::CameraBoardSocket::LEFT, _parameter.width, _parameter.height)
  );
  const cv::Mat intrinsic_right = convert_to_mat(calibration_data.getCameraIntrinsics(
    dai::CameraBoardSocket::RIGHT, _parameter.width, _parameter.height)
  );  
  const cv::Mat extrinsic = convert_to_mat(calibration_data.getCameraExtrinsics(
    dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT)
  );
  const auto distortion_left = calibration_data.getDistortionCoefficients(
    dai::CameraBoardSocket::LEFT
  );
  const auto distortion_right = calibration_data.getDistortionCoefficients(
    dai::CameraBoardSocket::RIGHT
  );

  std::cout << "intrinsic_left:\n" << intrinsic_left << std::endl;
  std::cout << "distortion_left:\n";
  for (const auto value : distortion_left) std::cout << value << " ";
  std::cout << "\ndistortion_right\n";
  for (const auto value: distortion_right) std::cout << value << " ";
  std::cout << "\nintrinsic_right:\n" << intrinsic_right << std::endl;
  std::cout << "extrinsic\n" << extrinsic << std::endl;

  cv::stereoRectify(
    intrinsic_left, distortion_left, intrinsic_right, distortion_right,
    cv::Size(_parameter.width, _parameter.height), extract_rotation(extrinsic),
    extract_translation(extrinsic), cv::Mat(), cv::Mat(), _projection_left, _projection_right, cv::Mat()
  );

  std::cout << "projection_left:\n" << _projection_left << std::endl;
  std::cout << "projection_right:\n" << _projection_right << std::endl;

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

} // end namespace stereo
} // end namespace perception
} // end namespace eduart
