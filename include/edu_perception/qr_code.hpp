/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <string>
#include <array>

#include <opencv2/opencv.hpp>

namespace eduart {
namespace perception {

struct QrCode {
  inline bool isValid() const { return text.empty();}

  std::string text;
  // The points should be sorted in this way:
  //
  // p_0  |  p_1
  //      |
  // ----(c)---- // in image coordinates: --> x
  //      |                               |       
  // P_2  |  p_3
  //
  // where (c) is the virtual center point.
  std::array<cv::Point2i, 4> point;
};

} // end namespace perception
} // end namespace eduart
