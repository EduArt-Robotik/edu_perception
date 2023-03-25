/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_perception/qr_code.hpp"

#include <opencv2/opencv.hpp>

#include <memory>

namespace zbar {
class ImageScanner;
} // end namespace zbar

namespace eduart {
namespace perception {
namespace detector {

class QrCodeDetector
{
public:
  struct Parameter {
    struct {
      float horizontal = 0.4f;
      float vertical = 0.2f;
    } roi_increase_rate;
    std::string qr_code_text_filter = "";
  };

  QrCodeDetector(const Parameter& parameter);

  /**
   * \brief Detects an single QR code. Either it will returns the first found code, thats because it is recommended
   *        to set up an filter string via parameters. This method is designed to find a unique QR code and optimized
   *        for it using an region of interest concept.
   * \param image Input gray scale image.
   * \throw Throws an std::runtime_error if an error occurs durring detection process.
   * \returns Returns the detected QR code or in case of non detection an empty QR code object.
   */
  QrCode detect(const cv::Mat& image);
  inline QrCode operator()(const cv::Mat& image) { return detect(image); }

  inline cv::Rect getCurrentRoi() const { return _roi; }

private:
  cv::Rect increaseRoi(
    const decltype(QrCode::point)& qr_code_points, const cv::Size image_size, const Parameter& parameter);

  const Parameter _parameter;

  std::shared_ptr<zbar::ImageScanner> _scanner;
  cv::Rect _roi;
};

} // end namespace detector
} // end namespace perception
} // end namespace eduart
