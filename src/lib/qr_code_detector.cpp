#include "edu_perception/detector/qr_code_detector.hpp"

#include <zbar.h>

namespace eduart {
namespace perception {
namespace detector {

static void sort_qr_code_points(decltype(QrCode::point)& qr_code_points)
{
  // Expecting the points represent a rectangle and the rectangle's orientation is less than 45°.
  // Then the center point (c) can be used to locate each point's ... :
  //
  // p_0  |  p_1
  //      |
  // ----(c)---- // in image coordinates: --> x
  //      |                               |       
  // P_2  |  p_3                        y v
  //
  const cv::Point2i center_point = (qr_code_points[0] + qr_code_points[1] + qr_code_points[2] + qr_code_points[3]) / 4;
  decltype(QrCode::point) sorted_points;
  std::uint8_t flags_sorted_points = 0x0f;

  for (const auto& point : qr_code_points) {
    // p_0
    if (point.x < center_point.x && point.y < center_point.y) {
      sorted_points[0] = point;
      flags_sorted_points &= ~(1 << 0);
    }
    // p_1
    else if (point.x > center_point.x && point.y < center_point.y) {
      sorted_points[1] = point;
      flags_sorted_points &= ~(1 << 1);
    }
    // p_2
    else if (point.x < center_point.x && point.y > center_point.y) {
      sorted_points[2] = point;
      flags_sorted_points &= ~(1 << 2);
    }
    // p_3
    else if (point.x > center_point.x && point.y > center_point.y) {
      sorted_points[3] = point;
      flags_sorted_points &= ~(1 << 3);
    }
    else {
      throw std::runtime_error("Can't locate point's quant. Expectations not fulfilled. Cancel sorting qr code points.");
    }
  }

  if (flags_sorted_points != 0) {
    throw std::runtime_error("Sorting of qr code points went wrong. Maybe expectations wasn't fulfilled");
  }

  // Sorting was successful.
  qr_code_points = sorted_points;
}

QrCodeDetector::QrCodeDetector(const Parameter& parameter)
  : _parameter(parameter)
  , _scanner(std::make_shared<zbar::ImageScanner>())
{
  _scanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _scanner->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

QrCode QrCodeDetector::detect(const cv::Mat &image)
{
  zbar::Image zbar_image(
    image.cols, image.rows, "Y800", image.data, image.cols * image.rows
  );

  zbar_image.set_crop(_roi.x, _roi.y, _roi.width, _roi.height);
  _scanner->scan(zbar_image);

  if (zbar_image.symbol_begin() == zbar_image.symbol_end()) {
    _roi = cv::Rect(0, 0, image.size().width, image.size().height);    
    return {};
  }

  QrCode qr_code;

  for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
    if (symbol->get_data().find(_parameter.qr_code_text_filter) != std::string::npos) {
      qr_code.text = symbol->get_data();

      for (std::size_t i = 0; i < qr_code.point.size(); ++i) {
        qr_code.point[i].x = symbol->get_location_x(i);
        qr_code.point[i].y = symbol->get_location_y(i);
      }

      _roi = increaseRoi(qr_code.point, image.size(), _parameter);
      sort_qr_code_points(qr_code.point);
      return qr_code;
    }
  }

  _roi = cv::Rect(0, 0, image.size().width, image.size().height);
  return qr_code;
}

cv::Rect QrCodeDetector::increaseRoi(
    const decltype(QrCode::point)& qr_code_points, const cv::Size image_size, const Parameter& parameter)
{
  const cv::Rect future_roi = cv::boundingRect(qr_code_points);
  const cv::Point top_left_corner(
    // Decrease x coordinate according given rate or down to image border.
    std::max(future_roi.x - static_cast<int>(future_roi.width  * parameter.roi_increase_rate.horizontal), 0),
    // Decrease x coordinate according given rate or down to image border.    
    std::max(future_roi.y - static_cast<int>(future_roi.height * parameter.roi_increase_rate.vertical), 0)
  );
  const cv::Point bottom_right_corner(
    // Increase x coordinate according given rate or up to image border.
    std::min(
      future_roi.br().x + static_cast<int>(future_roi.width  * parameter.roi_increase_rate.horizontal),
      image_size.width  - 1
    ),
    // Increase y coordinate according given rate or up to image border.
    std::min(
      future_roi.br().y + static_cast<int>(future_roi.height * parameter.roi_increase_rate.vertical),
      image_size.height - 1
    )
  );
  const cv::Rect future_roi_increased(top_left_corner, bottom_right_corner);

  return future_roi_increased;
}

} // end namespace detector
} // end namespace perception
} // end namespace eduart
