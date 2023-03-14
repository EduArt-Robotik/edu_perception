#include "qr_detection_and_pose_estimation.hpp"

// #include "depthai-shared/common/CameraBoardSocket.hpp"
// #include "depthai-shared/properties/MonoCameraProperties.hpp"
// #include "depthai/device/Device.hpp"
// #include "depthai/pipeline/Pipeline.hpp"
// #include "depthai/pipeline/node/MonoCamera.hpp"
// #include "depthai/pipeline/node/XLinkOut.hpp"

#include <cstddef>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <depthai/depthai.hpp>
#include <stdexcept>
#include <zbar.h>

#include <chrono>
#include <memory>

namespace eduart {
namespace perception {

using std::chrono::duration;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::round;
using namespace std::chrono_literals;

QrDetectionAndPoseEstimation::Parameter QrDetectionAndPoseEstimation::get_parameter(
  rclcpp::Node &ros_node, const Parameter& default_parameter)
{
  Parameter parameter;

  ros_node.declare_parameter<float>("camera.fps", default_parameter.camera.fps);
  ros_node.declare_parameter<std::string>("qr_text_filter", default_parameter.qr_text_filter);

  parameter.camera.fps = ros_node.get_parameter("camera.fps").as_double();
  parameter.qr_text_filter = ros_node.get_parameter("qr_text_filter").as_string();

  return parameter;
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
  , _parameter(get_parameter(*this, Parameter()))
  , _qr_code_scanner(std::make_shared<zbar::ImageScanner>())
{
  setupCameraPipeline(_parameter);

  // const auto timer_period = round<milliseconds>(duration<float>{1.0f / _parameter.camera.fps});
  _timer_processing_camera = create_wall_timer(
    1ms, std::bind(&QrDetectionAndPoseEstimation::callbackProcessingCamera, this)
  );
}

QrDetectionAndPoseEstimation::~QrDetectionAndPoseEstimation()
{

}

struct QrCode {
  std::string text;
  std::array<cv::Point2i, 4> point;
};

static QrCode decode_qr_code(const cv::Mat& image, const std::string& qr_text_filter, zbar::ImageScanner& scanner)
{
  zbar::Image zbar_image(
    image.cols, image.rows, "Y800", image.data, image.cols * image.rows
  );
  scanner.scan(zbar_image);

  if (zbar_image.symbol_begin() == zbar_image.symbol_end()) {
    throw std::runtime_error("No QR code detected in image.");
  }

  QrCode qr_code;

  for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
    if (symbol->get_data().find(qr_text_filter) != std::string::npos) {
      qr_code.text = symbol->get_data();

      for (std::size_t i = 0; i < qr_code.point.size(); ++i) {
        qr_code.point[i].x = symbol->get_location_x(i);
        qr_code.point[i].y = symbol->get_location_y(i);
      }

      return qr_code;
    }
  }

  throw std::runtime_error("No expected QR code found.");
}

static void draw_polygon_on_image(cv::Mat& image, const QrCode& qr_code)
{
  cv::polylines(image, qr_code.point, true, cv::Scalar(255), 3, cv::LINE_8);
}

void QrDetectionAndPoseEstimation::callbackProcessingCamera()
{
  try {
    const auto image_frame_left = _output_queue[Camera::Left]->get<dai::ImgFrame>();
    const auto image_frame_right = _output_queue[Camera::Right]->get<dai::ImgFrame>();

    if (image_frame_left == nullptr || image_frame_right == nullptr) {
      return;
    }

    cv::Mat cv_frame_left(
      image_frame_left->getHeight(), image_frame_left->getWidth(), CV_8UC1, image_frame_left->getData().data()
    );
    cv::Mat cv_frame_right(
      image_frame_right->getHeight(), image_frame_right->getWidth(), CV_8UC1, image_frame_right->getData().data()
    );

    const auto qr_code_left = decode_qr_code(
      cv_frame_left, _parameter.qr_text_filter, *_qr_code_scanner
    );
    const auto qr_code_right = decode_qr_code(
      cv_frame_right, _parameter.qr_text_filter, *_qr_code_scanner
    );
    draw_polygon_on_image(cv_frame_left, qr_code_left);
    draw_polygon_on_image(cv_frame_right, qr_code_right);
    cv::imshow("left", cv_frame_left);
    cv::imshow("right", cv_frame_right);

    cv::waitKey(1);


  }
  catch (const std::runtime_error& err) {
    RCLCPP_ERROR_STREAM(get_logger(), err.what());
  }
}

void QrDetectionAndPoseEstimation::setupCameraPipeline(const Parameter parameter)
{
  // Define pipeline.
  _camera_pipeline = std::make_shared<dai::Pipeline>();

  // Define node 1: camera left.
  _camera[Camera::Left] = _camera_pipeline->create<dai::node::MonoCamera>();
  _camera[Camera::Left]->setBoardSocket(dai::CameraBoardSocket::LEFT);
  _camera[Camera::Left]->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  _camera[Camera::Left]->setFps(parameter.camera.fps);

  // Define node 2: camera right.
  _camera[Camera::Right] = _camera_pipeline->create<dai::node::MonoCamera>();  
  _camera[Camera::Right]->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  _camera[Camera::Right]->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  _camera[Camera::Right]->setFps(parameter.camera.fps);

  // Define node 3: output camera left.
  _camera_output[Camera::Left] = _camera_pipeline->create<dai::node::XLinkOut>();
  _camera_output[Camera::Left]->setStreamName("camera_left");
  _camera[Camera::Left]->out.link(_camera_output[Camera::Left]->input);

  // Define node 4: output camera right.
  _camera_output[Camera::Right] = _camera_pipeline->create<dai::node::XLinkOut>();
  _camera_output[Camera::Right]->setStreamName("camera_right");
  _camera[Camera::Right]->out.link(_camera_output[Camera::Right]->input);

  // Initialize device and data queues.
  _camera_device = std::make_shared<dai::Device>(*_camera_pipeline);
  _output_queue[Camera::Left] = _camera_device->getOutputQueue("camera_left", 2, true);
  _output_queue[Camera::Right] = _camera_device->getOutputQueue("camera_right", 2, true);
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
