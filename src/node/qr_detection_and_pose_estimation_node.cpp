#include "edu_perception/stereo_inference.hpp"
#include "qr_detection_and_pose_estimation.hpp"

#include <Eigen/Geometry>

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
#include <cstddef>

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
  ros_node.declare_parameter<int>("camera.width", default_parameter.camera.width);
  ros_node.declare_parameter<int>("camera.height", default_parameter.camera.height);
  ros_node.declare_parameter<std::string>("qr_text_filter", default_parameter.qr_text_filter);
  ros_node.declare_parameter<bool>("debugging_on", default_parameter.debugging_on);

  parameter.camera.fps = ros_node.get_parameter("camera.fps").as_double();
  parameter.camera.width = ros_node.get_parameter("camera.width").as_int();
  parameter.camera.height = ros_node.get_parameter("camera.height").as_int();
  parameter.qr_text_filter = ros_node.get_parameter("qr_text_filter").as_string();
  parameter.debugging_on = ros_node.get_parameter("debugging_on").as_bool();

  return parameter;
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
  , _parameter(get_parameter(*this, Parameter()))
  , _qr_code_scanner(std::make_shared<zbar::ImageScanner>())
  , _qr_code_detector(std::make_shared<cv::QRCodeDetector>())
{
  setupCameraPipeline(_parameter);
  const StereoInference::Parameter stereo_inference_parameter{
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionWidth()),
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionHeight()),
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionWidth()),
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionHeight()) 
  };
  _stereo_inference = std::make_unique<StereoInference>(_camera_device, stereo_inference_parameter);
  _qr_code_scanner->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _qr_code_scanner->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  _pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("qr_code_pose", rclcpp::SensorDataQoS());

  // const auto timer_period = round<milliseconds>(duration<float>{1.0f / _parameter.camera.fps});
  // run with 1ms interval to try getting faster than camera delivers (queues configured to block...)
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

static QrCode decode_qr_code(const cv::Mat& image, const std::string& qr_text_filter, cv::QRCodeDetector& detector)
{
  (void)qr_text_filter;
  QrCode qr_code;
  std::vector<cv::Point2i> points;
  qr_code.text = detector.detectAndDecode(image, points);
  std::copy(points.begin(), points.end(), qr_code.point.begin());

  for (const auto& point : qr_code.point) {
    std::cout << "point: x = " << point.x << ", y = " << point.y << std::endl;
  }
  std::cout << std::endl;

  return qr_code;
}

static void draw_polygon_on_image(cv::Mat& image, const QrCode& qr_code)
{
  cv::polylines(image, qr_code.point, true, cv::Scalar(255), 3, cv::LINE_8);
  std::cout << "Qr Code: " << qr_code.text << std::endl;
  for (const auto& point : qr_code.point) {
    std::cout << "point: x = " << point.x << ", y = " << point.y << std::endl;
  }
  std::cout << std::endl;
}

static geometry_msgs::msg::Pose estimate_pose_of_qr_code(
  const StereoInference& inference, const QrCode& left_qr_code, const QrCode& right_qr_code)
{
  // Matching points are: point[0] --> point[1] == x axis
  //                      point[0] --> point[3] == y axis
  const Eigen::Vector3d origin = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[0].x, left_qr_code.point[0].y),
    Eigen::Vector2i(right_qr_code.point[0].x, right_qr_code.point[0].y)
  );
  const Eigen::Vector3d end_point_x_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[1].x, left_qr_code.point[1].y),
    Eigen::Vector2i(right_qr_code.point[1].x, right_qr_code.point[1].y)
  );
  const Eigen::Vector3d end_point_y_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[3].x, left_qr_code.point[3].y),
    Eigen::Vector2i(right_qr_code.point[3].x, right_qr_code.point[3].y)
  );
  const Eigen::Vector3d middle_point = (end_point_x_axis + end_point_y_axis) * 0.5;
  const Eigen::Vector3d x_axis = end_point_x_axis - origin;
  const Eigen::Vector3d y_axis = end_point_y_axis - origin;
  const Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  std::cout << "x_axis:\n" << x_axis << std::endl;
  std::cout << "y_axis:\n" << y_axis << std::endl;
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(z_axis, Eigen::Vector3d::UnitZ());

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

    std::cout << "Es beginnt..." << std::endl;
    const auto qr_code_left = decode_qr_code(
      cv_frame_left, _parameter.qr_text_filter, *_qr_code_scanner
    );
    const auto qr_code_right = decode_qr_code(
      cv_frame_right, _parameter.qr_text_filter, *_qr_code_scanner
    );
    // const auto qr_code_left = decode_qr_code(
    //   cv_frame_left, _parameter.qr_text_filter, *_qr_code_detector
    // );
    // const auto qr_code_right = decode_qr_code(
    //   cv_frame_right, _parameter.qr_text_filter, *_qr_code_detector
    // );
    if (qr_code_left.text == "" || qr_code_right.text == "") {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = _parameter.frame_id;
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.pose = estimate_pose_of_qr_code(
      *_stereo_inference, qr_code_left, qr_code_right
    );
    _pub_pose->publish(pose_msg);

    if (_parameter.debugging_on == true) {
      draw_polygon_on_image(cv_frame_left, qr_code_left);
      draw_polygon_on_image(cv_frame_right, qr_code_right);
      cv::imshow("left", cv_frame_left);
      cv::imshow("right", cv_frame_right);

      cv::waitKey(1);
    }
    std::cout << std::endl;
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

  // Define node 3: cropping left camera image.
  const std::size_t x_border = (_camera[Camera::Left]->getResolutionWidth() - parameter.camera.width) / 2;
  const std::size_t y_border = (_camera[Camera::Left]->getResolutionHeight() - parameter.camera.height) / 2;
  const float x_min = x_border / static_cast<float>(_camera[Camera::Left]->getResolutionWidth());
  const float y_min = y_border / static_cast<float>(_camera[Camera::Left]->getResolutionHeight());
  const float x_max = (x_border + _camera[Camera::Left]->getResolutionWidth())
    / static_cast<float>(_camera[Camera::Left]->getResolutionWidth());
  const float y_max = (y_border + _camera[Camera::Left]->getResolutionHeight())
    / static_cast<float>(_camera[Camera::Left]->getResolutionHeight());
  _image_manip[Camera::Left] = _camera_pipeline->create<dai::node::ImageManip>();
  _image_manip[Camera::Left]->initialConfig.setCropRect(x_min, y_min, x_max, y_max);
  _camera[Camera::Left]->out.link(_image_manip[Camera::Left]->inputImage);

  // Define node 4: cropping right camera image.
  _image_manip[Camera::Right] = _camera_pipeline->create<dai::node::ImageManip>();
  _image_manip[Camera::Right]->initialConfig.setCropRect(x_min, y_min, x_max, y_max);
  _camera[Camera::Right]->out.link(_image_manip[Camera::Right]->inputImage);

  // Define node 5: output camera left.
  _camera_output[Camera::Left] = _camera_pipeline->create<dai::node::XLinkOut>();
  _camera_output[Camera::Left]->setStreamName("camera_left");
  _image_manip[Camera::Left]->out.link(_camera_output[Camera::Left]->input);

  // Define node 6: output camera right.
  _camera_output[Camera::Right] = _camera_pipeline->create<dai::node::XLinkOut>();
  _camera_output[Camera::Right]->setStreamName("camera_right");
  _image_manip[Camera::Right]->out.link(_camera_output[Camera::Right]->input);

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
