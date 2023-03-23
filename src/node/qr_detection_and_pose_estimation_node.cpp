#include "edu_perception/stereo_inference.hpp"
#include "qr_detection_and_pose_estimation.hpp"

#include <Eigen/Geometry>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <depthai/depthai.hpp>
#include <stdexcept>
#include <zbar.h>

#include <chrono>
#include <memory>
#include <cstddef>
#include <future>

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
  ros_node.declare_parameter<std::string>("frame_id", default_parameter.frame_id);
  ros_node.declare_parameter<bool>("debugging_on", default_parameter.debugging_on);

  parameter.camera.fps = ros_node.get_parameter("camera.fps").as_double();
  parameter.camera.width = ros_node.get_parameter("camera.width").as_int();
  parameter.camera.height = ros_node.get_parameter("camera.height").as_int();
  parameter.qr_text_filter = ros_node.get_parameter("qr_text_filter").as_string();
  parameter.frame_id = ros_node.get_parameter("frame_id").as_string();
  parameter.debugging_on = ros_node.get_parameter("debugging_on").as_bool();

  return parameter;
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
  , _parameter(get_parameter(*this, Parameter()))
  , _qr_code_scanner{std::make_shared<zbar::ImageScanner>(), std::make_shared<zbar::ImageScanner>()}
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
  _qr_code_scanner[Camera::Left]->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _qr_code_scanner[Camera::Left]->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  _qr_code_scanner[Camera::Right]->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _qr_code_scanner[Camera::Right]->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  _pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("qr_code_pose", rclcpp::SensorDataQoS());
  _pub_debug_image = create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::QoS(2).reliable());

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
    return {};
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

  return qr_code;
}

// static QrCode decode_qr_code(const cv::Mat& image, const std::string& qr_text_filter, cv::QRCodeDetector& detector)
// {
//   (void)qr_text_filter;
//   QrCode qr_code;
//   std::vector<cv::Point2i> points;
//   qr_code.text = detector.detectAndDecode(image, points);
//   std::copy(points.begin(), points.end(), qr_code.point.begin());

//   for (const auto& point : qr_code.point) {
//     std::cout << "point: x = " << point.x << ", y = " << point.y << std::endl;
//   }
//   std::cout << std::endl;

//   return qr_code;
// }

static void draw_polygon_on_image(cv::Mat& image, const QrCode& qr_code)
{
  cv::polylines(image, qr_code.point, true, cv::Scalar(255), 3, cv::LINE_8);
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

  // Based on estimated coordinate axes build up pose of qr code.
  const Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(z_axis, Eigen::Vector3d::UnitZ());

  // Return result in ROS format.
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
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  try {
    auto stamp_start = get_clock()->now();
    const auto image_frame_left = _output_queue[Camera::Left]->get<dai::ImgFrame>();
    auto stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "get image frame left: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);
    stamp_start = stamp_stop;
    const auto image_frame_right = _output_queue[Camera::Right]->get<dai::ImgFrame>();
    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "get image frame right: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);

    if (image_frame_left == nullptr || image_frame_right == nullptr) {
      return;
    }

    cv::Mat cv_frame_left(
      image_frame_left->getHeight(), image_frame_left->getWidth(), CV_8UC1, image_frame_left->getData().data()
    );
    cv::Mat cv_frame_right(
      image_frame_right->getHeight(), image_frame_right->getWidth(), CV_8UC1, image_frame_right->getData().data()
    );
    stamp_start = get_clock()->now();
    // const auto qr_code_left = decode_qr_code(
    //   cv_frame_left, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Left]
    // );
    // stamp_stop = get_clock()->now();
    // RCLCPP_INFO(get_logger(), "decode left image: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);    
    // stamp_start = stamp_stop;
    // const auto qr_code_right = decode_qr_code(
    //   cv_frame_right, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Right]
    // );

    // Try with async function call to reduce execution time.
    auto future_left = std::async(
      std::launch::async, [&](){
        return decode_qr_code(
          cv_frame_left, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Left]
        );
      }
    );
    auto future_right = std::async(
      std::launch::async, [&](){
        return decode_qr_code(
          cv_frame_right, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Right]
        );
      }
    );

    const auto timeout = round<milliseconds>(duration<float>{1.0f / _parameter.camera.fps});
    do {
      std::this_thread::sleep_for(timeout / 10);

      if (future_left.wait_for(0ms) == std::future_status::ready
          && future_right.wait_for(0ms) == std::future_status::ready) {
        break;
      }
      if ((get_clock()->now() - stamp_start) >= timeout) {
        throw std::runtime_error("No QR code detected in time.");
      }
    } while (true);

    const auto qr_code_left = future_left.get();
    const auto qr_code_right = future_right.get();

    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "decode images: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);    
    // const auto qr_code_left = decode_qr_code(
    //   cv_frame_left, _parameter.qr_text_filter, *_qr_code_detector
    // );
    // const auto qr_code_right = decode_qr_code(
    //   cv_frame_right, _parameter.qr_text_filter, *_qr_code_detector
    // );
    // Providing a image for debugging if someone has subscripted to this topic.
    if (_pub_debug_image->get_subscription_count() > 0) {
      if (qr_code_left.text != "") {
        draw_polygon_on_image(cv_frame_left, qr_code_left);
      }
      if (qr_code_right.text != "") {
        draw_polygon_on_image(cv_frame_right, qr_code_right);
      }

      cv::Mat debug_output;
      cv::hconcat(cv_frame_left, cv_frame_right, debug_output);
      
      std_msgs::msg::Header header;
      header.frame_id = getFrameIdPrefix() + _parameter.frame_id;
      header.stamp = get_clock()->now();

      _pub_debug_image->publish(*cv_bridge::CvImage(header, "mono8", debug_output).toImageMsg());
    }
    // If the QR code is not found in both cameras cancel processing.
    if (qr_code_left.text == "" || qr_code_right.text == "") {
      return;
    }

    stamp_start = get_clock()->now();    
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = getFrameIdPrefix() + _parameter.frame_id;
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.pose = estimate_pose_of_qr_code(
      *_stereo_inference, qr_code_left, qr_code_right
    );
    _pub_pose->publish(pose_msg);
    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "estimate pose and publishing it: %ld us\n\n", (stamp_stop - stamp_start).nanoseconds() / 1000);

    if (_parameter.debugging_on == true) {
      draw_polygon_on_image(cv_frame_left, qr_code_left);
      draw_polygon_on_image(cv_frame_right, qr_code_right);
      cv::imshow("left", cv_frame_left);
      cv::imshow("right", cv_frame_right);

      cv::waitKey(1);
    }
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
  _camera[Camera::Left]->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
  _camera[Camera::Left]->initialControl.setAutoExposureEnable();
  _camera[Camera::Left]->initialControl.setSceneMode(dai::CameraControl::SceneMode::BARCODE);

  // Define node 2: camera right.
  _camera[Camera::Right] = _camera_pipeline->create<dai::node::MonoCamera>();  
  _camera[Camera::Right]->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  _camera[Camera::Right]->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
  _camera[Camera::Right]->setFps(parameter.camera.fps);
  _camera[Camera::Right]->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
  _camera[Camera::Right]->initialControl.setAutoExposureEnable();
  _camera[Camera::Right]->initialControl.setSceneMode(dai::CameraControl::SceneMode::BARCODE);

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

std::string QrDetectionAndPoseEstimation::getFrameIdPrefix() const
{
  // remove slash at the beginning
  std::string frame_id_prefix(get_effective_namespace().begin() + 1, get_effective_namespace().end());
  // add slash at the end if it is missing
  if (frame_id_prefix.back() != '/') {
    frame_id_prefix.push_back('/');
  }
  return frame_id_prefix;
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
