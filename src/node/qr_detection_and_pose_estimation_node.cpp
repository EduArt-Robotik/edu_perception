#include "edu_perception/stereo_inference.hpp"
#include "qr_detection_and_pose_estimation.hpp"

#include <Eigen/Geometry>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

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
    // static_cast<std::size_t>(_camera[Camera::Left]->getResolutionWidth()),
    // static_cast<std::size_t>(_camera[Camera::Left]->getResolutionHeight()),
    _parameter.camera.width,
    _parameter.camera.height,
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionWidth()),
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionHeight()) 
  };
  _stereo_inference = std::make_unique<StereoInference>(_camera_device, stereo_inference_parameter);

  _qr_code_scanner[Camera::Left]->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _qr_code_scanner[Camera::Left]->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  // _qr_code_scanner[Camera::Left]->enable_cache();
  _qr_code_scanner[Camera::Right]->set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  _qr_code_scanner[Camera::Right]->set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  // _qr_code_scanner[Camera::Right]->enable_cache();

  _camera_roi[Camera::Left] = cv::Rect(0, 0, _parameter.camera.width, _parameter.camera.height);
  _camera_roi[Camera::Right] = cv::Rect(0, 0, _parameter.camera.width, _parameter.camera.height);

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

static cv::Rect estimateFutureRoi(const QrCode& qr_code, const cv::Size image_size, const float size_increase_factor)
{
  const cv::Rect future_roi = cv::boundingRect(qr_code.point);
  std::cout << "Found ROI: " << future_roi << std::endl;
  const cv::Point top_left_corner(
    std::max(future_roi.x - static_cast<int>(future_roi.width  * (size_increase_factor - 1.0f)), 0),
    std::max(future_roi.y - static_cast<int>(future_roi.height * (size_increase_factor - 1.0f)), 0)
  );
  const cv::Point bottom_right_corner(
    std::min(future_roi.br().x + static_cast<int>(future_roi.width  * (size_increase_factor - 1.0f)), image_size.width  - 1),
    std::min(future_roi.br().y + static_cast<int>(future_roi.height * (size_increase_factor - 1.0f)), image_size.height - 1)
  );
  const cv::Rect future_roi_increased(top_left_corner, bottom_right_corner);
  std::cout << "Increased ROI: " << future_roi_increased << std::endl;
  return future_roi_increased;
}

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

static QrCode decode_qr_code(
  const cv::Mat& image, const std::string& qr_text_filter, zbar::ImageScanner& scanner, cv::Rect& camera_roi)
{
  zbar::Image zbar_image(
    image.cols, image.rows, "Y800", image.data, image.cols * image.rows
  );
  std::cout << "Setting crop: " << camera_roi << std::endl;
  zbar_image.set_crop(camera_roi.x, camera_roi.y, camera_roi.width, camera_roi.height);
  scanner.scan(zbar_image);

  if (zbar_image.symbol_begin() == zbar_image.symbol_end()) {
    camera_roi = cv::Rect(0, 0, image.size().width, image.size().height);    
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

      camera_roi = estimateFutureRoi(qr_code, image.size(), 1.2f);
      sort_qr_code_points(qr_code.point);
      return qr_code;
    }
  }

  camera_roi = cv::Rect(0, 0, image.size().width, image.size().height);
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
  std::cout << "qr code left points:\n";
  for (const auto& point : left_qr_code.point) {
    std::cout << "point: " << point << std::endl;
  }
  std::cout << "qr code right points:\n";
  for (const auto& point : right_qr_code.point) {
    std::cout << "point: " << point << std::endl;
  }  
  // Matching points are: point[0] --> point[1] == x axis
  //                      point[0] --> point[2] == y axis
  const Eigen::Vector3f origin = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[0].x, left_qr_code.point[0].y),
    Eigen::Vector2i(right_qr_code.point[0].x, right_qr_code.point[0].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f end_point_x_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[1].x, left_qr_code.point[1].y),
    Eigen::Vector2i(right_qr_code.point[1].x, right_qr_code.point[1].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f end_point_y_axis = inference.estimateSpatial(
    Eigen::Vector2i(left_qr_code.point[2].x, left_qr_code.point[2].y),
    Eigen::Vector2i(right_qr_code.point[2].x, right_qr_code.point[2].y)
  ).cwiseProduct(
    Eigen::Vector3f(1.0f, -1.0f, 1.0f)
  );
  const Eigen::Vector3f middle_point = (end_point_x_axis + end_point_y_axis) * 0.5;
  std::cout << "origin:\n" << origin << std::endl;
  std::cout << "end_point_x_axis:\n" << end_point_x_axis << std::endl;
  std::cout << "end_point_y_axis:\n" << end_point_y_axis << std::endl;
  // Based on estimated coordinate axes build up pose of qr code.
  const Eigen::Vector3f x_axis = end_point_x_axis - origin;
  const Eigen::Vector3f y_axis = end_point_y_axis - origin;
  const Eigen::Vector3f z_axis = x_axis.cross(y_axis);
  const Eigen::DiagonalMatrix<float, 3> camera_coordinate_sysytem(1.0f, 1.0f, 1.0f);
  const auto camera_coordinate_sysytem_inv = camera_coordinate_sysytem.inverse();

  Eigen::Matrix3f qr_code_coordinate_system;
  qr_code_coordinate_system.col(0) = x_axis.normalized();
  qr_code_coordinate_system.col(1) = y_axis.normalized();
  qr_code_coordinate_system.col(2) = z_axis.normalized();
  const Eigen::Matrix3f R = qr_code_coordinate_system * camera_coordinate_sysytem_inv;
  std::cout << "R:\n" << R << std::endl;
  const Angle angle_x = std::acos(y_axis.y() / y_axis.norm());
  const Angle angle_y = std::acos(z_axis.z() / z_axis.norm());
  const Angle angle_z = std::acos(x_axis.x() / x_axis.norm());
  std::cout << "x axis:\n" << x_axis << std::endl;
  std::cout << "y axis:\n" << y_axis << std::endl;  
  std::cout << "angle x = " << angle_x << std::endl;
  std::cout << "angle y = " << angle_y << std::endl;
  std::cout << "angle z = " << angle_z << std::endl;    
  // const Eigen::Quaterniond orientation = Eigen::Quaterniond::FromTwoVectors(z_axis, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf orientation(R);
  // orientation = Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX())
  //   * Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY())
  //   * Eigen::AngleAxisf(angle_z, Eigen::Vector3f::UnitZ());

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
    RCLCPP_INFO(get_logger(), "got image frame left: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);

    stamp_start = stamp_stop;
    const auto image_frame_right = _output_queue[Camera::Right]->get<dai::ImgFrame>();
    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "got image frame right: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);

    if (image_frame_left == nullptr || image_frame_right == nullptr) {
      return;
    }

    cv::Mat cv_frame_left(
      image_frame_left->getHeight(), image_frame_left->getWidth(), CV_8UC1, image_frame_left->getData().data()
    );
    cv::Mat cv_frame_right(
      image_frame_right->getHeight(), image_frame_right->getWidth(), CV_8UC1, image_frame_right->getData().data()
    );
    std::cout << "Image size left: " << cv_frame_left.size() << std::endl;
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
          cv_frame_left, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Left], _camera_roi[Camera::Left]
        );
      }
    );
    auto future_right = std::async(
      std::launch::async, [&](){
        return decode_qr_code(
          cv_frame_right, _parameter.qr_text_filter, *_qr_code_scanner[Camera::Right], _camera_roi[Camera::Right]
        );
      }
    );

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
      cv::rectangle(cv_frame_left, _camera_roi[Camera::Left], cv::Scalar(255), 3);
      cv::rectangle(cv_frame_right, _camera_roi[Camera::Right], cv::Scalar(255), 3);
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
  const float x_max = (_camera[Camera::Left]->getResolutionWidth() - x_border)
    / static_cast<float>(_camera[Camera::Left]->getResolutionWidth());
  const float y_max = (_camera[Camera::Left]->getResolutionHeight() - y_border)
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
