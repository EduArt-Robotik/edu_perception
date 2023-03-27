#include "qr_detection_and_pose_estimation.hpp"

#include <edu_perception/stereo/pose_estimation.hpp>
#include <edu_perception/transform.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <depthai/depthai.hpp>
#include <stdexcept>

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
  ros_node.declare_parameter<std::string>(
    "qr_code_detector.qr_code_text_filter", default_parameter.qr_code_detector.qr_code_text_filter);
  ros_node.declare_parameter<float>(
    "qr_code_detector.roi_increase_rate.horizontal",
    default_parameter.qr_code_detector.roi_increase_rate.horizontal);
  ros_node.declare_parameter<float>(
    "qr_code_detector.roi_increase_rate.vertical",
    default_parameter.qr_code_detector.roi_increase_rate.vertical);  
  ros_node.declare_parameter<std::string>("frame_id", default_parameter.frame_id);
  ros_node.declare_parameter<std::string>(
    "frame_id_object_origin", default_parameter.frame_id_object_origin);

  parameter.camera.fps = ros_node.get_parameter("camera.fps").as_double();
  parameter.camera.width = ros_node.get_parameter("camera.width").as_int();
  parameter.camera.height = ros_node.get_parameter("camera.height").as_int();
  parameter.qr_code_detector.qr_code_text_filter = ros_node.get_parameter(
    "qr_code_detector.qr_code_text_filter").as_string();
  parameter.qr_code_detector.roi_increase_rate.horizontal = ros_node.get_parameter(
    "qr_code_detector.roi_increase_rate.horizontal").as_double();
  parameter.qr_code_detector.roi_increase_rate.vertical = ros_node.get_parameter(
    "qr_code_detector.roi_increase_rate.vertical").as_double();
  parameter.frame_id = ros_node.get_parameter("frame_id").as_string();
  parameter.frame_id_object_origin = ros_node.get_parameter("frame_id_object_origin").as_string();

  return parameter;
}

static void draw_polygon_on_image(cv::Mat& image, const QrCode& qr_code)
{
  cv::polylines(image, qr_code.point, true, cv::Scalar(255), 3, cv::LINE_8);
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
  , _parameter(get_parameter(*this, Parameter()))
  , _qr_code_detector{
      std::make_shared<detector::QrCodeDetector>(_parameter.qr_code_detector),
      std::make_shared<detector::QrCodeDetector>(_parameter.qr_code_detector)
    }
  , _filter_orientation(_parameter.filter, decltype(geometry_msgs::msg::Pose::orientation){ })
  , _tf_buffer(std::make_unique<tf2_ros::Buffer>(get_clock()))
  , _tf_listener(std::make_shared<tf2_ros::TransformListener>(*_tf_buffer))
{
  setupCameraPipeline(_parameter);
  const stereo::StereoInference::Parameter stereo_inference_parameter{
    _parameter.camera.width,
    _parameter.camera.height,
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionWidth()),
    static_cast<std::size_t>(_camera[Camera::Left]->getResolutionHeight()) 
  };
  _stereo_inference = std::make_unique<stereo::StereoInference>(_camera_device, stereo_inference_parameter);
  _filter_orientation.clear();

  _pub_pose = create_publisher<geometry_msgs::msg::PoseStamped>("qr_code_pose", rclcpp::SensorDataQoS());
  _pub_debug_image = create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::QoS(2).reliable());

  // run with 1ms interval to try getting faster than camera delivers (queues configured to block...)
  _timer_processing_camera = create_wall_timer(
    1ms, std::bind(&QrDetectionAndPoseEstimation::callbackProcessingCamera, this)
  );
}

QrDetectionAndPoseEstimation::~QrDetectionAndPoseEstimation()
{

}

void QrDetectionAndPoseEstimation::callbackProcessingCamera()
{
  RCLCPP_INFO(get_logger(), __PRETTY_FUNCTION__);
  try {
    // Getting images from the camera pipeline.
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

    stamp_start = get_clock()->now();

    // Detect and decode QR codes in the received images.
    // Try with async function call to reduce execution time.
    auto future_left = std::async(
      std::launch::async, [&](){
        return _qr_code_detector[Camera::Left]->detect(cv_frame_left);
      }
    );
    auto future_right = std::async(
      std::launch::async, [&](){
        return _qr_code_detector[Camera::Right]->detect(cv_frame_right);
      }
    );

    const auto qr_code_left = future_left.get();
    const auto qr_code_right = future_right.get();

    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "decode images: %ld us", (stamp_stop - stamp_start).nanoseconds() / 1000);    

    // Providing debug images if it is requested.
    if (_pub_debug_image->get_subscription_count() > 0) {
      if (qr_code_left.text != "") {
        draw_polygon_on_image(cv_frame_left, qr_code_left);
      }
      if (qr_code_right.text != "") {
        draw_polygon_on_image(cv_frame_right, qr_code_right);
      }
      cv::rectangle(
        cv_frame_left, _qr_code_detector[Camera::Left]->getCurrentRoi(), cv::Scalar(255), 3
      );
      cv::rectangle(
        cv_frame_right, _qr_code_detector[Camera::Right]->getCurrentRoi(), cv::Scalar(255), 3
      );
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
    // Estimate pose from the detected QR codes.
    geometry_msgs::msg::PoseStamped pose_msg;

    pose_msg.header.frame_id = getFrameIdPrefix() + _parameter.frame_id;
    pose_msg.header.stamp = get_clock()->now();
    pose_msg.pose = stereo::estimate_pose_of_qr_code(
      *_stereo_inference, qr_code_left, qr_code_right
    );

    _filter_orientation(pose_msg.pose.orientation);
    pose_msg.pose.orientation = _filter_orientation.channel<0>().getValue();


    // Check if QR pose can be transformed in the given frame id.
    geometry_msgs::msg::TransformStamped t_qr_code_to_base_link;
    try {
      t_qr_code_to_base_link = _tf_buffer->lookupTransform(
        _parameter.frame_id_object_origin, "eduard/red/qr_code/rear", tf2::TimePointZero
      );
      // Transform is available. Transform pose in given frame id.
      pose_msg.pose = transform_pose(pose_msg.pose, t_qr_code_to_base_link.transform);
    }
    catch (const tf2::TransformException & ex) {
      // No transform available. Skip the pose transform.
    }

    _pub_pose->publish(pose_msg);
    stamp_stop = get_clock()->now();
    RCLCPP_INFO(get_logger(), "estimate pose and publishing it: %ld us\n\n", (stamp_stop - stamp_start).nanoseconds() / 1000);
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
