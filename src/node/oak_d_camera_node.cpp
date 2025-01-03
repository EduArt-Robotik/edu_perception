#include "oak_d_camera_node.hpp"
#include "depthai/pipeline/node/Camera.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include <depthai/depthai.hpp>

#include <opencv2/core/mat.hpp>

#include <rclcpp/executors.hpp>

namespace eduart {
namespace perception {

using namespace std::chrono_literals;

OakDCamera::Parameter OakDCamera::get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter)
{
  Parameter parameter;

  ros_node.declare_parameter<float>("fps", default_parameter.fps);
  ros_node.declare_parameter<std::uint8_t>("manual_focus", default_parameter.manual_focus);
  ros_node.declare_parameter<int>("width", default_parameter.width);
  ros_node.declare_parameter<int>("height", default_parameter.height);
  ros_node.declare_parameter<std::string>("device", default_parameter.device);
  ros_node.declare_parameter<std::string>("frame_id", default_parameter.frame_id);

  parameter.fps = ros_node.get_parameter("fps").as_double();
  parameter.manual_focus = ros_node.get_parameter("manual_focus").as_int();
  parameter.width = ros_node.get_parameter("width").as_int();
  parameter.height = ros_node.get_parameter("height").as_int();
  parameter.device = ros_node.get_parameter("device").as_string();
  parameter.frame_id = ros_node.get_parameter("frame_id").as_string();

  return parameter;
}

OakDCamera::OakDCamera()
  : rclcpp::Node("oak_d_camera")
  , _parameter(get_parameter(*this, _parameter))
{
  setupCameraPipeline(_parameter);

  // Publisher
  _pub_image = create_publisher<sensor_msgs::msg::Image>(
    "oak_d_camera/image_raw", rclcpp::QoS(2).reliable().durability_volatile()
  );
  _pub_camera_info = create_publisher<sensor_msgs::msg::CameraInfo>(
    "oak_d_camera/camera_info", rclcpp::QoS(2).reliable().transient_local()
  );
  publishCameraInfo(_camera_device, _parameter);

  // run with 1ms interval to try getting faster than camera delivers (queues configured to block...)
  _timer_processing_camera = create_wall_timer(
    1ms, std::bind(&OakDCamera::callbackProcessingCamera, this)
  );
}

OakDCamera::~OakDCamera()
{

}

void OakDCamera::callbackProcessingCamera()
{
  // Requesting Image
  const auto packet = _camera_output_queue->get<dai::ImgFrame>();

  if (packet == nullptr) {
    RCLCPP_ERROR(get_logger(), "No image received! Cancel processing camera.");
    return;
  }

  cv::Mat cv_image = _video_decoder(packet->getData().data(), packet->getData().size());
  // cv::Mat cv_image(image->getHeight(), image->getWidth(), CV_8UC1, image->getData().data());

  if (cv_image.empty()) {
    // do not publish empty image
    return;
  }

  // Publishing Image
  std_msgs::msg::Header header;
  header.frame_id = getFrameIdPrefix() + _parameter.frame_id;
  header.stamp = get_clock()->now();
  const auto msg = cv_bridge::CvImage(header, "mono8", cv_image).toImageMsg();
  _pub_image->publish(*msg);
}

void OakDCamera::setupCameraPipeline(const Parameter parameter)
{
  // Define pipeline.
  _camera_pipeline = std::make_shared<dai::Pipeline>();

  // Define Camera Node
  _camera = _camera_pipeline->create<dai::node::Camera>();
  _camera->setVideoSize(parameter.width, parameter.height);
  _camera->setFps(parameter.fps);
  _camera->setBoardSocket(dai::CameraBoardSocket::CENTER);
  _camera->initialControl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::OFF);
  _camera->initialControl.setManualFocus(_parameter.manual_focus);
  _camera->initialControl.setAutoExposureEnable();

  // Define Video Encoder
  _video_encoder = _camera_pipeline->create<dai::node::VideoEncoder>();
  _video_encoder->setDefaultProfilePreset(parameter.fps, dai::VideoEncoderProperties::Profile::H264_MAIN);
  _camera->video.link(_video_encoder->input);

  // Define Camera Output
  _camera_output = _camera_pipeline->create<dai::node::XLinkOut>();
  _camera_output->setStreamName("camera_video");
  _video_encoder->bitstream.link(_camera_output->input);

  // Initialize device and data queues.
  if (_parameter.isEthernet()) {
    RCLCPP_INFO(get_logger(), "try to connect camera using ethernet net (ip = %s)", _parameter.device.c_str());
    const auto device_info = dai::DeviceInfo(_parameter.device);
    _camera_device = std::make_shared<dai::Device>(*_camera_pipeline, device_info);    
  }
  else {
    RCLCPP_INFO(get_logger(), "try to connect camera using USB port.");
    _camera_device = std::make_shared<dai::Device>(*_camera_pipeline);
  }

  _camera_output_queue = _camera_device->getOutputQueue("camera_video", 2, true);
}

void OakDCamera::publishCameraInfo(std::shared_ptr<dai::Device> camera_device, const Parameter& parameter)
{
  const auto calibration_data = camera_device->readCalibration();
  const auto intrinsic = calibration_data.getCameraIntrinsics(
    dai::CameraBoardSocket::CENTER, parameter.width, parameter.height);
  const auto distortion = calibration_data.getDistortionCoefficients(
    dai::CameraBoardSocket::CENTER);
  // const auto distortion_model = calibration_data.getDistortionModel(
  //   dai::CameraBoardSocket::CENTER);

  // Fill camera info with calibration data
  sensor_msgs::msg::CameraInfo camera_info;

  camera_info.header.frame_id = _parameter.frame_id;
  camera_info.header.stamp = get_clock()->now();

  camera_info.height = parameter.height;
  camera_info.width = parameter.width;

  camera_info.distortion_model = "none";
  camera_info.d.resize(distortion.size());

  for (std::size_t i = 0; i < camera_info.d.size(); ++i) {
    camera_info.d[i] = distortion[i];
  }

  for (std::size_t i = 0; i < camera_info.k.size(); ++i) {
    camera_info.k[i] = intrinsic[i / 3][i % 3];
  }

  _pub_camera_info->publish(camera_info);
}

std::string OakDCamera::getFrameIdPrefix() const
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
  rclcpp::spin(std::make_shared<eduart::perception::OakDCamera>());
  rclcpp::shutdown();

  return 0;
}