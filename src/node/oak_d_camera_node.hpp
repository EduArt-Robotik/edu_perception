/**
 * Copyright EduArt Robotik GmbH 2024
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_perception/h264_decoder.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <regex>

// Forward Declarations
namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;

namespace node {
class Camera;
class ColorCamera;
class XLinkOut;
class VideoEncoder;
} // end namespace node
} // end namespace dai

namespace eduart {
namespace perception {

/**
 * \brief Captures images from OAK-D camera and publishes it. At the moment only the color camera is captured.
 * \todo make camera selection configurable.
 */
class OakDCamera : public rclcpp::Node
{
public:
  struct Parameter {
    float fps = 10.0f;
    std::size_t width = 1920;
    std::size_t height = 1080;
    std::string device = "192.168.3.110";
    std::string frame_id = "oak_d";

    inline bool isEthernet() const {
      const std::regex ipv4(
        "(([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\.){3}([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])");
      return std::regex_match(device, ipv4);
    }
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  OakDCamera();
  ~OakDCamera() override;

private:
  void callbackProcessingCamera();
  void setupCameraPipeline(const Parameter parameter);
  void publishCameraInfo(std::shared_ptr<dai::Device> camera_device, const Parameter& parameter);
  std::string getFrameIdPrefix() const;

  const Parameter _parameter;

  std::shared_ptr<dai::node::Camera> _camera;
  std::shared_ptr<dai::node::VideoEncoder> _video_encoder;
  std::shared_ptr<dai::node::XLinkOut> _camera_output;
  std::shared_ptr<dai::DataOutputQueue> _camera_output_queue;
  std::shared_ptr<dai::Pipeline> _camera_pipeline;
  std::shared_ptr<dai::Device> _camera_device;

  std::shared_ptr<rclcpp::TimerBase> _timer_processing_camera;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub_image;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> _pub_camera_info;

  H264Decoder _video_decoder;
};

} // end namespace perception
} // end namespace eduart
