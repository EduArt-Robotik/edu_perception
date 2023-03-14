/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <array>

// Forward Declarations
namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;

namespace node {
class MonoCamera;
class XLinkOut;
} // end namespace node
} // end namespace dai

namespace zbar {
class ImageScanner;
} // end namespace zbar

namespace eduart {
namespace perception {

class QrDetectionAndPoseEstimation : public rclcpp::Node
{
public:
  struct Parameter {
    struct {
      float fps = 5.0f;
    } camera;
    std::string qr_text_filter = "Eduard";
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  QrDetectionAndPoseEstimation();
  ~QrDetectionAndPoseEstimation() override;

private:
  void callbackProcessingCamera();
  void setupCameraPipeline(const Parameter parameter);

  struct Camera {
    enum {
      Left = 0,
      Right,
      Count
    };
  };

  Parameter _parameter;
  std::shared_ptr<dai::Pipeline> _camera_pipeline;
  std::shared_ptr<dai::Device> _camera_device;

  // \todo check if it is necessary to keep depthai objects alive, because it seems the pipeline keeps also
  //       a copy of the shared pointer.
  std::array<std::shared_ptr<dai::node::MonoCamera>, Camera::Count> _camera;
  std::array<std::shared_ptr<dai::node::XLinkOut>, Camera::Count> _camera_output;
  std::array<std::shared_ptr<dai::DataOutputQueue>, Camera::Count> _output_queue;

  std::shared_ptr<zbar::ImageScanner> _qr_code_scanner;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
  std::shared_ptr<rclcpp::TimerBase> _timer_processing_camera;
};

} // end namespace perception
} // end namespace eduart
