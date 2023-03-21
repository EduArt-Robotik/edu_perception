/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_perception/stereo_inference.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>

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
class ImageManip;
} // end namespace node
} // end namespace dai

namespace zbar {
class ImageScanner;
} // end namespace zbar

namespace cv {
class QRCodeDetector;
}

namespace eduart {
namespace perception {

class QrDetectionAndPoseEstimation : public rclcpp::Node
{
public:
  struct Parameter {
    struct {
      float fps = 5.0f;
      std::size_t width = 1280;
      std::size_t height = 800;
    } camera;
    std::string qr_text_filter = "Eduard";
    std::string frame_id = "qr_code_camera";
    bool debugging_on = false;
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
  std::unique_ptr<StereoInference> _stereo_inference;

  // \todo check if it is necessary to keep depthai objects alive, because it seems the pipeline keeps also
  //       a copy of the shared pointer.
  std::array<std::shared_ptr<dai::node::MonoCamera>, Camera::Count> _camera;
  std::array<std::shared_ptr<dai::node::ImageManip>, Camera::Count> _image_manip;
  std::array<std::shared_ptr<dai::node::XLinkOut>, Camera::Count> _camera_output;
  std::array<std::shared_ptr<dai::DataOutputQueue>, Camera::Count> _output_queue;

  std::shared_ptr<zbar::ImageScanner> _qr_code_scanner;
  std::shared_ptr<cv::QRCodeDetector> _qr_code_detector;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub_debug_image;
  std::shared_ptr<rclcpp::TimerBase> _timer_processing_camera;
};

} // end namespace perception
} // end namespace eduart
