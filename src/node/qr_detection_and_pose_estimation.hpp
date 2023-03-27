/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <edu_perception/detector/qr_code_detector.hpp>
#include <edu_perception/stereo/stereo_inference.hpp>
#include <edu_perception/low_pass_filter.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
    detector::QrCodeDetector::Parameter qr_code_detector = {
      { 0.2f, 0.4f }, "Eduard"
    };
    LowPassFilter<decltype(geometry_msgs::msg::Pose::orientation)>::Parameter filter = {
      { 1.0f }
    };
    std::string frame_id = "qr_code_camera"; // frame id of the used camera
    std::string frame_id_object_origin = "eduard/red/base_link"; // origin of the object qr code is mounted on
  };

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

  QrDetectionAndPoseEstimation();
  ~QrDetectionAndPoseEstimation() override;

private:
  void callbackProcessingCamera();
  void setupCameraPipeline(const Parameter parameter);
  std::string getFrameIdPrefix() const;

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
  std::array<std::shared_ptr<dai::node::ImageManip>, Camera::Count> _image_manip;
  std::array<std::shared_ptr<dai::node::XLinkOut>, Camera::Count> _camera_output;
  std::array<std::shared_ptr<dai::DataOutputQueue>, Camera::Count> _output_queue;

  std::array<std::shared_ptr<detector::QrCodeDetector>, Camera::Count> _qr_code_detector;
  std::unique_ptr<stereo::StereoInference> _stereo_inference;
  LowPassFilter<decltype(geometry_msgs::msg::Pose::orientation)> _filter_orientation;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> _pub_pose;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub_debug_image;
  std::shared_ptr<rclcpp::TimerBase> _timer_processing_camera;
  std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
};

} // end namespace perception
} // end namespace eduart
