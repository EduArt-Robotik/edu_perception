#include "apriltag_draw_node.hpp"

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>

#include <cv_bridge/cv_bridge.h>

#include <functional>

namespace eduart {
namespace perception {

AprilTagDraw::Parameter AprilTagDraw::get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter)
{
  (void)ros_node;
  (void)default_parameter;
  Parameter parameter;

  return parameter;
}

AprilTagDraw::AprilTagDraw()
  : rclcpp::Node("apriltag_draw")
  , _parameter(get_parameter(*this, _parameter))
{
  // subscriptions including time synchronizer
  const auto rmw_qos_profile = rclcpp::QoS(5).best_effort().get_rmw_qos_profile();
  _sub_image.subscribe(this, "image_raw", rmw_qos_profile);
  _sub_apriltag_detection.subscribe(this, "detection", rmw_qos_profile);

  _synchronizer = std::make_shared<Synchronizer>(_sub_image, _sub_apriltag_detection, 10);
  _synchronizer->registerCallback(
    std::bind(&AprilTagDraw::callbackSynchedDetection, this, std::placeholders::_1, std::placeholders::_2)
  );

  // publisher for drawn detections
  _pub_image = create_publisher<sensor_msgs::msg::Image>(
    "drawn_tags", rclcpp::QoS(5).reliable()
  );
}

AprilTagDraw::~AprilTagDraw()
{

}

void AprilTagDraw::callbackSynchedDetection(
    std::shared_ptr<const sensor_msgs::msg::Image> image,
    std::shared_ptr<const apriltag_msgs::msg::AprilTagDetectionArray> detection)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image, image->encoding);

    for (const auto& detection : detection->detections) {
      cv::line(
        cv_image->image,
        cv::Point(detection.corners[0].x, detection.corners[0].y),
        cv::Point(detection.corners[1].x, detection.corners[1].y),
        cv::Scalar(0, 255, 0)
      );
      cv::line(
        cv_image->image,
        cv::Point(detection.corners[1].x, detection.corners[1].y),
        cv::Point(detection.corners[2].x, detection.corners[2].y),
        cv::Scalar(0, 255, 0)
      );
      cv::line(
        cv_image->image,
        cv::Point(detection.corners[2].x, detection.corners[2].y),
        cv::Point(detection.corners[3].x, detection.corners[3].y),
        cv::Scalar(0, 255, 0)
      );
      cv::line(
        cv_image->image,
        cv::Point(detection.corners[3].x, detection.corners[3].y),
        cv::Point(detection.corners[0].x, detection.corners[0].y),
        cv::Scalar(0, 255, 0)
      );                  
    }

    _pub_image->publish(*cv_image->toImageMsg());
  }
  catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::perception::AprilTagDraw>());
  rclcpp::shutdown();

  return 0;
}