#include "qr_detection_and_pose_estimation.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

namespace eduart {
namespace perception {

QrDetectionAndPoseEstimation::Parameter QrDetectionAndPoseEstimation::get_parameter(
  rclcpp::Node &ros_node, const Parameter& default_parameter)
{
  (void)ros_node;
  (void)default_parameter;
  Parameter parameter;

  return parameter;
}

QrDetectionAndPoseEstimation::QrDetectionAndPoseEstimation()
  : rclcpp::Node("qr_detection_and_pose_estimation")
{

}

QrDetectionAndPoseEstimation::~QrDetectionAndPoseEstimation()
{

}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    colorCam->setInterleaved(true);
    colorCam->preview.link(xlinkOut->input);


    try {
        // Try connecting to device and start the pipeline
        dai::Device device(pipeline);

        // Get output queue
        auto preview = device.getOutputQueue("preview");

        cv::Mat frame;
        while (true) {

            // Receive 'preview' frame from device
            auto imgFrame = preview->get<dai::ImgFrame>();

            // Show the received 'preview' frame
            cv::imshow("preview", cv::Mat(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3, imgFrame->getData().data()));

            // Wait and check if 'q' pressed
            if (cv::waitKey(1) == 'q') return 0;

        }
    } catch (const std::runtime_error& err) {
        std::cout << err.what() << std::endl;
    }

  rclcpp::spin(std::make_shared<eduart::perception::QrDetectionAndPoseEstimation>());
  rclcpp::shutdown();

  return 0;
}
