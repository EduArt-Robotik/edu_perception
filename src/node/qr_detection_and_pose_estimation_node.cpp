/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<eduart::robot::iotbot::IotBot>());
  rclcpp::shutdown();

  return 0;
}
