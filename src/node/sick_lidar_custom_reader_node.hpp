/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <regex>

namespace eduart {
namespace perception {

class SickLidarCustomReader : public rclcpp::Node
{
public:
  struct Parameter {
    std::string lidar_ip_address = "192.168.0.70";
    std::uint16_t lidar_port = 2111;

    inline bool isIpAddressValid() const {
      const std::regex ipv4(
        "(([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\.){3}([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])");
      return std::regex_match(lidar_ip_address, ipv4);
    }
  };

  SickLidarCustomReader();
  ~SickLidarCustomReader() override;

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

private:
  const Parameter _parameter;

  std::shared_ptr<rclcpp::TimerBase> _timer_process_reading;

};

} // end namespace perception
} // end namespace eduart
