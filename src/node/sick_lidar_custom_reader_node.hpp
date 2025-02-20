/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_perception/msg/lidar_field_evaluation.hpp"

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
    
    std::vector<std::size_t> field_index = { 0, 2 };
    std::vector<std::string> field_name = { "warnfeld", "schutzfeld" };
  };

  SickLidarCustomReader();
  ~SickLidarCustomReader() override;

  static Parameter get_parameter(rclcpp::Node& ros_node, const Parameter& default_parameter);

private:
  void processReading();

  const Parameter _parameter;

  int _socket_fd = -1;
  std::uint32_t _stamp_last_field_state = 0;
  std::shared_ptr<rclcpp::TimerBase> _timer_process_reading;
  std::shared_ptr<rclcpp::Publisher<edu_perception::msg::LidarFieldEvaluation>> _pub_field_evaluation;
};

} // end namespace perception
} // end namespace eduart
