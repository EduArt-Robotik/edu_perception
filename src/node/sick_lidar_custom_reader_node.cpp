#include "sick_lidar_custom_reader_node.hpp"

namespace eduart {
namespace perception {

SickLidarCustomReader::Parameter SickLidarCustomReader::get_parameter(
  rclcpp::Node &ros_node, const Parameter &default_parameter)
{
  return default_parameter;
}

SickLidarCustomReader::SickLidarCustomReader()
  : rclcpp::Node("sick_lidar_custom_reader")
  , _parameter(get_parameter(*this, {}))
{

}

SickLidarCustomReader::~SickLidarCustomReader()
{

}

} // end namespace perception
} // end namespace eduart

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<eduart::perception::SickLidarCustomReader>());
  rclcpp::shutdown();

  return 0;
}
