#include "sick_lidar_custom_reader_node.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <netinet/in.h>
#include <net/if.h>

#include <arpa/inet.h>

namespace eduart {
namespace perception {

using namespace std::chrono_literals;

static constexpr std::array<std::uint8_t, 6> convert_field_state_to_msg = {
  edu_perception::msg::LidarField::NOT_CONFIGURED,
  edu_perception::msg::LidarField::INACTIVE,
  edu_perception::msg::LidarField::FREE,
  edu_perception::msg::LidarField::DETECTING_INFRINGED,
  edu_perception::msg::LidarField::INFRINGED,
  edu_perception::msg::LidarField::DETECTING_FREE
};

static std::uint32_t get_timestamp(const std::uint8_t rx_buffer[])
{
  std::uint32_t timestamp;
  unsigned int version;
  std::sscanf((char*)rx_buffer + 1, "sRA FieldEvaluationResult %u %x", &version, &timestamp);

  return timestamp;
}

static std::vector<edu_perception::msg::LidarField> deserialize(
  const std::uint8_t rx_buffer[], const std::size_t length, const SickLidarCustomReader::Parameter& parameter)
{
  std::vector<edu_perception::msg::LidarField> fields;
  std::vector<edu_perception::msg::LidarField::_state_type> field_state;
  std::size_t found_spaces = 0;

  // collect all field states
  for (std::size_t i = 0; i < length; ++i) {
    if (rx_buffer[i] == '\0') {
      // string end reached
      break;
    }
    // counting spaces
    if (rx_buffer[i] == ' ') {
      ++found_spaces;
      continue;
    }

    //else: some proper data
    if (found_spaces < 5) {
      // no field data
      continue;
    }
    //else: field data --> parse

    if (rx_buffer[i] - '0' >= static_cast<int>(convert_field_state_to_msg.size())) {
      // field state value is invalid
      std::cout << "invalid field state value = " << rx_buffer[i] << std::endl;
      continue;
    }

    field_state.push_back(convert_field_state_to_msg[rx_buffer[i] - '0']);
  }

  // pick only wanted fields using given field indicies
  for (std::size_t i = 0; i < parameter.field_index.size(); ++i) {
    if (parameter.field_index[i] >= field_state.size()) {
      // index is out of range --> skip
      continue;
    }

    fields.emplace_back();
    fields.back().name = parameter.field_name[i];
    fields.back().state = field_state[parameter.field_index[i]];
  }
  
  return fields;
}

SickLidarCustomReader::Parameter SickLidarCustomReader::get_parameter(
  rclcpp::Node &ros_node, const Parameter &default_parameter)
{
  (void)ros_node;
  return default_parameter;
}

SickLidarCustomReader::SickLidarCustomReader()
  : rclcpp::Node("sick_lidar_custom_reader")
  , _parameter(get_parameter(*this, {}))
{
  // Create socket and connect to Sick lidar.
  sockaddr_in socket_address;

  socket_address.sin_addr.s_addr = inet_addr(_parameter.lidar_ip_address.c_str());
  socket_address.sin_family =  AF_INET;
  socket_address.sin_port = htons(_parameter.lidar_port);
  _socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

  if (_socket_fd < 0) {
    throw std::runtime_error("problem occurred during creating TCP socket");
  }
  if (::connect(_socket_fd, (struct sockaddr*)&socket_address, sizeof(socket_address)) < 0) {
    ::close(_socket_fd);
    throw std::invalid_argument("error occurred during opening TCP socket");
  }

  // set timeout for reading socket
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 200000; // 200 ms
  setsockopt(_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

  // publisher
  _pub_field_evaluation = create_publisher<edu_perception::msg::LidarFieldEvaluation>(
    "field_evaluation", rclcpp::QoS(2).reliable().transient_local()
  );

  // start timer
  _timer_process_reading = create_timer(100ms, std::bind(&SickLidarCustomReader::processReading, this));
}

SickLidarCustomReader::~SickLidarCustomReader()
{
  if (_socket_fd > 0) {
    ::close(_socket_fd);
  }
}

void SickLidarCustomReader::processReading()
{
  // note: below implementation is designed for field evaluation only at the moment

  // field evaluation
  // message to request field evaluation results (manual picoscan150 s.169)
  constexpr std::uint8_t tx_buffer[] = {
    0x02, 0x73, 0x52, 0x4E, 0x20, 0x46, 0x69, 0x65, 0x6C, 0x64, 0x45, 0x76,
    0x61, 0x6C, 0x75, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x52, 0x65, 0x73, 0x75,
    0x6C, 0x74, 0x03 };

  if (::send(_socket_fd, tx_buffer, sizeof(tx_buffer), 0) != sizeof(tx_buffer)) {
    RCLCPP_ERROR(get_logger(), "failed to send field evaluation result request.");
    return;
  }

  std::uint8_t rx_buffer[500]; // measured number of bytes. It differs from lidar configuration somehow.
  const int received_bytes = ::recv(_socket_fd, rx_buffer, sizeof(rx_buffer), 0);

  if (received_bytes < 0) {
    RCLCPP_ERROR(get_logger(), "error during reading data from tcp socket.");
    return;
  }
  if (rx_buffer[0] != 0x02 || rx_buffer[received_bytes - 1] != 0x03) {
    // invalid message received
    RCLCPP_ERROR(get_logger(), "received message is invalid.");
    return;
  }

  rx_buffer[0] = ' '; // override special character
  rx_buffer[received_bytes - 1] = 0; // terminate buffer so it is a c string
  RCLCPP_INFO(get_logger(), "read %i bytes from socket. got: %s", received_bytes, rx_buffer);

  std::cout << "timestamp = " << get_timestamp(rx_buffer) << std::endl;
  const auto fields = deserialize(rx_buffer, received_bytes, _parameter);
  std::cout << "read " << fields.size() << " fields" << std::endl;

  const auto stamp = get_timestamp(rx_buffer);

  // if timestamp has not changed than the state has also not changed
  if (stamp == _stamp_last_field_state) {
    // no state change --> no publishing
    return;
  }
  // else: state changed --> publish it

  edu_perception::msg::LidarFieldEvaluation msg;
  msg.header.frame_id = "";
  msg.header.stamp = get_clock()->now();
  msg.fields = std::move(fields);
  _stamp_last_field_state = stamp;

  _pub_field_evaluation->publish(msg);
  // std::cout << "rx buffer: ";
  // for (std::size_t i = 0; i < received_bytes; ++i) {
  //   std::cout << std::hex << static_cast<int>(rx_buffer[i]) << ' ';
  // }
  // std::cout << std::dec << std::endl;
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
