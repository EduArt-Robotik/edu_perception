/**
 * Copyright EduArt Robotik GmbH 2023
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "angle.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Geometry>

#include <array>
#include <tuple>
#include <utility>

namespace eduart {
namespace perception {

template <typename ChannelType>
class LowPassFilerChannel
{
public:
  using data_type = ChannelType;

  LowPassFilerChannel(const ChannelType& initial_value = static_cast<ChannelType>(0))
    : _value(initial_value)
  { }

  void update(const ChannelType& input, const float filter_weight) {
    _value = (1.0f - filter_weight) * _value + filter_weight * input;
  }
  inline void clear() { _value = static_cast<ChannelType>(0); }
  inline ChannelType getValue() const { return _value; }
  operator const ChannelType&() const { return _value; }

protected:
  ChannelType _value;
};

template <>
class LowPassFilerChannel<decltype(geometry_msgs::msg::Pose::orientation)> : protected LowPassFilerChannel<Eigen::Quaternionf>
{
public:
  using LowPassFilerChannel<Eigen::Quaternionf>::LowPassFilerChannel;
  using LowPassFilerChannel<Eigen::Quaternionf>::operator const Eigen::Quaternionf&;

  LowPassFilerChannel(const decltype(geometry_msgs::msg::Pose::orientation)& initial_value)
    : LowPassFilerChannel<Eigen::Quaternionf>(
        Eigen::Quaternionf(initial_value.w, initial_value.x, initial_value.y, initial_value.z)
      )
  { }

  void update(const decltype(geometry_msgs::msg::Pose::orientation)& input, const float filter_weight) {
    _value = _value.slerp(filter_weight, Eigen::Quaternionf(input.w, input.x, input.y, input.z));
  }
  inline void clear() { _value = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f); }   
  decltype(geometry_msgs::msg::Pose::orientation) getValue() const {
    decltype(geometry_msgs::msg::Pose::orientation) orientation;
    orientation.w = _value.w();
    orientation.x = _value.x();
    orientation.y = _value.y();
    orientation.z = _value.z();

    return orientation;
  }
};

template <typename... ChannelDataType>
class LowPassFilter
{
public:
  struct Parameter {
    std::array<float, sizeof...(ChannelDataType)> weight;
  };

  LowPassFilter(const Parameter& parameter, const ChannelDataType&... initial_value)
    : _parameter(parameter)
    , _filter_channel{ LowPassFilerChannel<ChannelDataType>(initial_value)... }
  { }

  inline LowPassFilter& operator()(const ChannelDataType&... input_value) {
    updateFilterChannel(_filter_channel, input_value..., std::index_sequence_for<ChannelDataType...>{});
    return *this;
  }
  inline void clear() {
    clearFilterChannel(_filter_channel, std::index_sequence_for<ChannelDataType...>{});
  }
  template <std::size_t ChannelIndex>
  inline auto& channel() { return std::get<ChannelIndex>(_filter_channel); }
  template <std::size_t ChannelIndex>
  inline const auto& channel() const { return std::get<ChannelIndex>(_filter_channel); }

private:
  template <std::size_t... Is>
  void updateFilterChannel(
    std::tuple<LowPassFilerChannel<ChannelDataType>...>& filter_channel,
    const ChannelDataType&... new_value, std::index_sequence<Is...>)
  {
    ((std::get<Is>(filter_channel).update(new_value, _parameter.weight[Is])), ...);
  }
  template <std::size_t... Is>
  void clearFilterChannel(
    std::tuple<LowPassFilerChannel<ChannelDataType>...>& filter_channel, std::index_sequence<Is...>)
  {
    ((std::get<Is>(filter_channel).clear()), ...);
  }

  Parameter _parameter;
  std::tuple<LowPassFilerChannel<ChannelDataType>...> _filter_channel;
};

} // end namespace perception
} // end namespace eduart
