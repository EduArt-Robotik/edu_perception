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
#include <type_traits>

namespace eduart {
namespace perception {
namespace impl {

template <typename ChannelType, typename ExchangeType>
struct LowPassFilterChannelOperation;

template <typename DataType>
struct LowPassFilterChannelOperation<DataType, std::enable_if_t<std::is_floating_point_v<DataType>, DataType>> {
  static inline DataType update(const DataType input, const DataType previous_value, const float filter_weight) {
    return (1.0f - filter_weight) * previous_value + filter_weight * input;
  }
  static inline void clear(DataType& value) {
    value = static_cast<DataType>(0);
  }
  static inline DataType getValue(const DataType& value) { return value; } 
};
template <>
struct LowPassFilterChannelOperation<decltype(geometry_msgs::msg::Pose::position),
                                     decltype(geometry_msgs::msg::Pose::position)> {
  using DataType = decltype(geometry_msgs::msg::Pose::position);

  static inline DataType update(const DataType& input, const DataType& previous_value, const float filter_weight) {
    DataType updated_value;
    updated_value.x = static_cast<double>(1.0f - filter_weight) * previous_value.x + static_cast<double>(filter_weight) * input.x;
    updated_value.y = static_cast<double>(1.0f - filter_weight) * previous_value.y + static_cast<double>(filter_weight) * input.y;
    updated_value.z = static_cast<double>(1.0f - filter_weight) * previous_value.z + static_cast<double>(filter_weight) * input.z;

    return updated_value;
  }
  static inline void clear(DataType& value) {
    value.x = 0.0;
    value.y = 0.0;
    value.z = 0.0;
  }
  static inline DataType getValue(const DataType& value) { return value; } 
};
template <>
struct LowPassFilterChannelOperation<Eigen::Quaternionf, decltype(geometry_msgs::msg::Pose::orientation)> {
  static Eigen::Quaternionf update(
    const decltype(geometry_msgs::msg::Pose::orientation)& input, const Eigen::Quaternionf& previous_value,
    const float filter_weight)
  {
    return previous_value.slerp(filter_weight, Eigen::Quaternionf(input.w, input.x, input.y, input.z));
  }
  static inline void clear(Eigen::Quaternionf& value) {
    value = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
  }
  static inline decltype(geometry_msgs::msg::Pose::orientation) getValue(const Eigen::Quaternionf& value) {
    decltype(geometry_msgs::msg::Pose::orientation) orientation;
    orientation.w = value.w();
    orientation.x = value.x();
    orientation.y = value.y();
    orientation.z = value.z();

    return orientation;
  }
};

} // end namespace impl

template <typename ChannelType, typename ExchangeType = ChannelType>
class LowPassFilerChannel
{
public:
  using data_type = ChannelType;
  using exchange_type = ExchangeType;

  inline void clear() { impl::LowPassFilterChannelOperation<ChannelType, ExchangeType>::clear(_value); }
  inline ExchangeType getValue() const {
    return impl::LowPassFilterChannelOperation<ChannelType, ExchangeType>::getValue(_value);
  }
  inline operator const ChannelType&() const { return getValue(); }

protected:
  void update(const ExchangeType& input, const float filter_weight) {
    _value = impl::LowPassFilterChannelOperation<ChannelType, ExchangeType>::update(
      input, _value, filter_weight
    );
  }

  ChannelType _value;
};

template <typename ChannelType, typename ExchangeType = ChannelType>
struct LowPassFilter : public LowPassFilerChannel<ChannelType, ExchangeType>
{
  struct Parameter {
    float weight = 1.0f;
  };

  LowPassFilter(const Parameter& parameter) : _parameter(parameter) { }
  void update(const ExchangeType& input) {
    LowPassFilerChannel<ChannelType, ExchangeType>::update(input, _parameter.weight);
  }

private:
  Parameter _parameter;
};

// template <typename... ChannelDataType>
// class LowPassFilter
// {
// public:
//   struct Parameter {
//     std::array<float, sizeof...(ChannelDataType)> weight;
//   };

//   LowPassFilter(const Parameter& parameter, const ChannelDataType&... initial_value)
//     : _parameter(parameter)
//     , _filter_channel{ LowPassFilerChannel<ChannelDataType>(initial_value)... }
//   { }

//   inline LowPassFilter& operator()(const ChannelDataType&... input_value) {
//     updateFilterChannel(_filter_channel, input_value..., std::index_sequence_for<ChannelDataType...>{});
//     return *this;
//   }
//   inline void clear() {
//     clearFilterChannel(_filter_channel, std::index_sequence_for<ChannelDataType...>{});
//   }
//   template <std::size_t ChannelIndex>
//   inline auto& channel() { return std::get<ChannelIndex>(_filter_channel); }
//   template <std::size_t ChannelIndex>
//   inline const auto& channel() const { return std::get<ChannelIndex>(_filter_channel); }

// private:
//   template <std::size_t... Is>
//   void updateFilterChannel(
//     std::tuple<LowPassFilerChannel<ChannelDataType>...>& filter_channel,
//     const ChannelDataType&... new_value, std::index_sequence<Is...>)
//   {
//     ((std::get<Is>(filter_channel).update(new_value, _parameter.weight[Is])), ...);
//   }
//   template <std::size_t... Is>
//   void clearFilterChannel(
//     std::tuple<LowPassFilerChannel<ChannelDataType>...>& filter_channel, std::index_sequence<Is...>)
//   {
//     ((std::get<Is>(filter_channel).clear()), ...);
//   }

//   Parameter _parameter;
//   std::tuple<LowPassFilerChannel<ChannelDataType>...> _filter_channel;
// };

} // end namespace perception
} // end namespace eduart
