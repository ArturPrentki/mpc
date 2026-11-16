// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/mpc_trajectory.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__TRAITS_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wbc_control_msgs/msg/detail/mpc_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'steps'
#include "wbc_control_msgs/msg/detail/mpc_state_step__traits.hpp"

namespace wbc_control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MpcTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: steps
  {
    if (msg.steps.size() == 0) {
      out << "steps: []";
    } else {
      out << "steps: [";
      size_t pending_items = msg.steps.size();
      for (auto item : msg.steps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MpcTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: steps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.steps.size() == 0) {
      out << "steps: []\n";
    } else {
      out << "steps:\n";
      for (auto item : msg.steps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MpcTrajectory & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace wbc_control_msgs

namespace rosidl_generator_traits
{

[[deprecated("use wbc_control_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const wbc_control_msgs::msg::MpcTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  wbc_control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wbc_control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const wbc_control_msgs::msg::MpcTrajectory & msg)
{
  return wbc_control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wbc_control_msgs::msg::MpcTrajectory>()
{
  return "wbc_control_msgs::msg::MpcTrajectory";
}

template<>
inline const char * name<wbc_control_msgs::msg::MpcTrajectory>()
{
  return "wbc_control_msgs/msg/MpcTrajectory";
}

template<>
struct has_fixed_size<wbc_control_msgs::msg::MpcTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wbc_control_msgs::msg::MpcTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wbc_control_msgs::msg::MpcTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__TRAITS_HPP_
