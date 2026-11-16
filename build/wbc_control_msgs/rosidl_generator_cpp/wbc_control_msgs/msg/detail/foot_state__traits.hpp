// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_state.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__TRAITS_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wbc_control_msgs/msg/detail/foot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
// Member 'acceleration'
// Member 'contact_force'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace wbc_control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FootState & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: is_contact
  {
    out << "is_contact: ";
    rosidl_generator_traits::value_to_yaml(msg.is_contact, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    to_flow_style_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: contact_force
  {
    out << "contact_force: ";
    to_flow_style_yaml(msg.contact_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FootState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: is_contact
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_contact: ";
    rosidl_generator_traits::value_to_yaml(msg.is_contact, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration:\n";
    to_block_style_yaml(msg.acceleration, out, indentation + 2);
  }

  // member: contact_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_force:\n";
    to_block_style_yaml(msg.contact_force, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FootState & msg, bool use_flow_style = false)
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
  const wbc_control_msgs::msg::FootState & msg,
  std::ostream & out, size_t indentation = 0)
{
  wbc_control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wbc_control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const wbc_control_msgs::msg::FootState & msg)
{
  return wbc_control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wbc_control_msgs::msg::FootState>()
{
  return "wbc_control_msgs::msg::FootState";
}

template<>
inline const char * name<wbc_control_msgs::msg::FootState>()
{
  return "wbc_control_msgs/msg/FootState";
}

template<>
struct has_fixed_size<wbc_control_msgs::msg::FootState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wbc_control_msgs::msg::FootState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wbc_control_msgs::msg::FootState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__TRAITS_HPP_
