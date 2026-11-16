// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_contact_flags.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__TRAITS_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wbc_control_msgs/msg/detail/foot_contact_flags__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace wbc_control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FootContactFlags & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: fl
  {
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << ", ";
  }

  // member: fr
  {
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << ", ";
  }

  // member: rl
  {
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << ", ";
  }

  // member: rr
  {
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FootContactFlags & msg,
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

  // member: fl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fl: ";
    rosidl_generator_traits::value_to_yaml(msg.fl, out);
    out << "\n";
  }

  // member: fr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fr: ";
    rosidl_generator_traits::value_to_yaml(msg.fr, out);
    out << "\n";
  }

  // member: rl
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rl: ";
    rosidl_generator_traits::value_to_yaml(msg.rl, out);
    out << "\n";
  }

  // member: rr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rr: ";
    rosidl_generator_traits::value_to_yaml(msg.rr, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FootContactFlags & msg, bool use_flow_style = false)
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
  const wbc_control_msgs::msg::FootContactFlags & msg,
  std::ostream & out, size_t indentation = 0)
{
  wbc_control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wbc_control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const wbc_control_msgs::msg::FootContactFlags & msg)
{
  return wbc_control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wbc_control_msgs::msg::FootContactFlags>()
{
  return "wbc_control_msgs::msg::FootContactFlags";
}

template<>
inline const char * name<wbc_control_msgs::msg::FootContactFlags>()
{
  return "wbc_control_msgs/msg/FootContactFlags";
}

template<>
struct has_fixed_size<wbc_control_msgs::msg::FootContactFlags>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<wbc_control_msgs::msg::FootContactFlags>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<wbc_control_msgs::msg::FootContactFlags>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__TRAITS_HPP_
