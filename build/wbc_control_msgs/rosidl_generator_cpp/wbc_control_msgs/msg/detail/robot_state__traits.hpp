// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from wbc_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/robot_state.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "wbc_control_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'body_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'body_acceleration'
#include "geometry_msgs/msg/detail/accel__traits.hpp"
// Member 'feet'
#include "wbc_control_msgs/msg/detail/foot_state__traits.hpp"

namespace wbc_control_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: body_pose
  {
    out << "body_pose: ";
    to_flow_style_yaml(msg.body_pose, out);
    out << ", ";
  }

  // member: body_velocity
  {
    out << "body_velocity: ";
    to_flow_style_yaml(msg.body_velocity, out);
    out << ", ";
  }

  // member: body_acceleration
  {
    out << "body_acceleration: ";
    to_flow_style_yaml(msg.body_acceleration, out);
    out << ", ";
  }

  // member: joint_positions
  {
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []";
    } else {
      out << "joint_positions: [";
      size_t pending_items = msg.joint_positions.size();
      for (auto item : msg.joint_positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_velocities
  {
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []";
    } else {
      out << "joint_velocities: [";
      size_t pending_items = msg.joint_velocities.size();
      for (auto item : msg.joint_velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_accelerations
  {
    if (msg.joint_accelerations.size() == 0) {
      out << "joint_accelerations: []";
    } else {
      out << "joint_accelerations: [";
      size_t pending_items = msg.joint_accelerations.size();
      for (auto item : msg.joint_accelerations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feet
  {
    if (msg.feet.size() == 0) {
      out << "feet: []";
    } else {
      out << "feet: [";
      size_t pending_items = msg.feet.size();
      for (auto item : msg.feet) {
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
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: body_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_pose:\n";
    to_block_style_yaml(msg.body_pose, out, indentation + 2);
  }

  // member: body_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_velocity:\n";
    to_block_style_yaml(msg.body_velocity, out, indentation + 2);
  }

  // member: body_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_acceleration:\n";
    to_block_style_yaml(msg.body_acceleration, out, indentation + 2);
  }

  // member: joint_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []\n";
    } else {
      out << "joint_positions:\n";
      for (auto item : msg.joint_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []\n";
    } else {
      out << "joint_velocities:\n";
      for (auto item : msg.joint_velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_accelerations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_accelerations.size() == 0) {
      out << "joint_accelerations: []\n";
    } else {
      out << "joint_accelerations:\n";
      for (auto item : msg.joint_accelerations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feet
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feet.size() == 0) {
      out << "feet: []\n";
    } else {
      out << "feet:\n";
      for (auto item : msg.feet) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
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
  const wbc_control_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  wbc_control_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use wbc_control_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const wbc_control_msgs::msg::RobotState & msg)
{
  return wbc_control_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<wbc_control_msgs::msg::RobotState>()
{
  return "wbc_control_msgs::msg::RobotState";
}

template<>
inline const char * name<wbc_control_msgs::msg::RobotState>()
{
  return "wbc_control_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<wbc_control_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<wbc_control_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<wbc_control_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
