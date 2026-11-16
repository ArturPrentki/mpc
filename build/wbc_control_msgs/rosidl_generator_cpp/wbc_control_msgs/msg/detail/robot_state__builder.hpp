// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wbc_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/robot_state.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wbc_control_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wbc_control_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_feet
{
public:
  explicit Init_RobotState_feet(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::wbc_control_msgs::msg::RobotState feet(::wbc_control_msgs::msg::RobotState::_feet_type arg)
  {
    msg_.feet = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_joint_accelerations
{
public:
  explicit Init_RobotState_joint_accelerations(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_feet joint_accelerations(::wbc_control_msgs::msg::RobotState::_joint_accelerations_type arg)
  {
    msg_.joint_accelerations = std::move(arg);
    return Init_RobotState_feet(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_joint_velocities
{
public:
  explicit Init_RobotState_joint_velocities(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_joint_accelerations joint_velocities(::wbc_control_msgs::msg::RobotState::_joint_velocities_type arg)
  {
    msg_.joint_velocities = std::move(arg);
    return Init_RobotState_joint_accelerations(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_joint_positions
{
public:
  explicit Init_RobotState_joint_positions(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_joint_velocities joint_positions(::wbc_control_msgs::msg::RobotState::_joint_positions_type arg)
  {
    msg_.joint_positions = std::move(arg);
    return Init_RobotState_joint_velocities(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_body_acceleration
{
public:
  explicit Init_RobotState_body_acceleration(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_joint_positions body_acceleration(::wbc_control_msgs::msg::RobotState::_body_acceleration_type arg)
  {
    msg_.body_acceleration = std::move(arg);
    return Init_RobotState_joint_positions(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_body_velocity
{
public:
  explicit Init_RobotState_body_velocity(::wbc_control_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  Init_RobotState_body_acceleration body_velocity(::wbc_control_msgs::msg::RobotState::_body_velocity_type arg)
  {
    msg_.body_velocity = std::move(arg);
    return Init_RobotState_body_acceleration(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

class Init_RobotState_body_pose
{
public:
  Init_RobotState_body_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_body_velocity body_pose(::wbc_control_msgs::msg::RobotState::_body_pose_type arg)
  {
    msg_.body_pose = std::move(arg);
    return Init_RobotState_body_velocity(msg_);
  }

private:
  ::wbc_control_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wbc_control_msgs::msg::RobotState>()
{
  return wbc_control_msgs::msg::builder::Init_RobotState_body_pose();
}

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
