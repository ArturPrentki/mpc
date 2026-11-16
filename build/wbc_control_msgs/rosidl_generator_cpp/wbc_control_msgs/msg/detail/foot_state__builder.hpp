// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_state.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wbc_control_msgs/msg/detail/foot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wbc_control_msgs
{

namespace msg
{

namespace builder
{

class Init_FootState_contact_force
{
public:
  explicit Init_FootState_contact_force(::wbc_control_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  ::wbc_control_msgs::msg::FootState contact_force(::wbc_control_msgs::msg::FootState::_contact_force_type arg)
  {
    msg_.contact_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

class Init_FootState_acceleration
{
public:
  explicit Init_FootState_acceleration(::wbc_control_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_contact_force acceleration(::wbc_control_msgs::msg::FootState::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_FootState_contact_force(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

class Init_FootState_velocity
{
public:
  explicit Init_FootState_velocity(::wbc_control_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_acceleration velocity(::wbc_control_msgs::msg::FootState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_FootState_acceleration(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

class Init_FootState_position
{
public:
  explicit Init_FootState_position(::wbc_control_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_velocity position(::wbc_control_msgs::msg::FootState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_FootState_velocity(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

class Init_FootState_is_contact
{
public:
  explicit Init_FootState_is_contact(::wbc_control_msgs::msg::FootState & msg)
  : msg_(msg)
  {}
  Init_FootState_position is_contact(::wbc_control_msgs::msg::FootState::_is_contact_type arg)
  {
    msg_.is_contact = std::move(arg);
    return Init_FootState_position(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

class Init_FootState_name
{
public:
  Init_FootState_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FootState_is_contact name(::wbc_control_msgs::msg::FootState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_FootState_is_contact(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wbc_control_msgs::msg::FootState>()
{
  return wbc_control_msgs::msg::builder::Init_FootState_name();
}

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__BUILDER_HPP_
