// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/mpc_trajectory.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__BUILDER_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wbc_control_msgs/msg/detail/mpc_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wbc_control_msgs
{

namespace msg
{

namespace builder
{

class Init_MpcTrajectory_steps
{
public:
  explicit Init_MpcTrajectory_steps(::wbc_control_msgs::msg::MpcTrajectory & msg)
  : msg_(msg)
  {}
  ::wbc_control_msgs::msg::MpcTrajectory steps(::wbc_control_msgs::msg::MpcTrajectory::_steps_type arg)
  {
    msg_.steps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wbc_control_msgs::msg::MpcTrajectory msg_;
};

class Init_MpcTrajectory_header
{
public:
  Init_MpcTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MpcTrajectory_steps header(::wbc_control_msgs::msg::MpcTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MpcTrajectory_steps(msg_);
  }

private:
  ::wbc_control_msgs::msg::MpcTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wbc_control_msgs::msg::MpcTrajectory>()
{
  return wbc_control_msgs::msg::builder::Init_MpcTrajectory_header();
}

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__BUILDER_HPP_
