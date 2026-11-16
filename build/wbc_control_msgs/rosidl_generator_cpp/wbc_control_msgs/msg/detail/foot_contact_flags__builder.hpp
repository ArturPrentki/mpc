// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_contact_flags.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__BUILDER_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "wbc_control_msgs/msg/detail/foot_contact_flags__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace wbc_control_msgs
{

namespace msg
{

namespace builder
{

class Init_FootContactFlags_rr
{
public:
  explicit Init_FootContactFlags_rr(::wbc_control_msgs::msg::FootContactFlags & msg)
  : msg_(msg)
  {}
  ::wbc_control_msgs::msg::FootContactFlags rr(::wbc_control_msgs::msg::FootContactFlags::_rr_type arg)
  {
    msg_.rr = std::move(arg);
    return std::move(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootContactFlags msg_;
};

class Init_FootContactFlags_rl
{
public:
  explicit Init_FootContactFlags_rl(::wbc_control_msgs::msg::FootContactFlags & msg)
  : msg_(msg)
  {}
  Init_FootContactFlags_rr rl(::wbc_control_msgs::msg::FootContactFlags::_rl_type arg)
  {
    msg_.rl = std::move(arg);
    return Init_FootContactFlags_rr(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootContactFlags msg_;
};

class Init_FootContactFlags_fr
{
public:
  explicit Init_FootContactFlags_fr(::wbc_control_msgs::msg::FootContactFlags & msg)
  : msg_(msg)
  {}
  Init_FootContactFlags_rl fr(::wbc_control_msgs::msg::FootContactFlags::_fr_type arg)
  {
    msg_.fr = std::move(arg);
    return Init_FootContactFlags_rl(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootContactFlags msg_;
};

class Init_FootContactFlags_fl
{
public:
  explicit Init_FootContactFlags_fl(::wbc_control_msgs::msg::FootContactFlags & msg)
  : msg_(msg)
  {}
  Init_FootContactFlags_fr fl(::wbc_control_msgs::msg::FootContactFlags::_fl_type arg)
  {
    msg_.fl = std::move(arg);
    return Init_FootContactFlags_fr(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootContactFlags msg_;
};

class Init_FootContactFlags_header
{
public:
  Init_FootContactFlags_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FootContactFlags_fl header(::wbc_control_msgs::msg::FootContactFlags::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FootContactFlags_fl(msg_);
  }

private:
  ::wbc_control_msgs::msg::FootContactFlags msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::wbc_control_msgs::msg::FootContactFlags>()
{
  return wbc_control_msgs::msg::builder::Init_FootContactFlags_header();
}

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__BUILDER_HPP_
