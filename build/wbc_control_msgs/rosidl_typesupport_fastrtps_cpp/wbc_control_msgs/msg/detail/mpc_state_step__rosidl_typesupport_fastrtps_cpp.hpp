// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from wbc_control_msgs:msg/MpcStateStep.idl
// generated code does not contain a copyright notice

#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "wbc_control_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "wbc_control_msgs/msg/detail/mpc_state_step__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace wbc_control_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
cdr_serialize(
  const wbc_control_msgs::msg::MpcStateStep & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  wbc_control_msgs::msg::MpcStateStep & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
get_serialized_size(
  const wbc_control_msgs::msg::MpcStateStep & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
max_serialized_size_MpcStateStep(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
cdr_serialize_key(
  const wbc_control_msgs::msg::MpcStateStep & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
get_serialized_size_key(
  const wbc_control_msgs::msg::MpcStateStep & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
max_serialized_size_key_MpcStateStep(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace wbc_control_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_wbc_control_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, wbc_control_msgs, msg, MpcStateStep)();

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
