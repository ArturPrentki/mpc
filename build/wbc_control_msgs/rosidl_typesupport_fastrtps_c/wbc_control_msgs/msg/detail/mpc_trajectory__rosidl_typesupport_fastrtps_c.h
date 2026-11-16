// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice
#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "wbc_control_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "wbc_control_msgs/msg/detail/mpc_trajectory__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
bool cdr_serialize_wbc_control_msgs__msg__MpcTrajectory(
  const wbc_control_msgs__msg__MpcTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
bool cdr_deserialize_wbc_control_msgs__msg__MpcTrajectory(
  eprosima::fastcdr::Cdr &,
  wbc_control_msgs__msg__MpcTrajectory * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
size_t get_serialized_size_wbc_control_msgs__msg__MpcTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
size_t max_serialized_size_wbc_control_msgs__msg__MpcTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
bool cdr_serialize_key_wbc_control_msgs__msg__MpcTrajectory(
  const wbc_control_msgs__msg__MpcTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
size_t get_serialized_size_key_wbc_control_msgs__msg__MpcTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
size_t max_serialized_size_key_wbc_control_msgs__msg__MpcTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wbc_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, wbc_control_msgs, msg, MpcTrajectory)();

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
