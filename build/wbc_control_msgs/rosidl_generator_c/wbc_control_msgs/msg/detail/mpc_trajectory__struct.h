// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/mpc_trajectory.h"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'steps'
#include "wbc_control_msgs/msg/detail/mpc_state_step__struct.h"

/// Struct defined in msg/MpcTrajectory in the package wbc_control_msgs.
typedef struct wbc_control_msgs__msg__MpcTrajectory
{
  std_msgs__msg__Header header;
  /// Full sequence of trajectory steps over the MPC horizon
  wbc_control_msgs__msg__MpcStateStep__Sequence steps;
} wbc_control_msgs__msg__MpcTrajectory;

// Struct for a sequence of wbc_control_msgs__msg__MpcTrajectory.
typedef struct wbc_control_msgs__msg__MpcTrajectory__Sequence
{
  wbc_control_msgs__msg__MpcTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wbc_control_msgs__msg__MpcTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_H_
