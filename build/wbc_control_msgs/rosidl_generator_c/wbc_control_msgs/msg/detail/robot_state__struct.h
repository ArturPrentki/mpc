// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wbc_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/robot_state.h"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'body_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'body_acceleration'
#include "geometry_msgs/msg/detail/accel__struct.h"
// Member 'joint_positions'
// Member 'joint_velocities'
// Member 'joint_accelerations'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'feet'
#include "wbc_control_msgs/msg/detail/foot_state__struct.h"

/// Struct defined in msg/RobotState in the package wbc_control_msgs.
/**
  * msg/RobotState.msg
 */
typedef struct wbc_control_msgs__msg__RobotState
{
  /// Body SE3 Pose (Translation + Quaternion Orientation)
  geometry_msgs__msg__Pose body_pose;
  /// Pinocchio::Motion Spatial Velocity (Linear + Angular)
  geometry_msgs__msg__Twist body_velocity;
  /// Pinocchio::Motion Spatial Acceleration (Linear + Angular)
  geometry_msgs__msg__Accel body_acceleration;
  /// q
  rosidl_runtime_c__double__Sequence joint_positions;
  /// v
  rosidl_runtime_c__double__Sequence joint_velocities;
  /// a
  rosidl_runtime_c__double__Sequence joint_accelerations;
  /// Foot target states and optimized contact forces
  wbc_control_msgs__msg__FootState__Sequence feet;
} wbc_control_msgs__msg__RobotState;

// Struct for a sequence of wbc_control_msgs__msg__RobotState.
typedef struct wbc_control_msgs__msg__RobotState__Sequence
{
  wbc_control_msgs__msg__RobotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wbc_control_msgs__msg__RobotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_H_
