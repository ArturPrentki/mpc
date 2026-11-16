// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_state.h"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration'
// Member 'contact_force'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/FootState in the package wbc_control_msgs.
typedef struct wbc_control_msgs__msg__FootState
{
  /// Foot identifier ("FL", "FR", "RL", "RR")
  rosidl_runtime_c__String name;
  /// Contact state (true = stance, false = swing)
  bool is_contact;
  /// Foot target position (World Frame) ????
  geometry_msgs__msg__Point position;
  /// Foot target velocity
  geometry_msgs__msg__Vector3 velocity;
  /// Target XYZ acceleration (optional, but good for WBC)
  geometry_msgs__msg__Vector3 acceleration;
  /// Optimized ground reaction force (Fx, Fy, Fz)
  geometry_msgs__msg__Vector3 contact_force;
} wbc_control_msgs__msg__FootState;

// Struct for a sequence of wbc_control_msgs__msg__FootState.
typedef struct wbc_control_msgs__msg__FootState__Sequence
{
  wbc_control_msgs__msg__FootState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wbc_control_msgs__msg__FootState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_H_
