// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_contact_flags.h"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_H_

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

/// Struct defined in msg/FootContactFlags in the package wbc_control_msgs.
/**
  * FootContactFlags.msg
 */
typedef struct wbc_control_msgs__msg__FootContactFlags
{
  std_msgs__msg__Header header;
  bool fl;
  bool fr;
  bool rl;
  bool rr;
} wbc_control_msgs__msg__FootContactFlags;

// Struct for a sequence of wbc_control_msgs__msg__FootContactFlags.
typedef struct wbc_control_msgs__msg__FootContactFlags__Sequence
{
  wbc_control_msgs__msg__FootContactFlags * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} wbc_control_msgs__msg__FootContactFlags__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_H_
