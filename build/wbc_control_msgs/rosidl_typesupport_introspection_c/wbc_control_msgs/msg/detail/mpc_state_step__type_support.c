// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wbc_control_msgs:msg/MpcStateStep.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wbc_control_msgs/msg/detail/mpc_state_step__rosidl_typesupport_introspection_c.h"
#include "wbc_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wbc_control_msgs/msg/detail/mpc_state_step__functions.h"
#include "wbc_control_msgs/msg/detail/mpc_state_step__struct.h"


// Include directives for member types
// Member `time_offset`
#include "builtin_interfaces/msg/time.h"
// Member `time_offset`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `body_pose`
#include "geometry_msgs/msg/pose.h"
// Member `body_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `body_velocity`
#include "geometry_msgs/msg/twist.h"
// Member `body_velocity`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `body_acceleration`
#include "geometry_msgs/msg/accel.h"
// Member `body_acceleration`
#include "geometry_msgs/msg/detail/accel__rosidl_typesupport_introspection_c.h"
// Member `joint_positions`
// Member `joint_velocities`
// Member `joint_accelerations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `feet`
#include "wbc_control_msgs/msg/foot_state.h"
// Member `feet`
#include "wbc_control_msgs/msg/detail/foot_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wbc_control_msgs__msg__MpcStateStep__init(message_memory);
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_fini_function(void * message_memory)
{
  wbc_control_msgs__msg__MpcStateStep__fini(message_memory);
}

size_t wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_velocities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_velocities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_accelerations(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_accelerations(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_accelerations(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_accelerations(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_accelerations(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_accelerations(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_accelerations(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_accelerations(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__feet(
  const void * untyped_member)
{
  const wbc_control_msgs__msg__FootState__Sequence * member =
    (const wbc_control_msgs__msg__FootState__Sequence *)(untyped_member);
  return member->size;
}

const void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__feet(
  const void * untyped_member, size_t index)
{
  const wbc_control_msgs__msg__FootState__Sequence * member =
    (const wbc_control_msgs__msg__FootState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__feet(
  void * untyped_member, size_t index)
{
  wbc_control_msgs__msg__FootState__Sequence * member =
    (wbc_control_msgs__msg__FootState__Sequence *)(untyped_member);
  return &member->data[index];
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__feet(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const wbc_control_msgs__msg__FootState * item =
    ((const wbc_control_msgs__msg__FootState *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__feet(untyped_member, index));
  wbc_control_msgs__msg__FootState * value =
    (wbc_control_msgs__msg__FootState *)(untyped_value);
  *value = *item;
}

void wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__feet(
  void * untyped_member, size_t index, const void * untyped_value)
{
  wbc_control_msgs__msg__FootState * item =
    ((wbc_control_msgs__msg__FootState *)
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__feet(untyped_member, index));
  const wbc_control_msgs__msg__FootState * value =
    (const wbc_control_msgs__msg__FootState *)(untyped_value);
  *item = *value;
}

bool wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__feet(
  void * untyped_member, size_t size)
{
  wbc_control_msgs__msg__FootState__Sequence * member =
    (wbc_control_msgs__msg__FootState__Sequence *)(untyped_member);
  wbc_control_msgs__msg__FootState__Sequence__fini(member);
  return wbc_control_msgs__msg__FootState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[8] = {
  {
    "time_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, time_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, body_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, body_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, body_acceleration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, joint_positions),  // bytes offset in struct
    NULL,  // default value
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_positions,  // size() function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_positions,  // get_const(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_positions,  // get(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_positions,  // fetch(index, &value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_positions,  // assign(index, value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_positions  // resize(index) function pointer
  },
  {
    "joint_velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, joint_velocities),  // bytes offset in struct
    NULL,  // default value
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_velocities,  // size() function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_velocities,  // get_const(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_velocities,  // get(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_velocities,  // fetch(index, &value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_velocities,  // assign(index, value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_velocities  // resize(index) function pointer
  },
  {
    "joint_accelerations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, joint_accelerations),  // bytes offset in struct
    NULL,  // default value
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__joint_accelerations,  // size() function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__joint_accelerations,  // get_const(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__joint_accelerations,  // get(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__joint_accelerations,  // fetch(index, &value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__joint_accelerations,  // assign(index, value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__joint_accelerations  // resize(index) function pointer
  },
  {
    "feet",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__MpcStateStep, feet),  // bytes offset in struct
    NULL,  // default value
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__size_function__MpcStateStep__feet,  // size() function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_const_function__MpcStateStep__feet,  // get_const(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__get_function__MpcStateStep__feet,  // get(index) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__fetch_function__MpcStateStep__feet,  // fetch(index, &value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__assign_function__MpcStateStep__feet,  // assign(index, value) function pointer
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__resize_function__MpcStateStep__feet  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_members = {
  "wbc_control_msgs__msg",  // message namespace
  "MpcStateStep",  // message name
  8,  // number of fields
  sizeof(wbc_control_msgs__msg__MpcStateStep),
  false,  // has_any_key_member_
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array,  // message members
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_init_function,  // function to initialize message memory (memory has to be allocated)
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_type_support_handle = {
  0,
  &wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_members,
  get_message_typesupport_handle_function,
  &wbc_control_msgs__msg__MpcStateStep__get_type_hash,
  &wbc_control_msgs__msg__MpcStateStep__get_type_description,
  &wbc_control_msgs__msg__MpcStateStep__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wbc_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wbc_control_msgs, msg, MpcStateStep)() {
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Accel)();
  wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wbc_control_msgs, msg, FootState)();
  if (!wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_type_support_handle.typesupport_identifier) {
    wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wbc_control_msgs__msg__MpcStateStep__rosidl_typesupport_introspection_c__MpcStateStep_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
