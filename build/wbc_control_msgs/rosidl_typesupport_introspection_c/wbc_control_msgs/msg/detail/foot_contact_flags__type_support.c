// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "wbc_control_msgs/msg/detail/foot_contact_flags__rosidl_typesupport_introspection_c.h"
#include "wbc_control_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "wbc_control_msgs/msg/detail/foot_contact_flags__functions.h"
#include "wbc_control_msgs/msg/detail/foot_contact_flags__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  wbc_control_msgs__msg__FootContactFlags__init(message_memory);
}

void wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_fini_function(void * message_memory)
{
  wbc_control_msgs__msg__FootContactFlags__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__FootContactFlags, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fl",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__FootContactFlags, fl),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__FootContactFlags, fr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rl",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__FootContactFlags, rl),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs__msg__FootContactFlags, rr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_members = {
  "wbc_control_msgs__msg",  // message namespace
  "FootContactFlags",  // message name
  5,  // number of fields
  sizeof(wbc_control_msgs__msg__FootContactFlags),
  false,  // has_any_key_member_
  wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_member_array,  // message members
  wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_init_function,  // function to initialize message memory (memory has to be allocated)
  wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_type_support_handle = {
  0,
  &wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_members,
  get_message_typesupport_handle_function,
  &wbc_control_msgs__msg__FootContactFlags__get_type_hash,
  &wbc_control_msgs__msg__FootContactFlags__get_type_description,
  &wbc_control_msgs__msg__FootContactFlags__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_wbc_control_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, wbc_control_msgs, msg, FootContactFlags)() {
  wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_type_support_handle.typesupport_identifier) {
    wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &wbc_control_msgs__msg__FootContactFlags__rosidl_typesupport_introspection_c__FootContactFlags_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
