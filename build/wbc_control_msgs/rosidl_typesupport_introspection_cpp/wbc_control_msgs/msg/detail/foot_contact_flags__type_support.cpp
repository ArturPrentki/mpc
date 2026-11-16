// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wbc_control_msgs/msg/detail/foot_contact_flags__functions.h"
#include "wbc_control_msgs/msg/detail/foot_contact_flags__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace wbc_control_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FootContactFlags_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wbc_control_msgs::msg::FootContactFlags(_init);
}

void FootContactFlags_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wbc_control_msgs::msg::FootContactFlags *>(message_memory);
  typed_message->~FootContactFlags();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FootContactFlags_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::FootContactFlags, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fl",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::FootContactFlags, fl),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::FootContactFlags, fr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rl",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::FootContactFlags, rl),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::FootContactFlags, rr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FootContactFlags_message_members = {
  "wbc_control_msgs::msg",  // message namespace
  "FootContactFlags",  // message name
  5,  // number of fields
  sizeof(wbc_control_msgs::msg::FootContactFlags),
  false,  // has_any_key_member_
  FootContactFlags_message_member_array,  // message members
  FootContactFlags_init_function,  // function to initialize message memory (memory has to be allocated)
  FootContactFlags_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FootContactFlags_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FootContactFlags_message_members,
  get_message_typesupport_handle_function,
  &wbc_control_msgs__msg__FootContactFlags__get_type_hash,
  &wbc_control_msgs__msg__FootContactFlags__get_type_description,
  &wbc_control_msgs__msg__FootContactFlags__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace wbc_control_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wbc_control_msgs::msg::FootContactFlags>()
{
  return &::wbc_control_msgs::msg::rosidl_typesupport_introspection_cpp::FootContactFlags_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wbc_control_msgs, msg, FootContactFlags)() {
  return &::wbc_control_msgs::msg::rosidl_typesupport_introspection_cpp::FootContactFlags_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
