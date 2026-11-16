// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "wbc_control_msgs/msg/detail/mpc_trajectory__functions.h"
#include "wbc_control_msgs/msg/detail/mpc_trajectory__struct.hpp"
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

void MpcTrajectory_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) wbc_control_msgs::msg::MpcTrajectory(_init);
}

void MpcTrajectory_fini_function(void * message_memory)
{
  auto typed_message = static_cast<wbc_control_msgs::msg::MpcTrajectory *>(message_memory);
  typed_message->~MpcTrajectory();
}

size_t size_function__MpcTrajectory__steps(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<wbc_control_msgs::msg::MpcStateStep> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MpcTrajectory__steps(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<wbc_control_msgs::msg::MpcStateStep> *>(untyped_member);
  return &member[index];
}

void * get_function__MpcTrajectory__steps(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<wbc_control_msgs::msg::MpcStateStep> *>(untyped_member);
  return &member[index];
}

void fetch_function__MpcTrajectory__steps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const wbc_control_msgs::msg::MpcStateStep *>(
    get_const_function__MpcTrajectory__steps(untyped_member, index));
  auto & value = *reinterpret_cast<wbc_control_msgs::msg::MpcStateStep *>(untyped_value);
  value = item;
}

void assign_function__MpcTrajectory__steps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<wbc_control_msgs::msg::MpcStateStep *>(
    get_function__MpcTrajectory__steps(untyped_member, index));
  const auto & value = *reinterpret_cast<const wbc_control_msgs::msg::MpcStateStep *>(untyped_value);
  item = value;
}

void resize_function__MpcTrajectory__steps(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<wbc_control_msgs::msg::MpcStateStep> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MpcTrajectory_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::MpcTrajectory, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "steps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<wbc_control_msgs::msg::MpcStateStep>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(wbc_control_msgs::msg::MpcTrajectory, steps),  // bytes offset in struct
    nullptr,  // default value
    size_function__MpcTrajectory__steps,  // size() function pointer
    get_const_function__MpcTrajectory__steps,  // get_const(index) function pointer
    get_function__MpcTrajectory__steps,  // get(index) function pointer
    fetch_function__MpcTrajectory__steps,  // fetch(index, &value) function pointer
    assign_function__MpcTrajectory__steps,  // assign(index, value) function pointer
    resize_function__MpcTrajectory__steps  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MpcTrajectory_message_members = {
  "wbc_control_msgs::msg",  // message namespace
  "MpcTrajectory",  // message name
  2,  // number of fields
  sizeof(wbc_control_msgs::msg::MpcTrajectory),
  false,  // has_any_key_member_
  MpcTrajectory_message_member_array,  // message members
  MpcTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  MpcTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MpcTrajectory_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MpcTrajectory_message_members,
  get_message_typesupport_handle_function,
  &wbc_control_msgs__msg__MpcTrajectory__get_type_hash,
  &wbc_control_msgs__msg__MpcTrajectory__get_type_description,
  &wbc_control_msgs__msg__MpcTrajectory__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace wbc_control_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<wbc_control_msgs::msg::MpcTrajectory>()
{
  return &::wbc_control_msgs::msg::rosidl_typesupport_introspection_cpp::MpcTrajectory_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, wbc_control_msgs, msg, MpcTrajectory)() {
  return &::wbc_control_msgs::msg::rosidl_typesupport_introspection_cpp::MpcTrajectory_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
