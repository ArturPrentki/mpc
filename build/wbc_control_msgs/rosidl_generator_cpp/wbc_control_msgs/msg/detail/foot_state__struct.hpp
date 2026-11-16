// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_state.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'acceleration'
// Member 'contact_force'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wbc_control_msgs__msg__FootState __attribute__((deprecated))
#else
# define DEPRECATED__wbc_control_msgs__msg__FootState __declspec(deprecated)
#endif

namespace wbc_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FootState_
{
  using Type = FootState_<ContainerAllocator>;

  explicit FootState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    velocity(_init),
    acceleration(_init),
    contact_force(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->is_contact = false;
    }
  }

  explicit FootState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    position(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init),
    contact_force(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->is_contact = false;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _is_contact_type =
    bool;
  _is_contact_type is_contact;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _contact_force_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _contact_force_type contact_force;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__is_contact(
    const bool & _arg)
  {
    this->is_contact = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__contact_force(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->contact_force = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wbc_control_msgs::msg::FootState_<ContainerAllocator> *;
  using ConstRawPtr =
    const wbc_control_msgs::msg::FootState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::FootState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::FootState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wbc_control_msgs__msg__FootState
    std::shared_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wbc_control_msgs__msg__FootState
    std::shared_ptr<wbc_control_msgs::msg::FootState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootState_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->is_contact != other.is_contact) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->contact_force != other.contact_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootState_

// alias to use template instance with default allocator
using FootState =
  wbc_control_msgs::msg::FootState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_STATE__STRUCT_HPP_
