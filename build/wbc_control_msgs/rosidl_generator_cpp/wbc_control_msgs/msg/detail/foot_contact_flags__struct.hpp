// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_contact_flags.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wbc_control_msgs__msg__FootContactFlags __attribute__((deprecated))
#else
# define DEPRECATED__wbc_control_msgs__msg__FootContactFlags __declspec(deprecated)
#endif

namespace wbc_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FootContactFlags_
{
  using Type = FootContactFlags_<ContainerAllocator>;

  explicit FootContactFlags_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl = false;
      this->fr = false;
      this->rl = false;
      this->rr = false;
    }
  }

  explicit FootContactFlags_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl = false;
      this->fr = false;
      this->rl = false;
      this->rr = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _fl_type =
    bool;
  _fl_type fl;
  using _fr_type =
    bool;
  _fr_type fr;
  using _rl_type =
    bool;
  _rl_type rl;
  using _rr_type =
    bool;
  _rr_type rr;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__fl(
    const bool & _arg)
  {
    this->fl = _arg;
    return *this;
  }
  Type & set__fr(
    const bool & _arg)
  {
    this->fr = _arg;
    return *this;
  }
  Type & set__rl(
    const bool & _arg)
  {
    this->rl = _arg;
    return *this;
  }
  Type & set__rr(
    const bool & _arg)
  {
    this->rr = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> *;
  using ConstRawPtr =
    const wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wbc_control_msgs__msg__FootContactFlags
    std::shared_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wbc_control_msgs__msg__FootContactFlags
    std::shared_ptr<wbc_control_msgs::msg::FootContactFlags_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootContactFlags_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->fl != other.fl) {
      return false;
    }
    if (this->fr != other.fr) {
      return false;
    }
    if (this->rl != other.rl) {
      return false;
    }
    if (this->rr != other.rr) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootContactFlags_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootContactFlags_

// alias to use template instance with default allocator
using FootContactFlags =
  wbc_control_msgs::msg::FootContactFlags_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__STRUCT_HPP_
