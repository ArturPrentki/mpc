// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/mpc_trajectory.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_HPP_

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
// Member 'steps'
#include "wbc_control_msgs/msg/detail/mpc_state_step__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wbc_control_msgs__msg__MpcTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__wbc_control_msgs__msg__MpcTrajectory __declspec(deprecated)
#endif

namespace wbc_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcTrajectory_
{
  using Type = MpcTrajectory_<ContainerAllocator>;

  explicit MpcTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MpcTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _steps_type =
    std::vector<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>>;
  _steps_type steps;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__steps(
    const std::vector<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>> & _arg)
  {
    this->steps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wbc_control_msgs__msg__MpcTrajectory
    std::shared_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wbc_control_msgs__msg__MpcTrajectory
    std::shared_ptr<wbc_control_msgs::msg::MpcTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->steps != other.steps) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcTrajectory_

// alias to use template instance with default allocator
using MpcTrajectory =
  wbc_control_msgs::msg::MpcTrajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_TRAJECTORY__STRUCT_HPP_
