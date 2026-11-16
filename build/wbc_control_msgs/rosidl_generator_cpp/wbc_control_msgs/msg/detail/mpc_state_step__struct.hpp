// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from wbc_control_msgs:msg/MpcStateStep.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/mpc_state_step.hpp"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__STRUCT_HPP_
#define WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'time_offset'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'body_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'body_velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'body_acceleration'
#include "geometry_msgs/msg/detail/accel__struct.hpp"
// Member 'feet'
#include "wbc_control_msgs/msg/detail/foot_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__wbc_control_msgs__msg__MpcStateStep __attribute__((deprecated))
#else
# define DEPRECATED__wbc_control_msgs__msg__MpcStateStep __declspec(deprecated)
#endif

namespace wbc_control_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MpcStateStep_
{
  using Type = MpcStateStep_<ContainerAllocator>;

  explicit MpcStateStep_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_offset(_init),
    body_pose(_init),
    body_velocity(_init),
    body_acceleration(_init)
  {
    (void)_init;
  }

  explicit MpcStateStep_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : time_offset(_alloc, _init),
    body_pose(_alloc, _init),
    body_velocity(_alloc, _init),
    body_acceleration(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _time_offset_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _time_offset_type time_offset;
  using _body_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _body_pose_type body_pose;
  using _body_velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _body_velocity_type body_velocity;
  using _body_acceleration_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _body_acceleration_type body_acceleration;
  using _joint_positions_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_positions_type joint_positions;
  using _joint_velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_velocities_type joint_velocities;
  using _joint_accelerations_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_accelerations_type joint_accelerations;
  using _feet_type =
    std::vector<wbc_control_msgs::msg::FootState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<wbc_control_msgs::msg::FootState_<ContainerAllocator>>>;
  _feet_type feet;

  // setters for named parameter idiom
  Type & set__time_offset(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->time_offset = _arg;
    return *this;
  }
  Type & set__body_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->body_pose = _arg;
    return *this;
  }
  Type & set__body_velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->body_velocity = _arg;
    return *this;
  }
  Type & set__body_acceleration(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->body_acceleration = _arg;
    return *this;
  }
  Type & set__joint_positions(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_positions = _arg;
    return *this;
  }
  Type & set__joint_velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_velocities = _arg;
    return *this;
  }
  Type & set__joint_accelerations(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_accelerations = _arg;
    return *this;
  }
  Type & set__feet(
    const std::vector<wbc_control_msgs::msg::FootState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<wbc_control_msgs::msg::FootState_<ContainerAllocator>>> & _arg)
  {
    this->feet = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> *;
  using ConstRawPtr =
    const wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__wbc_control_msgs__msg__MpcStateStep
    std::shared_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__wbc_control_msgs__msg__MpcStateStep
    std::shared_ptr<wbc_control_msgs::msg::MpcStateStep_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MpcStateStep_ & other) const
  {
    if (this->time_offset != other.time_offset) {
      return false;
    }
    if (this->body_pose != other.body_pose) {
      return false;
    }
    if (this->body_velocity != other.body_velocity) {
      return false;
    }
    if (this->body_acceleration != other.body_acceleration) {
      return false;
    }
    if (this->joint_positions != other.joint_positions) {
      return false;
    }
    if (this->joint_velocities != other.joint_velocities) {
      return false;
    }
    if (this->joint_accelerations != other.joint_accelerations) {
      return false;
    }
    if (this->feet != other.feet) {
      return false;
    }
    return true;
  }
  bool operator!=(const MpcStateStep_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MpcStateStep_

// alias to use template instance with default allocator
using MpcStateStep =
  wbc_control_msgs::msg::MpcStateStep_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace wbc_control_msgs

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__MPC_STATE_STEP__STRUCT_HPP_
