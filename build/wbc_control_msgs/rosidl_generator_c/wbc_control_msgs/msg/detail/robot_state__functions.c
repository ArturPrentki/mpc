// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wbc_control_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "wbc_control_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `body_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `body_velocity`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `body_acceleration`
#include "geometry_msgs/msg/detail/accel__functions.h"
// Member `joint_positions`
// Member `joint_velocities`
// Member `joint_accelerations`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `feet`
#include "wbc_control_msgs/msg/detail/foot_state__functions.h"

bool
wbc_control_msgs__msg__RobotState__init(wbc_control_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // body_pose
  if (!geometry_msgs__msg__Pose__init(&msg->body_pose)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // body_velocity
  if (!geometry_msgs__msg__Twist__init(&msg->body_velocity)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // body_acceleration
  if (!geometry_msgs__msg__Accel__init(&msg->body_acceleration)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_positions, 0)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_velocities, 0)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__init(&msg->joint_accelerations, 0)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // feet
  if (!wbc_control_msgs__msg__FootState__Sequence__init(&msg->feet, 0)) {
    wbc_control_msgs__msg__RobotState__fini(msg);
    return false;
  }
  return true;
}

void
wbc_control_msgs__msg__RobotState__fini(wbc_control_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // body_pose
  geometry_msgs__msg__Pose__fini(&msg->body_pose);
  // body_velocity
  geometry_msgs__msg__Twist__fini(&msg->body_velocity);
  // body_acceleration
  geometry_msgs__msg__Accel__fini(&msg->body_acceleration);
  // joint_positions
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_positions);
  // joint_velocities
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_velocities);
  // joint_accelerations
  rosidl_runtime_c__double__Sequence__fini(&msg->joint_accelerations);
  // feet
  wbc_control_msgs__msg__FootState__Sequence__fini(&msg->feet);
}

bool
wbc_control_msgs__msg__RobotState__are_equal(const wbc_control_msgs__msg__RobotState * lhs, const wbc_control_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // body_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->body_pose), &(rhs->body_pose)))
  {
    return false;
  }
  // body_velocity
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->body_velocity), &(rhs->body_velocity)))
  {
    return false;
  }
  // body_acceleration
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->body_acceleration), &(rhs->body_acceleration)))
  {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_positions), &(rhs->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_velocities), &(rhs->joint_velocities)))
  {
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->joint_accelerations), &(rhs->joint_accelerations)))
  {
    return false;
  }
  // feet
  if (!wbc_control_msgs__msg__FootState__Sequence__are_equal(
      &(lhs->feet), &(rhs->feet)))
  {
    return false;
  }
  return true;
}

bool
wbc_control_msgs__msg__RobotState__copy(
  const wbc_control_msgs__msg__RobotState * input,
  wbc_control_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // body_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->body_pose), &(output->body_pose)))
  {
    return false;
  }
  // body_velocity
  if (!geometry_msgs__msg__Twist__copy(
      &(input->body_velocity), &(output->body_velocity)))
  {
    return false;
  }
  // body_acceleration
  if (!geometry_msgs__msg__Accel__copy(
      &(input->body_acceleration), &(output->body_acceleration)))
  {
    return false;
  }
  // joint_positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_positions), &(output->joint_positions)))
  {
    return false;
  }
  // joint_velocities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_velocities), &(output->joint_velocities)))
  {
    return false;
  }
  // joint_accelerations
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->joint_accelerations), &(output->joint_accelerations)))
  {
    return false;
  }
  // feet
  if (!wbc_control_msgs__msg__FootState__Sequence__copy(
      &(input->feet), &(output->feet)))
  {
    return false;
  }
  return true;
}

wbc_control_msgs__msg__RobotState *
wbc_control_msgs__msg__RobotState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__RobotState * msg = (wbc_control_msgs__msg__RobotState *)allocator.allocate(sizeof(wbc_control_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wbc_control_msgs__msg__RobotState));
  bool success = wbc_control_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wbc_control_msgs__msg__RobotState__destroy(wbc_control_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wbc_control_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wbc_control_msgs__msg__RobotState__Sequence__init(wbc_control_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__RobotState * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(wbc_control_msgs__msg__RobotState)) {
      return false;
    }
    data = (wbc_control_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(wbc_control_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wbc_control_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wbc_control_msgs__msg__RobotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
wbc_control_msgs__msg__RobotState__Sequence__fini(wbc_control_msgs__msg__RobotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      wbc_control_msgs__msg__RobotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

wbc_control_msgs__msg__RobotState__Sequence *
wbc_control_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__RobotState__Sequence * array = (wbc_control_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(wbc_control_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wbc_control_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wbc_control_msgs__msg__RobotState__Sequence__destroy(wbc_control_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wbc_control_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wbc_control_msgs__msg__RobotState__Sequence__are_equal(const wbc_control_msgs__msg__RobotState__Sequence * lhs, const wbc_control_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wbc_control_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wbc_control_msgs__msg__RobotState__Sequence__copy(
  const wbc_control_msgs__msg__RobotState__Sequence * input,
  wbc_control_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(wbc_control_msgs__msg__RobotState)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(wbc_control_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wbc_control_msgs__msg__RobotState * data =
      (wbc_control_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wbc_control_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wbc_control_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wbc_control_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
