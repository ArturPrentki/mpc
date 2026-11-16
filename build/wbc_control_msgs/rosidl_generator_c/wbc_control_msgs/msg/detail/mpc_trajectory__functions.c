// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wbc_control_msgs:msg/MpcTrajectory.idl
// generated code does not contain a copyright notice
#include "wbc_control_msgs/msg/detail/mpc_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `steps`
#include "wbc_control_msgs/msg/detail/mpc_state_step__functions.h"

bool
wbc_control_msgs__msg__MpcTrajectory__init(wbc_control_msgs__msg__MpcTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wbc_control_msgs__msg__MpcTrajectory__fini(msg);
    return false;
  }
  // steps
  if (!wbc_control_msgs__msg__MpcStateStep__Sequence__init(&msg->steps, 0)) {
    wbc_control_msgs__msg__MpcTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
wbc_control_msgs__msg__MpcTrajectory__fini(wbc_control_msgs__msg__MpcTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // steps
  wbc_control_msgs__msg__MpcStateStep__Sequence__fini(&msg->steps);
}

bool
wbc_control_msgs__msg__MpcTrajectory__are_equal(const wbc_control_msgs__msg__MpcTrajectory * lhs, const wbc_control_msgs__msg__MpcTrajectory * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // steps
  if (!wbc_control_msgs__msg__MpcStateStep__Sequence__are_equal(
      &(lhs->steps), &(rhs->steps)))
  {
    return false;
  }
  return true;
}

bool
wbc_control_msgs__msg__MpcTrajectory__copy(
  const wbc_control_msgs__msg__MpcTrajectory * input,
  wbc_control_msgs__msg__MpcTrajectory * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // steps
  if (!wbc_control_msgs__msg__MpcStateStep__Sequence__copy(
      &(input->steps), &(output->steps)))
  {
    return false;
  }
  return true;
}

wbc_control_msgs__msg__MpcTrajectory *
wbc_control_msgs__msg__MpcTrajectory__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__MpcTrajectory * msg = (wbc_control_msgs__msg__MpcTrajectory *)allocator.allocate(sizeof(wbc_control_msgs__msg__MpcTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wbc_control_msgs__msg__MpcTrajectory));
  bool success = wbc_control_msgs__msg__MpcTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wbc_control_msgs__msg__MpcTrajectory__destroy(wbc_control_msgs__msg__MpcTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wbc_control_msgs__msg__MpcTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wbc_control_msgs__msg__MpcTrajectory__Sequence__init(wbc_control_msgs__msg__MpcTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__MpcTrajectory * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(wbc_control_msgs__msg__MpcTrajectory)) {
      return false;
    }
    data = (wbc_control_msgs__msg__MpcTrajectory *)allocator.zero_allocate(size, sizeof(wbc_control_msgs__msg__MpcTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wbc_control_msgs__msg__MpcTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wbc_control_msgs__msg__MpcTrajectory__fini(&data[i - 1]);
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
wbc_control_msgs__msg__MpcTrajectory__Sequence__fini(wbc_control_msgs__msg__MpcTrajectory__Sequence * array)
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
      wbc_control_msgs__msg__MpcTrajectory__fini(&array->data[i]);
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

wbc_control_msgs__msg__MpcTrajectory__Sequence *
wbc_control_msgs__msg__MpcTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__MpcTrajectory__Sequence * array = (wbc_control_msgs__msg__MpcTrajectory__Sequence *)allocator.allocate(sizeof(wbc_control_msgs__msg__MpcTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wbc_control_msgs__msg__MpcTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wbc_control_msgs__msg__MpcTrajectory__Sequence__destroy(wbc_control_msgs__msg__MpcTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wbc_control_msgs__msg__MpcTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wbc_control_msgs__msg__MpcTrajectory__Sequence__are_equal(const wbc_control_msgs__msg__MpcTrajectory__Sequence * lhs, const wbc_control_msgs__msg__MpcTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wbc_control_msgs__msg__MpcTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wbc_control_msgs__msg__MpcTrajectory__Sequence__copy(
  const wbc_control_msgs__msg__MpcTrajectory__Sequence * input,
  wbc_control_msgs__msg__MpcTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(wbc_control_msgs__msg__MpcTrajectory)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(wbc_control_msgs__msg__MpcTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wbc_control_msgs__msg__MpcTrajectory * data =
      (wbc_control_msgs__msg__MpcTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wbc_control_msgs__msg__MpcTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wbc_control_msgs__msg__MpcTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wbc_control_msgs__msg__MpcTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
