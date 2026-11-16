// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice
#include "wbc_control_msgs/msg/detail/foot_contact_flags__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
wbc_control_msgs__msg__FootContactFlags__init(wbc_control_msgs__msg__FootContactFlags * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    wbc_control_msgs__msg__FootContactFlags__fini(msg);
    return false;
  }
  // fl
  // fr
  // rl
  // rr
  return true;
}

void
wbc_control_msgs__msg__FootContactFlags__fini(wbc_control_msgs__msg__FootContactFlags * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // fl
  // fr
  // rl
  // rr
}

bool
wbc_control_msgs__msg__FootContactFlags__are_equal(const wbc_control_msgs__msg__FootContactFlags * lhs, const wbc_control_msgs__msg__FootContactFlags * rhs)
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
  // fl
  if (lhs->fl != rhs->fl) {
    return false;
  }
  // fr
  if (lhs->fr != rhs->fr) {
    return false;
  }
  // rl
  if (lhs->rl != rhs->rl) {
    return false;
  }
  // rr
  if (lhs->rr != rhs->rr) {
    return false;
  }
  return true;
}

bool
wbc_control_msgs__msg__FootContactFlags__copy(
  const wbc_control_msgs__msg__FootContactFlags * input,
  wbc_control_msgs__msg__FootContactFlags * output)
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
  // fl
  output->fl = input->fl;
  // fr
  output->fr = input->fr;
  // rl
  output->rl = input->rl;
  // rr
  output->rr = input->rr;
  return true;
}

wbc_control_msgs__msg__FootContactFlags *
wbc_control_msgs__msg__FootContactFlags__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootContactFlags * msg = (wbc_control_msgs__msg__FootContactFlags *)allocator.allocate(sizeof(wbc_control_msgs__msg__FootContactFlags), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wbc_control_msgs__msg__FootContactFlags));
  bool success = wbc_control_msgs__msg__FootContactFlags__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wbc_control_msgs__msg__FootContactFlags__destroy(wbc_control_msgs__msg__FootContactFlags * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wbc_control_msgs__msg__FootContactFlags__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wbc_control_msgs__msg__FootContactFlags__Sequence__init(wbc_control_msgs__msg__FootContactFlags__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootContactFlags * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(wbc_control_msgs__msg__FootContactFlags)) {
      return false;
    }
    data = (wbc_control_msgs__msg__FootContactFlags *)allocator.zero_allocate(size, sizeof(wbc_control_msgs__msg__FootContactFlags), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wbc_control_msgs__msg__FootContactFlags__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wbc_control_msgs__msg__FootContactFlags__fini(&data[i - 1]);
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
wbc_control_msgs__msg__FootContactFlags__Sequence__fini(wbc_control_msgs__msg__FootContactFlags__Sequence * array)
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
      wbc_control_msgs__msg__FootContactFlags__fini(&array->data[i]);
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

wbc_control_msgs__msg__FootContactFlags__Sequence *
wbc_control_msgs__msg__FootContactFlags__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootContactFlags__Sequence * array = (wbc_control_msgs__msg__FootContactFlags__Sequence *)allocator.allocate(sizeof(wbc_control_msgs__msg__FootContactFlags__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wbc_control_msgs__msg__FootContactFlags__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wbc_control_msgs__msg__FootContactFlags__Sequence__destroy(wbc_control_msgs__msg__FootContactFlags__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wbc_control_msgs__msg__FootContactFlags__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wbc_control_msgs__msg__FootContactFlags__Sequence__are_equal(const wbc_control_msgs__msg__FootContactFlags__Sequence * lhs, const wbc_control_msgs__msg__FootContactFlags__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wbc_control_msgs__msg__FootContactFlags__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wbc_control_msgs__msg__FootContactFlags__Sequence__copy(
  const wbc_control_msgs__msg__FootContactFlags__Sequence * input,
  wbc_control_msgs__msg__FootContactFlags__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(wbc_control_msgs__msg__FootContactFlags)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(wbc_control_msgs__msg__FootContactFlags);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wbc_control_msgs__msg__FootContactFlags * data =
      (wbc_control_msgs__msg__FootContactFlags *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wbc_control_msgs__msg__FootContactFlags__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wbc_control_msgs__msg__FootContactFlags__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wbc_control_msgs__msg__FootContactFlags__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
