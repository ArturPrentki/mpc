// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice
#include "wbc_control_msgs/msg/detail/foot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `acceleration`
// Member `contact_force`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
wbc_control_msgs__msg__FootState__init(wbc_control_msgs__msg__FootState * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    wbc_control_msgs__msg__FootState__fini(msg);
    return false;
  }
  // is_contact
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    wbc_control_msgs__msg__FootState__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    wbc_control_msgs__msg__FootState__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    wbc_control_msgs__msg__FootState__fini(msg);
    return false;
  }
  // contact_force
  if (!geometry_msgs__msg__Vector3__init(&msg->contact_force)) {
    wbc_control_msgs__msg__FootState__fini(msg);
    return false;
  }
  return true;
}

void
wbc_control_msgs__msg__FootState__fini(wbc_control_msgs__msg__FootState * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // is_contact
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // contact_force
  geometry_msgs__msg__Vector3__fini(&msg->contact_force);
}

bool
wbc_control_msgs__msg__FootState__are_equal(const wbc_control_msgs__msg__FootState * lhs, const wbc_control_msgs__msg__FootState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // is_contact
  if (lhs->is_contact != rhs->is_contact) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // contact_force
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->contact_force), &(rhs->contact_force)))
  {
    return false;
  }
  return true;
}

bool
wbc_control_msgs__msg__FootState__copy(
  const wbc_control_msgs__msg__FootState * input,
  wbc_control_msgs__msg__FootState * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // is_contact
  output->is_contact = input->is_contact;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // contact_force
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->contact_force), &(output->contact_force)))
  {
    return false;
  }
  return true;
}

wbc_control_msgs__msg__FootState *
wbc_control_msgs__msg__FootState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootState * msg = (wbc_control_msgs__msg__FootState *)allocator.allocate(sizeof(wbc_control_msgs__msg__FootState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(wbc_control_msgs__msg__FootState));
  bool success = wbc_control_msgs__msg__FootState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
wbc_control_msgs__msg__FootState__destroy(wbc_control_msgs__msg__FootState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    wbc_control_msgs__msg__FootState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
wbc_control_msgs__msg__FootState__Sequence__init(wbc_control_msgs__msg__FootState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootState * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(wbc_control_msgs__msg__FootState)) {
      return false;
    }
    data = (wbc_control_msgs__msg__FootState *)allocator.zero_allocate(size, sizeof(wbc_control_msgs__msg__FootState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = wbc_control_msgs__msg__FootState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        wbc_control_msgs__msg__FootState__fini(&data[i - 1]);
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
wbc_control_msgs__msg__FootState__Sequence__fini(wbc_control_msgs__msg__FootState__Sequence * array)
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
      wbc_control_msgs__msg__FootState__fini(&array->data[i]);
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

wbc_control_msgs__msg__FootState__Sequence *
wbc_control_msgs__msg__FootState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  wbc_control_msgs__msg__FootState__Sequence * array = (wbc_control_msgs__msg__FootState__Sequence *)allocator.allocate(sizeof(wbc_control_msgs__msg__FootState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = wbc_control_msgs__msg__FootState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
wbc_control_msgs__msg__FootState__Sequence__destroy(wbc_control_msgs__msg__FootState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    wbc_control_msgs__msg__FootState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
wbc_control_msgs__msg__FootState__Sequence__are_equal(const wbc_control_msgs__msg__FootState__Sequence * lhs, const wbc_control_msgs__msg__FootState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!wbc_control_msgs__msg__FootState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
wbc_control_msgs__msg__FootState__Sequence__copy(
  const wbc_control_msgs__msg__FootState__Sequence * input,
  wbc_control_msgs__msg__FootState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(wbc_control_msgs__msg__FootState)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(wbc_control_msgs__msg__FootState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    wbc_control_msgs__msg__FootState * data =
      (wbc_control_msgs__msg__FootState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!wbc_control_msgs__msg__FootState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          wbc_control_msgs__msg__FootState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!wbc_control_msgs__msg__FootState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
