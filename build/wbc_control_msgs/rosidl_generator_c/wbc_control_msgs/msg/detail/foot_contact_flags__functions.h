// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from wbc_control_msgs:msg/FootContactFlags.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "wbc_control_msgs/msg/foot_contact_flags.h"


#ifndef WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__FUNCTIONS_H_
#define WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "wbc_control_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "wbc_control_msgs/msg/detail/foot_contact_flags__struct.h"

/// Initialize msg/FootContactFlags message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * wbc_control_msgs__msg__FootContactFlags
 * )) before or use
 * wbc_control_msgs__msg__FootContactFlags__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__init(wbc_control_msgs__msg__FootContactFlags * msg);

/// Finalize msg/FootContactFlags message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
void
wbc_control_msgs__msg__FootContactFlags__fini(wbc_control_msgs__msg__FootContactFlags * msg);

/// Create msg/FootContactFlags message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * wbc_control_msgs__msg__FootContactFlags__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
wbc_control_msgs__msg__FootContactFlags *
wbc_control_msgs__msg__FootContactFlags__create(void);

/// Destroy msg/FootContactFlags message.
/**
 * It calls
 * wbc_control_msgs__msg__FootContactFlags__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
void
wbc_control_msgs__msg__FootContactFlags__destroy(wbc_control_msgs__msg__FootContactFlags * msg);

/// Check for msg/FootContactFlags message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__are_equal(const wbc_control_msgs__msg__FootContactFlags * lhs, const wbc_control_msgs__msg__FootContactFlags * rhs);

/// Copy a msg/FootContactFlags message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__copy(
  const wbc_control_msgs__msg__FootContactFlags * input,
  wbc_control_msgs__msg__FootContactFlags * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
const rosidl_type_hash_t *
wbc_control_msgs__msg__FootContactFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
const rosidl_runtime_c__type_description__TypeDescription *
wbc_control_msgs__msg__FootContactFlags__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
const rosidl_runtime_c__type_description__TypeSource *
wbc_control_msgs__msg__FootContactFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
wbc_control_msgs__msg__FootContactFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/FootContactFlags messages.
/**
 * It allocates the memory for the number of elements and calls
 * wbc_control_msgs__msg__FootContactFlags__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__Sequence__init(wbc_control_msgs__msg__FootContactFlags__Sequence * array, size_t size);

/// Finalize array of msg/FootContactFlags messages.
/**
 * It calls
 * wbc_control_msgs__msg__FootContactFlags__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
void
wbc_control_msgs__msg__FootContactFlags__Sequence__fini(wbc_control_msgs__msg__FootContactFlags__Sequence * array);

/// Create array of msg/FootContactFlags messages.
/**
 * It allocates the memory for the array and calls
 * wbc_control_msgs__msg__FootContactFlags__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
wbc_control_msgs__msg__FootContactFlags__Sequence *
wbc_control_msgs__msg__FootContactFlags__Sequence__create(size_t size);

/// Destroy array of msg/FootContactFlags messages.
/**
 * It calls
 * wbc_control_msgs__msg__FootContactFlags__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
void
wbc_control_msgs__msg__FootContactFlags__Sequence__destroy(wbc_control_msgs__msg__FootContactFlags__Sequence * array);

/// Check for msg/FootContactFlags message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__Sequence__are_equal(const wbc_control_msgs__msg__FootContactFlags__Sequence * lhs, const wbc_control_msgs__msg__FootContactFlags__Sequence * rhs);

/// Copy an array of msg/FootContactFlags messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
bool
wbc_control_msgs__msg__FootContactFlags__Sequence__copy(
  const wbc_control_msgs__msg__FootContactFlags__Sequence * input,
  wbc_control_msgs__msg__FootContactFlags__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // WBC_CONTROL_MSGS__MSG__DETAIL__FOOT_CONTACT_FLAGS__FUNCTIONS_H_
