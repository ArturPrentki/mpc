// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from wbc_control_msgs:msg/FootState.idl
// generated code does not contain a copyright notice

#include "wbc_control_msgs/msg/detail/foot_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_wbc_control_msgs
const rosidl_type_hash_t *
wbc_control_msgs__msg__FootState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0xa3, 0xb9, 0x56, 0x58, 0xb3, 0xc6, 0xe3,
      0x9a, 0xe0, 0xe1, 0x21, 0xd3, 0xbb, 0xc5, 0xe9,
      0x1a, 0xb7, 0x40, 0x36, 0xa8, 0x97, 0x92, 0x05,
      0x63, 0x07, 0x38, 0x26, 0xa9, 0x5f, 0x91, 0x5b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/vector3__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Vector3__EXPECTED_HASH = {1, {
    0xcc, 0x12, 0xfe, 0x83, 0xe4, 0xc0, 0x27, 0x19,
    0xf1, 0xce, 0x80, 0x70, 0xbf, 0xd1, 0x4a, 0xec,
    0xd4, 0x0f, 0x75, 0xa9, 0x66, 0x96, 0xa6, 0x7a,
    0x2a, 0x1f, 0x37, 0xf7, 0xdb, 0xb0, 0x76, 0x5d,
  }};
#endif

static char wbc_control_msgs__msg__FootState__TYPE_NAME[] = "wbc_control_msgs/msg/FootState";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Vector3__TYPE_NAME[] = "geometry_msgs/msg/Vector3";

// Define type names, field names, and default values
static char wbc_control_msgs__msg__FootState__FIELD_NAME__name[] = "name";
static char wbc_control_msgs__msg__FootState__FIELD_NAME__is_contact[] = "is_contact";
static char wbc_control_msgs__msg__FootState__FIELD_NAME__position[] = "position";
static char wbc_control_msgs__msg__FootState__FIELD_NAME__velocity[] = "velocity";
static char wbc_control_msgs__msg__FootState__FIELD_NAME__acceleration[] = "acceleration";
static char wbc_control_msgs__msg__FootState__FIELD_NAME__contact_force[] = "contact_force";

static rosidl_runtime_c__type_description__Field wbc_control_msgs__msg__FootState__FIELDS[] = {
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__is_contact, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {wbc_control_msgs__msg__FootState__FIELD_NAME__contact_force, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription wbc_control_msgs__msg__FootState__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Vector3__TYPE_NAME, 25, 25},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
wbc_control_msgs__msg__FootState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {wbc_control_msgs__msg__FootState__TYPE_NAME, 30, 30},
      {wbc_control_msgs__msg__FootState__FIELDS, 6, 6},
    },
    {wbc_control_msgs__msg__FootState__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Vector3__EXPECTED_HASH, geometry_msgs__msg__Vector3__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Vector3__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string name                         # Foot identifier (\"FL\", \"FR\", \"RL\", \"RR\")\n"
  "bool is_contact                     # Contact state (true = stance, false = swing)\n"
  "geometry_msgs/Point position        # Foot target position (World Frame) ????\n"
  "geometry_msgs/Vector3 velocity       # Foot target velocity\n"
  "geometry_msgs/Vector3 acceleration    # Target XYZ acceleration (optional, but good for WBC)\n"
  "geometry_msgs/Vector3 contact_force  # Optimized ground reaction force (Fx, Fy, Fz)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
wbc_control_msgs__msg__FootState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {wbc_control_msgs__msg__FootState__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 476, 476},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
wbc_control_msgs__msg__FootState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *wbc_control_msgs__msg__FootState__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Vector3__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
