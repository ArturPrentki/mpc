#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "wbc_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__FootContactFlags() -> *const std::ffi::c_void;
}

#[link(name = "wbc_control_msgs__rosidl_generator_c")]
extern "C" {
    fn wbc_control_msgs__msg__FootContactFlags__init(msg: *mut FootContactFlags) -> bool;
    fn wbc_control_msgs__msg__FootContactFlags__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FootContactFlags>, size: usize) -> bool;
    fn wbc_control_msgs__msg__FootContactFlags__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FootContactFlags>);
    fn wbc_control_msgs__msg__FootContactFlags__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FootContactFlags>, out_seq: *mut rosidl_runtime_rs::Sequence<FootContactFlags>) -> bool;
}

// Corresponds to wbc_control_msgs__msg__FootContactFlags
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// FootContactFlags.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootContactFlags {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fl: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub fr: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rl: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rr: bool,

}



impl Default for FootContactFlags {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wbc_control_msgs__msg__FootContactFlags__init(&mut msg as *mut _) {
        panic!("Call to wbc_control_msgs__msg__FootContactFlags__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FootContactFlags {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootContactFlags__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootContactFlags__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootContactFlags__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FootContactFlags {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FootContactFlags where Self: Sized {
  const TYPE_NAME: &'static str = "wbc_control_msgs/msg/FootContactFlags";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__FootContactFlags() }
  }
}


#[link(name = "wbc_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__FootState() -> *const std::ffi::c_void;
}

#[link(name = "wbc_control_msgs__rosidl_generator_c")]
extern "C" {
    fn wbc_control_msgs__msg__FootState__init(msg: *mut FootState) -> bool;
    fn wbc_control_msgs__msg__FootState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FootState>, size: usize) -> bool;
    fn wbc_control_msgs__msg__FootState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FootState>);
    fn wbc_control_msgs__msg__FootState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FootState>, out_seq: *mut rosidl_runtime_rs::Sequence<FootState>) -> bool;
}

// Corresponds to wbc_control_msgs__msg__FootState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootState {
    /// Foot identifier ("FL", "FR", "RL", "RR")
    pub name: rosidl_runtime_rs::String,

    /// Contact state (true = stance, false = swing)
    pub is_contact: bool,

    /// Foot target position (World Frame) ????
    pub position: geometry_msgs::msg::rmw::Point,

    /// Foot target velocity
    pub velocity: geometry_msgs::msg::rmw::Vector3,

    /// Target XYZ acceleration (optional, but good for WBC)
    pub acceleration: geometry_msgs::msg::rmw::Vector3,

    /// Optimized ground reaction force (Fx, Fy, Fz)
    pub contact_force: geometry_msgs::msg::rmw::Vector3,

}



impl Default for FootState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wbc_control_msgs__msg__FootState__init(&mut msg as *mut _) {
        panic!("Call to wbc_control_msgs__msg__FootState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FootState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__FootState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FootState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FootState where Self: Sized {
  const TYPE_NAME: &'static str = "wbc_control_msgs/msg/FootState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__FootState() }
  }
}


#[link(name = "wbc_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__MpcStateStep() -> *const std::ffi::c_void;
}

#[link(name = "wbc_control_msgs__rosidl_generator_c")]
extern "C" {
    fn wbc_control_msgs__msg__MpcStateStep__init(msg: *mut MpcStateStep) -> bool;
    fn wbc_control_msgs__msg__MpcStateStep__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MpcStateStep>, size: usize) -> bool;
    fn wbc_control_msgs__msg__MpcStateStep__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MpcStateStep>);
    fn wbc_control_msgs__msg__MpcStateStep__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MpcStateStep>, out_seq: *mut rosidl_runtime_rs::Sequence<MpcStateStep>) -> bool;
}

// Corresponds to wbc_control_msgs__msg__MpcStateStep
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// msg/MpcStateStep.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MpcStateStep {
    /// Time delta/offset relative to trajectory start
    pub time_offset: builtin_interfaces::msg::rmw::Time,

    /// Body SE3 Pose (Translation + Quaternion Orientation)
    pub body_pose: geometry_msgs::msg::rmw::Pose,

    /// Pinocchio::Motion Spatial Velocity (Linear + Angular)
    pub body_velocity: geometry_msgs::msg::rmw::Twist,

    /// Pinocchio::Motion Spatial Acceleration (Linear + Angular)
    pub body_acceleration: geometry_msgs::msg::rmw::Accel,

    /// q
    pub joint_positions: rosidl_runtime_rs::Sequence<f64>,

    /// v
    pub joint_velocities: rosidl_runtime_rs::Sequence<f64>,

    /// a
    pub joint_accelerations: rosidl_runtime_rs::Sequence<f64>,

    /// Foot target states and optimized contact forces
    pub feet: rosidl_runtime_rs::Sequence<super::super::msg::rmw::FootState>,

}



impl Default for MpcStateStep {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wbc_control_msgs__msg__MpcStateStep__init(&mut msg as *mut _) {
        panic!("Call to wbc_control_msgs__msg__MpcStateStep__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MpcStateStep {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcStateStep__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcStateStep__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcStateStep__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MpcStateStep {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MpcStateStep where Self: Sized {
  const TYPE_NAME: &'static str = "wbc_control_msgs/msg/MpcStateStep";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__MpcStateStep() }
  }
}


#[link(name = "wbc_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__MpcTrajectory() -> *const std::ffi::c_void;
}

#[link(name = "wbc_control_msgs__rosidl_generator_c")]
extern "C" {
    fn wbc_control_msgs__msg__MpcTrajectory__init(msg: *mut MpcTrajectory) -> bool;
    fn wbc_control_msgs__msg__MpcTrajectory__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MpcTrajectory>, size: usize) -> bool;
    fn wbc_control_msgs__msg__MpcTrajectory__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MpcTrajectory>);
    fn wbc_control_msgs__msg__MpcTrajectory__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MpcTrajectory>, out_seq: *mut rosidl_runtime_rs::Sequence<MpcTrajectory>) -> bool;
}

// Corresponds to wbc_control_msgs__msg__MpcTrajectory
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MpcTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// Full sequence of trajectory steps over the MPC horizon
    pub steps: rosidl_runtime_rs::Sequence<super::super::msg::rmw::MpcStateStep>,

}



impl Default for MpcTrajectory {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wbc_control_msgs__msg__MpcTrajectory__init(&mut msg as *mut _) {
        panic!("Call to wbc_control_msgs__msg__MpcTrajectory__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MpcTrajectory {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcTrajectory__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcTrajectory__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__MpcTrajectory__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MpcTrajectory {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MpcTrajectory where Self: Sized {
  const TYPE_NAME: &'static str = "wbc_control_msgs/msg/MpcTrajectory";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__MpcTrajectory() }
  }
}


#[link(name = "wbc_control_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__RobotState() -> *const std::ffi::c_void;
}

#[link(name = "wbc_control_msgs__rosidl_generator_c")]
extern "C" {
    fn wbc_control_msgs__msg__RobotState__init(msg: *mut RobotState) -> bool;
    fn wbc_control_msgs__msg__RobotState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotState>, size: usize) -> bool;
    fn wbc_control_msgs__msg__RobotState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotState>);
    fn wbc_control_msgs__msg__RobotState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotState>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotState>) -> bool;
}

// Corresponds to wbc_control_msgs__msg__RobotState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// msg/RobotState.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotState {
    /// Body SE3 Pose (Translation + Quaternion Orientation)
    pub body_pose: geometry_msgs::msg::rmw::Pose,

    /// Pinocchio::Motion Spatial Velocity (Linear + Angular)
    pub body_velocity: geometry_msgs::msg::rmw::Twist,

    /// Pinocchio::Motion Spatial Acceleration (Linear + Angular)
    pub body_acceleration: geometry_msgs::msg::rmw::Accel,

    /// q
    pub joint_positions: rosidl_runtime_rs::Sequence<f64>,

    /// v
    pub joint_velocities: rosidl_runtime_rs::Sequence<f64>,

    /// a
    pub joint_accelerations: rosidl_runtime_rs::Sequence<f64>,

    /// Foot target states and optimized contact forces
    pub feet: rosidl_runtime_rs::Sequence<super::super::msg::rmw::FootState>,

}



impl Default for RobotState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wbc_control_msgs__msg__RobotState__init(&mut msg as *mut _) {
        panic!("Call to wbc_control_msgs__msg__RobotState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__RobotState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__RobotState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wbc_control_msgs__msg__RobotState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotState where Self: Sized {
  const TYPE_NAME: &'static str = "wbc_control_msgs/msg/RobotState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wbc_control_msgs__msg__RobotState() }
  }
}


