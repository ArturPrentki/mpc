#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to wbc_control_msgs__msg__FootContactFlags
/// FootContactFlags.msg

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootContactFlags {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FootContactFlags::default())
  }
}

impl rosidl_runtime_rs::Message for FootContactFlags {
  type RmwMsg = super::msg::rmw::FootContactFlags;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        fl: msg.fl,
        fr: msg.fr,
        rl: msg.rl,
        rr: msg.rr,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      fl: msg.fl,
      fr: msg.fr,
      rl: msg.rl,
      rr: msg.rr,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      fl: msg.fl,
      fr: msg.fr,
      rl: msg.rl,
      rr: msg.rr,
    }
  }
}


// Corresponds to wbc_control_msgs__msg__FootState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FootState {
    /// Foot identifier ("FL", "FR", "RL", "RR")
    pub name: std::string::String,

    /// Contact state (true = stance, false = swing)
    pub is_contact: bool,

    /// Foot target position (World Frame) ????
    pub position: geometry_msgs::msg::Point,

    /// Foot target velocity
    pub velocity: geometry_msgs::msg::Vector3,

    /// Target XYZ acceleration (optional, but good for WBC)
    pub acceleration: geometry_msgs::msg::Vector3,

    /// Optimized ground reaction force (Fx, Fy, Fz)
    pub contact_force: geometry_msgs::msg::Vector3,

}



impl Default for FootState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::FootState::default())
  }
}

impl rosidl_runtime_rs::Message for FootState {
  type RmwMsg = super::msg::rmw::FootState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
        is_contact: msg.is_contact,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.acceleration)).into_owned(),
        contact_force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.contact_force)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        name: msg.name.as_str().into(),
      is_contact: msg.is_contact,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.acceleration)).into_owned(),
        contact_force: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.contact_force)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      name: msg.name.to_string(),
      is_contact: msg.is_contact,
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      acceleration: geometry_msgs::msg::Vector3::from_rmw_message(msg.acceleration),
      contact_force: geometry_msgs::msg::Vector3::from_rmw_message(msg.contact_force),
    }
  }
}


// Corresponds to wbc_control_msgs__msg__MpcStateStep
/// msg/MpcStateStep.msg

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MpcStateStep {
    /// Time delta/offset relative to trajectory start
    pub time_offset: builtin_interfaces::msg::Time,

    /// Body SE3 Pose (Translation + Quaternion Orientation)
    pub body_pose: geometry_msgs::msg::Pose,

    /// Pinocchio::Motion Spatial Velocity (Linear + Angular)
    pub body_velocity: geometry_msgs::msg::Twist,

    /// Pinocchio::Motion Spatial Acceleration (Linear + Angular)
    pub body_acceleration: geometry_msgs::msg::Accel,

    /// q
    pub joint_positions: Vec<f64>,

    /// v
    pub joint_velocities: Vec<f64>,

    /// a
    pub joint_accelerations: Vec<f64>,

    /// Foot target states and optimized contact forces
    pub feet: Vec<super::msg::FootState>,

}



impl Default for MpcStateStep {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MpcStateStep::default())
  }
}

impl rosidl_runtime_rs::Message for MpcStateStep {
  type RmwMsg = super::msg::rmw::MpcStateStep;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_offset: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.time_offset)).into_owned(),
        body_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.body_pose)).into_owned(),
        body_velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.body_velocity)).into_owned(),
        body_acceleration: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Owned(msg.body_acceleration)).into_owned(),
        joint_positions: msg.joint_positions.as_slice().into(),
        joint_velocities: msg.joint_velocities.as_slice().into(),
        joint_accelerations: msg.joint_accelerations.as_slice().into(),
        feet: msg.feet
          .into_iter()
          .map(|elem| super::msg::FootState::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_offset: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.time_offset)).into_owned(),
        body_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_pose)).into_owned(),
        body_velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_velocity)).into_owned(),
        body_acceleration: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_acceleration)).into_owned(),
        joint_positions: msg.joint_positions.as_slice().into(),
        joint_velocities: msg.joint_velocities.as_slice().into(),
        joint_accelerations: msg.joint_accelerations.as_slice().into(),
        feet: msg.feet
          .iter()
          .map(|elem| super::msg::FootState::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_offset: builtin_interfaces::msg::Time::from_rmw_message(msg.time_offset),
      body_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.body_pose),
      body_velocity: geometry_msgs::msg::Twist::from_rmw_message(msg.body_velocity),
      body_acceleration: geometry_msgs::msg::Accel::from_rmw_message(msg.body_acceleration),
      joint_positions: msg.joint_positions.into(),
      joint_velocities: msg.joint_velocities.into(),
      joint_accelerations: msg.joint_accelerations.into(),
      feet: msg.feet
          .into_iter()
          .map(super::msg::FootState::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to wbc_control_msgs__msg__MpcTrajectory

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MpcTrajectory {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// Full sequence of trajectory steps over the MPC horizon
    pub steps: Vec<super::msg::MpcStateStep>,

}



impl Default for MpcTrajectory {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MpcTrajectory::default())
  }
}

impl rosidl_runtime_rs::Message for MpcTrajectory {
  type RmwMsg = super::msg::rmw::MpcTrajectory;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        steps: msg.steps
          .into_iter()
          .map(|elem| super::msg::MpcStateStep::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        steps: msg.steps
          .iter()
          .map(|elem| super::msg::MpcStateStep::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      steps: msg.steps
          .into_iter()
          .map(super::msg::MpcStateStep::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to wbc_control_msgs__msg__RobotState
/// msg/RobotState.msg

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotState {
    /// Body SE3 Pose (Translation + Quaternion Orientation)
    pub body_pose: geometry_msgs::msg::Pose,

    /// Pinocchio::Motion Spatial Velocity (Linear + Angular)
    pub body_velocity: geometry_msgs::msg::Twist,

    /// Pinocchio::Motion Spatial Acceleration (Linear + Angular)
    pub body_acceleration: geometry_msgs::msg::Accel,

    /// q
    pub joint_positions: Vec<f64>,

    /// v
    pub joint_velocities: Vec<f64>,

    /// a
    pub joint_accelerations: Vec<f64>,

    /// Foot target states and optimized contact forces
    pub feet: Vec<super::msg::FootState>,

}



impl Default for RobotState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotState::default())
  }
}

impl rosidl_runtime_rs::Message for RobotState {
  type RmwMsg = super::msg::rmw::RobotState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        body_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.body_pose)).into_owned(),
        body_velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.body_velocity)).into_owned(),
        body_acceleration: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Owned(msg.body_acceleration)).into_owned(),
        joint_positions: msg.joint_positions.as_slice().into(),
        joint_velocities: msg.joint_velocities.as_slice().into(),
        joint_accelerations: msg.joint_accelerations.as_slice().into(),
        feet: msg.feet
          .into_iter()
          .map(|elem| super::msg::FootState::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        body_pose: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_pose)).into_owned(),
        body_velocity: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_velocity)).into_owned(),
        body_acceleration: geometry_msgs::msg::Accel::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_acceleration)).into_owned(),
        joint_positions: msg.joint_positions.as_slice().into(),
        joint_velocities: msg.joint_velocities.as_slice().into(),
        joint_accelerations: msg.joint_accelerations.as_slice().into(),
        feet: msg.feet
          .iter()
          .map(|elem| super::msg::FootState::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      body_pose: geometry_msgs::msg::Pose::from_rmw_message(msg.body_pose),
      body_velocity: geometry_msgs::msg::Twist::from_rmw_message(msg.body_velocity),
      body_acceleration: geometry_msgs::msg::Accel::from_rmw_message(msg.body_acceleration),
      joint_positions: msg.joint_positions.into(),
      joint_velocities: msg.joint_velocities.into(),
      joint_accelerations: msg.joint_accelerations.into(),
      feet: msg.feet
          .into_iter()
          .map(super::msg::FootState::from_rmw_message)
          .collect(),
    }
  }
}


