#pragma once
// Computes the dynamics quantities the WBC needs from the measured state and
// estimates foot contacts from the measured joint torques.

#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/types.hpp"

#include <memory>

namespace wbc {

struct ContactDetectionConfig {
  bool enabled = true;
  double touchdown_force = 15.0;  // [N] vertical force to declare contact
  double liftoff_force = 10.0;    // [N] vertical force to declare liftoff
  // After activation all feet are assumed in contact for this long, so that
  // a previous controller's (or zero) torque reading cannot drop the stance.
  double activation_grace_time = 0.2;  // [s]
  // A foot planned in contact but not detected reaches down. If it stops
  // descending (speed below stall_velocity_ratio * reach speed) for
  // stall_time it has hit the ground: declare contact without waiting for the
  // force estimate (a lightly loaded foot may never reach touchdown_force).
  double stall_velocity_ratio = 0.3;
  double stall_time = 0.02;  // [s]
  // A newly declared contact is kept at least this long.
  double min_stance_time = 0.05;  // [s]
};

class DynamicSystem {
public:
  DynamicSystem(const RobotModel &robot, const ContactDetectionConfig &config);

  // Needs state.q, state.v, state.tau_meas. Fills all dynamics fields, foot
  // kinematics, the estimated ground reaction forces and (if enabled) the
  // detected contact flags (with hysteresis on the previous flags).
  void update(SystemState &state);

  const RobotModel &robot() const { return robot_; }

private:
  const RobotModel &robot_;
  ContactDetectionConfig config_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::MatrixXd J6_;
  std::vector<Eigen::MatrixXd> J_leg_;  // 3 x n_leg_joints per foot
  std::vector<Eigen::VectorXd> rhs_leg_;
};

}  // namespace wbc
