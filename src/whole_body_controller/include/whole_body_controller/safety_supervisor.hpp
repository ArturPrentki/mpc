#pragma once
// Last line of defence between the WBC and the motors.
//
//   OK       WBC output passes through (torques clamped to the URDF limits)
//   HOLDING  WBC failed: repeat the last good command for `hold_time`;
//            returns to OK as soon as the WBC succeeds again
//   DAMPING  latched: zero stiffness, joint damping only. Entered when the
//            WBC keeps failing, the command is not finite, or the base tilts
//            beyond `max_tilt`. Cleared only by reset() (controller restart).
//
// The controller never stops commanding the drives because of a solver
// failure (the old WBC returned ERROR, or sent zero torque).

#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/types.hpp"

#include <string>

namespace wbc {

struct SafetyConfig {
  double max_tilt = 1.0;        // [rad] angle between base z and world z
  double hold_time = 0.05;      // [s]
  double damping_kd = 2.0;      // [Nm s/rad] joint damping in DAMPING
};

class SafetySupervisor {
public:
  enum class State { OK, HOLDING, DAMPING };

  SafetySupervisor(const RobotModel &robot, const SafetyConfig &config);

  void reset();

  // `cmd` is the WBC command (ignored when wbc_ok is false) and is replaced
  // by the command that must be sent. Returns the resulting state.
  State step(double now, bool wbc_ok, const SystemState &state,
             JointCommand &cmd);

  State state() const { return state_; }
  const std::string &reason() const { return reason_; }

private:
  void enterDamping(const std::string &reason);
  void clamp(JointCommand &cmd) const;

  const RobotModel &robot_;
  SafetyConfig config_;
  State state_ = State::OK;
  std::string reason_;
  double fault_since_ = 0.0;
  bool has_last_good_ = false;
  JointCommand last_good_;
  Eigen::VectorXd q_joints_, v_joints_;
};

const char *toString(SafetySupervisor::State state);

}  // namespace wbc
