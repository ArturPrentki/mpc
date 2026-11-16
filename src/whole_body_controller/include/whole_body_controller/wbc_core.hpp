#pragma once
// ROS-independent whole-body control pipeline:
//   measured state -> dynamics/contact estimation -> reference sampling ->
//   contact modes -> prioritized WBC -> safety supervisor -> joint command
//
// The ros2_control plugin only moves data between hardware interfaces /
// topics and this class, which keeps the controller unit-testable.

#include "whole_body_controller/dynamic_system.hpp"
#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/safety_supervisor.hpp"
#include "whole_body_controller/trajectory_interpolator.hpp"
#include "whole_body_controller/wbic.hpp"

#include <memory>
#include <string>

namespace wbc {

struct CoreConfig {
  RobotConfig robot;
  WbicConfig wbic;
  ContactDetectionConfig contact;
  SafetyConfig safety;
  double fade_time = 0.2;  // [s] see TrajectoryInterpolator
};

class WbcCore {
public:
  // Throws std::runtime_error on an invalid URDF/config or solver setup.
  WbcCore(const std::string &urdf_xml, const CoreConfig &config);

  const RobotModel &robot() const { return *robot_; }

  // Call with the measured state (q, v, tau_meas filled) when the controller
  // becomes active. Assumes the robot is standing on all feet and anchors the
  // stand target at the current pose.
  void activate(double now, SystemState &state);

  // Non-RT thread.
  void setTrajectory(std::shared_ptr<const Trajectory> trajectory) {
    interpolator_.set(std::move(trajectory));
  }

  // One control tick. `state` must hold q, v and tau_meas; the remaining
  // fields are filled. Returns the command to send (already safe/clamped).
  const JointCommand &update(double now, SystemState &state);

  // Diagnostics of the last tick
  const TargetState &target() const { return target_; }
  const WbcOutput &output() const { return output_; }
  const std::vector<ContactMode> &contactModes() const { return modes_; }
  TrajectoryInterpolator::Mode referenceMode() const { return ref_mode_; }
  SafetySupervisor::State safetyState() const { return safety_.state(); }
  const std::string &safetyReason() const { return safety_.reason(); }
  bool wbcOk() const { return wbc_ok_; }

private:
  void anchorStand(const SystemState &state);

  CoreConfig config_;
  std::unique_ptr<RobotModel> robot_;
  DynamicSystem dynamics_;
  Wbic wbic_;
  TrajectoryInterpolator interpolator_;
  SafetySupervisor safety_;

  TargetState stand_target_, target_;
  WbcOutput output_;
  JointCommand command_;
  std::vector<ContactMode> modes_;
  TrajectoryInterpolator::Mode ref_mode_ = TrajectoryInterpolator::Mode::EXPIRED;
  bool wbc_ok_ = false;
  double activation_time_ = 0.0;
  double last_update_ = 0.0;
  std::vector<double> stall_timer_;       // per foot
  std::vector<double> stance_hold_until_; // per foot
};

}  // namespace wbc
