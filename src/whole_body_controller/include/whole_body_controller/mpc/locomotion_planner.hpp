#pragma once
// Turns a velocity command and the measured state into a reference trajectory
// for the WBC:
//
//   command smoothing -> gait timing -> footholds -> swing trajectories ->
//   SRBD convex MPC -> dense Trajectory (base, feet, contact forces)
//
// Called at the MPC rate (e.g. 50 Hz) from a non-real-time thread. Everything
// robot-specific comes from RobotModel (URDF) and the config.

#include "whole_body_controller/mpc/gait_scheduler.hpp"
#include "whole_body_controller/mpc/srbd_mpc.hpp"
#include "whole_body_controller/mpc/swing_trajectory.hpp"
#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/trajectory_interpolator.hpp"

#include <memory>
#include <vector>

namespace wbc {

struct VelocityCommand {
  double vx = 0.0;  // [m/s]   base yaw frame
  double vy = 0.0;  // [m/s]   base yaw frame
  double wz = 0.0;  // [rad/s]
};

struct LocomotionConfig {
  GaitConfig gait;
  double step_height = 0.08;  // [m]
  SrbdMpcConfig mpc;

  // Footholds: nominal stance under the hips + v * T_stance / 2 +
  // feedback_gain * sqrt(h/g) * (v_measured - v_command) + centrifugal term,
  // clamped to max_foothold_offset around the nominal stance.
  double foothold_feedback_gain = 0.5;
  double max_foothold_offset = 0.15;  // [m]

  // Command limits and smoothing
  double max_vx = 0.6, max_vy = 0.3, max_wz = 1.0;
  double max_lin_acc = 1.0;  // [m/s^2]
  double max_yaw_acc = 2.0;  // [rad/s^2]
  double active_threshold = 0.02;  // |command| that starts the gait
  double stop_delay = 0.5;  // [s] zero command before the gait stops
  bool always_walk = false; // keep stepping in place at zero command

  double body_height_offset = 0.0;  // [m] added to the nominal height

  double output_dt = 0.005;       // [s] spacing of the trajectory points
  double output_horizon = 0.25;   // [s] length of the trajectory sent
};

// Measured state used by the planner (snapshot from the control loop).
struct PlannerInput {
  double time = 0.0;
  Eigen::Vector3d base_pos = Eigen::Vector3d::Zero();
  Eigen::Matrix3d base_rot = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 1> base_vel_world =
      Eigen::Matrix<double, 6, 1>::Zero();  // [lin; ang], world
  std::vector<Eigen::Vector3d> feet;        // measured foot positions, world
};

class LocomotionPlanner {
public:
  LocomotionPlanner(const RobotModel &robot, const LocomotionConfig &config);

  // Call when the controller activates (robot standing on all feet).
  void reset(const PlannerInput &input);

  // Returns nullptr if the MPC failed (the WBC then keeps/fades the last one).
  std::shared_ptr<Trajectory> update(const PlannerInput &input,
                                     const VelocityCommand &command);

  const GaitScheduler &gait() const { return gait_; }

private:
  struct Reference {
    Eigen::Vector3d base;  // base position (world)
    double yaw;
    Eigen::Vector3d v;     // base linear velocity (world)
  };
  Reference reference(double tau) const;
  Eigen::Vector3d foothold(int foot, double touchdown_time) const;
  // Foot position at absolute time t (stance: fixed point, swing: curve)
  SwingSample footAt(int foot, double t) const;
  void updateCommand(double t, const VelocityCommand &request);

  const RobotModel &robot_;
  LocomotionConfig config_;
  GaitScheduler gait_;
  SrbdMpc mpc_;
  int nf_;

  // Planning context of the current update
  double t_ = 0.0;
  double yaw0_ = 0.0;
  Eigen::Vector3d base0_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_meas_ = Eigen::Vector3d::Zero();
  double ground_z_ = 0.0;

  VelocityCommand cmd_;
  double last_time_ = 0.0;
  double inactive_since_ = 0.0;
  bool initialized_ = false;

  std::vector<Eigen::Vector3d> stance_pos_;   // last stance position per foot
  std::vector<Eigen::Vector3d> swing_start_;  // liftoff position per foot
  std::vector<double> swing_liftoff_;         // liftoff time of that swing
};

}  // namespace wbc
