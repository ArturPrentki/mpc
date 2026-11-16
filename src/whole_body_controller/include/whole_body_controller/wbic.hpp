#pragma once
// Whole-Body Impulse Control style prioritized whole-body controller
// (Kim et al., "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse
// Control and Model Predictive Control", 2019).
//
// Stage 1 - strict task hierarchy (null-space projection, dynamically
//           consistent pseudo-inverses):
//             0. stance feet do not accelerate        (constraint)
//             1. base orientation
//             2. base position
//             3. swing feet / late-touchdown reach
//             4. joint posture                        (only uses what is left)
// Stage 2 - small QP that relaxes the floating-base accelerations as little as
//           possible to satisfy the floating-base dynamics with physically
//           admissible contact forces (friction pyramid, normal force bounds,
//           joint torque limits), and keeps forces close to the MPC reference.
//
// Task gains are accelerations (1/s^2, 1/s), i.e. independent of the robot's
// mass, so the same gains transfer reasonably between robots.

#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/types.hpp"

#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace OsqpEigen {
class Solver;
}

namespace wbc {

struct WbicConfig {
  double kp_base_lin = 80.0, kd_base_lin = 18.0;
  double kp_base_ang = 100.0, kd_base_ang = 20.0;
  double kp_swing = 350.0, kd_swing = 37.0;
  double kp_posture = 0.0, kd_posture = 10.0;
  double late_touchdown_velocity = 0.5;  // [m/s] downward reach speed

  double mu = 0.6;           // friction pyramid coefficient
  double fz_min = 0.0;       // [N] min normal force of a stance foot
  double fz_max = 500.0;     // [N] max normal force of a stance foot
  double torque_limit_scale = 1.0;  // fraction of URDF effort limit usable

  // Relaxing the base acceleration must be much more expensive than moving
  // the contact forces away from their reference, otherwise the base tasks
  // are traded away: 0.1 m/s^2 of relaxation costs as much as 100 N.
  double weight_floating_base = 1000.0;  // relaxation of base acceleration
  double weight_force = 0.001;           // tracking of the MPC force reference
  double pinv_threshold = 1e-8;       // singular value cut-off

  // Joint PD sent with the feed-forward torque, per foot contact mode.
  // Stance legs: targets follow the WBC solution one control step ahead
  // (q + v dt, v + qdd dt), so the PD only corrects deviations from it.
  // Swing legs: targets from leg IK of the foot reference.
  double joint_kp_stance = 0.0, joint_kd_stance = 1.0;
  double joint_kp_swing = 20.0, joint_kd_swing = 1.0;
  double control_dt = 0.001;  // [s] used for the stance joint targets
};

class Wbic {
public:
  Wbic(const RobotModel &robot, const WbicConfig &config);
  ~Wbic();

  // Builds the QP solver. Returns false on failure.
  bool initialize();

  // Returns false if no admissible solution was found; `out` is then
  // unchanged except for diagnostics.
  bool solve(const SystemState &state, const TargetState &target,
             const std::vector<ContactMode> &modes, WbcOutput &out);

  const WbicConfig &config() const { return config_; }

private:
  void computeTaskAccelerations(const SystemState &state,
                                const TargetState &target,
                                const std::vector<ContactMode> &modes);
  void addTask(const Eigen::Ref<const Eigen::MatrixXd> &J,
               const Eigen::Ref<const Eigen::VectorXd> &xdd_minus_drift);
  void dcPinv(const Eigen::MatrixXd &J, Eigen::MatrixXd &Jinv);
  void swingJointTargets(const SystemState &state, const TargetState &target,
                         const std::vector<ContactMode> &modes,
                         JointCommand &cmd) const;
  bool solveQp(const SystemState &state, const TargetState &target,
               const std::vector<ContactMode> &modes, bool torque_limits);

  const RobotModel &robot_;
  WbicConfig config_;
  int nv_, nj_, nf_, nx_, nc_;

  // Stage 1 workspace
  Eigen::MatrixXd Minv_, N_, Jpre_, Jpre_inv_, I_;
  Eigen::VectorXd qdd_cmd_;
  Eigen::MatrixXd S_joints_;  // nj x nv selection (config order)

  // Stage 2 (QP) workspace
  std::unique_ptr<OsqpEigen::Solver> solver_;
  Eigen::MatrixXd A_dense_;
  Eigen::SparseMatrix<double> A_, H_;
  Eigen::VectorXd g_, lb_, ub_, x_;
};

}  // namespace wbc
