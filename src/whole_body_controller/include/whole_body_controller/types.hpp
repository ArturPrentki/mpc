#pragma once
// Robot-agnostic state containers shared by the WBC core.
//
// Conventions
// -----------
//  * q, v, a use Pinocchio's layout: q = [base pos (3), base quat xyzw (4),
//    joints (in Pinocchio order)], v = [base lin (3), base ang (3), joints].
//    Base velocities are expressed in the LOCAL base frame (Pinocchio
//    free-flyer convention).
//  * Joint vectors that are NOT inside q/v (torques, gains, desired joint
//    positions sent to the drives) use the CONFIG order, i.e. the order of the
//    `joint_names` parameter. RobotModel converts between the two.
//  * Foot quantities are in the world (odom) frame / LOCAL_WORLD_ALIGNED.
//  * Feet use the order of the `foot_frames` parameter.

#include <Eigen/Dense>
#include <vector>

namespace wbc {

struct FootState {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  // measured: estimated ground reaction force; target: MPC force reference
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  // measured: detected contact; target: planned contact
  bool contact = false;
};

// Measured state plus the dynamics quantities derived from it.
struct SystemState {
  Eigen::VectorXd q;          // nq
  Eigen::VectorXd v;          // nv
  Eigen::VectorXd tau_meas;   // nj, config order
  std::vector<FootState> feet;

  // Filled by DynamicSystem::update()
  Eigen::MatrixXd M;          // nv x nv (full, symmetric)
  Eigen::VectorXd nle;        // nv
  Eigen::MatrixXd J_base;     // 6 x nv, LOCAL_WORLD_ALIGNED [lin; ang]
  Eigen::Matrix<double, 6, 1> base_drift = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::MatrixXd J_feet;     // 3nf x nv, LOCAL_WORLD_ALIGNED linear
  Eigen::VectorXd feet_drift; // 3nf, classical acceleration bias (Jdot*v)
  Eigen::Vector3d base_pos = Eigen::Vector3d::Zero();      // world
  Eigen::Matrix3d base_rot = Eigen::Matrix3d::Identity();  // world <- base
  Eigen::Matrix<double, 6, 1> base_vel_world =
      Eigen::Matrix<double, 6, 1>::Zero();                 // [lin; ang]

  void resize(int nq, int nv, int nj, int nf) {
    q = Eigen::VectorXd::Zero(nq);
    q[6] = 1.0;
    v = Eigen::VectorXd::Zero(nv);
    tau_meas = Eigen::VectorXd::Zero(nj);
    feet.assign(nf, FootState{});
    M = Eigen::MatrixXd::Zero(nv, nv);
    nle = Eigen::VectorXd::Zero(nv);
    J_base = Eigen::MatrixXd::Zero(6, nv);
    J_feet = Eigen::MatrixXd::Zero(3 * nf, nv);
    feet_drift = Eigen::VectorXd::Zero(3 * nf);
  }
};

// Reference coming from the planner (MPC) or from the stand fallback.
struct TargetState {
  Eigen::VectorXd q;  // nq
  Eigen::VectorXd v;  // nv (base twist local)
  Eigen::VectorXd a;  // nv (base accel local)
  std::vector<FootState> feet;

  void resize(int nq, int nv, int nf) {
    q = Eigen::VectorXd::Zero(nq);
    q[6] = 1.0;
    v = Eigen::VectorXd::Zero(nv);
    a = Eigen::VectorXd::Zero(nv);
    feet.assign(nf, FootState{});
  }
};

// Joint-level command. All vectors in config order.
struct JointCommand {
  Eigen::VectorXd tau_ff;
  Eigen::VectorXd q_des;
  Eigen::VectorXd v_des;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;

  void resize(int nj) {
    tau_ff = Eigen::VectorXd::Zero(nj);
    q_des = Eigen::VectorXd::Zero(nj);
    v_des = Eigen::VectorXd::Zero(nj);
    kp = Eigen::VectorXd::Zero(nj);
    kd = Eigen::VectorXd::Zero(nj);
  }
  bool allFinite() const {
    return tau_ff.allFinite() && q_des.allFinite() && v_des.allFinite() &&
           kp.allFinite() && kd.allFinite();
  }
};

enum class ContactMode {
  STANCE,          // planned and detected: hard contact constraint
  SWING,           // planned swing: track foot trajectory
  LATE_TOUCHDOWN,  // planned contact but not detected yet: reach down
};

struct WbcOutput {
  Eigen::VectorXd qdd;     // nv
  Eigen::VectorXd forces;  // 3nf, world
  JointCommand cmd;
  bool torque_limited = false;  // torque limits had to be relaxed
  double base_dynamics_residual = 0.0;

  void resize(int nv, int nj, int nf) {
    qdd = Eigen::VectorXd::Zero(nv);
    forces = Eigen::VectorXd::Zero(3 * nf);
    cmd.resize(nj);
  }
};

}  // namespace wbc
