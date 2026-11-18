#pragma once
// Shared test fixtures: robot configurations for Go2 and Gardo and a small
// rigid-contact simulator (point contacts solved with a KKT system, Baumgarte
// stabilization, semi-implicit Euler).
//
// Contacts are either pinned/released manually (pin(), release()) or, with
// setGround(z), handled automatically as unilateral contacts: a foot is
// released when holding it would need a pulling force, and a foot reaching
// the ground lands with an inelastic impact.

#include "whole_body_controller/wbc_core.hpp"

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#ifndef WBC_TEST_DATA_DIR
#error "WBC_TEST_DATA_DIR must be defined"
#endif

namespace wbc_test {
using namespace wbc;

inline std::string readFile(const std::string &name) {
  std::ifstream in(std::string(WBC_TEST_DATA_DIR) + "/" + name);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

struct RobotCase {
  std::string name;
  std::string urdf_file;
  CoreConfig config;
};

inline void PrintTo(const RobotCase &c, std::ostream *os) { *os << c.name; }

inline RobotCase go2Case() {
  RobotCase c;
  c.name = "go2";
  c.urdf_file = "go2.urdf";
  auto &r = c.config.robot;
  r.base_frame = "base";
  r.foot_frames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  for (const char *leg : {"FL", "FR", "RL", "RR"}) {
    for (const char *j : {"_hip_joint", "_thigh_joint", "_calf_joint"}) {
      r.joint_names.push_back(std::string(leg) + j);
    }
    r.nominal_joint_positions.insert(r.nominal_joint_positions.end(),
                                     {0.0, 0.8, -1.5});
  }
  c.config.safety.damping_kd = 2.0;
  return c;
}

inline RobotCase gardoCase() {
  RobotCase c;
  c.name = "gardo";
  c.urdf_file = "gardo.urdf";
  auto &r = c.config.robot;
  r.base_frame = "TORSO";
  // Config order deliberately differs from Pinocchio's (LF, LH, RF, RH)
  r.foot_frames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  for (const char *leg : {"LF", "RF", "LH", "RH"}) {
    for (const char *j : {"_HAA", "_HFE", "_KFE"}) {
      r.joint_names.push_back(std::string(leg) + j);
    }
    r.nominal_joint_positions.insert(r.nominal_joint_positions.end(),
                                     {0.0, 0.7, -1.4});
  }
  c.config.wbic.joint_kd_stance = 2.0;
  c.config.wbic.joint_kd_swing = 2.0;
  c.config.wbic.joint_kp_swing = 60.0;
  c.config.safety.damping_kd = 5.0;
  return c;
}

// ---------------------------------------------------------------------------
// Minimal rigid-contact simulator
// ---------------------------------------------------------------------------
class ContactSim {
public:
  explicit ContactSim(const RobotModel &robot)
      : robot_(robot), model_(robot.model()), data_(model_) {
    anchors_.resize(robot.nf());
    active_.assign(robot.nf(), false);
  }

  Eigen::VectorXd q, v;
  Eigen::VectorXd tau_applied;  // config order, what the drives produced
  std::vector<Eigen::Vector3d> forces;  // last contact forces (world)

  void setState(const Eigen::VectorXd &q0) {
    q = q0;
    v.setZero(model_.nv);
    tau_applied.setZero(robot_.nj());
    forces.assign(robot_.nf(), Eigen::Vector3d::Zero());
    pinocchio::framesForwardKinematics(model_, data_, q);
  }
  Eigen::Vector3d footPos(int f) {
    pinocchio::framesForwardKinematics(model_, data_, q);
    return data_.oMf[robot_.footFrameId(f)].translation();
  }
  void pin(int f) {
    anchors_[f] = footPos(f);
    active_[f] = true;
  }
  void release(int f) { active_[f] = false; }
  bool pinned(int f) const { return active_[f]; }

  // Automatic unilateral contacts against a flat ground at height z.
  void setGround(double z) {
    auto_contacts_ = true;
    ground_ = z;
  }
  // Largest |f_tangential| / f_normal seen on a contact carrying at least
  // friction_min_normal_force (friction demand of loaded feet; contacts that
  // are just landing or lifting carry a few newtons and would only slip a
  // little on real ground).
  double max_friction_ratio = 0.0;
  double friction_min_normal_force = 1.0;

  // Applies a joint command like a drive with software PD would.
  void step(const JointCommand &cmd, double dt) {
    Eigen::VectorXd qj(robot_.nj()), vj(robot_.nj());
    robot_.jointsFromQ(q, qj);
    robot_.jointsFromV(v, vj);
    tau_applied = cmd.tau_ff + cmd.kp.cwiseProduct(cmd.q_des - qj) +
                  cmd.kd.cwiseProduct(cmd.v_des - vj);
    tau_applied = tau_applied.cwiseMax(-robot_.effortLimits())
                      .cwiseMin(robot_.effortLimits());
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
    robot_.jointsToV(tau_applied, tau);

    pinocchio::crba(model_, data_, q);
    M_ = data_.M;
    M_.triangularView<Eigen::StrictlyLower>() =
        M_.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, data_, q, v);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::forwardKinematics(model_, data_, q, v,
                                 Eigen::VectorXd::Zero(model_.nv));
    pinocchio::updateFramePlacements(model_, data_);

    const int nf = robot_.nf();
    std::vector<Eigen::MatrixXd> Jc(nf);
    std::vector<Eigen::Vector3d> rhs_c(nf);
    Eigen::MatrixXd J6(6, model_.nv);
    for (int f = 0; f < nf; ++f) {
      const auto fid = robot_.footFrameId(f);
      J6.setZero();
      pinocchio::getFrameJacobian(model_, data_, fid,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6);
      Jc[f] = J6.topRows<3>();
      const Eigen::Vector3d gamma =
          pinocchio::getFrameClassicalAcceleration(
              model_, data_, fid, pinocchio::LOCAL_WORLD_ALIGNED)
              .linear();
      const Eigen::Vector3d err = data_.oMf[fid].translation() - anchors_[f];
      rhs_c[f] = -gamma - 400.0 * err - 40.0 * (Jc[f] * v);
    }

    Eigen::VectorXd a;
    for (int attempt = 0; attempt <= nf; ++attempt) {
      int nc = 0;
      for (bool act : active_) nc += act;
      const int n = model_.nv + 3 * nc;
      Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n, n);
      Eigen::VectorXd rhs(n);
      K.topLeftCorner(model_.nv, model_.nv) = M_;
      rhs.head(model_.nv) = tau - nle;
      int r = 0;
      for (int f = 0; f < nf; ++f) {
        if (!active_[f]) continue;
        K.block(model_.nv + 3 * r, 0, 3, model_.nv) = Jc[f];
        K.block(0, model_.nv + 3 * r, model_.nv, 3) = -Jc[f].transpose();
        rhs.segment<3>(model_.nv + 3 * r) = rhs_c[f];
        ++r;
      }
      const Eigen::VectorXd sol = K.fullPivLu().solve(rhs);
      a = sol.head(model_.nv);
      r = 0;
      int worst = -1;
      double worst_fz = 0.0;
      for (int f = 0; f < nf; ++f) {
        forces[f].setZero();
        if (!active_[f]) continue;
        forces[f] = sol.segment<3>(model_.nv + 3 * r++);
        if (forces[f].z() < worst_fz) {
          worst_fz = forces[f].z();
          worst = f;
        }
      }
      // Unilateral: the ground cannot pull. Release the worst foot, resolve.
      if (!auto_contacts_ || worst < 0) break;
      active_[worst] = false;
    }
    for (int f = 0; f < nf; ++f) {
      if (active_[f] && forces[f].z() > friction_min_normal_force) {
        max_friction_ratio = std::max(
            max_friction_ratio, forces[f].head<2>().norm() / forces[f].z());
      }
    }
    v += a * dt;

    if (auto_contacts_) {
      // Touchdown: a free foot at or below the ground moving down lands with
      // an inelastic impact (velocity projected onto the contact constraints)
      bool landed = false;
      for (int f = 0; f < nf; ++f) {
        if (active_[f]) continue;
        const Eigen::Vector3d p = data_.oMf[robot_.footFrameId(f)].translation();
        if (p.z() <= ground_ && (Jc[f] * v).z() < 0.0) {
          active_[f] = true;
          anchors_[f] = Eigen::Vector3d(p.x(), p.y(), ground_);
          landed = true;
        }
      }
      if (landed) {
        int nc = 0;
        for (bool act : active_) nc += act;
        Eigen::MatrixXd J(3 * nc, model_.nv);
        int r = 0;
        for (int f = 0; f < nf; ++f) {
          if (active_[f]) J.middleRows<3>(3 * r++) = Jc[f];
        }
        const Eigen::MatrixXd MinvJt = M_.ldlt().solve(J.transpose());
        v -= MinvJt * (J * MinvJt).ldlt().solve(J * v);
      }
    }
    q = pinocchio::integrate(model_, q, v * dt);
  }

  // A push: sudden change of the base velocity (world frame), then the
  // feet on the ground are held (velocity projected onto their constraints,
  // like an impact).
  void push(const Eigen::Vector3d &dv_world, const Eigen::Vector3d &dw_world) {
    const Eigen::Matrix3d R =
        Eigen::Quaterniond(q[6], q[3], q[4], q[5]).toRotationMatrix();
    v.head<3>() += R.transpose() * dv_world;
    v.segment<3>(3) += R.transpose() * dw_world;
    int nc = 0;
    for (bool act : active_) nc += act;
    if (nc == 0) return;
    pinocchio::crba(model_, data_, q);
    Eigen::MatrixXd M = data_.M;
    M.triangularView<Eigen::StrictlyLower>() =
        M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    Eigen::MatrixXd J(3 * nc, model_.nv), J6(6, model_.nv);
    int r = 0;
    for (int f = 0; f < robot_.nf(); ++f) {
      if (!active_[f]) continue;
      J6.setZero();
      pinocchio::getFrameJacobian(model_, data_, robot_.footFrameId(f),
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6);
      J.middleRows<3>(3 * r++) = J6.topRows<3>();
    }
    const Eigen::MatrixXd MinvJt = M.ldlt().solve(J.transpose());
    v -= MinvJt * (J * MinvJt).ldlt().solve(J * v);
  }

  void fillMeasured(SystemState &s) const {
    s.q = q;
    s.v = v;
    s.tau_meas = tau_applied;
  }

private:
  const RobotModel &robot_;
  const pinocchio::Model &model_;
  pinocchio::Data data_;
  std::vector<Eigen::Vector3d> anchors_;
  std::vector<bool> active_;
  Eigen::MatrixXd M_;
  bool auto_contacts_ = false;
  double ground_ = 0.0;
};

inline double orientationError(const Eigen::VectorXd &q, const Eigen::VectorXd &q_ref) {
  const Eigen::Quaterniond a(q[6], q[3], q[4], q[5]);
  const Eigen::Quaterniond b(q_ref[6], q_ref[3], q_ref[4], q_ref[5]);
  return a.angularDistance(b);
}

}  // namespace wbc_test
