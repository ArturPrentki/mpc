#include "whole_body_controller/wbic.hpp"

#include <OsqpEigen/OsqpEigen.h>
#include <pinocchio/spatial/explog.hpp>

#include <cmath>

namespace wbc {

namespace {
// Copies values of `dense` into the existing non-zeros of `sparse`.
void updateSparse(Eigen::SparseMatrix<double> &sparse,
                  const Eigen::MatrixXd &dense) {
  for (int k = 0; k < sparse.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sparse, k); it; ++it) {
      it.valueRef() = dense(it.row(), it.col());
    }
  }
}
}  // namespace

Wbic::Wbic(const RobotModel &robot, const WbicConfig &config)
    : robot_(robot), config_(config), nv_(robot.nv()), nj_(robot.nj()),
      nf_(robot.nf()) {
  // QP: x = [delta_base_acc (6); forces (3 nf)]
  // rows: floating-base dynamics (6), friction pyramid (4 nf),
  //       normal force (nf), joint torques (nj)
  nx_ = 6 + 3 * nf_;
  nc_ = 6 + 5 * nf_ + nj_;

  Minv_.setZero(nv_, nv_);
  N_.setZero(nv_, nv_);
  I_ = Eigen::MatrixXd::Identity(nv_, nv_);
  qdd_cmd_.setZero(nv_);
  S_joints_.setZero(nj_, nv_);
  for (int j = 0; j < nj_; ++j) S_joints_(j, robot_.jointVIdx(j)) = 1.0;

  A_dense_.setZero(nc_, nx_);
  g_.setZero(nx_);
  lb_.setZero(nc_);
  ub_.setZero(nc_);
  x_.setZero(nx_);
}

Wbic::~Wbic() = default;

bool Wbic::initialize() {
  // Sparsity templates (structural non-zeros set to 1)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nx_, nx_);
  H.diagonal().setOnes();
  H_ = H.sparseView();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc_, nx_);
  A.topRows(6).setOnes();                       // dynamics: [M_fb | -Jc_fb^T]
  for (int f = 0; f < nf_; ++f) {
    A.block(6 + 4 * f, 6 + 3 * f, 4, 3).setOnes();  // pyramid
    A(6 + 4 * nf_ + f, 6 + 3 * f + 2) = 1.0;        // fz
  }
  A.bottomRows(nj_).setOnes();                  // torques
  A_ = A.sparseView();
  H_.makeCompressed();
  A_.makeCompressed();

  solver_ = std::make_unique<OsqpEigen::Solver>();
  auto &s = *solver_;
  s.settings()->setVerbosity(false);
  s.settings()->setWarmStart(true);
  s.settings()->setAdaptiveRho(true);
  s.settings()->setPolish(true);
  s.settings()->setAbsoluteTolerance(1e-5);
  s.settings()->setRelativeTolerance(1e-5);
  s.settings()->setMaxIteration(4000);
  s.data()->setNumberOfVariables(nx_);
  s.data()->setNumberOfConstraints(nc_);
  lb_.setConstant(-OsqpEigen::INFTY);
  ub_.setConstant(OsqpEigen::INFTY);
  if (!s.data()->setHessianMatrix(H_) || !s.data()->setGradient(g_) ||
      !s.data()->setLinearConstraintsMatrix(A_) ||
      !s.data()->setLowerBound(lb_) || !s.data()->setUpperBound(ub_)) {
    return false;
  }
  return s.initSolver();
}

void Wbic::dcPinv(const Eigen::MatrixXd &J, Eigen::MatrixXd &Jinv) {
  // J^# = M^-1 J^T (J M^-1 J^T)^+
  const Eigen::MatrixXd MinvJt = Minv_ * J.transpose();
  const Eigen::MatrixXd lambda_inv = J * MinvJt;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      lambda_inv, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto &sv = svd.singularValues();
  const double cut =
      config_.pinv_threshold * std::max(1.0, sv.size() ? sv[0] : 0.0);
  Eigen::VectorXd sv_inv = sv;
  for (int i = 0; i < sv.size(); ++i) sv_inv[i] = sv[i] > cut ? 1.0 / sv[i] : 0.0;
  Jinv = MinvJt * (svd.matrixV() * sv_inv.asDiagonal() *
                   svd.matrixU().transpose());
}

void Wbic::addTask(const Eigen::Ref<const Eigen::MatrixXd> &J,
                   const Eigen::Ref<const Eigen::VectorXd> &xdd_minus_drift) {
  Jpre_ = J * N_;
  dcPinv(Jpre_, Jpre_inv_);
  qdd_cmd_ += Jpre_inv_ * (xdd_minus_drift - J * qdd_cmd_);
  N_ = N_ * (I_ - Jpre_inv_ * Jpre_);
}

void Wbic::computeTaskAccelerations(const SystemState &s,
                                    const TargetState &t,
                                    const std::vector<ContactMode> &modes) {
  Minv_ = s.M.ldlt().solve(I_);
  qdd_cmd_.setZero();
  N_ = I_;

  // --- 0. stance contacts ---------------------------------------------------
  int ns = 0;
  for (auto m : modes) ns += (m == ContactMode::STANCE);
  if (ns > 0) {
    Eigen::MatrixXd Jc(3 * ns, nv_);
    Eigen::VectorXd rhs(3 * ns);
    int r = 0;
    for (int f = 0; f < nf_; ++f) {
      if (modes[f] != ContactMode::STANCE) continue;
      Jc.middleRows<3>(3 * r) = s.J_feet.middleRows<3>(3 * f);
      rhs.segment<3>(3 * r) = -s.feet_drift.segment<3>(3 * f);
      ++r;
    }
    addTask(Jc, rhs);
  }

  // --- 1. base orientation (world frame) --------------------------------------
  const Eigen::Quaterniond quat_t(t.q[6], t.q[3], t.q[4], t.q[5]);
  const Eigen::Matrix3d R = s.base_rot;
  const Eigen::Matrix3d Rt = quat_t.normalized().toRotationMatrix();
  const Eigen::Vector3d e_ori = R * pinocchio::log3(R.transpose() * Rt);
  const Eigen::Vector3d w_t = Rt * t.v.segment<3>(3);
  const Eigen::Vector3d dw_t = Rt * t.a.segment<3>(3);
  const Eigen::Vector3d xdd_ori = dw_t + config_.kp_base_ang * e_ori +
                                  config_.kd_base_ang *
                                      (w_t - s.base_vel_world.tail<3>());
  addTask(s.J_base.bottomRows<3>(), xdd_ori - s.base_drift.tail<3>());

  // --- 2. base position (world frame) ----------------------------------------
  const Eigen::Vector3d v_t = Rt * t.v.head<3>();
  const Eigen::Vector3d a_t = Rt * t.a.head<3>();
  const Eigen::Vector3d xdd_pos =
      a_t + config_.kp_base_lin * (t.q.head<3>() - s.base_pos) +
      config_.kd_base_lin * (v_t - s.base_vel_world.head<3>());
  addTask(s.J_base.topRows<3>(), xdd_pos - s.base_drift.head<3>());

  // --- 3. swing feet ---------------------------------------------------------
  for (int f = 0; f < nf_; ++f) {
    if (modes[f] == ContactMode::STANCE) continue;
    const auto &foot = s.feet[f];
    const auto &ref = t.feet[f];
    Eigen::Vector3d xdd;
    if (modes[f] == ContactMode::SWING) {
      xdd = ref.acc + config_.kp_swing * (ref.pos - foot.pos) +
            config_.kd_swing * (ref.vel - foot.vel);
    } else {  // LATE_TOUCHDOWN: hold xy, move down at constant speed
      Eigen::Vector3d v_reach(0.0, 0.0, -config_.late_touchdown_velocity);
      xdd = config_.kd_swing * (v_reach - foot.vel);
      xdd.head<2>() += config_.kp_swing * (ref.pos - foot.pos).head<2>();
    }
    addTask(s.J_feet.middleRows<3>(3 * f), xdd - s.feet_drift.segment<3>(3 * f));
  }

  // --- 4. joint posture ------------------------------------------------------
  Eigen::VectorXd xdd_joint(nj_);
  for (int j = 0; j < nj_; ++j) {
    const int qi = robot_.jointQIdx(j), vi = robot_.jointVIdx(j);
    xdd_joint[j] = t.a[vi] + config_.kp_posture * (t.q[qi] - s.q[qi]) +
                   config_.kd_posture * (t.v[vi] - s.v[vi]);
  }
  addTask(S_joints_, xdd_joint);
}

bool Wbic::solveQp(const SystemState &s, const TargetState &t,
                   const std::vector<ContactMode> &modes, bool torque_limits) {
  A_dense_.setZero();
  g_.setZero();

  // Floating-base dynamics: M_fb (qdd_cmd + [d;0]) + nle_fb = Jc_fb^T f
  A_dense_.block(0, 0, 6, 6) = s.M.topLeftCorner(6, 6);
  A_dense_.block(0, 6, 6, 3 * nf_) = -s.J_feet.leftCols(6).transpose();
  const Eigen::VectorXd fb_rhs =
      -(s.M.topRows(6) * qdd_cmd_ + s.nle.head<6>());
  lb_.head<6>() = fb_rhs;
  ub_.head<6>() = fb_rhs;

  int n_stance = 0;
  for (auto m : modes) n_stance += (m == ContactMode::STANCE);
  const double fz_share =
      n_stance > 0 ? robot_.totalMass() * 9.81 / n_stance : 0.0;

  Eigen::DiagonalMatrix<double, Eigen::Dynamic> H(nx_);
  H.diagonal().head<6>().setConstant(config_.weight_floating_base);
  H.diagonal().tail(3 * nf_).setConstant(config_.weight_force);

  const double mu = config_.mu;
  for (int f = 0; f < nf_; ++f) {
    const int c = 6 + 3 * f;
    const int rp = 6 + 4 * f;
    const int rz = 6 + 4 * nf_ + f;
    A_dense_.block<4, 3>(rp, c) << 1, 0, -mu, -1, 0, -mu, 0, 1, -mu, 0, -1, -mu;
    A_dense_(rz, c + 2) = 1.0;
    lb_.segment<4>(rp).setConstant(-OsqpEigen::INFTY);
    ub_.segment<4>(rp).setZero();
    if (modes[f] == ContactMode::STANCE) {
      lb_[rz] = config_.fz_min;
      ub_[rz] = config_.fz_max;
      Eigen::Vector3d f_ref = t.feet[f].force;
      if (f_ref.squaredNorm() < 1e-12) f_ref = Eigen::Vector3d(0, 0, fz_share);
      g_.segment<3>(c) = -config_.weight_force * f_ref;
    } else {
      lb_[rz] = 0.0;
      ub_[rz] = 0.0;
    }
  }

  // Joint torques: tau = M_j (qdd_cmd + [d;0]) + nle_j - Jc_j^T f
  for (int j = 0; j < nj_; ++j) {
    const int vi = robot_.jointVIdx(j);
    const int r = 6 + 5 * nf_ + j;
    A_dense_.block(r, 0, 1, 6) = s.M.block(vi, 0, 1, 6);
    A_dense_.block(r, 6, 1, 3 * nf_) = -s.J_feet.col(vi).transpose();
    const double offset = s.M.row(vi).dot(qdd_cmd_) + s.nle[vi];
    const double lim = robot_.effortLimits()[j] * config_.torque_limit_scale;
    if (torque_limits) {
      lb_[r] = -lim - offset;
      ub_[r] = lim - offset;
    } else {
      lb_[r] = -OsqpEigen::INFTY;
      ub_[r] = OsqpEigen::INFTY;
    }
  }

  if (!A_dense_.allFinite() || !g_.allFinite() || !lb_.allFinite() ||
      !ub_.allFinite()) {
    // INFTY is finite in OSQP; anything else non-finite is a bug upstream.
    return false;
  }

  for (int i = 0; i < nx_; ++i) H_.coeffRef(i, i) = H.diagonal()[i];
  updateSparse(A_, A_dense_);
  auto &solver = *solver_;
  if (!solver.updateHessianMatrix(H_) || !solver.updateGradient(g_) ||
      !solver.updateLinearConstraintsMatrix(A_) ||
      !solver.updateBounds(lb_, ub_)) {
    return false;
  }
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
  const auto status = solver.getStatus();
  if (status != OsqpEigen::Status::Solved &&
      status != OsqpEigen::Status::SolvedInaccurate) {
    return false;
  }
  x_ = solver.getSolution();
  return x_.allFinite();
}

void Wbic::swingJointTargets(const SystemState &s, const TargetState &t,
                             const std::vector<ContactMode> &modes,
                             JointCommand &cmd) const {
  // Joint targets of non-stance legs that agree with the foot target (one
  // Newton step of leg IK relative to the moving base), so that the joint PD
  // helps the swing task instead of pulling the leg towards the planner's
  // joint angles.
  for (int f = 0; f < nf_; ++f) {
    if (modes[f] == ContactMode::STANCE) continue;
    const auto &leg = robot_.legJoints(f);
    const int n = static_cast<int>(leg.size());
    Eigen::MatrixXd Jl(3, n);
    Eigen::VectorXd ql(n), vl(n);
    for (int k = 0; k < n; ++k) {
      const int vi = robot_.jointVIdx(leg[k]);
      Jl.col(k) = s.J_feet.block<3, 1>(3 * f, vi);
      ql[k] = s.q[robot_.jointQIdx(leg[k])];
      vl[k] = s.v[vi];
    }
    const auto &foot = s.feet[f];
    const auto &ref = t.feet[f];
    Eigen::Vector3d dp = ref.pos - foot.pos;
    // velocity of the foot due to the leg joints only
    const Eigen::Vector3d v_leg = Jl * vl;
    Eigen::Vector3d dv = ref.vel - foot.vel;
    if (modes[f] == ContactMode::LATE_TOUCHDOWN) {
      dp.z() = 0.0;
      dv = Eigen::Vector3d(0.0, 0.0, -config_.late_touchdown_velocity) - foot.vel;
    }
    const auto dec = Jl.completeOrthogonalDecomposition();
    const Eigen::VectorXd dq = dec.solve(dp);
    const Eigen::VectorXd vd = dec.solve(v_leg + dv);
    for (int k = 0; k < n; ++k) {
      cmd.q_des[leg[k]] = ql[k] + dq[k];
      cmd.v_des[leg[k]] = vd[k];
    }
  }
}

bool Wbic::solve(const SystemState &s, const TargetState &t,
                 const std::vector<ContactMode> &modes, WbcOutput &out) {
  if (static_cast<int>(modes.size()) != nf_ || !solver_) return false;
  computeTaskAccelerations(s, t, modes);
  if (!qdd_cmd_.allFinite()) return false;

  bool limited = false;
  if (!solveQp(s, t, modes, true)) {
    // Torque limits made the problem infeasible: solve without them; the
    // command is clamped downstream.
    limited = true;
    if (!solveQp(s, t, modes, false)) return false;
  }

  out.qdd = qdd_cmd_;
  out.qdd.head<6>() += x_.head<6>();
  out.forces = x_.tail(3 * nf_);
  const Eigen::VectorXd tau_full =
      s.M * out.qdd + s.nle - s.J_feet.transpose() * out.forces;
  out.base_dynamics_residual = tau_full.head<6>().norm();
  out.torque_limited = limited;

  auto &cmd = out.cmd;
  for (int j = 0; j < nj_; ++j) {
    const int qi = robot_.jointQIdx(j), vi = robot_.jointVIdx(j);
    cmd.tau_ff[j] = tau_full[vi];
    // non-stance legs are overwritten by swingJointTargets()
    cmd.q_des[j] = s.q[qi] + s.v[vi] * config_.control_dt;
    cmd.v_des[j] = s.v[vi] + out.qdd[vi] * config_.control_dt;
    const int f = robot_.footOfJoint(j);
    const bool stance = f >= 0 && modes[f] == ContactMode::STANCE;
    cmd.kp[j] = stance ? config_.joint_kp_stance : config_.joint_kp_swing;
    cmd.kd[j] = stance ? config_.joint_kd_stance : config_.joint_kd_swing;
  }
  swingJointTargets(s, t, modes, cmd);
  return cmd.allFinite();
}

}  // namespace wbc
