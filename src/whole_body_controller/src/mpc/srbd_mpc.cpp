#include "whole_body_controller/mpc/srbd_mpc.hpp"

#include <OsqpEigen/OsqpEigen.h>

#include <cmath>
#include <stdexcept>

namespace wbc {

namespace {
constexpr double kGravity = 9.81;

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return S;
}

Eigen::Matrix3d rotZ(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void updateSparse(Eigen::SparseMatrix<double> &sparse,
                  const Eigen::MatrixXd &dense) {
  for (int k = 0; k < sparse.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sparse, k); it; ++it) {
      it.valueRef() = dense(it.row(), it.col());
    }
  }
}
}  // namespace

SrbdMpc::SrbdMpc(double mass, const Eigen::Matrix3d &inertia_body,
                 int num_feet, const SrbdMpcConfig &config)
    : mass_(mass), inertia_body_(inertia_body), nf_(num_feet),
      nu_(3 * num_feet), N_(config.horizon), config_(config) {
  if (N_ < 2 || !(config_.dt > 0.0) || !(mass_ > 0.0) || nf_ < 1) {
    throw std::invalid_argument("invalid SRBD MPC configuration");
  }
  // z = [x_1 .. x_N, u_0 .. u_N-1]
  n_var_ = 12 * N_ + nu_ * N_;
  // dynamics (12 N) + per step and foot: 4 pyramid rows + 1 normal force row
  n_con_ = 12 * N_ + 5 * nf_ * N_;

  // --- sparsity templates -----------------------------------------------
  Eigen::MatrixXd At = Eigen::MatrixXd::Zero(n_con_, n_var_);
  for (int k = 0; k < N_; ++k) {
    const int r = 12 * k;
    At.block(r, 12 * k, 12, 12).diagonal().setOnes();     // x_k+1
    if (k > 0) At.block(r, 12 * (k - 1), 12, 12).setOnes();  // -A x_k
    At.block(r, 12 * N_ + nu_ * k, 12, nu_).setOnes();       // -B u_k
  }
  for (int k = 0; k < N_; ++k) {
    for (int f = 0; f < nf_; ++f) {
      const int r = 12 * N_ + 5 * (nf_ * k + f);
      const int c = 12 * N_ + nu_ * k + 3 * f;
      At.block(r, c, 4, 3).setOnes();
      At(r + 4, c + 2) = 1.0;
    }
  }
  A_ = At.sparseView();
  A_.makeCompressed();
  A_dense_ = Eigen::MatrixXd::Zero(n_con_, n_var_);

  // Constant Hessian: Q on the states, R on the forces
  std::vector<Eigen::Triplet<double>> h;
  const double fw = config_.force_weight / std::pow(mass_ * kGravity, 2);
  for (int k = 0; k < N_; ++k) {
    for (int i = 0; i < 12; ++i) {
      h.emplace_back(12 * k + i, 12 * k + i,
                     2.0 * std::max(config_.state_weights[i], 1e-9));
    }
    for (int i = 0; i < nu_; ++i) {
      const int c = 12 * N_ + nu_ * k + i;
      h.emplace_back(c, c, 2.0 * fw);
    }
  }
  weights_ = config_.state_weights;
  H_.resize(n_var_, n_var_);
  H_.setFromTriplets(h.begin(), h.end());
  H_.makeCompressed();

  g_ = Eigen::VectorXd::Zero(n_var_);
  lb_ = Eigen::VectorXd::Zero(n_con_);
  ub_ = Eigen::VectorXd::Zero(n_con_);
  xs_.assign(N_ + 1, Vector12d::Zero());
  us_.assign(N_, Eigen::VectorXd::Zero(nu_));

  solver_ = std::make_unique<OsqpEigen::Solver>();
  auto &s = *solver_;
  s.settings()->setVerbosity(false);
  s.settings()->setWarmStart(true);
  s.settings()->setPolish(true);
  s.settings()->setAdaptiveRho(true);
  s.settings()->setAbsoluteTolerance(1e-5);
  s.settings()->setRelativeTolerance(1e-5);
  s.settings()->setMaxIteration(4000);
  s.data()->setNumberOfVariables(n_var_);
  s.data()->setNumberOfConstraints(n_con_);
  if (!s.data()->setHessianMatrix(H_) || !s.data()->setGradient(g_) ||
      !s.data()->setLinearConstraintsMatrix(A_) ||
      !s.data()->setLowerBound(lb_) || !s.data()->setUpperBound(ub_) ||
      !s.initSolver()) {
    throw std::runtime_error("Failed to initialize the MPC QP solver");
  }
  initialized_ = true;
}

SrbdMpc::~SrbdMpc() = default;

bool SrbdMpc::setStateWeights(const Vector12d &weights) {
  if (weights == weights_) return true;
  weights_ = weights;
  for (int k = 0; k < N_; ++k) {
    for (int i = 0; i < 12; ++i) {
      H_.coeffRef(12 * k + i, 12 * k + i) = 2.0 * std::max(weights_[i], 1e-9);
    }
  }
  return solver_->updateHessianMatrix(H_);
}

void SrbdMpc::stepMatrices(double yaw, const std::vector<Eigen::Vector3d> &r,
                           Eigen::Matrix<double, 12, 12> &A,
                           Eigen::MatrixXd &B, Vector12d &d) const {
  const double dt = config_.dt;
  const Eigen::Matrix3d Rz = rotZ(yaw);
  const Eigen::Matrix3d Iw_inv =
      (Rz * inertia_body_ * Rz.transpose()).inverse();

  // Continuous Ac, Bc, gc
  Eigen::Matrix<double, 12, 12> Ac = Eigen::Matrix<double, 12, 12>::Zero();
  Ac.block<3, 3>(0, 6) = Rz.transpose();
  Ac.block<3, 3>(3, 9).setIdentity();
  Eigen::MatrixXd Bc = Eigen::MatrixXd::Zero(12, nu_);
  for (int f = 0; f < nf_; ++f) {
    Bc.block<3, 3>(6, 3 * f) = Iw_inv * skew(r[f]);
    Bc.block<3, 3>(9, 3 * f) = Eigen::Matrix3d::Identity() / mass_;
  }
  Vector12d gc = Vector12d::Zero();
  gc[11] = -kGravity;

  // Ac^2 = 0 for this model, so the second-order hold is exact in Ac:
  A = Eigen::Matrix<double, 12, 12>::Identity() + Ac * dt;
  B = Bc * dt + 0.5 * dt * dt * (Ac * Bc);
  d = gc * dt + 0.5 * dt * dt * (Ac * gc);
}

Vector12d SrbdMpc::derivative(const Vector12d &x, const Eigen::VectorXd &u,
                              const std::vector<Eigen::Vector3d> &r) const {
  const Eigen::Matrix3d Rz = rotZ(x[2]);
  const Eigen::Matrix3d Iw_inv =
      (Rz * inertia_body_ * Rz.transpose()).inverse();
  Vector12d dx = Vector12d::Zero();
  dx.segment<3>(0) = Rz.transpose() * x.segment<3>(6);
  dx.segment<3>(3) = x.segment<3>(9);
  Eigen::Vector3d torque = Eigen::Vector3d::Zero();
  Eigen::Vector3d force = Eigen::Vector3d::Zero();
  for (int f = 0; f < nf_; ++f) {
    const Eigen::Vector3d fi = u.segment<3>(3 * f);
    torque += r[f].cross(fi);
    force += fi;
  }
  dx.segment<3>(6) = Iw_inv * torque;
  dx.segment<3>(9) = force / mass_;
  dx[11] -= kGravity;
  return dx;
}

bool SrbdMpc::solve(const SrbdMpcProblem &p) {
  if (static_cast<int>(p.x_ref.size()) != N_ ||
      static_cast<int>(p.yaw.size()) != N_ ||
      static_cast<int>(p.r.size()) != N_ ||
      static_cast<int>(p.contact.size()) != N_) {
    return false;
  }
  A_dense_.setZero();
  Eigen::Matrix<double, 12, 12> A;
  Eigen::MatrixXd B(12, nu_);
  Vector12d d;

  for (int k = 0; k < N_; ++k) {
    stepMatrices(p.yaw[k], p.r[k], A, B, d);
    const int r = 12 * k;
    A_dense_.block(r, 12 * k, 12, 12).setIdentity();
    A_dense_.block(r, 12 * N_ + nu_ * k, 12, nu_) = -B;
    if (k == 0) {
      lb_.segment<12>(r) = A * p.x0 + d;
    } else {
      A_dense_.block(r, 12 * (k - 1), 12, 12) = -A;
      lb_.segment<12>(r) = d;
    }
    ub_.segment<12>(r) = lb_.segment<12>(r);

    // Tracking cost gradient: (x - x_ref)^T Q (x - x_ref)
    g_.segment<12>(12 * k) =
        -2.0 * weights_.cwiseProduct(p.x_ref[k]);
  }

  // Forces are regularized towards an equal share of the weight on the
  // stance feet (not towards zero, which would make letting the body sag at
  // the end of the horizon look optimal).
  const double fw = config_.force_weight / std::pow(mass_ * kGravity, 2);
  for (int k = 0; k < N_; ++k) {
    int n_stance = 0;
    for (int f = 0; f < nf_; ++f) n_stance += p.contact[k][f];
    const double share = n_stance > 0 ? mass_ * kGravity / n_stance : 0.0;
    for (int f = 0; f < nf_; ++f) {
      const int c = 12 * N_ + nu_ * k + 3 * f;
      g_.segment<3>(c).setZero();
      if (p.contact[k][f]) g_[c + 2] = -2.0 * fw * share;
    }
  }

  const double mu = config_.mu;
  for (int k = 0; k < N_; ++k) {
    for (int f = 0; f < nf_; ++f) {
      const int r = 12 * N_ + 5 * (nf_ * k + f);
      const int c = 12 * N_ + nu_ * k + 3 * f;
      A_dense_.block<4, 3>(r, c) << 1, 0, -mu, -1, 0, -mu, 0, 1, -mu, 0, -1,
          -mu;
      A_dense_(r + 4, c + 2) = 1.0;
      lb_.segment<4>(r).setConstant(-OsqpEigen::INFTY);
      ub_.segment<4>(r).setZero();
      lb_[r + 4] = 0.0;
      ub_[r + 4] = p.contact[k][f] ? config_.fz_max : 0.0;
    }
  }

  if (!A_dense_.allFinite() || !lb_.allFinite() || !ub_.allFinite() ||
      !g_.allFinite()) {
    return false;
  }
  updateSparse(A_, A_dense_);
  auto &s = *solver_;
  if (!s.updateGradient(g_) || !s.updateLinearConstraintsMatrix(A_) ||
      !s.updateBounds(lb_, ub_)) {
    return false;
  }
  if (s.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
  const auto status = s.getStatus();
  if (status != OsqpEigen::Status::Solved &&
      status != OsqpEigen::Status::SolvedInaccurate) {
    return false;
  }
  const Eigen::VectorXd z = s.getSolution();
  if (!z.allFinite()) return false;

  xs_[0] = p.x0;
  for (int k = 0; k < N_; ++k) {
    xs_[k + 1] = z.segment<12>(12 * k);
    us_[k] = z.segment(12 * N_ + nu_ * k, nu_);
  }
  return true;
}

}  // namespace wbc
