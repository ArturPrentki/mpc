#pragma once
// Convex MPC on the single-rigid-body model (Di Carlo et al., "Dynamic
// Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control",
// 2018).
//
// State (12):  x = [roll pitch yaw, p_com (world), omega (world), v_com (world)]
// Input (3nf): u = ground reaction forces of all feet (world)
//
//   d/dt rpy   = Rz(yaw)^T omega          (small roll/pitch)
//   d/dt p     = v
//   d/dt omega = I_w^-1 sum r_i x f_i     (I_w = Rz I_body Rz^T)
//   d/dt v     = sum f_i / m + g
//
// linearized around the reference yaw and the planned foot positions, then
// discretized (exact for this model up to O(dt^3)). The whole horizon is one
// sparse QP with hard friction-pyramid and normal-force limits, solved with
// OSQP.

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace OsqpEigen {
class Solver;
}

namespace wbc {

using Vector12d = Eigen::Matrix<double, 12, 1>;

struct SrbdMpcConfig {
  int horizon = 14;     // steps
  double dt = 0.03;     // [s]
  double mu = 0.6;      // friction pyramid coefficient
  double fz_max = 500.0;  // [N] per foot
  // State weights: rpy, p, omega, v
  Vector12d state_weights =
      (Vector12d() << 50, 50, 10, 30, 30, 100, 1, 1, 1, 5, 5, 5).finished();
  // Weight on the deviation of each force from its share of the weight,
  // with forces normalized by the robot weight (m g) so that the trade-off
  // does not depend on the robot's mass.
  double force_weight = 0.5;
};

struct SrbdMpcProblem {
  Vector12d x0;
  std::vector<Vector12d> x_ref;  // horizon entries, reference of x_1..x_N
  std::vector<double> yaw;       // horizon entries, linearization yaw of step k
  // horizon x nf: foot position relative to the CoM (world) and contact flag
  // during step k (from t_k to t_k+1)
  std::vector<std::vector<Eigen::Vector3d>> r;
  std::vector<std::vector<bool>> contact;
};

class SrbdMpc {
public:
  SrbdMpc(double mass, const Eigen::Matrix3d &inertia_body, int num_feet,
          const SrbdMpcConfig &config);
  ~SrbdMpc();

  // Returns false if OSQP did not find a solution.
  bool solve(const SrbdMpcProblem &problem);

  // Predicted states x_0..x_N (x_0 = problem.x0) and forces u_0..u_N-1.
  const std::vector<Vector12d> &states() const { return xs_; }
  const std::vector<Eigen::VectorXd> &forces() const { return us_; }

  // Continuous-time state derivative for given forces (used for output).
  Vector12d derivative(const Vector12d &x, const Eigen::VectorXd &u,
                       const std::vector<Eigen::Vector3d> &r) const;

private:
  void stepMatrices(double yaw, const std::vector<Eigen::Vector3d> &r,
                    Eigen::Matrix<double, 12, 12> &A, Eigen::MatrixXd &B,
                    Vector12d &d) const;

  double mass_;
  Eigen::Matrix3d inertia_body_;
  int nf_, nu_, N_;
  SrbdMpcConfig config_;
  int n_var_, n_con_;

  std::unique_ptr<OsqpEigen::Solver> solver_;
  Eigen::SparseMatrix<double> H_, A_;
  Eigen::MatrixXd A_dense_;
  Eigen::VectorXd g_, lb_, ub_;
  std::vector<Vector12d> xs_;
  std::vector<Eigen::VectorXd> us_;
  bool initialized_ = false;
};

}  // namespace wbc
