#pragma once
#include "whole_body_controller/system_state.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <OsqpEigen/OsqpEigen.h>

namespace wbc {
class Optimizer {
public:
  Optimizer(const Eigen::Vector<double, 6> &base_weights,
            const Eigen::Vector<double, 12> &joint_weights,
            const Eigen::Vector<double, 12> &force_weights,
            const Eigen::Vector3d &foot_weights, double kp_base_lin,
            double kd_base_lin, double kp_base_ang, double kd_base_ang,
            double kp_foot, double kd_foot, double fz_max, double mu);
  ~Optimizer();

  // initialize using dynamic system as reference for matrices
  bool initialize();

  bool solve(const SystemState &current, const TargetState &target,
             ControlOutput &output);

private:
  // constants in run-time
  const Eigen::Matrix<double, 6, 6> optimization_weight_base_;
  const Eigen::Matrix<double, 12, 12> optimization_weight_joint_;
  const Eigen::Matrix<double, Consts::n_forces, Consts::n_forces>
      optimization_weight_force_;
  const Eigen::Matrix3d optimization_weight_foot_;

  const Eigen::Matrix<double, 4, 3> friction_pyramid_;
  const double kp_base_lin_, kd_base_lin_;
  const double kp_base_ang_, kd_base_ang_;
  const double kp_foot_;
  const double kd_foot_;
  const double fz_max; // Max normal force per leg
  const double mu;     // 1.5;       // Friction coefficient
  void build_cost(const SystemState &current_state,
                  const TargetState &target_state);
  void build_dynamics_constraints(
      const Eigen::Vector<double, Consts::base_dof> &hu,
      const Eigen::MatrixXd &mass_matrix,
      const Eigen::Matrix<double, 12, Consts::nq_dot> &contact_jacobian);
  void build_contact_constraints(const SystemState &system_state);
  void build_friction_constraints(const SystemState &system_state);

  OsqpEigen::Solver solver_;
  // Pre-allocated Matrices
  std::vector<int> leg_v_idx_;
  Eigen::Matrix<double, Consts::num_variables, Consts::num_variables>
      dense_hesian_;
  Eigen::Matrix<double, Consts::num_constraints, Consts::num_variables>
      dense_linear_constraints_;
  Eigen::SparseMatrix<double> hessian_;
  Eigen::SparseMatrix<double> linear_constraints_;

  Eigen::Matrix<double, 12, Consts::nq_dot> S_joint;
  Eigen::Matrix<double, Consts::num_variables, 1> gradient_;
  Eigen::Matrix<double, Consts::num_constraints, 1> lower_bound_, upper_bound_;

  // bool is_initialized_;
};
} // namespace wbc