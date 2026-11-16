#include "whole_body_controller/optimizer.hpp"
#include "pinocchio/spatial.hpp"
#include "whole_body_controller/system_state.hpp"
#include <Eigen/Dense>
#include <array>

namespace wbc {
template <typename SparseMatrixType, typename DenseMatrixType>
static void update_sparse_matrix(SparseMatrixType &sparse_mat,
                                 const DenseMatrixType &dense_mat) {
  for (int k = 0; k < sparse_mat.outerSize(); ++k) {
    for (typename SparseMatrixType::InnerIterator it(sparse_mat, k); it; ++it) {
      it.valueRef() = dense_mat(it.row(), it.col());
    }
  }
}

static Eigen::Matrix<double, 4, 3> construct_friction_pyramid(double mu) {
  Eigen::Matrix<double, 4, 3> friction_pyramid;
  friction_pyramid << 1, 0, -mu, -1, 0, -mu, 0, 1, -mu, 0, -1, -mu;
  return friction_pyramid;
}

Optimizer::Optimizer(const Eigen::Vector<double, 6> &base_weights,
                     const Eigen::Vector<double, 12> &joint_weights,
                     const Eigen::Vector<double, 12> &force_weights,
                     const Eigen::Vector3d &foot_weights, double kp_base_lin,
                     double kd_base_lin, double kp_base_ang, double kd_base_ang,
                     double kp_foot, double kd_foot, double fz_max, double mu)
    : optimization_weight_base_(base_weights.asDiagonal()),
      optimization_weight_joint_(joint_weights.asDiagonal()),
      optimization_weight_force_(force_weights.asDiagonal()),
      optimization_weight_foot_(foot_weights.asDiagonal()),
      friction_pyramid_(construct_friction_pyramid(mu)),
      kp_base_lin_(kp_base_lin), kd_base_lin_(kd_base_lin),
      kp_base_ang_(kp_base_ang), kd_base_ang_(kd_base_ang), kp_foot_(kp_foot),
      kd_foot_(kd_foot), fz_max(fz_max), mu(mu) {

  std::cout << "Base weights:\n"
            << optimization_weight_base_ << "\nJoint weights:\n"
            << optimization_weight_joint_ << "\nForce weights:\n"
            << optimization_weight_force_ << "\nFoot weights:\n"
            << optimization_weight_foot_ << "\n";
}
Optimizer::~Optimizer() {
  solver_.clearSolverVariables();
  solver_.clearSolver();
  solver_.settings()->resetDefaultSettings();
}
bool Optimizer::initialize() {
  dense_hesian_.setZero();
  dense_linear_constraints_.setZero();
  gradient_.setZero();
  lower_bound_.setZero();
  upper_bound_.setZero();

  // --- 1. Efficient Hessian Sparsity Template ---
  Eigen::Matrix<double, Consts::num_variables, Consts::num_variables>
      H_template;
  H_template.setZero();

  // Only allocate upper-triangular non-zeros for q_ddot block (18x18) and Force
  // block (12x12)
  H_template.topLeftCorner<Consts::nq_dot, Consts::nq_dot>()
      .triangularView<Eigen::Upper>()
      .setOnes();
  H_template.bottomRightCorner<Consts::n_forces, Consts::n_forces>()
      .setIdentity();

  hessian_ = H_template.sparseView();

  // --- 2. Efficient Linear Constraints Sparsity Template ---
  Eigen::Matrix<double, Consts::num_constraints, Consts::num_variables>
      A_template;
  A_template.setZero();

  // Dynamics: [ M_u (6x18) | -J_cu^T (6x12) ]
  A_template.block(0, 0, Consts::base_dof, Consts::num_variables).setOnes();

  // Contact Kinematics: [ J_c (12x18) | 0 (12x12) ]
  A_template.block(Consts::base_dof, 0, 12, Consts::nq_dot).setOnes();

  // Friction Pyramids & Fz: [ 0 (20x18) | Pyramids (20x12) ]
  A_template.block(18, Consts::nq_dot, 20, Consts::n_forces).setOnes();

  linear_constraints_ = A_template.sparseView();

  hessian_.makeCompressed();
  linear_constraints_.makeCompressed();

  // --- 3. Solver Settings ---
  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setAdaptiveRho(true);
  solver_.settings()->setAbsoluteTolerance(1e-4);
  solver_.settings()->setRelativeTolerance(1e-4);
  solver_.settings()->setMaxIteration(1000);

  // --- 4. Pass Workspace Data ---
  solver_.data()->setNumberOfVariables(Consts::num_variables);
  solver_.data()->setNumberOfConstraints(Consts::num_constraints);

  if (!solver_.data()->setHessianMatrix(hessian_))
    return false;
  if (!solver_.data()->setGradient(gradient_))
    return false;
  if (!solver_.data()->setLinearConstraintsMatrix(linear_constraints_))
    return false;
  if (!solver_.data()->setLowerBound(lower_bound_))
    return false;
  if (!solver_.data()->setUpperBound(upper_bound_))
    return false;

  return solver_.initSolver();
};

bool Optimizer::solve(const SystemState &system_state,
                      const TargetState &target_state, ControlOutput &output) {
  dense_hesian_.setZero();
  dense_linear_constraints_.setZero();
  gradient_.setZero();

  build_cost(system_state, target_state);
  build_dynamics_constraints(system_state.hu, system_state.mass_matrix,
                             system_state.foot_jac_lin_lwa);
  build_contact_constraints(system_state);
  build_friction_constraints(system_state);

  if (!dense_hesian_.allFinite() || !dense_linear_constraints_.allFinite()) {
    std::cerr << "[WBC ERROR] NaN/Inf detected in QP matrices!" << std::endl;
    output.q_ddot.setZero();
    output.contact_forces.setZero();
    return false;
  }

  update_sparse_matrix(hessian_, dense_hesian_);
  update_sparse_matrix(linear_constraints_, dense_linear_constraints_);

  if (!solver_.updateHessianMatrix(hessian_) ||
      !solver_.updateGradient(gradient_) ||
      !solver_.updateLinearConstraintsMatrix(linear_constraints_) ||
      !solver_.updateBounds(lower_bound_, upper_bound_)) {
    std::cerr << "[WBC ERROR] Failed to update OSQP matrices!" << std::endl;
    return false;
  }

  if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError ||
      solver_.getStatus() != OsqpEigen::Status::Solved) {
    std::cerr << "[WBC WARNING] OSQP failed to find optimal solution! Status: "
              << static_cast<int>(solver_.getStatus()) << std::endl;
    output.q_ddot.setZero();
    output.contact_forces.setZero();
    return false;
  }

  const auto &solution = solver_.getSolution();
  output.q_ddot = solution.head<Consts::nq_dot>();
  output.contact_forces = solution.tail<Consts::n_forces>();

  return true;
}
void Optimizer::build_cost(const SystemState &system_state,
                           const TargetState &target) {

  // 1. BASE TASK (SE3 Tracking in LWA Frame)
  // Pinocchio quaternion order: q[3]=x, q[4]=y, q[5]=z, q[6]=w
  Eigen::Quaterniond quat_curr(system_state.q[6], system_state.q[3],
                               system_state.q[4], system_state.q[5]);
  Eigen::Quaterniond quat_targ(target.q[6], target.q[3], target.q[4],
                               target.q[5]);

  pinocchio::SE3 pose_curr_odom(quat_curr, system_state.q.head<3>());
  pinocchio::SE3 pose_targ_odom(quat_targ, target.q.head<3>());

  pinocchio::Motion twist_err_local =
      pinocchio::log6(pose_curr_odom.actInv(pose_targ_odom));

  Eigen::Matrix<double, 6, 1> spatial_error_lwa;
  spatial_error_lwa << pose_curr_odom.rotation() * twist_err_local.linear(),
      pose_curr_odom.rotation() * twist_err_local.angular();

  Eigen::Vector3d v_curr_lwa =
      pose_curr_odom.rotation() * system_state.q_dot.head<3>();
  Eigen::Vector3d w_curr_lwa =
      pose_curr_odom.rotation() * system_state.q_dot.segment<3>(3);
  Eigen::Vector3d v_targ_lwa =
      pose_targ_odom.rotation() * target.q_dot.head<3>();
  Eigen::Vector3d w_targ_lwa =
      pose_targ_odom.rotation() * target.q_dot.segment<3>(3);

  Eigen::Matrix<double, 6, 1> vel_error_lwa;
  vel_error_lwa << (v_targ_lwa - v_curr_lwa), (w_targ_lwa - w_curr_lwa);

  Eigen::Matrix<double, 6, 1> a_ff_lwa;
  a_ff_lwa << pose_targ_odom.rotation() * target.q_ddot.head<3>(),
      pose_targ_odom.rotation() * target.q_ddot.segment<3>(3);

  Eigen::Matrix<double, 6, 1> base_pd_lwa;
  base_pd_lwa.head<3>() = kp_base_lin_ * spatial_error_lwa.head<3>() +
                          kd_base_lin_ * vel_error_lwa.head<3>();
  base_pd_lwa.tail<3>() = kp_base_ang_ * spatial_error_lwa.tail<3>() +
                          kd_base_ang_ * vel_error_lwa.tail<3>();

  Eigen::Matrix<double, 6, 1> rhs_base =
      a_ff_lwa + base_pd_lwa - system_state.base_drift_lwa;

  // 2. JOINT POSTURAL TASK
  Eigen::Vector<double, 12> j_acc =
      target.q_ddot.tail<12>() +
      target.j_kp.cwiseProduct(target.q.tail<12>() -
                               system_state.q.tail<12>()) +
      target.j_kd.cwiseProduct(target.q_dot.tail<12>() -
                               system_state.q_dot.tail<12>());

  // 3. HESSIAN & GRADIENT POPULATION
  // Base Task
  dense_hesian_.topLeftCorner<Consts::nq_dot, Consts::nq_dot>() =
      2.0 * system_state.base_jacobian_lwa.transpose() *
      optimization_weight_base_ * system_state.base_jacobian_lwa;

  gradient_.head<Consts::nq_dot>() =
      -2.0 * system_state.base_jacobian_lwa.transpose() *
      optimization_weight_base_ * rhs_base;

  // Diagonal Regularization for Numerical Stability
  constexpr double acc_regularization = 1e-4;
  dense_hesian_.topLeftCorner<Consts::nq_dot, Consts::nq_dot>()
      .diagonal()
      .array() += acc_regularization;

  // Joint Task
  dense_hesian_.block<12, 12>(6, 6) += 2.0 * optimization_weight_joint_;
  gradient_.segment<12>(6) -= 2.0 * optimization_weight_joint_ * j_acc;

  // Force Regularization
  Eigen::Vector<double, 12> adjusted_contact_forces =
      target.contact_forces_odom;
      for (int i = 0; i < 4; ++i) {
        if (!system_state.foot_state[i].contact) {
        // if leg is not actually contacting the ground.
        // Contact forces should be zero!
        adjusted_contact_forces.segment<3>(3 * i).setZero();
    }
  }

  dense_hesian_.bottomRightCorner(Consts::n_forces, Consts::n_forces) =
      2.0 * optimization_weight_force_;
  gradient_.tail(Consts::n_forces) =
      -2.0 * optimization_weight_force_ * adjusted_contact_forces;//target.contact_forces_odom;

  // 4. SWING and REFLEX FOOT TASKS
  for (int i = 0; i < 4; ++i) {
    bool mpc_contact = target.foot_state[i].contact;
    bool wbc_contact = system_state.foot_state[i].contact;

    // ONLY add a Cartesian task if the foot is physically in the air (!wbc_contact).
    // If wbc_contact == true, build_contact_constraints already locks the foot with
    // a hard equality constraint (J_f * q_ddot = -J_dot * q_dot).
    if (!wbc_contact) {
      Eigen::Matrix<double, 3, Consts::nq_dot> J_f =
          system_state.foot_jac_lin_lwa.middleRows<3>(3 * i);

      Eigen::Vector3d rhs_foot;

      if (!mpc_contact) {
        // 1. NOMINAL SWING
        rhs_foot = kp_foot_ * (target.foot_state[i].foot_pos_world -
                               system_state.foot_state[i].foot_pos_world) +
                   kd_foot_ * (target.foot_state[i].foot_vel_lwa -
                               system_state.foot_state[i].foot_vel_lwa) +
                   (target.foot_state[i].foot_acc_lwa -
                    system_state.foot_drift_lin_lwa[i]);
      } else {
        // 2. LATE TOUCHDOWN REFLEX
        Eigen::Vector3d reach_vel(0.0, 0.0, -0.5); 
        rhs_foot =
            kd_foot_ * (reach_vel - system_state.foot_state[i].foot_vel_lwa) -
            system_state.foot_drift_lin_lwa[i];
      }

      dense_hesian_.topLeftCorner<Consts::nq_dot, Consts::nq_dot>() +=
          2.0 * J_f.transpose() * optimization_weight_foot_ * J_f;

      gradient_.head<Consts::nq_dot>() -=
          2.0 * J_f.transpose() * optimization_weight_foot_ * rhs_foot;
    }
  }
}

void Optimizer::build_dynamics_constraints(
    const Eigen::Vector<double, Consts::base_dof> &hu,
    const Eigen::MatrixXd &mass_matrix,
    const Eigen::Matrix<double, 12, Consts::nq_dot> &contact_jacobian) {

  dense_linear_constraints_.block(0, 0, Consts::base_dof, Consts::nq_dot) =
      mass_matrix.topRows(Consts::base_dof);

  Eigen::Matrix<double, 6, 12> J_c_top_trans =
      contact_jacobian.transpose().topRows(Consts::base_dof);

  dense_linear_constraints_.block(0, Consts::nq_dot, Consts::base_dof, 12) =
      -J_c_top_trans;

  lower_bound_.segment(0, Consts::base_dof) = -hu;
  upper_bound_.segment(0, Consts::base_dof) = -hu;
}

void Optimizer::build_contact_constraints(const SystemState &system_state) {
  dense_linear_constraints_.block(6, 0, 12, Consts::nq_dot) =
      system_state.foot_jac_lin_lwa;

  for (int i = 0; i < 4; ++i) {
    // if foot is actually in contact - constrain its acceleration
    if (system_state.foot_state[i].contact) {
      // Stance: J_c * qdd = -gamma_c
      lower_bound_.segment(6 + 3 * i, 3) = -system_state.foot_drift_lin_lwa[i];
      upper_bound_.segment(6 + 3 * i, 3) = -system_state.foot_drift_lin_lwa[i];
    } else {
      // Swing: Unconstrained acceleration constraint
      lower_bound_.segment(6 + 3 * i, 3).setConstant(-OsqpEigen::INFTY);
      upper_bound_.segment(6 + 3 * i, 3).setConstant(OsqpEigen::INFTY);
    }
  }
}

void Optimizer::build_friction_constraints(const SystemState &system_state) {
  for (int i = 0; i < 4; ++i) {
    int f_idx = Consts::nq_dot + 3 * i;

    dense_linear_constraints_.block(18 + 4 * i, f_idx, 4, 3) =
        friction_pyramid_;
    dense_linear_constraints_(34 + i, f_idx + 2) = 1.0; // Fz selector

    if (system_state.foot_state[i].contact) {
      // Friction pyramid: -inf <= A_fric * F <= 0
      lower_bound_.segment(18 + 4 * i, 4).setConstant(-OsqpEigen::INFTY);
      upper_bound_.segment(18 + 4 * i, 4).setConstant(0.0);

      // Normal Force: Fz_min <= Fz <= Fz_max (Ground pushes UP)
      lower_bound_(34 + i) = 0.0;
      upper_bound_(34 + i) = fz_max;
    } else {
      // Swing Foot: Friction pyramid unconstrained to prevent dual degeneracy
      lower_bound_.segment(18 + 4 * i, 4).setConstant(-OsqpEigen::INFTY);
      upper_bound_.segment(18 + 4 * i, 4).setConstant(0.0);

      // Zero normal force directly zeroes all contact forces (F_x = F_y = F_z =
      // 0)
      lower_bound_(34 + i) = 0.0;
      upper_bound_(34 + i) = 0.0;
    }
  }
}

} // namespace wbc