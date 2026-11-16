#pragma once
#include "Eigen/Dense"
#include "array"
#include <Eigen/src/Core/Matrix.h>

namespace wbc {
struct Consts {
  static constexpr int nq = 19;
  static constexpr int nq_dot = 18;
  static constexpr int base_dof = 6;
  static constexpr int actuated_dof = 12;
  static constexpr int n_forces = 12;
  static constexpr int num_variables = Consts::nq_dot + n_forces;
  static constexpr int num_constraints = 38;
};

// feet represented in local world aligned frame
struct FootState {
  bool contact; // actual contact tracking

  Eigen::Vector3d foot_pos_world;
  Eigen::Vector3d foot_vel_lwa;
  Eigen::Vector3d foot_acc_lwa;
};
inline std::ostream &operator<<(std::ostream &os, const FootState &fs) {
  os << "{ contact: " << (fs.contact ? "true" : "false") << ", pos: ["
     << fs.foot_pos_world.transpose() << "]";
  return os;
}

// World reference of q, q_dot
struct SystemState {
  
  Eigen::Vector<double, 19> q;
  Eigen::Vector<double, 18> q_dot;
  Eigen::Vector<double, 12> j_torq;

  std::array<FootState, 4> foot_state;
  Eigen::Matrix<double, Consts::base_dof, Consts::nq_dot>
      base_jacobian_lwa;
  Eigen::Matrix<double, 12, Consts::nq_dot> foot_jac_lin_lwa;
  std::array<Eigen::Vector3d, 4> foot_drift_lin_lwa;
  Eigen::Vector<double, 6> base_drift_lwa;
  Eigen::Matrix<double, Consts::base_dof, 1> hu;
  Eigen::MatrixXd mass_matrix;
};

inline std::ostream &operator<<(std::ostream &os, const SystemState &ss) {
  os << "SystemState {\n"
     << "Base: {\n"
     << "  q: [" << ss.q.head<7>().transpose() << "]\n"
     << "  v: [" << ss.q_dot.head<6>().transpose() << "]\n"
     << "}\nJoints: {\n"
     << "  q: [" << ss.q.tail<12>().transpose() << "]\n"
     << "  v: [" << ss.q_dot.tail<12>().transpose() << "]\n}\n"
     << "Feet: [\n";
  for (size_t i = 0; i < ss.foot_state.size(); ++i) {
    os << ss.foot_state[i] << "\n"; //(i + 1 < ss.fs.size() ? ", " : "");
  }
  os << " ]\n";
  return os;
}

struct TargetState {
  Eigen::Vector<double, 19> q;
  Eigen::Vector<double, 18> q_dot;
  Eigen::Vector<double, 18> q_ddot;
  Eigen::Vector<double, 12> contact_forces_odom;

  std::array<FootState, 4> foot_state;
  Eigen::Vector<double, 12> j_kp;
  Eigen::Vector<double, 12> j_kd;
};
inline std::ostream &operator<<(std::ostream &os, const TargetState &ts) {
  os << "TargetState {\n"
     << "  q: [" << ts.q.transpose() << "]\n"
    //  << "  q_dot: [" << ts.q_dot.transpose() << "]\n"
    //  << "  q_ddot: [" << ts.q_ddot.transpose() << "]\n"

     << "  fs: [ ";
  for (size_t i = 0; i < ts.foot_state.size(); ++i) {
    os << ts.foot_state[i] << "\n";//(i + 1 < ts.fs.size() ? ", " : "");
  }
  os << " ]\n"
    //  << "  contact_forces: [" << ts.contact_forces.transpose() << "]\n"
     << "}";
  return os;
}
struct ControlOutput {
  Eigen::Vector<double, 18> q_ddot;
  Eigen::Vector<double, 12> contact_forces; //?
  Eigen::Vector<double, 12> torque;
};
inline std::ostream &operator<<(std::ostream &os, const ControlOutput &co) {
  os << "ControlOutput {\n"
     << "  q_ddot: [" << co.q_ddot.transpose() << "]\n"
     << "  contact_forces: [" << co.contact_forces.transpose() << "]\n"
     << "  torque: [" << co.torque.transpose() << "]\n"
     << "}";
  return os;
}
} // namespace wbc