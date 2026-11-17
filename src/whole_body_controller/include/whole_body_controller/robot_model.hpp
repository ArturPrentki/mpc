#pragma once
// Robot description for the WBC: everything robot-specific is derived from the
// URDF (through Pinocchio) plus a small RobotConfig. No robot names, sizes or
// geometry are hard-coded anywhere else in the controller.

#include "whole_body_controller/types.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <vector>

namespace wbc {

struct RobotConfig {
  std::string base_frame;                     // e.g. "base", "TORSO"
  std::vector<std::string> foot_frames;       // contact points, any count
  std::vector<std::string> joint_names;       // actuated joints, config order
  std::vector<double> nominal_joint_positions;  // config order
  double nominal_height = 0.0;  // base height when standing; <= 0: from FK
};

class RobotModel {
public:
  // Throws std::runtime_error with a readable message if the URDF and the
  // config do not match.
  static RobotModel fromUrdf(const std::string &urdf_xml,
                             const RobotConfig &config);
  RobotModel(const pinocchio::Model &model, const RobotConfig &config);

  const pinocchio::Model &model() const { return model_; }
  const RobotConfig &config() const { return config_; }
  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  int nj() const { return static_cast<int>(joint_q_idx_.size()); }
  int nf() const { return static_cast<int>(foot_ids_.size()); }
  double totalMass() const { return total_mass_; }

  pinocchio::FrameIndex baseFrameId() const { return base_id_; }
  pinocchio::FrameIndex footFrameId(int f) const { return foot_ids_[f]; }
  // Config joint index -> Pinocchio q / v index
  int jointQIdx(int j) const { return joint_q_idx_[j]; }
  int jointVIdx(int j) const { return joint_v_idx_[j]; }
  // Config joint indices of the joints moving foot f (its kinematic chain)
  const std::vector<int> &legJoints(int f) const { return leg_joints_[f]; }
  // Foot each config joint belongs to (-1 if none)
  int footOfJoint(int j) const { return joint_foot_[j]; }

  const Eigen::VectorXd &effortLimits() const { return effort_limits_; }
  double nominalHeight() const { return nominal_height_; }
  // Foot positions relative to the base projected on the ground, in the
  // yaw-aligned base frame, z = 0 (ground).
  const std::vector<Eigen::Vector3d> &nominalFootOffsets() const {
    return nominal_feet_;
  }
  // Whole-body centre of mass in the base frame and rotational inertia about
  // it (base axes), both at the nominal standing pose. Used by the
  // single-rigid-body MPC.
  const Eigen::Vector3d &comOffset() const { return com_offset_; }
  const Eigen::Matrix3d &bodyInertia() const { return body_inertia_; }

  // --- conversions between config order and Pinocchio layout -------------
  void jointsToQ(const Eigen::VectorXd &joints, Eigen::VectorXd &q) const;
  void jointsToV(const Eigen::VectorXd &joints, Eigen::VectorXd &v) const;
  void jointsFromQ(const Eigen::VectorXd &q, Eigen::VectorXd &joints) const;
  void jointsFromV(const Eigen::VectorXd &v, Eigen::VectorXd &joints) const;

  // --- helpers (allocate: NOT for the 1 kHz loop) -------------------------
  // Full configuration with nominal joints, base at pos, yaw only.
  Eigen::VectorXd nominalConfiguration(const Eigen::Vector3d &base_pos,
                                       double yaw) const;
  // Damped least-squares IK on the leg joints so that every foot reaches its
  // world target; the floating base in q stays fixed. Returns max error [m].
  double solveFootIK(Eigen::VectorXd &q,
                     const std::vector<Eigen::Vector3d> &feet_world,
                     int max_iter = 100) const;
  // Standing target: base above (x, y) at ground_z + nominal height with the
  // given yaw, feet at the nominal stance on the ground, all feet in contact
  // carrying an equal share of the weight.
  TargetState standTarget(const Eigen::Vector3d &base_xy, double yaw,
                          double ground_z) const;

private:
  pinocchio::Model model_;
  RobotConfig config_;
  pinocchio::FrameIndex base_id_ = 0;
  std::vector<pinocchio::FrameIndex> foot_ids_;
  std::vector<int> joint_q_idx_, joint_v_idx_, joint_foot_;
  std::vector<std::vector<int>> leg_joints_;
  Eigen::VectorXd effort_limits_;
  Eigen::VectorXd q_nom_joints_;
  double nominal_height_ = 0.0;
  double total_mass_ = 0.0;
  std::vector<Eigen::Vector3d> nominal_feet_;
  Eigen::Vector3d com_offset_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d body_inertia_ = Eigen::Matrix3d::Identity();
};

// Yaw of a rotation matrix (ZYX convention).
inline double yawOf(const Eigen::Matrix3d &R) {
  return std::atan2(R(1, 0), R(0, 0));
}

}  // namespace wbc
