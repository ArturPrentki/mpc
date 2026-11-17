#include "whole_body_controller/robot_model.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace wbc {

RobotModel RobotModel::fromUrdf(const std::string &urdf_xml,
                                const RobotConfig &config) {
  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml,
                                       pinocchio::JointModelFreeFlyer(), model);
  } catch (const std::exception &e) {
    throw std::runtime_error(std::string("Failed to parse URDF: ") + e.what());
  }
  return RobotModel(model, config);
}

RobotModel::RobotModel(const pinocchio::Model &model, const RobotConfig &config)
    : model_(model), config_(config) {
  std::ostringstream err;

  // --- floating base --------------------------------------------------------
  if (model_.njoints < 2 || model_.joints[1].shortname() != "JointModelFreeFlyer" ||
      model_.joints[1].idx_v() != 0) {
    throw std::runtime_error(
        "Model must have a free-flyer root joint as its first joint");
  }
  if (!model_.existFrame(config_.base_frame)) {
    throw std::runtime_error("base_frame '" + config_.base_frame +
                             "' not found in URDF");
  }
  base_id_ = model_.getFrameId(config_.base_frame);
  if (model_.frames[base_id_].parentJoint != 1) {
    throw std::runtime_error("base_frame '" + config_.base_frame +
                             "' must be rigidly attached to the floating base");
  }

  // --- joints ---------------------------------------------------------------
  if (config_.joint_names.empty()) {
    throw std::runtime_error("joint_names is empty");
  }
  if (static_cast<int>(config_.joint_names.size()) != model_.nv - 6) {
    err << "joint_names has " << config_.joint_names.size()
        << " entries but the URDF has " << model_.nv - 6
        << " actuated DoF; every non-fixed joint must be listed";
    throw std::runtime_error(err.str());
  }
  for (const auto &name : config_.joint_names) {
    if (!model_.existJointName(name)) {
      throw std::runtime_error("joint '" + name + "' not found in URDF");
    }
    const auto jid = model_.getJointId(name);
    if (model_.joints[jid].nv() != 1 || model_.joints[jid].nq() != 1) {
      throw std::runtime_error("joint '" + name +
                               "' must be a 1-DoF revolute/prismatic joint");
    }
    if (std::find(joint_q_idx_.begin(), joint_q_idx_.end(),
                  model_.joints[jid].idx_q()) != joint_q_idx_.end()) {
      throw std::runtime_error("joint '" + name + "' listed twice");
    }
    joint_q_idx_.push_back(model_.joints[jid].idx_q());
    joint_v_idx_.push_back(model_.joints[jid].idx_v());
  }
  const int n = nj();
  effort_limits_.resize(n);
  for (int j = 0; j < n; ++j) {
    double lim = model_.effortLimit[joint_v_idx_[j]];
    if (!(lim > 0.0) || !std::isfinite(lim)) {
      throw std::runtime_error("joint '" + config_.joint_names[j] +
                               "' has no valid effort limit in the URDF");
    }
    effort_limits_[j] = lim;
  }

  if (static_cast<int>(config_.nominal_joint_positions.size()) != n) {
    err << "nominal_joint_positions has "
        << config_.nominal_joint_positions.size() << " entries, expected " << n;
    throw std::runtime_error(err.str());
  }
  q_nom_joints_ = Eigen::Map<const Eigen::VectorXd>(
      config_.nominal_joint_positions.data(), n);

  // --- feet -----------------------------------------------------------------
  if (config_.foot_frames.empty()) {
    throw std::runtime_error("foot_frames is empty");
  }
  joint_foot_.assign(n, -1);
  for (const auto &name : config_.foot_frames) {
    if (!model_.existFrame(name)) {
      throw std::runtime_error("foot frame '" + name + "' not found in URDF");
    }
    const auto fid = model_.getFrameId(name);
    foot_ids_.push_back(fid);

    // Config joints on the chain root -> foot
    std::vector<int> chain;
    const auto &support = model_.supports[model_.frames[fid].parentJoint];
    for (const auto jid : support) {
      if (jid < 2) continue;  // universe, free-flyer
      const int vidx = model_.joints[jid].idx_v();
      auto it = std::find(joint_v_idx_.begin(), joint_v_idx_.end(), vidx);
      if (it == joint_v_idx_.end()) continue;
      const int j = static_cast<int>(it - joint_v_idx_.begin());
      chain.push_back(j);
      joint_foot_[j] = static_cast<int>(foot_ids_.size()) - 1;
    }
    if (chain.empty()) {
      throw std::runtime_error("foot frame '" + name +
                               "' is not moved by any actuated joint");
    }
    leg_joints_.push_back(chain);
  }

  total_mass_ = 0.0;
  for (const auto &inertia : model_.inertias) total_mass_ += inertia.mass();

  // --- nominal stance from forward kinematics -------------------------------
  pinocchio::Data data(model_);
  Eigen::VectorXd q = nominalConfiguration(Eigen::Vector3d::Zero(), 0.0);
  pinocchio::framesForwardKinematics(model_, data, q);
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto fid : foot_ids_) {
    lowest = std::min(lowest, data.oMf[fid].translation().z());
  }
  if (!(lowest < 0.0)) {
    throw std::runtime_error(
        "With nominal_joint_positions the feet are not below the base");
  }
  nominal_height_ =
      config_.nominal_height > 0.0 ? config_.nominal_height : -lowest;
  for (const auto fid : foot_ids_) {
    Eigen::Vector3d p = data.oMf[fid].translation();
    p.z() = 0.0;
    nominal_feet_.push_back(p);
  }

  // Composite rigid body at the nominal pose (base at the origin, identity
  // orientation, so world axes = base axes).
  const pinocchio::SE3 &oMb = data.oMf[base_id_];
  pinocchio::ccrba(model_, data, q, Eigen::VectorXd::Zero(model_.nv));
  com_offset_ = oMb.actInv(data.com[0]);
  body_inertia_ = oMb.rotation().transpose() * data.Ig.inertia().matrix() *
                  oMb.rotation();
}

void RobotModel::jointsToQ(const Eigen::VectorXd &joints,
                           Eigen::VectorXd &q) const {
  for (int j = 0; j < nj(); ++j) q[joint_q_idx_[j]] = joints[j];
}
void RobotModel::jointsToV(const Eigen::VectorXd &joints,
                           Eigen::VectorXd &v) const {
  for (int j = 0; j < nj(); ++j) v[joint_v_idx_[j]] = joints[j];
}
void RobotModel::jointsFromQ(const Eigen::VectorXd &q,
                             Eigen::VectorXd &joints) const {
  for (int j = 0; j < nj(); ++j) joints[j] = q[joint_q_idx_[j]];
}
void RobotModel::jointsFromV(const Eigen::VectorXd &v,
                             Eigen::VectorXd &joints) const {
  for (int j = 0; j < nj(); ++j) joints[j] = v[joint_v_idx_[j]];
}

Eigen::VectorXd RobotModel::nominalConfiguration(const Eigen::Vector3d &base_pos,
                                                 double yaw) const {
  Eigen::VectorXd q = pinocchio::neutral(model_);
  q.head<3>() = base_pos;
  Eigen::Quaterniond quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  q.segment<4>(3) = quat.coeffs();  // x y z w
  jointsToQ(q_nom_joints_, q);
  return q;
}

double RobotModel::solveFootIK(Eigen::VectorXd &q,
                               const std::vector<Eigen::Vector3d> &feet_world,
                               int max_iter) const {
  pinocchio::Data data(model_);
  Eigen::MatrixXd J6(6, model_.nv);
  double max_err = 0.0;
  for (int it = 0; it < max_iter; ++it) {
    pinocchio::computeJointJacobians(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    max_err = 0.0;
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(model_.nv);
    for (int f = 0; f < nf(); ++f) {
      const Eigen::Vector3d e =
          feet_world[f] - data.oMf[foot_ids_[f]].translation();
      max_err = std::max(max_err, e.norm());
      J6.setZero();
      pinocchio::getFrameJacobian(model_, data, foot_ids_[f],
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6);
      const auto &leg = leg_joints_[f];
      Eigen::MatrixXd Jl(3, leg.size());
      for (size_t k = 0; k < leg.size(); ++k) {
        Jl.col(k) = J6.block<3, 1>(0, joint_v_idx_[leg[k]]);
      }
      constexpr double damping = 1e-6;
      const Eigen::VectorXd dql =
          Jl.transpose() *
          (Jl * Jl.transpose() + damping * Eigen::Matrix3d::Identity())
              .ldlt()
              .solve(e);
      for (size_t k = 0; k < leg.size(); ++k) dv[joint_v_idx_[leg[k]]] = dql[k];
    }
    if (max_err < 1e-6) break;
    q = pinocchio::integrate(model_, q, dv);
  }
  return max_err;
}

TargetState RobotModel::standTarget(const Eigen::Vector3d &base_xy, double yaw,
                                    double ground_z) const {
  TargetState t;
  t.resize(nq(), nv(), nf());
  const Eigen::Vector3d base(base_xy.x(), base_xy.y(),
                             ground_z + nominal_height_);
  t.q = nominalConfiguration(base, yaw);
  const Eigen::Matrix3d Rz =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<Eigen::Vector3d> feet;
  const double fz = total_mass_ * 9.81 / nf();
  for (int f = 0; f < nf(); ++f) {
    Eigen::Vector3d p = Rz * nominal_feet_[f];
    p.x() += base_xy.x();
    p.y() += base_xy.y();
    p.z() = ground_z;
    feet.push_back(p);
    t.feet[f].pos = p;
    t.feet[f].contact = true;
    t.feet[f].force = Eigen::Vector3d(0.0, 0.0, fz);
  }
  solveFootIK(t.q, feet);
  return t;
}

}  // namespace wbc
