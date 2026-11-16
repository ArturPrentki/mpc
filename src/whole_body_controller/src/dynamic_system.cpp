#include "whole_body_controller/dynamic_system.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "whole_body_controller/system_state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <array>
#include <iostream>
#include <memory>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <vector>

namespace wbc {
static std::array<int, 4>
initialize_frame_id_arr(const pinocchio::Model &model,
                        const std::vector<std::string> &arg) {
  std::array<int, 4> temp{};
  for (int i = 0; i < 4; ++i) {
    temp[i] = model.getFrameId(arg[i]);
  }
  return temp;
}
static std::array<int, 4>
initialize_joint_id_arr(const pinocchio::Model &model,
                        const std::vector<std::string> &arg) {
  std::array<int, 4> temp{};
  for (int i = 0; i < 4; ++i) {
    temp[i] = model.getJointId(arg[i]);
  }
  return temp;
}
static std::array<int, 12>
initialize_v_id_arr(const pinocchio::Model &model,
                    const std::vector<std::string> &arg) {
  std::array<int, 12> temp{};
  for (int i = 0; i < 12; ++i) {
    temp[i] = model.joints[model.getJointId(arg[i])].idx_v();
  }

  return temp;
}
static bool check_if_all_exist(const pinocchio::Model &model,
                               const std::vector<std::string> &frames) {
  for (const auto &frame : frames) {
    if (!model.existFrame(frame)) {
      std::cout << "MODEL IS INVALID, because " << frame << " doesn't exist!\n";
      return false;
    }
  }
  return true;
}
bool DynamicSystem::validate_model(
    const pinocchio::Model model, const std::string &base_name,
    const std::vector<std::string> &hip_names,
    const std::vector<std::string> &thigh_names,
    const std::vector<std::string> &calf_names,
    const std::vector<std::string> &feet_names,
    const std::vector<std::string> &joints_names) {
  if (!model.existFrame(base_name))
    return false;
  if (!check_if_all_exist(model, hip_names))
    return false;
  if (!check_if_all_exist(model, thigh_names))
    return false;
  if (!check_if_all_exist(model, calf_names))
    return false;
  if (!check_if_all_exist(model, feet_names))
    return false;
  if (!check_if_all_exist(model, joints_names))
    return false;
  return true;
}

DynamicSystem::DynamicSystem(const pinocchio::Model model,
                             const std::string &base_name,
                             const std::vector<std::string> &hip_names,
                             const std::vector<std::string> &thigh_names,
                             const std::vector<std::string> &calf_names,
                             const std::vector<std::string> &feet_names,
                             const std::vector<std::string> &joints_names)
    : model(model), base_frame_id_(model.getFrameId(base_name)),
      hip_ids_(initialize_joint_id_arr(model, hip_names)),
      thigh_ids_(initialize_joint_id_arr(model, thigh_names)),
      calf_ids_(initialize_joint_id_arr(model, calf_names)),
      foot_ids_(initialize_frame_id_arr(model, feet_names)),
      joint_v_indices_(initialize_v_id_arr(model, joints_names)) {
  pin_data_ = std::make_unique<pinocchio::Data>(model);
  kin_data_ = std::make_unique<pinocchio::Data>(model);
}

void DynamicSystem::inverse_kinematics(TargetState &target) const {
  // 1. Reconstruct floating-base state
  Eigen::Vector3d body_pos_odom(target.q[0], target.q[1], target.q[2]);
  Eigen::Quaterniond body_quat_odom(target.q[6], target.q[3], target.q[4],
                                    target.q[5]); // (w, x, y, z)

  // Pinocchio floating base velocities are in the LOCAL body frame.
  Eigen::Vector3d v_body_local(target.q_dot[0], target.q_dot[1],
                               target.q_dot[2]);
  Eigen::Vector3d w_body_local(target.q_dot[3], target.q_dot[4],
                               target.q_dot[5]);

  // Rotate local velocities into the Odom frame for world-frame arithmetic
  Eigen::Vector3d v_body_odom = body_quat_odom * v_body_local;
  Eigen::Vector3d w_body_odom = body_quat_odom * w_body_local;

  // SE3(R, p) represents the pose of the base in odom, transforming points FROM
  // base TO odom.
  pinocchio::SE3 base_to_odom(body_quat_odom.toRotationMatrix(), body_pos_odom);

  for (int i = 0; i < 4; ++i) {
    // --- 2. TRANSFORMS & POSITION IK ---
    const pinocchio::SE3 &base_to_hip = model.jointPlacements[hip_ids_[i]];

    // T_odom_hip = T_odom_base * T_base_hip
    pinocchio::SE3 hip_to_odom = base_to_odom * base_to_hip;

    // Transform foot position from Odom down to local Hip frame using inverse
    Eigen::Vector3d foot_in_hip =
        hip_to_odom.actInv(target.foot_state[i].foot_pos_world);

    double x = foot_in_hip.x();
    double y = foot_in_hip.y();
    double z = foot_in_hip.z();

    // Link lengths from Pinocchio joint placements
    const pinocchio::SE3 &hip_to_thigh = model.jointPlacements[thigh_ids_[i]];
    double mul = (hip_to_thigh.translation().y() > 0) ? 1.0 : -1.0;
    double l_hip = hip_to_thigh.translation().norm() * mul;
    const pinocchio::SE3 &thigh_to_calf = model.jointPlacements[calf_ids_[i]];
    double l_thigh = thigh_to_calf.translation().norm();
    const pinocchio::SE3 &calf_to_foot = model.frames[foot_ids_[i]].placement;
    double l_calf = calf_to_foot.translation().norm();

    // Position IK
    double L2 = x * x + y * y + z * z - l_hip * l_hip;
    double L = std::sqrt(std::max(0.0, L2));

    double q1 = std::atan2(y, -z) - std::atan2(l_hip, L);
    double cos_q3 =
        (L2 - l_thigh * l_thigh - l_calf * l_calf) / (2.0 * l_thigh * l_calf);
    double q3 = -std::acos(std::clamp(cos_q3, -1.0, 1.0));
    double q2 = std::atan2(-x, L) - std::atan2(l_calf * std::sin(q3),
                                               l_thigh + l_calf * std::cos(q3));

    // Save joint angles
    target.q.segment<3>(7 + i * 3) << q1, q2, q3;

    // --- 3. VELOCITY IK (q_dot) ---

    // Vector from Base Origin to Foot (in Odom frame)
    Eigen::Vector3d r_foot_world =
        target.foot_state[i].foot_pos_world - body_pos_odom;

    // Velocity of the foot relative to the moving base, expressed in Odom frame
    Eigen::Vector3d v_foot_rel_world = target.foot_state[i].foot_vel_lwa -
                                       v_body_odom -
                                       w_body_odom.cross(r_foot_world);

    // Rotate the relative velocity into the local Hip frame
    // R^T rotates from odom to hip
    Eigen::Vector3d v_hip =
        hip_to_odom.rotation().transpose() * v_foot_rel_world;

    // Construct 3x3 Analytical Leg Jacobian J_leg in Hip Frame: v_hip = J_leg *
    // qdot
    double s1 = std::sin(q1), c1 = std::cos(q1);
    double s2 = std::sin(q2), c2 = std::cos(q2);
    double s23 = std::sin(q2 + q3), c23 = std::cos(q2 + q3);

    Eigen::Matrix3d J_leg;

    // Column 1: d(foot)/dq1
    J_leg(0, 0) = 0.0;
    J_leg(1, 0) = -l_hip * s1 + c1 * (l_thigh * c2 + l_calf * c23);
    J_leg(2, 0) = l_hip * c1 + s1 * (l_thigh * c2 + l_calf * c23);

    // Column 2: d(foot)/dq2
    J_leg(0, 1) = -l_thigh * c2 - l_calf * c23;
    J_leg(1, 1) = -s1 * (l_thigh * s2 + l_calf * s23);
    J_leg(2, 1) = c1 * (l_thigh * s2 + l_calf * s23);

    // Column 3: d(foot)/dq3
    J_leg(0, 2) = -l_calf * c23;
    J_leg(1, 2) = -l_calf * s1 * s23;
    J_leg(2, 2) = l_calf * c1 * s23;

    // Solve q_dot = J_leg^-1 * v_hip
    Eigen::Vector3d qdot = J_leg.colPivHouseholderQr().solve(v_hip);
    target.q_dot.segment<3>(6 + i * 3) = qdot;
  }
}

void DynamicSystem::inverse_dynamics(const SystemState &system_state,
                                     ControlOutput &target) const {

  Eigen::Matrix<double, 18, 1> tau_full =
      pin_data_->M * target.q_ddot + pin_data_->nle -
      system_state.foot_jac_lin_lwa.transpose() * target.contact_forces;
  if (tau_full.head<6>().norm() > 1.0) {
    std::cout << "WARNING: WBC violated floating base dynamics! Residual: "
              << tau_full.head<6>().transpose() << std::endl;
    target.torque.setZero();
  } else {

    for (int i = 0; i < 12; ++i) {
      target.torque[i] = tau_full[joint_v_indices_[i]];
    }
  }
}

void DynamicSystem::update_dynamics(SystemState &system_state) {
  pinocchio::computeAllTerms(model, *pin_data_, system_state.q,
                             system_state.q_dot);
  pinocchio::updateFramePlacements(model, *pin_data_);
  pinocchio::forwardKinematics(model, *pin_data_, system_state.q,
                               system_state.q_dot, q_ddot_zero_);
  system_state.base_jacobian_lwa.setZero(); // <-- ADD THIS
  pinocchio::getFrameJacobian(model, *pin_data_, base_frame_id_,
                              pinocchio::LOCAL_WORLD_ALIGNED,
                              system_state.base_jacobian_lwa);

  pin_data_->M.triangularView<Eigen::StrictlyLower>() =
      pin_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

  // formulate jacobians
  // populate foot acceleration biases
  system_state.base_drift_lwa = pinocchio::getFrameClassicalAcceleration(
      model, *pin_data_, base_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Matrix<double, 6, Consts::nq_dot> J_temp_foot;

  system_state.mass_matrix = pin_data_->M;
  system_state.hu = pin_data_->nle.head<Consts::base_dof>();
  for (int i = 0; i < 4; ++i) {
    J_temp_foot.setZero();
    pinocchio::getFrameJacobian(model, *pin_data_, foot_ids_[i],
                                pinocchio::LOCAL_WORLD_ALIGNED, J_temp_foot);

    system_state.foot_jac_lin_lwa.block<3, Consts::nq_dot>(3 * i, 0) =
        J_temp_foot.topRows(3);

    system_state.foot_state[i].foot_pos_world =
        pin_data_->oMf[foot_ids_[i]].translation();
    system_state.foot_state[i].foot_vel_lwa =
        pinocchio::getFrameVelocity(model, *pin_data_, foot_ids_[i],
                                    pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
    system_state.foot_drift_lin_lwa[i] =
        pinocchio::getFrameClassicalAcceleration(
            model, *pin_data_, foot_ids_[i], pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();
    Eigen::Matrix3d J_leg_linear;
    Eigen::Vector3d tau_leg; // = system_state.j_torq.block<3,1>(3*i,0);

    for (int j = 0; j < 3; ++j) {
      // The index of the joint in your custom arrays (0 to 11)
      int joint_idx = i * 3 + j;

      // Slot the specific column and torque together
      J_leg_linear.col(j) = system_state.foot_jac_lin_lwa.block<3, 1>(
          3 * i, joint_v_indices_[joint_idx]);
      tau_leg(j) = system_state.j_torq(joint_idx);
    }

    // Eigen::Vector3d force_lwa = -J_leg_linear.transpose().inverse() *
    // tau_leg;
    Eigen::Vector3d tau_leg_active =
        tau_leg - system_state.hu.segment<3>(6 + i * 3);
    Eigen::Vector3d force_lwa =
        -J_leg_linear.transpose().colPivHouseholderQr().solve(tau_leg_active);
    // Hysteresis 
    if(system_state.foot_state[i].contact)
    {
      // check for liftoff
      system_state.foot_state[i].contact = (force_lwa.z() > 9.81);  
    }else
    {
      // check for touchdown
      system_state.foot_state[i].contact = (force_lwa.z() > 15);
    }
  }
}
} // namespace wbc