#include "whole_body_controller/dynamic_system.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace wbc {

DynamicSystem::DynamicSystem(const RobotModel &robot,
                             const ContactDetectionConfig &config)
    : robot_(robot), config_(config),
      data_(std::make_unique<pinocchio::Data>(robot.model())),
      J6_(Eigen::MatrixXd::Zero(6, robot.nv())) {
  for (int f = 0; f < robot_.nf(); ++f) {
    const auto n = robot_.legJoints(f).size();
    J_leg_.emplace_back(Eigen::MatrixXd::Zero(3, n));
    rhs_leg_.emplace_back(Eigen::VectorXd::Zero(n));
  }
}

void DynamicSystem::update(SystemState &s) {
  const auto &model = robot_.model();
  auto &data = *data_;

  // M (upper triangle), nle, joint Jacobians, kinematics with a = 0 so that
  // frame accelerations are the drift terms Jdot * v.
  pinocchio::computeAllTerms(model, data, s.q, s.v);
  pinocchio::forwardKinematics(model, data, s.q, s.v,
                               Eigen::VectorXd::Zero(model.nv));
  pinocchio::updateFramePlacements(model, data);

  s.M = data.M;
  s.M.triangularView<Eigen::StrictlyLower>() =
      s.M.transpose().triangularView<Eigen::StrictlyLower>();
  s.nle = data.nle;

  const auto base = robot_.baseFrameId();
  s.J_base.setZero();
  pinocchio::getFrameJacobian(model, data, base, pinocchio::LOCAL_WORLD_ALIGNED,
                              s.J_base);
  s.base_drift = pinocchio::getFrameClassicalAcceleration(
                     model, data, base, pinocchio::LOCAL_WORLD_ALIGNED)
                     .toVector();
  s.base_pos = data.oMf[base].translation();
  s.base_rot = data.oMf[base].rotation();
  s.base_vel_world =
      pinocchio::getFrameVelocity(model, data, base,
                                  pinocchio::LOCAL_WORLD_ALIGNED)
          .toVector();

  for (int f = 0; f < robot_.nf(); ++f) {
    const auto fid = robot_.footFrameId(f);
    J6_.setZero();
    pinocchio::getFrameJacobian(model, data, fid,
                                pinocchio::LOCAL_WORLD_ALIGNED, J6_);
    s.J_feet.middleRows<3>(3 * f) = J6_.topRows<3>();
    s.feet_drift.segment<3>(3 * f) =
        pinocchio::getFrameClassicalAcceleration(
            model, data, fid, pinocchio::LOCAL_WORLD_ALIGNED)
            .linear();

    auto &foot = s.feet[f];
    foot.pos = data.oMf[fid].translation();
    foot.vel = pinocchio::getFrameVelocity(model, data, fid,
                                           pinocchio::LOCAL_WORLD_ALIGNED)
                   .linear();

    // Ground reaction force from the leg rows of the equation of motion,
    // neglecting the leg's inertial term:
    //   tau_leg = nle_leg - J_leg^T f   =>   J_leg^T f = nle_leg - tau_leg
    const auto &leg = robot_.legJoints(f);
    for (size_t k = 0; k < leg.size(); ++k) {
      const int vidx = robot_.jointVIdx(leg[k]);
      J_leg_[f].col(k) = J6_.block<3, 1>(0, vidx);
      rhs_leg_[f][k] = s.nle[vidx] - s.tau_meas[leg[k]];
    }
    foot.force = J_leg_[f]
                     .transpose()
                     .completeOrthogonalDecomposition()
                     .solve(rhs_leg_[f]);

    if (config_.enabled) {
      const double fz = foot.force.z();
      foot.contact = foot.contact ? (fz > config_.liftoff_force)
                                  : (fz > config_.touchdown_force);
    }
  }
}

}  // namespace wbc
