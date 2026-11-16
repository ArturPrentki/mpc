#include "whole_body_controller/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace wbc {

SafetySupervisor::SafetySupervisor(const RobotModel &robot,
                                   const SafetyConfig &config)
    : robot_(robot), config_(config) {
  last_good_.resize(robot_.nj());
  q_joints_.setZero(robot_.nj());
  v_joints_.setZero(robot_.nj());
  reason_.reserve(128);
}

void SafetySupervisor::reset() {
  state_ = State::OK;
  reason_.clear();
  has_last_good_ = false;
}

void SafetySupervisor::enterDamping(const std::string &reason) {
  if (state_ != State::DAMPING) reason_ = reason;
  state_ = State::DAMPING;
}

void SafetySupervisor::clamp(JointCommand &cmd) const {
  const auto &lim = robot_.effortLimits();
  cmd.tau_ff = cmd.tau_ff.cwiseMax(-lim).cwiseMin(lim);
}

SafetySupervisor::State SafetySupervisor::step(double now, bool wbc_ok,
                                               const SystemState &s,
                                               JointCommand &cmd) {
  robot_.jointsFromQ(s.q, q_joints_);
  robot_.jointsFromV(s.v, v_joints_);

  if (state_ != State::DAMPING) {
    const double tilt = std::acos(std::clamp(s.base_rot(2, 2), -1.0, 1.0));
    if (!s.q.allFinite() || !s.v.allFinite()) {
      enterDamping("non-finite robot state");
    } else if (tilt > config_.max_tilt) {
      enterDamping("base tilt exceeded max_tilt");
    }
  }

  if (state_ != State::DAMPING) {
    const bool ok = wbc_ok && cmd.allFinite();
    if (ok) {
      state_ = State::OK;
      clamp(cmd);
      last_good_ = cmd;
      has_last_good_ = true;
    } else if (!has_last_good_) {
      enterDamping("WBC failed before producing a valid command");
    } else {
      if (state_ == State::OK) {
        state_ = State::HOLDING;
        fault_since_ = now;
      }
      if (now - fault_since_ > config_.hold_time) {
        enterDamping("WBC failed for longer than hold_time");
      } else {
        cmd = last_good_;
      }
    }
  }

  if (state_ == State::DAMPING) {
    cmd.tau_ff.setZero();
    cmd.q_des = q_joints_;
    cmd.v_des.setZero();
    cmd.kp.setZero();
    cmd.kd.setConstant(config_.damping_kd);
  }
  return state_;
}

const char *toString(SafetySupervisor::State state) {
  switch (state) {
  case SafetySupervisor::State::OK: return "OK";
  case SafetySupervisor::State::HOLDING: return "HOLDING";
  case SafetySupervisor::State::DAMPING: return "DAMPING";
  }
  return "?";
}

}  // namespace wbc
