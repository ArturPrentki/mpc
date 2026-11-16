#include "whole_body_controller/wbc_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace wbc {

namespace {
std::unique_ptr<RobotModel> buildRobot(const std::string &urdf,
                                       const RobotConfig &config) {
  return std::make_unique<RobotModel>(RobotModel::fromUrdf(urdf, config));
}
}  // namespace

WbcCore::WbcCore(const std::string &urdf_xml, const CoreConfig &config)
    : config_(config), robot_(buildRobot(urdf_xml, config.robot)),
      dynamics_(*robot_, config.contact), wbic_(*robot_, config.wbic),
      interpolator_(*robot_, config.fade_time),
      safety_(*robot_, config.safety) {
  if (!wbic_.initialize()) {
    throw std::runtime_error("Failed to initialize the WBC QP solver");
  }
  target_.resize(robot_->nq(), robot_->nv(), robot_->nf());
  stand_target_ = target_;
  output_.resize(robot_->nv(), robot_->nj(), robot_->nf());
  command_.resize(robot_->nj());
  modes_.assign(robot_->nf(), ContactMode::STANCE);
  stall_timer_.assign(robot_->nf(), 0.0);
  stance_hold_until_.assign(robot_->nf(), 0.0);
}

void WbcCore::anchorStand(const SystemState &s) {
  // Stand where the robot is: current x, y, yaw; ground at the mean height of
  // the feet that are (or are assumed to be) on the ground.
  double ground = 0.0;
  int n = 0;
  for (const auto &foot : s.feet) {
    if (foot.contact) {
      ground += foot.pos.z();
      ++n;
    }
  }
  if (n == 0) {
    for (const auto &foot : s.feet) ground += foot.pos.z();
    n = static_cast<int>(s.feet.size());
  }
  ground /= n;
  stand_target_ = robot_->standTarget(s.base_pos, yawOf(s.base_rot), ground);
}

void WbcCore::activate(double now, SystemState &state) {
  activation_time_ = now;
  last_update_ = now;
  std::fill(stall_timer_.begin(), stall_timer_.end(), 0.0);
  std::fill(stance_hold_until_.begin(), stance_hold_until_.end(), now);
  if (state.feet.size() != static_cast<size_t>(robot_->nf())) {
    state.resize(robot_->nq(), robot_->nv(), robot_->nj(), robot_->nf());
  }
  for (auto &foot : state.feet) foot.contact = true;
  dynamics_.update(state);
  for (auto &foot : state.feet) foot.contact = true;  // assume standing
  interpolator_.clear();
  safety_.reset();
  anchorStand(state);
  target_ = stand_target_;
  ref_mode_ = TrajectoryInterpolator::Mode::EXPIRED;
}

const JointCommand &WbcCore::update(double now, SystemState &state) {
  const double dt = std::max(0.0, now - last_update_);
  last_update_ = now;
  dynamics_.update(state);

  const auto previous_mode = ref_mode_;
  ref_mode_ = interpolator_.sample(now, stand_target_, target_);
  if (previous_mode == TrajectoryInterpolator::Mode::TRACKING &&
      ref_mode_ != TrajectoryInterpolator::Mode::TRACKING) {
    // Planner stopped: stand where we are, not where we were activated.
    anchorStand(state);
    ref_mode_ = interpolator_.sample(now, stand_target_, target_);
  }

  const bool in_grace =
      now - activation_time_ < config_.contact.activation_grace_time;
  const auto &cc = config_.contact;
  const double v_stall =
      -cc.stall_velocity_ratio * config_.wbic.late_touchdown_velocity;
  for (int f = 0; f < robot_->nf(); ++f) {
    auto &foot = state.feet[f];
    const bool planned = target_.feet[f].contact;
    if (in_grace || now < stance_hold_until_[f]) foot.contact = true;

    // Stall detection while reaching down (uses last tick's mode)
    if (planned && modes_[f] == ContactMode::LATE_TOUCHDOWN && !foot.contact &&
        foot.vel.z() > v_stall) {
      stall_timer_[f] += dt;
    } else {
      stall_timer_[f] = 0.0;
    }
    if (cc.enabled && stall_timer_[f] >= cc.stall_time) {
      foot.contact = true;
      stall_timer_[f] = 0.0;
    }

    const bool detected = !cc.enabled || foot.contact;
    const ContactMode mode = !planned  ? ContactMode::SWING
                             : detected ? ContactMode::STANCE
                                        : ContactMode::LATE_TOUCHDOWN;
    if (mode == ContactMode::STANCE && modes_[f] != ContactMode::STANCE) {
      stance_hold_until_[f] = now + cc.min_stance_time;
    }
    modes_[f] = mode;
  }

  wbc_ok_ = wbic_.solve(state, target_, modes_, output_);
  if (wbc_ok_) command_ = output_.cmd;
  safety_.step(now, wbc_ok_, state, command_);
  return command_;
}

}  // namespace wbc
