#include "whole_body_controller/mpc/locomotion_planner.hpp"

#include <algorithm>
#include <cmath>

namespace wbc {

namespace {
constexpr double kGravity = 9.81;

Eigen::Matrix3d rotZ(double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

// ZYX Euler angles (roll, pitch, yaw) of R
Eigen::Vector3d rpyOf(const Eigen::Matrix3d &R) {
  const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
  return {std::atan2(R(2, 1), R(2, 2)), pitch, std::atan2(R(1, 0), R(0, 0))};
}

Eigen::Matrix3d rotRpy(const Eigen::Vector3d &rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

double approach(double value, double target, double max_step) {
  return value + std::clamp(target - value, -max_step, max_step);
}
}  // namespace

LocomotionPlanner::LocomotionPlanner(const RobotModel &robot,
                                     const LocomotionConfig &config)
    : robot_(robot), config_(config), gait_(config.gait),
      mpc_(robot.totalMass(), robot.bodyInertia(), robot.nf(), config.mpc),
      nf_(robot.nf()) {
  if (gait_.numFeet() != nf_) {
    throw std::invalid_argument(
        "gait phase_offsets must have one entry per foot");
  }
  stance_pos_.assign(nf_, Eigen::Vector3d::Zero());
  swing_start_.assign(nf_, Eigen::Vector3d::Zero());
  swing_liftoff_.assign(nf_, GaitScheduler::kNever);
  footholds_.assign(nf_, Eigen::Vector3d::Zero());
}

void LocomotionPlanner::reset(const PlannerInput &in) {
  gait_ = GaitScheduler(config_.gait);
  cmd_ = VelocityCommand{};
  last_time_ = in.time;
  inactive_since_ = in.time;
  double z = 0.0;
  for (int f = 0; f < nf_; ++f) {
    stance_pos_[f] = in.feet[f];
    footholds_[f] = in.feet[f];
    swing_liftoff_[f] = GaitScheduler::kNever;
    z += in.feet[f].z();
  }
  ground_z_ = z / nf_;
  initialized_ = true;
}

void LocomotionPlanner::updateCommand(double t, const VelocityCommand &req) {
  const double dt = std::clamp(t - last_time_, 0.0, 0.1);
  last_time_ = t;
  const auto &c = config_;
  const double vx = std::clamp(req.vx, -c.max_vx, c.max_vx);
  const double vy = std::clamp(req.vy, -c.max_vy, c.max_vy);
  const double wz = std::clamp(req.wz, -c.max_wz, c.max_wz);

  const bool active = std::hypot(vx, vy) > c.active_threshold ||
                      std::abs(wz) > c.active_threshold || c.always_walk;
  if (active) {
    inactive_since_ = t;
    gait_.start(t);
  } else if (t - inactive_since_ > c.stop_delay) {
    gait_.stop(t);
  }
  // Only move once the feet can follow; ramp down while stopping.
  const bool can_move = gait_.walking(t) && !gait_.stopping();
  const double tvx = can_move ? vx : 0.0;
  const double tvy = can_move ? vy : 0.0;
  const double twz = can_move ? wz : 0.0;
  cmd_.vx = approach(cmd_.vx, tvx, c.max_lin_acc * dt);
  cmd_.vy = approach(cmd_.vy, tvy, c.max_lin_acc * dt);
  cmd_.wz = approach(cmd_.wz, twz, c.max_yaw_acc * dt);
}

LocomotionPlanner::Reference LocomotionPlanner::reference(double tau) const {
  // Constant body-frame velocity and yaw rate from the current pose:
  // closed-form integral of Rz(yaw0 + wz s) [vx vy] ds.
  const double w = cmd_.wz;
  const double yaw = yaw0_ + w * tau;
  Eigen::Vector2d dxy;
  if (std::abs(w) < 1e-6) {
    dxy = rotZ(yaw0_).topLeftCorner<2, 2>() *
          Eigen::Vector2d(cmd_.vx, cmd_.vy) * tau;
  } else {
    const double s0 = std::sin(yaw0_), c0 = std::cos(yaw0_);
    const double s1 = std::sin(yaw), c1 = std::cos(yaw);
    Eigen::Matrix2d M;
    M << s1 - s0, c1 - c0, -(c1 - c0), s1 - s0;
    dxy = M * Eigen::Vector2d(cmd_.vx, cmd_.vy) / w;
  }
  Reference r;
  r.base = Eigen::Vector3d(base0_.x() + dxy.x(), base0_.y() + dxy.y(),
                           ground_z_ + robot_.nominalHeight() +
                               config_.body_height_offset);
  r.yaw = yaw;
  r.v = rotZ(yaw) * Eigen::Vector3d(cmd_.vx, cmd_.vy, 0.0);
  return r;
}

Eigen::Vector3d LocomotionPlanner::foothold(int f, double td) const {
  const Reference ref = reference(std::max(0.0, td - t_));
  const Eigen::Vector3d nominal =
      ref.base + rotZ(ref.yaw) * robot_.nominalFootOffsets()[f];
  const double h = robot_.nominalHeight();
  Eigen::Vector3d offset = ref.v * (0.5 * gait_.stanceDuration());
  // capture-point style feedback on the velocity error (now)
  const Eigen::Vector3d v_cmd_now =
      rotZ(yaw0_) * Eigen::Vector3d(cmd_.vx, cmd_.vy, 0.0);
  offset += config_.foothold_feedback_gain * std::sqrt(h / kGravity) *
            (v_meas_ - v_cmd_now);
  // centrifugal compensation while turning
  offset += 0.5 * h / kGravity *
            ref.v.cross(Eigen::Vector3d(0.0, 0.0, cmd_.wz));
  offset.z() = 0.0;
  const double n = offset.norm();
  if (n > config_.max_foothold_offset) {
    offset *= config_.max_foothold_offset / n;
  }
  Eigen::Vector3d p = nominal + offset;
  p.z() = ground_z_;
  return p;
}

SwingSample LocomotionPlanner::footAt(int f, double t) const {
  double lo, td;
  if (gait_.swingInterval(f, t, lo, td)) {
    // swing starts from the recorded liftoff point if this is the current
    // swing, otherwise from the foothold of the previous touchdown
    // (a future swing starts where the foot stands just before its liftoff)
    const Eigen::Vector3d p0 =
        lo == swing_liftoff_[f] ? swing_start_[f]
        : lo <= t_              ? stance_pos_[f]
                                : footAt(f, lo - 1e-6).pos;
    return swingTrajectory(p0, foothold(f, td), config_.step_height,
                           gait_.swingDuration(), t - lo);
  }
  SwingSample s;
  s.vel.setZero();
  s.acc.setZero();
  const double lo_last = gait_.lastLiftoff(f, t);
  const double td_last = lo_last + gait_.swingDuration();
  if (lo_last == GaitScheduler::kNever || td_last <= t_) {
    s.pos = stance_pos_[f];  // still on the current stance point
  } else {
    s.pos = foothold(f, td_last);  // landed after now
  }
  return s;
}

std::shared_ptr<Trajectory> LocomotionPlanner::update(
    const PlannerInput &in, const VelocityCommand &request) {
  if (!initialized_) reset(in);
  t_ = in.time;
  updateCommand(t_, request);

  const Eigen::Matrix3d &R = in.base_rot;
  const Eigen::Vector3d rpy = rpyOf(R);
  yaw0_ = rpy.z();
  base0_ = in.base_pos;
  v_meas_ = in.base_vel_world.head<3>();
  v_meas_.z() = 0.0;

  // --- feet bookkeeping -----------------------------------------------------
  double z_sum = 0.0;
  int z_n = 0;
  for (int f = 0; f < nf_; ++f) {
    double lo, td;
    if (gait_.swingInterval(f, t_, lo, td)) {
      if (lo != swing_liftoff_[f]) {
        swing_liftoff_[f] = lo;
        swing_start_[f] = stance_pos_[f];
      }
      footholds_[f] = foothold(f, td);
    } else {
      stance_pos_[f] = in.feet[f];
      footholds_[f] = in.feet[f];
      z_sum += in.feet[f].z();
      ++z_n;
    }
  }
  if (z_n > 0) {
    // flat ground at the height of the stance feet (low-pass)
    ground_z_ += 0.2 * (z_sum / z_n - ground_z_);
  }

  // --- MPC problem ------------------------------------------------------------
  const int N = config_.mpc.horizon;
  const double dt = config_.mpc.dt;
  const Eigen::Vector3d c_off = robot_.comOffset();
  SrbdMpcProblem prob;
  const Eigen::Vector3d omega = in.base_vel_world.tail<3>();
  const Eigen::Vector3d com0 = in.base_pos + R * c_off;
  prob.x0 << rpy, com0, omega,
      in.base_vel_world.head<3>() + omega.cross(R * c_off);
  prob.x_ref.resize(N);
  prob.yaw.resize(N);
  prob.r.assign(N, std::vector<Eigen::Vector3d>(nf_));
  prob.contact.assign(N, std::vector<bool>(nf_));
  for (int k = 0; k < N; ++k) {
    const Reference ref_k = reference(k * dt);
    const Reference ref_next = reference((k + 1) * dt);
    const Eigen::Matrix3d Rz_next = rotZ(ref_next.yaw);
    Vector12d xr;
    xr << 0.0, 0.0, ref_next.yaw, ref_next.base + Rz_next * c_off, 0.0, 0.0,
        cmd_.wz, ref_next.v + Eigen::Vector3d(0, 0, cmd_.wz).cross(Rz_next * c_off);
    prob.x_ref[k] = xr;
    prob.yaw[k] = ref_k.yaw;
    const Eigen::Vector3d com_k = ref_k.base + rotZ(ref_k.yaw) * c_off;
    const double t_k = t_ + k * dt;
    for (int f = 0; f < nf_; ++f) {
      // contact over [t_k, t_k+1): sample in the middle of the step
      prob.contact[k][f] = gait_.contact(f, t_k + 0.5 * dt);
      prob.r[k][f] = footAt(f, t_k).pos - com_k;
    }
  }
  if (!mpc_.solve(prob)) return nullptr;

  // --- dense output trajectory -------------------------------------------------
  auto traj = std::make_shared<Trajectory>();
  traj->stamp = t_;
  const double horizon = std::min(config_.output_horizon, N * dt);
  const int n_out = static_cast<int>(std::floor(horizon / config_.output_dt));
  const auto &xs = mpc_.states();
  const auto &us = mpc_.forces();
  std::vector<Eigen::Vector3d> r_now(nf_);
  for (int j = 0; j <= n_out; ++j) {
    const double tau = j * config_.output_dt;
    const int k = std::min(static_cast<int>(tau / dt), N - 1);
    const double a = std::clamp((tau - k * dt) / dt, 0.0, 1.0);
    const Vector12d x = (1.0 - a) * xs[k] + a * xs[k + 1];
    const Eigen::VectorXd &u = us[k];

    TrajectoryPoint pt;
    pt.time_offset = tau;
    TargetState &s = pt.state;
    s.resize(robot_.nq(), robot_.nv(), nf_);
    const Eigen::Matrix3d Rb = rotRpy(x.head<3>());
    const Eigen::Vector3d w = x.segment<3>(6);
    const Eigen::Vector3d c_w = Rb * c_off;
    const Eigen::Vector3d base = x.segment<3>(3) - c_w;
    s.q = robot_.nominalConfiguration(base, 0.0);
    s.q.segment<4>(3) = Eigen::Quaterniond(Rb).coeffs();
    const Eigen::Vector3d v_base = x.segment<3>(9) - w.cross(c_w);
    s.v.head<3>() = Rb.transpose() * v_base;
    s.v.segment<3>(3) = Rb.transpose() * w;

    const double t_abs = t_ + tau;
    for (int f = 0; f < nf_; ++f) {
      const SwingSample fs = footAt(f, t_abs);
      auto &foot = s.feet[f];
      foot.pos = fs.pos;
      foot.vel = fs.vel;
      foot.acc = fs.acc;
      foot.contact = gait_.contact(f, t_abs);
      foot.force = foot.contact ? Eigen::Vector3d(u.segment<3>(3 * f))
                                : Eigen::Vector3d::Zero();
      r_now[f] = fs.pos - x.segment<3>(3);
    }
    const Vector12d dx = mpc_.derivative(x, u, r_now);
    s.a.head<3>() = Rb.transpose() * dx.segment<3>(9);
    s.a.segment<3>(3) = Rb.transpose() * dx.segment<3>(6);
    traj->points.push_back(std::move(pt));
  }
  return traj;
}

}  // namespace wbc
