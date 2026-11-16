#include "whole_body_controller/trajectory_interpolator.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>

namespace wbc {

TrajectoryInterpolator::TrajectoryInterpolator(const RobotModel &robot,
                                               double fade_time)
    : robot_(robot), fade_time_(std::max(fade_time, 1e-3)) {}

void TrajectoryInterpolator::set(std::shared_ptr<const Trajectory> trajectory) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = std::move(trajectory);
  has_pending_ = true;
}

void TrajectoryInterpolator::clear() { set(nullptr); }

void TrajectoryInterpolator::interpolate(const TargetState &a,
                                         const TargetState &b, double alpha,
                                         TargetState &out) const {
  // SE(3)/joint-consistent interpolation of the configuration
  out.q = pinocchio::interpolate(robot_.model(), a.q, b.q, alpha);
  out.v = (1.0 - alpha) * a.v + alpha * b.v;
  out.a = (1.0 - alpha) * a.a + alpha * b.a;
  for (size_t f = 0; f < out.feet.size(); ++f) {
    const auto &fa = a.feet[f];
    const auto &fb = b.feet[f];
    auto &fo = out.feet[f];
    fo.pos = (1.0 - alpha) * fa.pos + alpha * fb.pos;
    fo.vel = (1.0 - alpha) * fa.vel + alpha * fb.vel;
    fo.acc = (1.0 - alpha) * fa.acc + alpha * fb.acc;
    fo.force = (1.0 - alpha) * fa.force + alpha * fb.force;
    // A planner node's contact flag holds over the interval that follows it
    fo.contact = fa.contact;
  }
}

TrajectoryInterpolator::Mode
TrajectoryInterpolator::sample(double now, const TargetState &fallback,
                               TargetState &out) {
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock() && has_pending_) {
      active_.swap(pending_);
      has_pending_ = false;
    }
  }

  if (!active_ || active_->points.empty()) {
    out = fallback;
    return Mode::EXPIRED;
  }
  const auto &pts = active_->points;
  const double t = std::max(0.0, now - active_->stamp);
  const double t_end = pts.back().time_offset;

  if (t <= t_end && pts.size() > 1) {
    // first point whose time offset is greater than t
    auto it = std::upper_bound(
        pts.begin(), pts.end(), t,
        [](double value, const TrajectoryPoint &p) { return value < p.time_offset; });
    if (it == pts.begin()) it = std::next(it);
    if (it == pts.end()) it = std::prev(pts.end());
    const auto &pb = *it;
    const auto &pa = *std::prev(it);
    const double dt = pb.time_offset - pa.time_offset;
    const double alpha =
        dt > 1e-9 ? std::clamp((t - pa.time_offset) / dt, 0.0, 1.0) : 1.0;
    interpolate(pa.state, pb.state, alpha, out);
    return Mode::TRACKING;
  }

  const double s = (t - t_end) / fade_time_;
  if (s < 1.0) {
    // last -> fallback; velocities and accelerations fade out
    interpolate(pts.back().state, fallback, s, out);
    out.v = (1.0 - s) * pts.back().state.v;
    out.a.setZero();
    for (size_t f = 0; f < out.feet.size(); ++f) {
      out.feet[f].vel = (1.0 - s) * pts.back().state.feet[f].vel;
      out.feet[f].acc.setZero();
    }
    return Mode::FADING;
  }

  out = fallback;
  return Mode::EXPIRED;
}

const char *toString(TrajectoryInterpolator::Mode mode) {
  switch (mode) {
  case TrajectoryInterpolator::Mode::TRACKING: return "TRACKING";
  case TrajectoryInterpolator::Mode::FADING: return "FADING";
  case TrajectoryInterpolator::Mode::EXPIRED: return "EXPIRED";
  }
  return "?";
}

}  // namespace wbc
