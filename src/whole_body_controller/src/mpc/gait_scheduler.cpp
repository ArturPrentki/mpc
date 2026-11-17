#include "whole_body_controller/mpc/gait_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wbc {

GaitScheduler::GaitScheduler(const GaitConfig &config) : config_(config) {
  if (!(config_.cycle_time > 0.0)) {
    throw std::invalid_argument("gait cycle_time must be positive");
  }
  if (!(config_.duty_factor > 0.0 && config_.duty_factor < 1.0)) {
    throw std::invalid_argument("gait duty_factor must be in (0, 1)");
  }
  if (config_.phase_offsets.empty()) {
    throw std::invalid_argument("gait phase_offsets is empty");
  }
  for (auto &o : config_.phase_offsets) {
    o = o - std::floor(o);  // wrap into [0, 1)
  }
  swing_ = (1.0 - config_.duty_factor) * config_.cycle_time;
}

void GaitScheduler::start(double t) {
  if (started_ && stop_ == kInf) return;  // already walking
  if (started_ && walking(t)) {
    // Restarted while still finishing the stop: cancel the stop. Keep the
    // phase so feet that are swinging are not disturbed.
    stop_ = kInf;
    return;
  }
  started_ = true;
  t0_ = t;
  stop_ = kInf;
}

void GaitScheduler::stop(double t) {
  if (!started_ || stop_ < kInf) return;
  stop_ = t;
}

double GaitScheduler::lastLiftoff(int f, double t) const {
  if (!started_ || t < t0_) return kNever;
  const double T = config_.cycle_time;
  const double off = config_.phase_offsets[f];
  // liftoff_n = t0 + (off + n) T  <= t
  double n = std::floor((t - t0_) / T - off);
  if (n < 0.0) return kNever;
  double lo = t0_ + (off + n) * T;
  if (lo >= stop_) {
    // last liftoff strictly before the stop
    n = std::ceil((stop_ - t0_) / T - off) - 1.0;
    if (n < 0.0) return kNever;
    lo = t0_ + (off + n) * T;
  }
  return lo;
}

bool GaitScheduler::swingInterval(int f, double t, double &liftoff,
                                  double &touchdown) const {
  const double lo = lastLiftoff(f, t);
  if (lo == kNever || t >= lo + swing_) return false;
  liftoff = lo;
  touchdown = lo + swing_;
  return true;
}

bool GaitScheduler::contact(int f, double t) const {
  double lo, td;
  return !swingInterval(f, t, lo, td);
}

bool GaitScheduler::walking(double t) const {
  if (!started_) return false;
  if (stop_ == kInf) return true;
  if (t < stop_) return true;
  for (int f = 0; f < numFeet(); ++f) {
    double lo, td;
    if (swingInterval(f, t, lo, td)) return true;
  }
  return false;
}

}  // namespace wbc
