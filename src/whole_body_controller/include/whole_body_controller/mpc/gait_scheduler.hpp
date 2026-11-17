#pragma once
// Periodic gait timing.
//
// After start(t0), foot f lifts off at t0 + (offset_f + n) * cycle_time for
// n = 0, 1, ..., swings for (1 - duty_factor) * cycle_time and then stands.
// Every foot is in stance before its first liftoff, so starting never begins
// in the middle of a swing. After stop(ts) no new liftoff happens at or after
// ts; swings already in progress finish normally.

#include <limits>
#include <vector>

namespace wbc {

struct GaitConfig {
  double cycle_time = 0.5;               // [s]
  double duty_factor = 0.5;              // stance fraction of a cycle
  std::vector<double> phase_offsets;     // per foot, in [0, 1)
};

class GaitScheduler {
public:
  static constexpr double kNever = -std::numeric_limits<double>::infinity();

  explicit GaitScheduler(const GaitConfig &config);

  void start(double t);  // no effect if already walking
  void stop(double t);   // no effect if not walking or already stopping

  // True between start() and the end of the last swing after stop().
  bool walking(double t) const;
  bool stopping() const { return stop_ < kInf; }

  bool contact(int foot, double t) const;
  // Latest liftoff time <= t, or kNever.
  double lastLiftoff(int foot, double t) const;
  // If the foot is in swing at t, returns true and its liftoff/touchdown.
  bool swingInterval(int foot, double t, double &liftoff,
                     double &touchdown) const;

  double swingDuration() const { return swing_; }
  double stanceDuration() const { return config_.cycle_time - swing_; }
  int numFeet() const { return static_cast<int>(config_.phase_offsets.size()); }

private:
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  GaitConfig config_;
  double swing_;
  bool started_ = false;
  double t0_ = 0.0;
  double stop_ = kInf;
};

}  // namespace wbc
