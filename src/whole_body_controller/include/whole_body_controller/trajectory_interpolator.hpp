#pragma once
// Holds the latest planner trajectory and samples it at the control rate.
//
// Times are absolute seconds on the controller clock. Each point carries its
// own time offset from the trajectory stamp, so non-uniform horizons work.
//
//   TRACKING  stamp <= now <= stamp + last offset: interpolate between points
//   FADING    up to `fade_time` past the end: blend the last point into the
//             fallback (stand) target, velocities decay to zero
//   EXPIRED   later than that, or no trajectory: fallback target
//
// set() is called from a non-real-time thread; sample() never blocks.

#include "whole_body_controller/robot_model.hpp"
#include "whole_body_controller/types.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace wbc {

struct TrajectoryPoint {
  double time_offset = 0.0;  // [s] from the trajectory stamp
  TargetState state;
};

struct Trajectory {
  double stamp = 0.0;  // [s]
  std::vector<TrajectoryPoint> points;
};

class TrajectoryInterpolator {
public:
  enum class Mode { TRACKING, FADING, EXPIRED };

  TrajectoryInterpolator(const RobotModel &robot, double fade_time);

  // Non-RT thread. Points must have strictly increasing time offsets.
  void set(std::shared_ptr<const Trajectory> trajectory);
  // Drops the current trajectory (e.g. on activation).
  void clear();

  // RT thread.
  Mode sample(double now, const TargetState &fallback, TargetState &out);

private:
  void interpolate(const TargetState &a, const TargetState &b, double alpha,
                   TargetState &out) const;

  const RobotModel &robot_;
  double fade_time_;
  std::mutex mutex_;
  std::shared_ptr<const Trajectory> pending_;
  bool has_pending_ = false;
  std::shared_ptr<const Trajectory> active_;  // RT side only
};

const char *toString(TrajectoryInterpolator::Mode mode);

}  // namespace wbc
