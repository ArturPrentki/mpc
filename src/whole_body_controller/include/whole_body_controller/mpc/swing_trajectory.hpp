#pragma once
// Swing foot trajectory: minimum-jerk from liftoff to touchdown in all three
// axes, plus a vertical bump of height `height` at mid-swing. Position,
// velocity and acceleration are zero-velocity/zero-acceleration at both ends.

#include <Eigen/Dense>
#include <algorithm>

namespace wbc {

struct SwingSample {
  Eigen::Vector3d pos, vel, acc;
};

inline SwingSample swingTrajectory(const Eigen::Vector3d &p0,
                                   const Eigen::Vector3d &p1, double height,
                                   double duration, double t) {
  const double s = std::clamp(t / duration, 0.0, 1.0);
  const double s2 = s * s, s3 = s2 * s;
  // minimum jerk b(s) = 10 s^3 - 15 s^4 + 6 s^5
  const double b = s3 * (10.0 - 15.0 * s + 6.0 * s2);
  const double db = 30.0 * s2 * (1.0 - s) * (1.0 - s);
  const double ddb = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s);
  // bump c(s) = 64 s^3 (1 - s)^3, max 1 at s = 0.5
  const double u = 1.0 - s;
  const double c = 64.0 * s3 * u * u * u;
  const double dc = 192.0 * s2 * u * u * (u - s);
  const double ddc = 384.0 * s * u * (u * u - 3.0 * s * u + s * s);

  const double inv = 1.0 / duration;
  SwingSample out;
  const Eigen::Vector3d d = p1 - p0;
  out.pos = p0 + d * b;
  out.vel = d * db * inv;
  out.acc = d * ddb * inv * inv;
  out.pos.z() += height * c;
  out.vel.z() += height * dc * inv;
  out.acc.z() += height * ddc * inv * inv;
  return out;
}

}  // namespace wbc
