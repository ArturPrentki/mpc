// Tests of the locomotion planner (gait, swing, SRBD MPC) and closed-loop
// walking with the WBC on both robots. The simulator uses unilateral point
// contacts: feet leave the ground only when they stop pushing and land with
// an inelastic impact, so these runs exercise the real contact sequence.

#include "test_common.hpp"
#include "whole_body_controller/mpc/locomotion_planner.hpp"

#include <gtest/gtest.h>

#include <functional>

using namespace wbc;
using namespace wbc_test;

namespace {

LocomotionConfig locomotionConfig(const RobotCase &c) {
  LocomotionConfig l;
  l.gait.cycle_time = 0.5;
  l.gait.duty_factor = 0.5;
  l.gait.phase_offsets = {0.0, 0.5, 0.5, 0.0};  // trot, foot_frames order
  l.step_height = c.name == "gardo" ? 0.10 : 0.08;
  l.max_foothold_offset = c.name == "gardo" ? 0.18 : 0.12;
  return l;
}

PlannerInput plannerInput(double t, const SystemState &s) {
  PlannerInput in;
  in.time = t;
  in.base_pos = s.base_pos;
  in.base_rot = s.base_rot;
  in.base_vel_world = s.base_vel_world;
  for (const auto &f : s.feet) in.feet.push_back(f.pos);
  return in;
}

struct RunStats {
  bool fell = false;
  bool damped = false;
  int planner_failures = 0;
  double min_height = 1e9, max_height = -1e9, max_tilt = 0.0;
  double max_friction_ratio = 0.0;
  Eigen::Vector3d start_pos, end_pos;
  double start_yaw = 0.0, end_yaw = 0.0;
  // average velocity over the measurement window (world, base yaw frame)
  Eigen::Vector3d avg_vel_body = Eigen::Vector3d::Zero();
  double avg_yaw_rate = 0.0;
  int liftoffs = 0;  // in the measurement window
};

// Runs WBC at 1 kHz and the planner at 50 Hz (synchronously) on the
// unilateral-contact simulator.
class Harness {
public:
  explicit Harness(const RobotCase &c, LocomotionConfig loco = {})
      : core(readFile(c.urdf_file), c.config), sim(core.robot()),
        planner(core.robot(), loco.gait.phase_offsets.empty()
                                  ? locomotionConfig(c)
                                  : loco) {
    const auto &robot = core.robot();
    sim.setState(robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0).q);
    for (int f = 0; f < robot.nf(); ++f) sim.pin(f);
    sim.setGround(0.0);
    sim.friction_min_normal_force =
        0.1 * robot.totalMass() * 9.81 / robot.nf();
    s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());
    sim.fillMeasured(s);
    core.activate(0.0, s);
    planner.reset(plannerInput(0.0, s));
  }

  RunStats run(double duration,
               const std::function<VelocityCommand(double)> &command,
               double measure_from) {
    RunStats st;
    const double dt = 1e-3;
    const int mpc_every = 20;
    const int nf = core.robot().nf();
    std::vector<bool> was_pinned(nf, true);
    double yaw_prev = yawOf(s.base_rot), yaw_unwrapped = yaw_prev;
    double yaw_at_measure = 0.0;
    Eigen::Vector3d pos_at_measure = Eigen::Vector3d::Zero();
    bool measuring = false;
    st.start_pos = sim.q.head<3>();
    st.start_yaw = yaw_prev;
    for (int k = 0; k < static_cast<int>(duration / dt); ++k) {
      const double t = t0_ + k * dt;
      sim.fillMeasured(s);
      if (k % mpc_every == 0) {
        // dynamics are filled by the previous core.update(); refresh for the
        // planner snapshot
        auto traj = planner.update(plannerInput(t, s), command(t - t0_));
        if (traj) {
          core.setTrajectory(traj);
        } else {
          ++st.planner_failures;
        }
      }
      const auto &cmd = core.update(t, s);
      if (core.safetyState() == SafetySupervisor::State::DAMPING) {
        st.damped = true;
        break;
      }
      sim.step(cmd, dt);

      const double h = sim.q[2];
      if (h < 0.5 * core.robot().nominalHeight()) {
        st.fell = true;
        break;
      }
      const double yaw = yawOf(s.base_rot);
      yaw_unwrapped += std::remainder(yaw - yaw_prev, 2.0 * M_PI);
      yaw_prev = yaw;
      if (t - t0_ >= measure_from) {
        if (!measuring) {
          measuring = true;
          pos_at_measure = sim.q.head<3>();
          yaw_at_measure = yaw_unwrapped;
        }
        st.min_height = std::min(st.min_height, h);
        st.max_height = std::max(st.max_height, h);
        st.max_tilt = std::max(
            st.max_tilt, std::acos(std::clamp(s.base_rot(2, 2), -1.0, 1.0)));
      }
      for (int f = 0; f < nf; ++f) {
        if (measuring && was_pinned[f] && !sim.pinned(f)) ++st.liftoffs;
        was_pinned[f] = sim.pinned(f);
      }
    }
    const double window = duration - measure_from;
    st.end_pos = sim.q.head<3>();
    st.end_yaw = yaw_unwrapped;
    if (measuring && window > 0) {
      const Eigen::Vector3d d = (st.end_pos - pos_at_measure) / window;
      // express in the mean heading over the window
      const double mean_yaw = 0.5 * (yaw_at_measure + yaw_unwrapped);
      st.avg_vel_body =
          Eigen::AngleAxisd(-mean_yaw, Eigen::Vector3d::UnitZ()) * d;
      st.avg_yaw_rate = (yaw_unwrapped - yaw_at_measure) / window;
    }
    st.max_friction_ratio = sim.max_friction_ratio;
    t0_ += duration;
    return st;
  }

  WbcCore core;
  ContactSim sim;
  LocomotionPlanner planner;
  SystemState s;

private:
  double t0_ = 0.0;
};

// The controllers enforce a friction pyramid |fx|, |fy| <= mu fz (mu = 0.6),
// i.e. |f_tangential| <= sqrt(2) mu fz.
constexpr double kMaxFrictionRatio = 0.6 * 1.41421356 + 0.02;

// Offset of the body from the middle of its feet, and heading error w.r.t.
// the feet, after fitting the nominal stance to the feet (independent of the
// planner's implementation).
struct StanceError {
  double offset;  // [m] horizontal
  double heading; // [rad]
};
StanceError stanceError(const RobotModel &robot, ContactSim &sim) {
  const auto &o = robot.nominalFootOffsets();
  const int nf = robot.nf();
  std::vector<Eigen::Vector2d> p(nf);
  Eigen::Vector2d pm = Eigen::Vector2d::Zero(), om = pm;
  for (int f = 0; f < nf; ++f) {
    p[f] = sim.footPos(f).head<2>();
    pm += p[f];
    om += o[f].head<2>();
  }
  pm /= nf;
  om /= nf;
  // best yaw by brute force (keeps this check independent of the planner)
  double best_yaw = 0.0, best = 1e18;
  for (double yaw = -M_PI; yaw < M_PI; yaw += 1e-4) {
    const Eigen::Rotation2Dd Rz(yaw);
    double err = 0.0;
    for (int f = 0; f < nf; ++f)
      err += (p[f] - pm - Rz * (o[f].head<2>() - om)).squaredNorm();
    if (err < best) {
      best = err;
      best_yaw = yaw;
    }
  }
  const Eigen::Vector2d anchor = pm - Eigen::Rotation2Dd(best_yaw) * om;
  const Eigen::Matrix3d R =
      Eigen::Quaterniond(sim.q[6], sim.q[3], sim.q[4], sim.q[5])
          .toRotationMatrix();
  return {(sim.q.head<2>() - anchor).norm(),
          std::abs(std::remainder(yawOf(R) - best_yaw, 2.0 * M_PI))};
}

VelocityCommand constant(double vx, double vy, double wz) {
  VelocityCommand c;
  c.vx = vx;
  c.vy = vy;
  c.wz = wz;
  return c;
}

}  // namespace

// ===========================================================================
// Gait scheduler
// ===========================================================================
TEST(Gait, TrotTimingStartAndStop) {
  GaitConfig cfg;
  cfg.cycle_time = 0.5;
  cfg.duty_factor = 0.5;
  cfg.phase_offsets = {0.0, 0.5, 0.5, 0.0};
  GaitScheduler g(cfg);
  for (int f = 0; f < 4; ++f) EXPECT_TRUE(g.contact(f, 0.1));
  EXPECT_FALSE(g.walking(0.0));

  g.start(1.0);
  // feet 0 and 3 swing in [1.0, 1.25), feet 1 and 2 in [1.25, 1.5)
  EXPECT_FALSE(g.contact(0, 1.1));
  EXPECT_TRUE(g.contact(1, 1.1));
  EXPECT_TRUE(g.contact(2, 1.1));
  EXPECT_FALSE(g.contact(3, 1.1));
  EXPECT_TRUE(g.contact(0, 1.3));
  EXPECT_FALSE(g.contact(1, 1.3));
  EXPECT_FALSE(g.contact(0, 1.6));  // next cycle
  double lo, td;
  ASSERT_TRUE(g.swingInterval(1, 1.3, lo, td));
  EXPECT_DOUBLE_EQ(lo, 1.25);
  EXPECT_DOUBLE_EQ(td, 1.5);
  EXPECT_DOUBLE_EQ(g.lastLiftoff(0, 1.7), 1.5);
  EXPECT_EQ(g.lastLiftoff(1, 1.2), GaitScheduler::kNever);

  // Stop at 1.6: foot 0 (lifted at 1.5) finishes its swing, no new liftoff
  g.stop(1.6);
  EXPECT_TRUE(g.stopping());
  EXPECT_FALSE(g.contact(0, 1.7));
  EXPECT_TRUE(g.walking(1.7));
  EXPECT_TRUE(g.contact(1, 1.8));  // would have lifted at 1.75
  EXPECT_FALSE(g.walking(1.8));
  for (int f = 0; f < 4; ++f) EXPECT_TRUE(g.contact(f, 5.0));

  // Restart begins a fresh cycle
  g.start(6.0);
  EXPECT_TRUE(g.walking(6.0));
  EXPECT_FALSE(g.contact(0, 6.1));
  EXPECT_TRUE(g.contact(1, 6.1));
}

TEST(Gait, RejectsInvalidConfig) {
  GaitConfig cfg;
  cfg.phase_offsets = {0.0};
  cfg.duty_factor = 1.0;
  EXPECT_THROW(GaitScheduler{cfg}, std::invalid_argument);
  cfg.duty_factor = 0.5;
  cfg.cycle_time = 0.0;
  EXPECT_THROW(GaitScheduler{cfg}, std::invalid_argument);
}

// ===========================================================================
// Swing trajectory
// ===========================================================================
TEST(Swing, EndpointsApexAndDerivatives) {
  const Eigen::Vector3d p0(0.1, 0.2, 0.0), p1(0.3, 0.1, 0.02);
  const double T = 0.25, h = 0.08;
  auto s0 = swingTrajectory(p0, p1, h, T, 0.0);
  auto s1 = swingTrajectory(p0, p1, h, T, T);
  EXPECT_TRUE(s0.pos.isApprox(p0));
  EXPECT_TRUE(s1.pos.isApprox(p1));
  EXPECT_LT(s0.vel.norm() + s1.vel.norm() + s0.acc.norm() + s1.acc.norm(),
            1e-9);
  auto mid = swingTrajectory(p0, p1, h, T, 0.5 * T);
  EXPECT_NEAR(mid.pos.z(), 0.5 * (p0.z() + p1.z()) + h, 1e-12);
  // finite differences
  for (double t : {0.03, 0.1, 0.17, 0.22}) {
    const double e = 1e-6;
    auto a = swingTrajectory(p0, p1, h, T, t - e);
    auto b = swingTrajectory(p0, p1, h, T, t + e);
    auto c = swingTrajectory(p0, p1, h, T, t);
    EXPECT_LT(((b.pos - a.pos) / (2 * e) - c.vel).norm(), 1e-5);
    EXPECT_LT(((b.vel - a.vel) / (2 * e) - c.acc).norm(), 1e-3);
  }
}

// ===========================================================================
// SRBD MPC
// ===========================================================================
class PerRobotLoco : public ::testing::TestWithParam<RobotCase> {};

TEST_P(PerRobotLoco, MpcStandingForcesCarryTheWeight) {
  const auto &c = GetParam();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  SrbdMpcConfig cfg;
  SrbdMpc mpc(robot.totalMass(), robot.bodyInertia(), robot.nf(), cfg);
  const auto stand = robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0);
  const Eigen::Vector3d base(stand.q[0], stand.q[1], stand.q[2]);
  const Eigen::Vector3d com = base + robot.comOffset();
  SrbdMpcProblem p;
  p.x0 << 0, 0, 0, com, 0, 0, 0, 0, 0, 0;
  p.x_ref.assign(cfg.horizon, p.x0);
  p.yaw.assign(cfg.horizon, 0.0);
  p.r.assign(cfg.horizon, std::vector<Eigen::Vector3d>(robot.nf()));
  p.contact.assign(cfg.horizon, std::vector<bool>(robot.nf(), true));
  for (int k = 0; k < cfg.horizon; ++k)
    for (int f = 0; f < robot.nf(); ++f) p.r[k][f] = stand.feet[f].pos - com;
  ASSERT_TRUE(mpc.solve(p));
  const double mg = robot.totalMass() * 9.81;
  // The last steps are weakly constrained (little state cost left after
  // them) and are never executed: only the first output_horizon is sent.
  for (int k = 0; k < cfg.horizon; ++k) {
    const double tol = k < cfg.horizon - 2 ? 0.01 : 0.05;
    Eigen::Vector3d total = Eigen::Vector3d::Zero();
    for (int f = 0; f < robot.nf(); ++f) total += mpc.forces()[k].segment<3>(3 * f);
    EXPECT_NEAR(total.z(), mg, tol * mg) << "step " << k;
    EXPECT_LT(total.head<2>().norm(), tol * mg);
  }
  for (int k = 0; k < cfg.horizon - 2; ++k) {
    EXPECT_LT((mpc.states()[k] - p.x0).norm(), 2e-3) << "step " << k;
  }

  // Swing feet get no force, stance feet stay inside the friction pyramid
  for (int k = 0; k < cfg.horizon; ++k) {
    p.contact[k][0] = p.contact[k][3] = false;  // trot pair in the air
  }
  ASSERT_TRUE(mpc.solve(p));
  for (int k = 0; k < cfg.horizon; ++k) {
    const auto &u = mpc.forces()[k];
    EXPECT_LT(u.segment<3>(0).norm(), 1e-3);
    EXPECT_LT(u.segment<3>(9).norm(), 1e-3);
    for (int f : {1, 2}) {
      const Eigen::Vector3d fi = u.segment<3>(3 * f);
      EXPECT_GE(fi.z(), -1e-3);
      EXPECT_LE(std::abs(fi.x()), cfg.mu * fi.z() + 1e-2);
      EXPECT_LE(std::abs(fi.y()), cfg.mu * fi.z() + 1e-2);
    }
  }
}

// ===========================================================================
// Closed loop: planner + WBC + unilateral-contact simulation
// ===========================================================================
TEST_P(PerRobotLoco, StandsWithoutSteppingAtZeroCommand) {
  Harness h(GetParam());
  const auto st = h.run(1.5, [](double) { return constant(0, 0, 0); }, 0.2);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_EQ(st.planner_failures, 0);
  EXPECT_EQ(st.liftoffs, 0);
  EXPECT_LT((st.end_pos - st.start_pos).head<2>().norm(), 0.01);
  EXPECT_LT(st.max_tilt, 0.02);
}

TEST_P(PerRobotLoco, TrotsInPlace) {
  const auto &c = GetParam();
  auto loco = locomotionConfig(c);
  loco.always_walk = true;
  Harness h(c, loco);
  const auto st = h.run(4.0, [](double) { return constant(0, 0, 0); }, 1.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_EQ(st.planner_failures, 0);
  EXPECT_GE(st.liftoffs, 4 * 5);  // 3 s window = 6 cycles x 4 feet
  const double h0 = h.core.robot().nominalHeight();
  EXPECT_GT(st.min_height, h0 - 0.03);
  EXPECT_LT(st.max_height, h0 + 0.03);
  EXPECT_LT(st.max_tilt, 0.08);
  EXPECT_LT((st.end_pos - st.start_pos).head<2>().norm(), 0.10);
  EXPECT_LT(std::abs(st.end_yaw - st.start_yaw), 0.1);
  EXPECT_LT(st.max_friction_ratio, kMaxFrictionRatio);
}

TEST_P(PerRobotLoco, WalksForward) {
  const auto &c = GetParam();
  const double v = c.name == "gardo" ? 0.4 : 0.3;
  Harness h(c);
  const auto st = h.run(5.0, [v](double) { return constant(v, 0, 0); }, 2.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_EQ(st.planner_failures, 0);
  EXPECT_NEAR(st.avg_vel_body.x(), v, 0.25 * v);
  EXPECT_LT(std::abs(st.avg_vel_body.y()), 0.05);
  EXPECT_LT(std::abs(st.end_yaw - st.start_yaw), 0.15);
  EXPECT_LT(st.max_tilt, 0.1);
  EXPECT_LT(st.max_friction_ratio, kMaxFrictionRatio);
}

TEST_P(PerRobotLoco, WalksSideways) {
  Harness h(GetParam());
  const auto st = h.run(4.0, [](double) { return constant(0, 0.15, 0); }, 2.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_NEAR(st.avg_vel_body.y(), 0.15, 0.05);
  EXPECT_LT(std::abs(st.avg_vel_body.x()), 0.05);
  EXPECT_LT(st.max_tilt, 0.1);
}

TEST_P(PerRobotLoco, TurnsInPlace) {
  Harness h(GetParam());
  const auto st = h.run(4.0, [](double) { return constant(0, 0, 0.6); }, 2.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_NEAR(st.avg_yaw_rate, 0.6, 0.15);
  EXPECT_LT(st.max_tilt, 0.1);
}

TEST_P(PerRobotLoco, StopsAndStandsAfterWalking) {
  const auto &c = GetParam();
  Harness h(c);
  auto st = h.run(3.0, [](double) { return constant(0.3, 0, 0); }, 2.0);
  ASSERT_FALSE(st.fell);
  st = h.run(2.5, [](double) { return constant(0, 0, 0); }, 1.5);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_FALSE(h.planner.gait().walking(1e9));
  EXPECT_EQ(st.liftoffs, 0) << "still stepping 1.5 s after the stop";
  EXPECT_LT(st.avg_vel_body.head<2>().norm(), 0.02);
  for (int f = 0; f < h.core.robot().nf(); ++f) EXPECT_TRUE(h.sim.pinned(f));
  // re-centred over its feet after walking
  const auto e = stanceError(h.core.robot(), h.sim);
  EXPECT_LT(e.offset, 0.01);
  EXPECT_LT(e.heading, 0.02);
}

TEST_P(PerRobotLoco, StandingHoldsAgainstSmallPush) {
  Harness h(GetParam());
  auto zero = [](double) { return constant(0, 0, 0); };
  ASSERT_FALSE(h.run(1.0, zero, 0.5).fell);
  const Eigen::Vector3d start = h.sim.q.head<3>();
  h.sim.push(Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d::Zero());
  const auto st = h.run(2.5, zero, 0.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_EQ(st.liftoffs, 0) << "a small push should not trigger stepping";
  // returns to where it stood (the feet did not move)
  EXPECT_LT((h.sim.q.head<2>() - start.head<2>()).norm(), 0.01);
  const auto e = stanceError(h.core.robot(), h.sim);
  EXPECT_LT(e.offset, 0.01);
  EXPECT_LT(e.heading, 0.02);
}

TEST_P(PerRobotLoco, StandingStepsToRecoverFromLargePush) {
  Harness h(GetParam());
  auto zero = [](double) { return constant(0, 0, 0); };
  ASSERT_FALSE(h.run(1.0, zero, 0.5).fell);
  h.sim.push(Eigen::Vector3d(0.0, 0.4, 0.0), Eigen::Vector3d::Zero());
  auto st = h.run(2.0, zero, 0.0);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  EXPECT_GT(st.liftoffs, 0) << "a large push should trigger catch steps";
  // settles: stops stepping, stands centred over its feet
  st = h.run(1.5, zero, 0.5);
  ASSERT_FALSE(st.fell);
  EXPECT_EQ(st.liftoffs, 0);
  EXPECT_LT(st.avg_vel_body.head<2>().norm(), 0.02);
  const auto e = stanceError(h.core.robot(), h.sim);
  EXPECT_LT(e.offset, 0.015);
  EXPECT_LT(e.heading, 0.03);
}

TEST_P(PerRobotLoco, StandingReturnsToFeetHeadingAfterTwist) {
  Harness h(GetParam());
  auto zero = [](double) { return constant(0, 0, 0); };
  ASSERT_FALSE(h.run(1.0, zero, 0.5).fell);
  h.sim.push(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.6));
  const auto st = h.run(2.5, zero, 1.5);
  ASSERT_FALSE(st.fell);
  ASSERT_FALSE(st.damped) << h.core.safetyReason();
  const auto e = stanceError(h.core.robot(), h.sim);
  EXPECT_LT(e.heading, 0.02);
  EXPECT_LT(e.offset, 0.01);
}

INSTANTIATE_TEST_SUITE_P(Robots, PerRobotLoco,
                         ::testing::Values(go2Case(), gardoCase()),
                         [](const auto &info) { return info.param.name; });
