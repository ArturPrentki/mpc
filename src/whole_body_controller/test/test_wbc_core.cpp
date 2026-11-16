// Unit and closed-loop tests of the ROS-independent WBC core on both robots
// (Go2 test platform and Gardo). The closed-loop tests use a small rigid
// contact simulator: bilateral point contacts solved with a KKT system and
// Baumgarte stabilization, semi-implicit Euler at 1 kHz.

#include "whole_body_controller/wbc_core.hpp"

#include <gtest/gtest.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cmath>
#include <fstream>
#include <sstream>

#ifndef WBC_TEST_DATA_DIR
#error "WBC_TEST_DATA_DIR must be defined"
#endif

using namespace wbc;

namespace {

std::string readFile(const std::string &name) {
  std::ifstream in(std::string(WBC_TEST_DATA_DIR) + "/" + name);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

struct RobotCase {
  std::string name;
  std::string urdf_file;
  CoreConfig config;
};

void PrintTo(const RobotCase &c, std::ostream *os) { *os << c.name; }

RobotCase go2Case() {
  RobotCase c;
  c.name = "go2";
  c.urdf_file = "go2.urdf";
  auto &r = c.config.robot;
  r.base_frame = "base";
  r.foot_frames = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
  for (const char *leg : {"FL", "FR", "RL", "RR"}) {
    for (const char *j : {"_hip_joint", "_thigh_joint", "_calf_joint"}) {
      r.joint_names.push_back(std::string(leg) + j);
    }
    r.nominal_joint_positions.insert(r.nominal_joint_positions.end(),
                                     {0.0, 0.8, -1.5});
  }
  c.config.safety.damping_kd = 2.0;
  return c;
}

RobotCase gardoCase() {
  RobotCase c;
  c.name = "gardo";
  c.urdf_file = "gardo.urdf";
  auto &r = c.config.robot;
  r.base_frame = "TORSO";
  // Config order deliberately differs from Pinocchio's (LF, LH, RF, RH)
  r.foot_frames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  for (const char *leg : {"LF", "RF", "LH", "RH"}) {
    for (const char *j : {"_HAA", "_HFE", "_KFE"}) {
      r.joint_names.push_back(std::string(leg) + j);
    }
    r.nominal_joint_positions.insert(r.nominal_joint_positions.end(),
                                     {0.0, 0.7, -1.4});
  }
  c.config.wbic.joint_kd_stance = 2.0;
  c.config.wbic.joint_kd_swing = 2.0;
  c.config.wbic.joint_kp_swing = 60.0;
  c.config.safety.damping_kd = 5.0;
  return c;
}

// ---------------------------------------------------------------------------
// Minimal rigid-contact simulator
// ---------------------------------------------------------------------------
class ContactSim {
public:
  explicit ContactSim(const RobotModel &robot)
      : robot_(robot), model_(robot.model()), data_(model_) {
    anchors_.resize(robot.nf());
    active_.assign(robot.nf(), false);
  }

  Eigen::VectorXd q, v;
  Eigen::VectorXd tau_applied;  // config order, what the drives produced
  std::vector<Eigen::Vector3d> forces;  // last contact forces (world)

  void setState(const Eigen::VectorXd &q0) {
    q = q0;
    v.setZero(model_.nv);
    tau_applied.setZero(robot_.nj());
    forces.assign(robot_.nf(), Eigen::Vector3d::Zero());
    pinocchio::framesForwardKinematics(model_, data_, q);
  }
  Eigen::Vector3d footPos(int f) {
    pinocchio::framesForwardKinematics(model_, data_, q);
    return data_.oMf[robot_.footFrameId(f)].translation();
  }
  void pin(int f) {
    anchors_[f] = footPos(f);
    active_[f] = true;
  }
  void release(int f) { active_[f] = false; }
  bool pinned(int f) const { return active_[f]; }

  // Applies a joint command like a drive with software PD would.
  void step(const JointCommand &cmd, double dt) {
    Eigen::VectorXd qj(robot_.nj()), vj(robot_.nj());
    robot_.jointsFromQ(q, qj);
    robot_.jointsFromV(v, vj);
    tau_applied = cmd.tau_ff + cmd.kp.cwiseProduct(cmd.q_des - qj) +
                  cmd.kd.cwiseProduct(cmd.v_des - vj);
    tau_applied = tau_applied.cwiseMax(-robot_.effortLimits())
                      .cwiseMin(robot_.effortLimits());
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
    robot_.jointsToV(tau_applied, tau);

    pinocchio::crba(model_, data_, q);
    Eigen::MatrixXd M = data_.M;
    M.triangularView<Eigen::StrictlyLower>() =
        M.transpose().triangularView<Eigen::StrictlyLower>();
    const Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, data_, q, v);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::forwardKinematics(model_, data_, q, v,
                                 Eigen::VectorXd::Zero(model_.nv));
    pinocchio::updateFramePlacements(model_, data_);

    int nc = 0;
    for (bool a : active_) nc += a;
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(model_.nv + 3 * nc, model_.nv + 3 * nc);
    Eigen::VectorXd rhs(model_.nv + 3 * nc);
    K.topLeftCorner(model_.nv, model_.nv) = M;
    rhs.head(model_.nv) = tau - nle;
    Eigen::MatrixXd J6(6, model_.nv);
    int r = 0;
    for (int f = 0; f < robot_.nf(); ++f) {
      forces[f].setZero();
      if (!active_[f]) continue;
      const auto fid = robot_.footFrameId(f);
      J6.setZero();
      pinocchio::getFrameJacobian(model_, data_, fid,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J6);
      const Eigen::MatrixXd Jc = J6.topRows<3>();
      const Eigen::Vector3d gamma =
          pinocchio::getFrameClassicalAcceleration(
              model_, data_, fid, pinocchio::LOCAL_WORLD_ALIGNED)
              .linear();
      const Eigen::Vector3d err = data_.oMf[fid].translation() - anchors_[f];
      const Eigen::Vector3d vel = Jc * v;
      K.block(model_.nv + 3 * r, 0, 3, model_.nv) = Jc;
      K.block(0, model_.nv + 3 * r, model_.nv, 3) = -Jc.transpose();
      rhs.segment<3>(model_.nv + 3 * r) = -gamma - 400.0 * err - 40.0 * vel;
      ++r;
    }
    const Eigen::VectorXd sol = K.fullPivLu().solve(rhs);
    const Eigen::VectorXd a = sol.head(model_.nv);
    r = 0;
    for (int f = 0; f < robot_.nf(); ++f) {
      if (!active_[f]) continue;
      forces[f] = sol.segment<3>(model_.nv + 3 * r++);
    }
    v += a * dt;
    q = pinocchio::integrate(model_, q, v * dt);
  }

  void fillMeasured(SystemState &s) const {
    s.q = q;
    s.v = v;
    s.tau_meas = tau_applied;
  }

private:
  const RobotModel &robot_;
  const pinocchio::Model &model_;
  pinocchio::Data data_;
  std::vector<Eigen::Vector3d> anchors_;
  std::vector<bool> active_;
};

double orientationError(const Eigen::VectorXd &q, const Eigen::VectorXd &q_ref) {
  const Eigen::Quaterniond a(q[6], q[3], q[4], q[5]);
  const Eigen::Quaterniond b(q_ref[6], q_ref[3], q_ref[4], q_ref[5]);
  return a.angularDistance(b);
}

}  // namespace

// ===========================================================================
// RobotModel
// ===========================================================================
TEST(RobotModel, GardoJointOrderMapping) {
  const auto c = gardoCase();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  EXPECT_EQ(robot.nj(), 12);
  EXPECT_EQ(robot.nf(), 4);
  // Pinocchio orders LF, LH, RF, RH; config is LF, RF, LH, RH.
  const auto &model = robot.model();
  for (int j = 0; j < robot.nj(); ++j) {
    const auto jid = model.getJointId(c.config.robot.joint_names[j]);
    EXPECT_EQ(robot.jointQIdx(j), model.joints[jid].idx_q());
    EXPECT_EQ(robot.jointVIdx(j), model.joints[jid].idx_v());
  }
  EXPECT_NE(robot.jointVIdx(3), 9);  // RF_HAA is not Pinocchio's 4th joint
  // RF foot is moved by config joints 3, 4, 5
  EXPECT_EQ(robot.legJoints(1), (std::vector<int>{3, 4, 5}));
  for (int j = 0; j < 12; ++j) EXPECT_EQ(robot.footOfJoint(j), j / 3);
  EXPECT_NEAR(robot.nominalHeight(), 0.434, 2e-3);
  EXPECT_GT(robot.totalMass(), 30.0);

  // Round trip config <-> Pinocchio
  Eigen::VectorXd joints = Eigen::VectorXd::LinSpaced(12, 0.1, 1.2);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.nq());
  robot.jointsToQ(joints, q);
  Eigen::VectorXd back(12);
  robot.jointsFromQ(q, back);
  EXPECT_TRUE(back.isApprox(joints));
}

TEST(RobotModel, RejectsInvalidConfig) {
  const std::string urdf = readFile("go2.urdf");
  auto cfg = go2Case().config.robot;
  auto bad = cfg;
  bad.base_frame = "TORSO";
  EXPECT_THROW(RobotModel::fromUrdf(urdf, bad), std::runtime_error);
  bad = cfg;
  bad.foot_frames[2] = "RL_FOOT";
  EXPECT_THROW(RobotModel::fromUrdf(urdf, bad), std::runtime_error);
  bad = cfg;
  bad.joint_names.pop_back();
  EXPECT_THROW(RobotModel::fromUrdf(urdf, bad), std::runtime_error);
  bad = cfg;
  bad.nominal_joint_positions.pop_back();
  EXPECT_THROW(RobotModel::fromUrdf(urdf, bad), std::runtime_error);
  EXPECT_NO_THROW(RobotModel::fromUrdf(urdf, cfg));
}

class PerRobot : public ::testing::TestWithParam<RobotCase> {};

TEST_P(PerRobot, StandTargetReachesFeetWithIK) {
  const auto &c = GetParam();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  const auto t = robot.standTarget(Eigen::Vector3d(1.0, -2.0, 7.0), 0.6, 0.1);
  EXPECT_NEAR(t.q[2], 0.1 + robot.nominalHeight(), 1e-12);
  pinocchio::Data data(robot.model());
  pinocchio::framesForwardKinematics(robot.model(), data, t.q);
  double fz_sum = 0.0;
  for (int f = 0; f < robot.nf(); ++f) {
    EXPECT_LT((data.oMf[robot.footFrameId(f)].translation() - t.feet[f].pos).norm(),
              1e-5);
    EXPECT_NEAR(t.feet[f].pos.z(), 0.1, 1e-12);
    EXPECT_TRUE(t.feet[f].contact);
    fz_sum += t.feet[f].force.z();
  }
  EXPECT_NEAR(fz_sum, robot.totalMass() * 9.81, 1e-6);
}

// ===========================================================================
// Closed loop
// ===========================================================================
TEST_P(PerRobot, StandsStillWithBalancedForces) {
  const auto &c = GetParam();
  WbcCore core(readFile(c.urdf_file), c.config);
  const auto &robot = core.robot();
  ContactSim sim(robot);
  sim.setState(robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0).q);
  for (int f = 0; f < robot.nf(); ++f) sim.pin(f);

  SystemState s;
  s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());
  sim.fillMeasured(s);
  core.activate(0.0, s);

  const double dt = 1e-3;
  for (int k = 0; k < 500; ++k) {
    sim.fillMeasured(s);
    const auto &cmd = core.update(k * dt, s);
    ASSERT_TRUE(core.wbcOk()) << "tick " << k;
    ASSERT_EQ(core.safetyState(), SafetySupervisor::State::OK);
    sim.step(cmd, dt);
  }
  const auto &out = core.output();
  Eigen::Vector3d total = Eigen::Vector3d::Zero();
  for (int f = 0; f < robot.nf(); ++f) {
    total += out.forces.segment<3>(3 * f);
    EXPECT_GT(out.forces[3 * f + 2], 0.0);
    EXPECT_EQ(core.contactModes()[f], ContactMode::STANCE);
  }
  EXPECT_NEAR(total.z(), robot.totalMass() * 9.81, 0.02 * robot.totalMass() * 9.81);
  EXPECT_LT(out.base_dynamics_residual, 1e-2);
  EXPECT_FALSE(out.torque_limited);
  EXPECT_LT((sim.q.head<3>() - core.target().q.head<3>()).norm(), 2e-3);
}

TEST_P(PerRobot, RecoversFromBasePerturbation) {
  const auto &c = GetParam();
  WbcCore core(readFile(c.urdf_file), c.config);
  const auto &robot = core.robot();
  ContactSim sim(robot);

  // Start 3 cm low, rolled 0.1 rad and pitched 0.05 rad, feet on the ground.
  const auto stand = robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0);
  Eigen::VectorXd q0 = stand.q;
  q0[2] -= 0.03;
  const Eigen::Quaterniond tilt(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()));
  q0.segment<4>(3) = tilt.coeffs();
  std::vector<Eigen::Vector3d> feet;
  for (const auto &foot : stand.feet) feet.push_back(foot.pos);
  robot.solveFootIK(q0, feet);
  sim.setState(q0);
  for (int f = 0; f < robot.nf(); ++f) sim.pin(f);

  SystemState s;
  s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());
  sim.fillMeasured(s);
  core.activate(0.0, s);
  // activate() anchors at the current (perturbed) pose; command the nominal
  // stand explicitly through a one-point trajectory held for 3 s.
  auto traj = std::make_shared<Trajectory>();
  traj->stamp = 0.0;
  traj->points.push_back({0.0, stand});
  traj->points.push_back({3.0, stand});
  core.setTrajectory(traj);

  const double dt = 1e-3;
  double min_fz = 1e9;
  for (int k = 0; k < 2500; ++k) {
    sim.fillMeasured(s);
    const auto &cmd = core.update(k * dt, s);
    ASSERT_TRUE(core.wbcOk()) << "tick " << k;
    ASSERT_NE(core.safetyState(), SafetySupervisor::State::DAMPING);
    sim.step(cmd, dt);
    if (k > 50)
      for (const auto &f : sim.forces) min_fz = std::min(min_fz, f.z());
  }
  EXPECT_EQ(core.referenceMode(), TrajectoryInterpolator::Mode::TRACKING);
  EXPECT_LT((sim.q.head<3>() - stand.q.head<3>()).norm(), 5e-3);
  EXPECT_LT(orientationError(sim.q, stand.q), 1e-2);
  EXPECT_GT(min_fz, 0.0) << "a foot would have needed to pull on the ground";
}

TEST_P(PerRobot, SwingsOneFootAndTouchesDown) {
  const auto &c = GetParam();
  WbcCore core(readFile(c.urdf_file), c.config);
  const auto &robot = core.robot();
  ContactSim sim(robot);
  const auto stand = robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0);
  sim.setState(stand.q);
  for (int f = 0; f < robot.nf(); ++f) sim.pin(f);

  SystemState s;
  s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());
  sim.fillMeasured(s);
  core.activate(0.0, s);

  // Shift the base towards the support triangle of feet 1..3, lift foot 0 by
  // 6 cm between t = 0.6 s and 1.2 s and put it back down.
  const int sw = 0;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (int f = 1; f < robot.nf(); ++f) centroid += stand.feet[f].pos;
  centroid /= robot.nf() - 1;
  const Eigen::Vector3d shift(0.5 * centroid.x(), 0.5 * centroid.y(), 0.0);
  const double t_lift = 0.6, t_land = 1.2, T = 2.0, h = 0.06;

  auto traj = std::make_shared<Trajectory>();
  traj->stamp = 0.0;
  for (double t = 0.0; t <= T + 1e-9; t += 0.02) {
    TargetState p = stand;
    const double ramp = std::min(1.0, t / 0.4);
    p.q.head<3>() += ramp * shift;
    p.v.head<3>() = (t < 0.4 ? 1.0 / 0.4 : 0.0) * shift;
    const bool swing = t >= t_lift && t < t_land;
    p.feet[sw].contact = !swing;
    p.feet[sw].force.setZero();
    if (swing) {
      const double ph = (t - t_lift) / (t_land - t_lift);
      p.feet[sw].pos.z() += h * std::sin(M_PI * ph);
      p.feet[sw].vel.z() = h * M_PI / (t_land - t_lift) * std::cos(M_PI * ph);
    }
    for (int f = 0; f < robot.nf(); ++f) {
      if (p.feet[f].contact && f != sw)
        p.feet[f].force = Eigen::Vector3d(0, 0, robot.totalMass() * 9.81 / 3.0);
    }
    traj->points.push_back({t, p});
  }
  core.setTrajectory(traj);

  const double dt = 1e-3;
  const double ground = stand.feet[sw].pos.z();
  double max_h = -1.0, max_base_err = 0.0;
  bool saw_swing = false, saw_stance_after = false;
  for (int k = 0; k < static_cast<int>(T / dt); ++k) {
    const double t = k * dt;
    sim.fillMeasured(s);
    const auto &cmd = core.update(t, s);
    ASSERT_TRUE(core.wbcOk()) << "tick " << k;
    ASSERT_NE(core.safetyState(), SafetySupervisor::State::DAMPING)
        << core.safetyReason();
    const auto mode = core.contactModes()[sw];
    if (mode == ContactMode::SWING && sim.pinned(sw)) sim.release(sw);
    if (!sim.pinned(sw) && t > t_land - 0.1 && sim.footPos(sw).z() <= ground)
      sim.pin(sw);  // touchdown
    saw_swing |= mode == ContactMode::SWING;
    saw_stance_after |= t > t_land && mode == ContactMode::STANCE;
    sim.step(cmd, dt);
    max_h = std::max(max_h, sim.footPos(sw).z() - ground);
    max_base_err = std::max(
        max_base_err, (sim.q.head<3>() - core.target().q.head<3>()).norm());
  }
  EXPECT_TRUE(saw_swing);
  EXPECT_TRUE(saw_stance_after) << "foot never went back to STANCE";
  EXPECT_NEAR(max_h, h, 0.012);
  EXPECT_LT(max_base_err, 0.02);
  EXPECT_TRUE(sim.pinned(sw));
}

INSTANTIATE_TEST_SUITE_P(Robots, PerRobot,
                         ::testing::Values(go2Case(), gardoCase()),
                         [](const auto &info) { return info.param.name; });

// ===========================================================================
// Safety supervisor
// ===========================================================================
TEST(Safety, HoldsLastCommandThenDamps) {
  const auto c = go2Case();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  SafetyConfig cfg;
  cfg.hold_time = 0.05;
  cfg.damping_kd = 3.0;
  SafetySupervisor sup(robot, cfg);
  SystemState s;
  s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());

  JointCommand good;
  good.resize(robot.nj());
  good.tau_ff.setConstant(1000.0);  // will be clamped
  JointCommand cmd = good;
  EXPECT_EQ(sup.step(0.0, true, s, cmd), SafetySupervisor::State::OK);
  EXPECT_TRUE(cmd.tau_ff.isApprox(robot.effortLimits()));

  JointCommand bad;
  bad.resize(robot.nj());
  bad.tau_ff.setConstant(std::nan(""));
  cmd = bad;
  EXPECT_EQ(sup.step(0.01, true, s, cmd), SafetySupervisor::State::HOLDING);
  EXPECT_TRUE(cmd.tau_ff.isApprox(robot.effortLimits()));
  cmd = bad;
  EXPECT_EQ(sup.step(0.03, false, s, cmd), SafetySupervisor::State::HOLDING);
  // recovers
  cmd = good;
  EXPECT_EQ(sup.step(0.04, true, s, cmd), SafetySupervisor::State::OK);
  // fails for longer than hold_time -> latched damping
  cmd = bad;
  sup.step(0.05, false, s, cmd);
  cmd = bad;
  EXPECT_EQ(sup.step(0.2, false, s, cmd), SafetySupervisor::State::DAMPING);
  EXPECT_TRUE(cmd.tau_ff.isZero());
  EXPECT_TRUE(cmd.kp.isZero());
  EXPECT_TRUE((cmd.kd.array() == 3.0).all());
  cmd = good;
  EXPECT_EQ(sup.step(0.3, true, s, cmd), SafetySupervisor::State::DAMPING);
  sup.reset();
  cmd = good;
  EXPECT_EQ(sup.step(0.4, true, s, cmd), SafetySupervisor::State::OK);
}

TEST(Safety, DampsWhenTipping) {
  const auto c = go2Case();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  SafetySupervisor sup(robot, SafetyConfig{});
  SystemState s;
  s.resize(robot.nq(), robot.nv(), robot.nj(), robot.nf());
  s.base_rot = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  JointCommand cmd;
  cmd.resize(robot.nj());
  EXPECT_EQ(sup.step(0.0, true, s, cmd), SafetySupervisor::State::DAMPING);
}

// ===========================================================================
// Trajectory interpolator
// ===========================================================================
TEST(Interpolator, TracksFadesAndExpires) {
  const auto c = go2Case();
  const auto robot = RobotModel::fromUrdf(readFile(c.urdf_file), c.config.robot);
  TrajectoryInterpolator interp(robot, 0.1);
  const auto fallback = robot.standTarget(Eigen::Vector3d::Zero(), 0.0, 0.0);
  TargetState out = fallback;

  EXPECT_EQ(interp.sample(0.0, fallback, out),
            TrajectoryInterpolator::Mode::EXPIRED);

  auto traj = std::make_shared<Trajectory>();
  traj->stamp = 10.0;
  TargetState a = fallback, b = fallback;
  a.q[0] = 0.0;
  b.q[0] = 0.3;
  b.v[0] = 1.0;
  a.feet[0].contact = true;
  b.feet[0].contact = false;
  // non-uniform spacing on purpose
  traj->points = {{0.0, a}, {0.3, b}, {0.35, b}};
  interp.set(traj);

  EXPECT_EQ(interp.sample(10.1, fallback, out),
            TrajectoryInterpolator::Mode::TRACKING);
  EXPECT_NEAR(out.q[0], 0.1, 1e-9);
  EXPECT_NEAR(out.v[0], 1.0 / 3.0, 1e-9);
  EXPECT_TRUE(out.feet[0].contact);  // contact of the interval start
  EXPECT_EQ(interp.sample(10.32, fallback, out),
            TrajectoryInterpolator::Mode::TRACKING);
  EXPECT_FALSE(out.feet[0].contact);

  EXPECT_EQ(interp.sample(10.40, fallback, out),
            TrajectoryInterpolator::Mode::FADING);
  EXPECT_NEAR(out.q[0], 0.15, 1e-9);  // half way back to the fallback
  EXPECT_NEAR(out.v[0], 0.5, 1e-9);

  EXPECT_EQ(interp.sample(10.5, fallback, out),
            TrajectoryInterpolator::Mode::EXPIRED);
  EXPECT_NEAR(out.q[0], 0.0, 1e-12);
  EXPECT_TRUE(out.feet[0].contact);
}
