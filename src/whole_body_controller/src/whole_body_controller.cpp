#include "whole_body_controller/whole_body_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <exception>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace wbc {

namespace {
constexpr const char *kEffort = hardware_interface::HW_IF_EFFORT;
constexpr const char *kPosition = hardware_interface::HW_IF_POSITION;
constexpr const char *kVelocity = hardware_interface::HW_IF_VELOCITY;
constexpr const char *kKp = hardware_interface::HW_IF_PROPORTIONAL_GAIN;
constexpr const char *kKd = hardware_interface::HW_IF_DERIVATIVE_GAIN;

double read(hardware_interface::LoanedStateInterface *itf, double fallback) {
  return itf ? itf->get_optional().value_or(fallback) : fallback;
}
void write(hardware_interface::LoanedCommandInterface *itf, double value) {
  if (itf) (void)itf->set_value(value);
}

template <typename Vec3>
void toMsg(const Eigen::Vector3d &v, Vec3 &msg) {
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
}

wbc_control_msgs::msg::RobotState
toRobotStateMsg(const RobotModel &robot, const Eigen::VectorXd &q,
                const Eigen::VectorXd &v, const Eigen::VectorXd &a,
                const std::vector<FootState> &feet,
                const Eigen::VectorXd *forces) {
  wbc_control_msgs::msg::RobotState msg;
  msg.body_pose.position.x = q[0];
  msg.body_pose.position.y = q[1];
  msg.body_pose.position.z = q[2];
  msg.body_pose.orientation.x = q[3];
  msg.body_pose.orientation.y = q[4];
  msg.body_pose.orientation.z = q[5];
  msg.body_pose.orientation.w = q[6];
  toMsg(v.segment<3>(0), msg.body_velocity.linear);
  toMsg(v.segment<3>(3), msg.body_velocity.angular);
  toMsg(a.segment<3>(0), msg.body_acceleration.linear);
  toMsg(a.segment<3>(3), msg.body_acceleration.angular);
  Eigen::VectorXd joints(robot.nj());
  robot.jointsFromQ(q, joints);
  msg.joint_positions.assign(joints.data(), joints.data() + joints.size());
  robot.jointsFromV(v, joints);
  msg.joint_velocities.assign(joints.data(), joints.data() + joints.size());
  robot.jointsFromV(a, joints);
  msg.joint_accelerations.assign(joints.data(), joints.data() + joints.size());
  for (size_t f = 0; f < feet.size(); ++f) {
    wbc_control_msgs::msg::FootState fs;
    fs.name = robot.config().foot_frames[f];
    toMsg(feet[f].pos, fs.position);
    toMsg(feet[f].vel, fs.velocity);
    toMsg(feet[f].acc, fs.acceleration);
    toMsg(forces ? Eigen::Vector3d(forces->segment<3>(3 * f)) : feet[f].force,
          fs.contact_force);
    fs.is_contact = feet[f].contact;
    msg.feet.push_back(fs);
  }
  return msg;
}
}  // namespace

// ---------------------------------------------------------------------------
// lifecycle
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WBC::on_init() {
  try {
    param_listener_ =
        std::make_shared<whole_body_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

CoreConfig WBC::makeCoreConfig() const {
  const auto &p = params_;
  CoreConfig c;
  c.robot.base_frame = p.base_frame;
  c.robot.foot_frames = p.foot_frames;
  c.robot.joint_names = p.joint_names;
  c.robot.nominal_joint_positions = p.nominal_joint_positions;
  c.robot.nominal_height = p.nominal_height;

  auto &w = c.wbic;
  w.kp_base_lin = p.task.kp_base_lin;
  w.kd_base_lin = p.task.kd_base_lin;
  w.kp_base_ang = p.task.kp_base_ang;
  w.kd_base_ang = p.task.kd_base_ang;
  w.kp_swing = p.task.kp_swing;
  w.kd_swing = p.task.kd_swing;
  w.kp_posture = p.task.kp_posture;
  w.kd_posture = p.task.kd_posture;
  w.late_touchdown_velocity = p.task.late_touchdown_velocity;
  w.mu = p.qp.mu;
  w.fz_min = p.qp.fz_min;
  w.fz_max = p.qp.fz_max;
  w.torque_limit_scale = p.qp.torque_limit_scale;
  w.weight_floating_base = p.qp.weight_floating_base;
  w.weight_force = p.qp.weight_force;
  w.joint_kp_stance = p.joint_gains.kp_stance;
  w.joint_kd_stance = p.joint_gains.kd_stance;
  w.joint_kp_swing = p.joint_gains.kp_swing;
  w.joint_kd_swing = p.joint_gains.kd_swing;
  w.control_dt = 1.0 / std::max<double>(get_update_rate(), 1.0);

  auto &cd = c.contact;
  cd.enabled = p.contact_detection.enabled;
  cd.touchdown_force = p.contact_detection.touchdown_force;
  cd.liftoff_force = p.contact_detection.liftoff_force;
  cd.activation_grace_time = p.contact_detection.activation_grace_time;
  cd.stall_velocity_ratio = p.contact_detection.stall_velocity_ratio;
  cd.stall_time = p.contact_detection.stall_time;
  cd.min_stance_time = p.contact_detection.min_stance_time;

  c.safety.max_tilt = p.safety.max_tilt;
  c.safety.hold_time = p.safety.hold_time;
  c.safety.damping_kd = p.safety.damping_kd;
  c.fade_time = p.fade_time;
  return c;
}

LocomotionConfig WBC::makeLocomotionConfig() const {
  const auto &p = params_;
  LocomotionConfig l;
  l.gait.cycle_time = p.gait.cycle_time;
  l.gait.duty_factor = p.gait.duty_factor;
  l.gait.phase_offsets = p.gait.phase_offsets;
  l.step_height = p.gait.step_height;
  l.mpc.horizon = static_cast<int>(p.mpc.horizon);
  l.mpc.dt = p.mpc.dt;
  l.mpc.mu = p.mpc.mu;
  l.mpc.fz_max = p.mpc.fz_max;
  for (int i = 0; i < 12; ++i) l.mpc.state_weights[i] = p.mpc.state_weights[i];
  l.mpc.force_weight = p.mpc.force_weight;
  l.foothold_feedback_gain = p.foothold.feedback_gain;
  l.max_foothold_offset = p.foothold.max_offset;
  l.max_vx = p.command.max_vx;
  l.max_vy = p.command.max_vy;
  l.max_wz = p.command.max_wz;
  l.max_lin_acc = p.command.max_lin_acc;
  l.max_yaw_acc = p.command.max_yaw_acc;
  l.active_threshold = p.command.active_threshold;
  l.stop_delay = p.command.stop_delay;
  l.always_walk = p.command.always_walk;
  l.body_height_offset = p.mpc.body_height_offset;
  l.output_dt = p.mpc.output_dt;
  l.output_horizon = p.mpc.output_horizon;
  return l;
}

bool WBC::buildCore(const std::string &urdf) {
  std::lock_guard<std::mutex> lock(core_mutex_);
  if (core_ready_) return true;
  try {
    auto core = std::make_shared<WbcCore>(urdf, makeCoreConfig());
    const auto &r = core->robot();
    state_.resize(r.nq(), r.nv(), r.nj(), r.nf());
    joint_pos_.setZero(r.nj());
    joint_vel_.setZero(r.nj());
    joint_eff_.setZero(r.nj());
    tau_out_.setZero(r.nj());
    if (params_.mpc.enabled) {
      planner_ = std::make_unique<LocomotionPlanner>(core->robot(),
                                                     makeLocomotionConfig());
      snapshot_.feet.assign(r.nf(), Eigen::Vector3d::Zero());
    }
    core_ = core;
    core_ready_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
                "WBC ready: %d joints, %d feet, mass %.2f kg, nominal height "
                "%.3f m, joint PD %s",
                r.nj(), r.nf(), r.totalMass(), r.nominalHeight(),
                hardware_pd_ ? "in the drives" : "in software");
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Cannot build the controller for this robot: %s", e.what());
    return false;
  }
}

controller_interface::CallbackReturn
WBC::on_configure(const rclcpp_lifecycle::State &) {
  params_ = param_listener_->get_params();
  const auto &p = params_;
  auto logger = get_node()->get_logger();

  if (p.joint_names.size() != p.nominal_joint_positions.size()) {
    RCLCPP_ERROR(logger, "joint_names (%zu) and nominal_joint_positions (%zu) "
                         "must have the same length",
                 p.joint_names.size(), p.nominal_joint_positions.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (p.base_state_interfaces.size() != 13) {
    RCLCPP_ERROR(logger, "base_state_interfaces must list 13 interfaces "
                         "(pos xyz, quat xyzw, lin vel xyz, ang vel xyz)");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!hasCommandInterface(kEffort)) {
    RCLCPP_ERROR(logger, "command_interfaces must contain 'effort'");
    return controller_interface::CallbackReturn::ERROR;
  }
  hardware_pd_ = hasCommandInterface(kPosition) &&
                 hasCommandInterface(kVelocity) && hasCommandInterface(kKp) &&
                 hasCommandInterface(kKd);

  stopPlanner();
  core_ready_ = false;
  planner_.reset();
  core_.reset();
  const std::string &urdf = get_robot_description();
  if (!urdf.empty()) {
    if (!buildCore(urdf)) return controller_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(logger, "No robot description from the controller manager, "
                        "waiting for /robot_description");
    urdf_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (!core_ready_) buildCore(msg->data);
        });
  }

  if (p.mpc.enabled) {
    if (p.gait.phase_offsets.size() != p.foot_frames.size()) {
      RCLCPP_ERROR(logger, "gait.phase_offsets needs one entry per foot (%zu)",
                   p.foot_frames.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    trajectory_sub_.reset();
    command_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        p.command.topic, rclcpp::QoS(10),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          command_.vx = msg->linear.x;
          command_.vy = msg->linear.y;
          command_.wz = msg->angular.z;
          command_stamp_ = std::chrono::steady_clock::now();
        });
    RCLCPP_INFO(logger, "Built-in MPC enabled: velocity commands on %s",
                p.command.topic.c_str());
  } else {
    command_sub_.reset();
    trajectory_sub_ =
        get_node()->create_subscription<wbc_control_msgs::msg::MpcTrajectory>(
            p.trajectory_topic, rclcpp::QoS(1),
            [this](const wbc_control_msgs::msg::MpcTrajectory::SharedPtr msg) {
              onTrajectory(*msg);
            });
    RCLCPP_INFO(logger, "Built-in MPC disabled: following %s",
                p.trajectory_topic.c_str());
  }
  contacts_pub_ =
      get_node()->create_publisher<wbc_control_msgs::msg::FootContactFlags>(
          "/foot_contacts", rclcpp::QoS(1));
  state_pub_ = get_node()->create_publisher<wbc_control_msgs::msg::RobotState>(
      "/robot_state", rclcpp::QoS(1));
  target_pub_ = get_node()->create_publisher<wbc_control_msgs::msg::RobotState>(
      "/target_state", rclcpp::QoS(1));
  return controller_interface::CallbackReturn::SUCCESS;
}

bool WBC::hasCommandInterface(const std::string &name) const {
  return std::find(params_.command_interfaces.begin(),
                   params_.command_interfaces.end(),
                   name) != params_.command_interfaces.end();
}

controller_interface::InterfaceConfiguration
WBC::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joint_names) {
    for (const auto &itf : params_.command_interfaces) {
      config.names.push_back(joint + "/" + itf);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
WBC::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joint_names) {
    config.names.push_back(joint + "/" + kPosition);
    config.names.push_back(joint + "/" + kVelocity);
    if (params_.contact_detection.enabled) {
      config.names.push_back(joint + "/" + kEffort);
    }
  }
  for (const auto &var : params_.base_state_interfaces) {
    config.names.push_back(params_.estimator_name + "/" + var);
  }
  return config;
}

controller_interface::CallbackReturn
WBC::on_activate(const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();
  const auto start = std::chrono::steady_clock::now();
  while (!core_ready_ &&
         std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
    RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 1000,
                         "Waiting for the robot description...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!core_ready_) {
    RCLCPP_ERROR(logger, "No valid robot description, cannot activate");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Map loaned interfaces by name; config order everywhere.
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *> st;
  std::unordered_map<std::string, hardware_interface::LoanedCommandInterface *>
      cm;
  for (auto &itf : state_interfaces_) st[itf.get_name()] = &itf;
  for (auto &itf : command_interfaces_) cm[itf.get_name()] = &itf;
  auto state_itf = [&](const std::string &name, bool required) {
    auto it = st.find(name);
    if (it == st.end()) {
      if (required) throw std::runtime_error("missing state interface " + name);
      return static_cast<hardware_interface::LoanedStateInterface *>(nullptr);
    }
    return it->second;
  };
  auto cmd_itf = [&](const std::string &name) {
    auto it = cm.find(name);
    return it == cm.end() ? nullptr : it->second;
  };

  try {
    base_ifs_.clear();
    for (const auto &var : params_.base_state_interfaces) {
      base_ifs_.push_back(state_itf(params_.estimator_name + "/" + var, true));
    }
    pos_ifs_.clear();
    vel_ifs_.clear();
    eff_ifs_.clear();
    cmd_ifs_.clear();
    for (const auto &j : params_.joint_names) {
      pos_ifs_.push_back(state_itf(j + "/" + kPosition, true));
      vel_ifs_.push_back(state_itf(j + "/" + kVelocity, true));
      eff_ifs_.push_back(state_itf(j + "/" + kEffort,
                                   params_.contact_detection.enabled));
      JointCommandInterfaces c;
      c.effort = cmd_itf(j + "/" + kEffort);
      c.position = cmd_itf(j + "/" + kPosition);
      c.velocity = cmd_itf(j + "/" + kVelocity);
      c.kp = cmd_itf(j + "/" + kKp);
      c.kd = cmd_itf(j + "/" + kKd);
      if (!c.effort) throw std::runtime_error("missing " + j + "/effort");
      cmd_ifs_.push_back(c);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!readState()) {
    RCLCPP_ERROR(logger, "State estimate is not valid, cannot activate");
    return controller_interface::CallbackReturn::ERROR;
  }
  const double now = get_node()->now().seconds();
  core_->activate(now, state_);
  last_safety_ = core_->safetyState();
  last_ref_mode_ = core_->referenceMode();
  tick_ = 0;
  if (planner_) {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_ = VelocityCommand{};  // never start walking on a stale command
      command_stamp_ = std::chrono::steady_clock::now();
    }
    storeSnapshot(now);
    planner_->reset(snapshot_);
    startPlanner();
  }
  RCLCPP_INFO(logger, "WBC active, standing at the current pose");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WBC::on_deactivate(const rclcpp_lifecycle::State &) {
  stopPlanner();
  if (!cmd_ifs_.empty()) writeDamping();
  return controller_interface::CallbackReturn::SUCCESS;
}

WBC::~WBC() { stopPlanner(); }

// ---------------------------------------------------------------------------
// built-in locomotion planner (non-RT thread)
// ---------------------------------------------------------------------------
void WBC::storeSnapshot(double time) {
  // RT side: never blocks; the planner skips a cycle if it holds the lock
  std::unique_lock<std::mutex> lock(snapshot_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  snapshot_.time = time;
  snapshot_.base_pos = state_.base_pos;
  snapshot_.base_rot = state_.base_rot;
  snapshot_.base_vel_world = state_.base_vel_world;
  for (size_t f = 0; f < snapshot_.feet.size(); ++f) {
    snapshot_.feet[f] = state_.feet[f].pos;
  }
  snapshot_fresh_ = true;
}

void WBC::startPlanner() {
  stopPlanner();
  planner_running_ = true;
  planner_thread_ = std::thread([this] { plannerLoop(); });
}

void WBC::stopPlanner() {
  planner_running_ = false;
  if (planner_thread_.joinable()) planner_thread_.join();
}

void WBC::plannerLoop() {
  using clock = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / params_.mpc.rate));
  const auto timeout = std::chrono::duration<double>(params_.command.timeout);
  auto logger = get_node()->get_logger();
  auto next = clock::now();
  int failures = 0;
  PlannerInput input;
  while (planner_running_) {
    next += period;
    const auto now = clock::now();
    if (next < now) next = now;  // overran: do not try to catch up
    std::this_thread::sleep_until(next);
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (!snapshot_fresh_) continue;
      input = snapshot_;
      snapshot_fresh_ = false;
    }
    VelocityCommand cmd;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      cmd = command_;
      if (timeout.count() > 0.0 && clock::now() - command_stamp_ > timeout) {
        cmd = VelocityCommand{};
      }
    }
    try {
      auto traj = planner_->update(input, cmd);
      if (traj) {
        core_->setTrajectory(std::move(traj));
        failures = 0;
      } else if (++failures % 10 == 1) {
        RCLCPP_WARN(logger, "MPC failed to solve (%d in a row); the WBC fades "
                            "to standing if this persists",
                    failures);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(logger, "Locomotion planner error: %s", e.what());
    }
  }
}

// ---------------------------------------------------------------------------
// planner input (non-RT thread)
// ---------------------------------------------------------------------------
void WBC::onTrajectory(const wbc_control_msgs::msg::MpcTrajectory &msg) {
  if (!core_ready_) return;
  const auto &robot = core_->robot();
  auto logger = get_node()->get_logger();
  auto clock = get_node()->get_clock();

  auto traj = std::make_shared<Trajectory>();
  traj->stamp = rclcpp::Time(msg.header.stamp, clock->get_clock_type()).seconds();

  bool all_zero = true;
  for (const auto &step : msg.steps) {
    all_zero &= step.time_offset.sec == 0 && step.time_offset.nanosec == 0;
  }

  Eigen::VectorXd joints(robot.nj());
  double last_t = -1.0;
  for (size_t k = 0; k < msg.steps.size(); ++k) {
    const auto &step = msg.steps[k];
    if (static_cast<int>(step.joint_positions.size()) != robot.nj() ||
        static_cast<int>(step.joint_velocities.size()) != robot.nj() ||
        static_cast<int>(step.joint_accelerations.size()) != robot.nj() ||
        static_cast<int>(step.feet.size()) != robot.nf()) {
      RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
                           "Ignoring trajectory: expected %d joints and %d "
                           "feet per step",
                           robot.nj(), robot.nf());
      return;
    }
    const double t =
        all_zero ? k * params_.default_step_dt
                 : step.time_offset.sec + 1e-9 * step.time_offset.nanosec;
    if (t <= last_t) continue;  // e.g. impulse nodes with the same time
    last_t = t;

    TrajectoryPoint pt;
    pt.time_offset = t;
    auto &s = pt.state;
    s.resize(robot.nq(), robot.nv(), robot.nf());
    const auto &bp = step.body_pose;
    s.q.head<7>() << bp.position.x, bp.position.y, bp.position.z,
        bp.orientation.x, bp.orientation.y, bp.orientation.z, bp.orientation.w;
    const double qn = s.q.segment<4>(3).norm();
    if (!(qn > 1e-6)) return;
    s.q.segment<4>(3) /= qn;
    const auto &bv = step.body_velocity;
    s.v.head<6>() << bv.linear.x, bv.linear.y, bv.linear.z, bv.angular.x,
        bv.angular.y, bv.angular.z;
    const auto &ba = step.body_acceleration;
    s.a.head<6>() << ba.linear.x, ba.linear.y, ba.linear.z, ba.angular.x,
        ba.angular.y, ba.angular.z;
    joints = Eigen::Map<const Eigen::VectorXd>(step.joint_positions.data(),
                                               robot.nj());
    robot.jointsToQ(joints, s.q);
    joints = Eigen::Map<const Eigen::VectorXd>(step.joint_velocities.data(),
                                               robot.nj());
    robot.jointsToV(joints, s.v);
    joints = Eigen::Map<const Eigen::VectorXd>(step.joint_accelerations.data(),
                                               robot.nj());
    robot.jointsToV(joints, s.a);
    for (int f = 0; f < robot.nf(); ++f) {
      const auto &fm = step.feet[f];
      auto &fs = s.feet[f];
      fs.pos << fm.position.x, fm.position.y, fm.position.z;
      fs.vel << fm.velocity.x, fm.velocity.y, fm.velocity.z;
      fs.acc << fm.acceleration.x, fm.acceleration.y, fm.acceleration.z;
      fs.force << fm.contact_force.x, fm.contact_force.y, fm.contact_force.z;
      fs.contact = fm.is_contact;
    }
    if (!s.q.allFinite() || !s.v.allFinite() || !s.a.allFinite()) {
      RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
                           "Ignoring trajectory with non-finite values");
      return;
    }
    traj->points.push_back(std::move(pt));
  }
  if (traj->points.empty()) return;
  core_->setTrajectory(traj);
}

// ---------------------------------------------------------------------------
// real-time loop
// ---------------------------------------------------------------------------
bool WBC::readState() {
  const auto &robot = core_->robot();
  auto &q = state_.q;
  for (int i = 0; i < 7; ++i) q[i] = read(base_ifs_[i], i == 6 ? 1.0 : 0.0);
  const double qn = q.segment<4>(3).norm();
  if (!(qn > 1e-6)) return false;
  q.segment<4>(3) /= qn;
  for (int i = 0; i < 6; ++i) state_.v[i] = read(base_ifs_[7 + i], 0.0);

  for (int j = 0; j < robot.nj(); ++j) {
    joint_pos_[j] = read(pos_ifs_[j], 0.0);
    joint_vel_[j] = read(vel_ifs_[j], 0.0);
    joint_eff_[j] = read(eff_ifs_[j], 0.0);
  }
  robot.jointsToQ(joint_pos_, state_.q);
  robot.jointsToV(joint_vel_, state_.v);
  state_.tau_meas = joint_eff_;
  return state_.q.allFinite() && state_.v.allFinite();
}

void WBC::writeCommand(const JointCommand &cmd) {
  const auto &lim = core_->robot().effortLimits();
  for (size_t j = 0; j < cmd_ifs_.size(); ++j) {
    const auto &c = cmd_ifs_[j];
    if (hardware_pd_) {
      write(c.effort, cmd.tau_ff[j]);
      write(c.position, cmd.q_des[j]);
      write(c.velocity, cmd.v_des[j]);
      write(c.kp, cmd.kp[j]);
      write(c.kd, cmd.kd[j]);
    } else {
      // Effort-only drives: run the joint PD here at the control rate.
      const double tau = cmd.tau_ff[j] +
                         cmd.kp[j] * (cmd.q_des[j] - joint_pos_[j]) +
                         cmd.kd[j] * (cmd.v_des[j] - joint_vel_[j]);
      tau_out_[j] = std::clamp(tau, -lim[j], lim[j]);
      write(c.effort, tau_out_[j]);
      write(c.position, cmd.q_des[j]);
      write(c.velocity, cmd.v_des[j]);
    }
  }
}

void WBC::writeDamping() {
  for (size_t j = 0; j < cmd_ifs_.size(); ++j) {
    const auto &c = cmd_ifs_[j];
    const double vel = j < static_cast<size_t>(joint_vel_.size())
                           ? joint_vel_[j]
                           : 0.0;
    if (hardware_pd_) {
      write(c.effort, 0.0);
      write(c.kp, 0.0);
      write(c.kd, params_.safety.damping_kd);
      write(c.velocity, 0.0);
      if (j < static_cast<size_t>(joint_pos_.size()))
        write(c.position, joint_pos_[j]);
    } else {
      write(c.effort, -params_.safety.damping_kd * vel);
    }
  }
}

controller_interface::return_type WBC::update(const rclcpp::Time &time,
                                              const rclcpp::Duration &) {
  if (!core_ready_) return controller_interface::return_type::OK;

  if (!readState()) {
    // Estimator output unusable: damp, never stop commanding the drives.
    writeDamping();
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000, "Invalid state estimate, damping joints");
    return controller_interface::return_type::OK;
  }

  const auto &cmd = core_->update(time.seconds(), state_);
  writeCommand(cmd);
  if (planner_) storeSnapshot(time.seconds());

  // Log state changes (rare) rather than every tick
  auto logger = get_node()->get_logger();
  if (core_->safetyState() != last_safety_) {
    last_safety_ = core_->safetyState();
    if (last_safety_ == SafetySupervisor::State::OK) {
      RCLCPP_WARN(logger, "Safety: back to OK");
    } else {
      RCLCPP_ERROR(logger, "Safety: %s (%s)", toString(last_safety_),
                   core_->safetyReason().c_str());
    }
  }
  if (core_->referenceMode() != last_ref_mode_) {
    last_ref_mode_ = core_->referenceMode();
    RCLCPP_INFO(logger, "Reference: %s", toString(last_ref_mode_));
  }

  // Contact flags feed the state estimator: publish every tick.
  if (core_->robot().nf() == 4) {
    const auto &modes = core_->contactModes();
    wbc_control_msgs::msg::FootContactFlags flags;
    flags.header.stamp = time;
    flags.fl = modes[0] == ContactMode::STANCE;
    flags.fr = modes[1] == ContactMode::STANCE;
    flags.rl = modes[2] == ContactMode::STANCE;
    flags.rr = modes[3] == ContactMode::STANCE;
    contacts_pub_->publish(flags);
  }

  if (params_.publish_decimation > 0 &&
      tick_++ % static_cast<uint64_t>(params_.publish_decimation) == 0) {
    publishDiagnostics(time);
  }
  return controller_interface::return_type::OK;
}

void WBC::publishDiagnostics(const rclcpp::Time &) {
  const auto &robot = core_->robot();
  const auto &out = core_->output();
  const auto &modes = core_->contactModes();

  auto measured = toRobotStateMsg(robot, state_.q, state_.v, out.qdd,
                                  state_.feet, &out.forces);
  for (size_t f = 0; f < measured.feet.size(); ++f) {
    measured.feet[f].is_contact = modes[f] == ContactMode::STANCE;
  }
  state_pub_->publish(measured);

  const auto &t = core_->target();
  target_pub_->publish(toRobotStateMsg(robot, t.q, t.v, t.a, t.feet, nullptr));

}

}  // namespace wbc

PLUGINLIB_EXPORT_CLASS(wbc::WBC, controller_interface::ControllerInterface)
