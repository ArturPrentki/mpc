#include "whole_body_controller/whole_body_controller.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "wbc_control_msgs/msg/foot_contact_flags.hpp"
#include "wbc_control_msgs/msg/foot_state.hpp"
#include "wbc_control_msgs/msg/mpc_state_step.hpp"
#include "wbc_control_msgs/msg/mpc_trajectory.hpp"
#include "wbc_control_msgs/msg/robot_state.hpp"
#include "whole_body_controller/control_interpolator.hpp"
#include "whole_body_controller/optimizer.hpp"

#include <Eigen/src/Core/Map.h>
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
#include <cmath>
#include <controller_interface/controller_interface_base.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "whole_body_controller/dynamic_system.hpp"
#include "whole_body_controller/system_state.hpp"

#include <cstddef>
#include <hardware_interface/loaned_command_interface.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <string>
#include <tf2_ros/buffer_interface.hpp>
#include <unordered_map>
#include <vector>

namespace wbc {

#define DEFAULT_J 1e-3, 1e-3, 1e-3
#define DEFAULT_F 1e-2, 1e-2, 1e-2
controller_interface::CallbackReturn WBC::on_init() {
  try {
    param_listener_ =
        std::make_shared<whole_body_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WBC::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

  auto base_weights = as_e_vector<6>(params_.base_weights);
  auto joint_weights = as_e_vector<12>(params_.joint_weights);
  auto force_weights = as_e_vector<12>(params_.force_weights);
  auto foot_weights = as_e_vector<3>(params_.foot_weights);

  // base_state_int_ = params_.base_state_int;

  optimizer_ = std::make_unique<Optimizer>(
      base_weights, joint_weights, force_weights, foot_weights,
      params_.kp_base_lin, params_.kd_base_lin, params_.kp_base_ang,
      params_.kd_base_ang, params_.kp_foot, params_.kd_foot, params_.max_fz,
      params_.mu);

  if (!optimizer_->initialize()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Optimizer wasnt initialized correctly!");
    return controller_interface::CallbackReturn::FAILURE;
  }
  if (params_.joint_suffixes.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'joint_suffixes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.legs_prefixes.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "'legs_prefixes' parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  /////////////////////////////////////////////////////////////////////////////////////
  mpc_trj_sub =
      get_node()->create_subscription<wbc_control_msgs::msg::MpcTrajectory>(
          "/mpc_trajectory", rclcpp::QoS(1),
          [this](const wbc_control_msgs::msg::MpcTrajectory msg) {
            trajectory_update(msg);
          });
  fc_pub =
      get_node()->create_publisher<wbc_control_msgs::msg::FootContactFlags>(
          "/foot_contacts", rclcpp::QoS(1));

  state_pub = get_node()->create_publisher<wbc_control_msgs::msg::RobotState>(
      "/robot_state", rclcpp::QoS(1));

  target_state_pub = get_node()->create_publisher<wbc_control_msgs::msg::RobotState>(
      "/target_state", rclcpp::QoS(1));
  ////////////////////////////////////////////////////////////////////////////////////

  urdf_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (urdf_received_) {
          return;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Got URDF!");
        std::string base_name = "base";
        std::string urdf = msg->data;

        std::vector<std::string> hip_names;
        std::vector<std::string> thigh_names;
        std::vector<std::string> calf_names;
        std::vector<std::string> feet_names;

        std::vector<std::string> joints_names;

        for (const auto &leg : params_.legs_prefixes) {
          hip_names.push_back(leg + params_.joint_suffixes[0]);
          thigh_names.push_back(leg + params_.joint_suffixes[1]);
          calf_names.push_back(leg + params_.joint_suffixes[2]);
          feet_names.push_back(leg + params_.foot_suffix);
          RCLCPP_INFO_STREAM(get_node()->get_logger(),
                             "\n" << hip_names.back() << "\n"
                                  << thigh_names.back() << "\n"
                                  << calf_names.back() << "\n"
                                  << feet_names.back() << "\n");
          for (const auto &joint : params_.joint_suffixes) {
            joints_names.push_back(leg + joint);
          }
        }
        pinocchio::Model model;
        pinocchio::urdf::buildModelFromXML(
            urdf, pinocchio::JointModelFreeFlyer(), model);
        if (!DynamicSystem::validate_model(model, base_name, hip_names,
                                           thigh_names, calf_names, feet_names,
                                           joints_names)) {
          RCLCPP_ERROR(get_node()->get_logger(), "Model is bad!");
          return;
        }
        dynamic_system_ = std::make_unique<DynamicSystem>(
            model, base_name, hip_names, thigh_names, calf_names, feet_names,
            joints_names);

        interpolator_ = std::make_unique<ControlInterpolator>(model);
        urdf_received_ = true;
      });
  system_state.foot_jac_lin_lwa.setZero();
  system_state.base_jacobian_lwa.setZero();
  system_state.j_torq.setZero();
  system_state.hu.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WBC::command_interface_configuration() const {

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto &leg : params_.legs_prefixes) {
    for (const auto &joint : params_.joint_suffixes) {
      const std::string j = leg + joint + "/";
      config.names.push_back(j + hardware_interface::HW_IF_EFFORT);
      config.names.push_back(j + hardware_interface::HW_IF_POSITION);
      config.names.push_back(j + hardware_interface::HW_IF_VELOCITY);
      config.names.push_back(j + hardware_interface::HW_IF_PROPORTIONAL_GAIN);
      config.names.push_back(j + hardware_interface::HW_IF_DERIVATIVE_GAIN);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
WBC::state_interface_configuration() const {

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // claim leg state interfaces
  for (const auto &leg : params_.legs_prefixes) {
    for (const auto &joint : params_.joint_suffixes) {
      const std::string j = leg + joint + "/";
      config.names.push_back(j + hardware_interface::HW_IF_EFFORT);
      config.names.push_back(j + hardware_interface::HW_IF_POSITION);
      config.names.push_back(j + hardware_interface::HW_IF_VELOCITY);
    }
  }
  // Claim Floating Base State from the Estimator
  for (const auto &var : params_.base_state_int) {
    config.names.push_back("state_estimator/" + var);
  }
  return config;
}

controller_interface::CallbackReturn
WBC::on_deactivate(const rclcpp_lifecycle::State &) {
  TargetState ts;
  ts.q_ddot.setZero();
  ts.q_dot.setZero();
  ts.q.setZero();
  ts.j_kp.setZero();
  ts.j_kd.setOnes();
  ts.j_kd *= 20;
  ControlOutput op;
  op.q_ddot.setZero();
  op.torque.setZero();
  op.contact_forces.setZero();
  send_system_state(ts, op);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
WBC::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  auto start_time = get_node()->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);

  while (!urdf_received_ && (get_node()->now() - start_time) < timeout) {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                         1000, "Waiting for URDF on /robot_description...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  if (!urdf_received_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Timed out waiting for URDF on /robot_description.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  /////////////////////////////////////////////////////////////////////////
  // store state interfaces in maps
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface *>
      st_map;
  std::unordered_map<std::string, hardware_interface::LoanedCommandInterface *>
      cm_map;

  for (auto &interface : state_interfaces_) {
    st_map[interface.get_name()] = &interface;
  }
  for (auto &interface : command_interfaces_) {
    cm_map[interface.get_name()] = &interface;
  }
  for (size_t i = 0; i < params_.base_state_int.size(); ++i) {
    BaseIfs bi = static_cast<BaseIfs>(i);
    base_interfaces_names[bi] = "state_estimator/" + params_.base_state_int[i];
    base_interfaces_map[bi] = st_map[base_interfaces_names[bi]];
  }

  std::vector<std::string> actuator_interfaces = {
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT,
      hardware_interface::HW_IF_PROPORTIONAL_GAIN,
      hardware_interface::HW_IF_DERIVATIVE_GAIN,

  };

  // construct what the names are supposed to be
  for (size_t i = 0; i < params_.legs_prefixes.size(); ++i) {
    for (size_t j = 0; j < 3; ++j) {   // 3 actuators per leg
      for (size_t k = 0; k < 5; ++k) { // 5 interfaces per actuator
        // 3 state interfaces per act
        Legs l = static_cast<Legs>(i);
        Actuator a = static_cast<Actuator>(j);
        ActIfs ai = static_cast<ActIfs>(k);
        std::string name = params_.legs_prefixes[i] +
                           params_.joint_suffixes[j] + "/" +
                           actuator_interfaces[k];
        leg_map_names_[l][a][ai] = name;
        leg_state_interfaces_map[l][a][ai] = st_map[name];
        leg_command_interfaces_map[l][a][ai] = cm_map[name];
      }
    }
  }

  system_state.q.setZero();
  system_state.q[6] = 1.;
  system_state.q_dot.setZero();

  __first_update = true;
  return controller_interface::CallbackReturn::SUCCESS;
}
// callback
void WBC::trajectory_update(const wbc_control_msgs::msg::MpcTrajectory &msg) {
  std::vector<TargetState> sys;
  rclcpp::Time msg_time(msg.header.stamp);
  rclcpp::Time now = get_node()->get_clock()->now();
  // Calculate age directly
  rclcpp::Duration age = now - msg_time;

  RCLCPP_INFO(get_node()->get_logger(), "Message age: %.3f ms",
              age.seconds() * 1000.0);

  for (auto &target : msg.steps) {
    TargetState ts;
    // these are in world frame
    ts.q.topRows(7) << target.body_pose.position.x, target.body_pose.position.y,
        target.body_pose.position.z, target.body_pose.orientation.x,
        target.body_pose.orientation.y, target.body_pose.orientation.z,
        target.body_pose.orientation.w;
    //
    ts.q.tail<12>() = Eigen::Map<const Eigen::Vector<double, 12>>(
        target.joint_positions.data(), 12);

    ts.q_dot.topRows(6) << target.body_velocity.linear.x,
        target.body_velocity.linear.y, target.body_velocity.linear.z,
        target.body_velocity.angular.x, target.body_velocity.angular.y,
        target.body_velocity.angular.z;
    ts.q_dot.tail<12>() = Eigen::Map<const Eigen::Vector<double, 12>>(
        target.joint_velocities.data(), 12);

    ts.q_ddot.topRows(6) << target.body_acceleration.linear.x,
        target.body_acceleration.linear.y, target.body_acceleration.linear.z,
        target.body_acceleration.angular.x, target.body_acceleration.angular.y,
        target.body_acceleration.angular.z;
    ts.q_ddot.tail<12>() = Eigen::Map<const Eigen::Vector<double, 12>>(
        target.joint_accelerations.data(), 12);
    // feet are in odom frame too
    int id = 0;

    for (auto &foot : target.feet) {
      ts.foot_state[id].foot_pos_world << foot.position.x, foot.position.y,
          foot.position.z;

      ts.foot_state[id].foot_vel_lwa << foot.velocity.x, foot.velocity.y,
          foot.velocity.z;

      ts.foot_state[id].foot_acc_lwa << foot.acceleration.x,
          foot.acceleration.y, foot.acceleration.z;

      ts.contact_forces_odom.middleRows(3 * id, 3) << foot.contact_force.x,
          foot.contact_force.y, foot.contact_force.z;

      ts.foot_state[id].contact = foot.is_contact;

      // if (foot.is_contact) {
      //   // STANCE: Foot is pinned by hard constraints. Base task handles
      //   // kinematics. Zeroing Kp prevents the joints from fighting the base.
      //   ts.j_kp.segment<3>(3 * id).setZero();
      //   ts.j_kd.segment<3>(3 * id).setConstant(
      //       1.0); // Tiny damping to regularize null-space
      // } else {
      //   // SWING: Safe to track MPC joint targets to keep the leg looking
      //   // natural
      // }
      ts.j_kp.segment<3>(3 * id).setConstant(100.0);
      ts.j_kd.segment<3>(3 * id).setConstant(25.0);

      ++id;
    };

    // ts.j_kp.setOnes();
    // ts.j_kd.setOnes();
    // ts.j_kp *= 100.;
    // ts.j_kd *= 10.0;

    // dynamic_system_->inverse_kinematics(ts);

    sys.push_back(ts);
  }

  interpolator_->set_target_from_non_rt(sys, msg.header.stamp);
}

void WBC::update_system_state() {
  // this is WORLD frame
  system_state.q.topRows(7)
      << base_interfaces_map.at(BaseIfs::POS_X)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::POS_Y)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::POS_Z)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::Q_X)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::Q_Y)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::Q_Z)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::Q_W)->get_optional().value_or(1.);

  // this is LOCAL velocity!
  system_state.q_dot.topRows(6)
      << base_interfaces_map.at(BaseIfs::VEL_X)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::VEL_Y)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::VEL_Z)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::RVEL_X)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::RVEL_Y)->get_optional().value_or(0.),
      base_interfaces_map.at(BaseIfs::RVEL_Z)->get_optional().value_or(0.);

  int num_of_legs = leg_state_interfaces_map.size();
  int num_of_act = leg_state_interfaces_map.at(Legs::LF).size();

  for (int i = 0; i < num_of_legs; ++i)
    for (int j = 0; j < num_of_act; ++j) {
      auto &actuator = leg_state_interfaces_map.at(static_cast<Legs>(i))
                           .at(static_cast<Actuator>(j));
      system_state.q[7 + i * num_of_act + j] =
          actuator.at(ActIfs::POS)->get_optional().value_or(0.);

      system_state.q_dot[6 + i * num_of_act + j] =
          actuator.at(ActIfs::VEL)->get_optional().value_or(0.);

      system_state.j_torq[i * num_of_act + j] =
          actuator.at(ActIfs::TRQ)->get_optional().value_or(0.);
    }
  dynamic_system_->update_dynamics(system_state);
}

void WBC::send_system_state(const TargetState &target,
                            const ControlOutput &output) {
  int num_of_legs = leg_command_interfaces_map.size();
  int n_o_a = leg_command_interfaces_map.at(Legs::LF).size();
  auto q = target.q.tail<12>();
  auto q_dot = target.q_dot.tail<12>();

  for (int i = 0; i < num_of_legs; ++i)
    for (int j = 0; j < n_o_a; ++j) {
      auto &coms = leg_command_interfaces_map.at(static_cast<Legs>(i))
                       .at(static_cast<Actuator>(j));

      (void)coms.at(ActIfs::POS)->set_value(q[i * n_o_a + j]);
      (void)coms.at(ActIfs::VEL)->set_value(q_dot[i * n_o_a + j]);
      (void)coms.at(ActIfs::TRQ)->set_value(output.torque[i * n_o_a + j]);
      (void)coms.at(ActIfs::SKP)->set_value(target.j_kp[i * n_o_a + j]);
      (void)coms.at(ActIfs::SKD)->set_value(target.j_kd[i * n_o_a + j]);
    }
}

void WBC::publish_target_state(const TargetState &target) {
  wbc_control_msgs::msg::RobotState msg;
  Eigen::Map<Eigen::Vector3d>(&msg.body_pose.position.x) = target.q.head<3>();
  Eigen::Map<Eigen::Vector4d>(&msg.body_pose.orientation.x) =
      target.q.segment<4>(3);

  Eigen::Map<Eigen::Vector3d>(&msg.body_velocity.linear.x) =
      target.q_dot.head<3>();
  Eigen::Map<Eigen::Vector3d>(&msg.body_velocity.angular.x) =
      target.q_dot.segment<3>(3);

  // Acceleration
  Eigen::Map<Eigen::Vector3d>(&msg.body_acceleration.linear.x) =
      target.q_ddot.head<3>();
  Eigen::Map<Eigen::Vector3d>(&msg.body_acceleration.angular.x) =
      target.q_ddot.segment<3>(3);

  const double *tail;
  tail = target.q.tail<12>().data();
  msg.joint_positions.assign(tail, tail + 12);

  tail = target.q_dot.tail<12>().data();
  msg.joint_velocities.assign(tail, tail + 12);

  tail = target.q_ddot.tail<12>().data();
  msg.joint_accelerations.assign(tail, tail + 12);

  for (int i = 0; i < 4; ++i) {
    wbc_control_msgs::msg::FootState fs;
    Eigen::Map<Eigen::Vector3d>(&fs.position.x) =
        target.foot_state[i].foot_pos_world;
    Eigen::Map<Eigen::Vector3d>(&fs.velocity.x) =
        target.foot_state[i].foot_vel_lwa;
    Eigen::Map<Eigen::Vector3d>(&fs.acceleration.x) = target.foot_state[i].foot_acc_lwa;

    Eigen::Map<Eigen::Vector3d>(&fs.contact_force.x) =
        target.contact_forces_odom.segment<3>(3 * i);

    fs.is_contact = target.foot_state[i].contact;
    msg.feet.push_back(fs);
  }
  target_state_pub->publish(msg);
}

void WBC::publish_robot_state(const ControlOutput &optimized) {
  wbc_control_msgs::msg::RobotState msg;
  Eigen::Map<Eigen::Vector3d>(&msg.body_pose.position.x) =
      system_state.q.head<3>();
  Eigen::Map<Eigen::Vector4d>(&msg.body_pose.orientation.x) =
      system_state.q.segment<4>(3);

  Eigen::Map<Eigen::Vector3d>(&msg.body_velocity.linear.x) =
      system_state.q_dot.head<3>();
  Eigen::Map<Eigen::Vector3d>(&msg.body_velocity.angular.x) =
      system_state.q_dot.segment<3>(3);

  // Acceleration
  Eigen::Map<Eigen::Vector3d>(&msg.body_acceleration.linear.x) =
      optimized.q_ddot.head<3>();
  Eigen::Map<Eigen::Vector3d>(&msg.body_acceleration.angular.x) =
      optimized.q_ddot.segment<3>(3);

  const double *tail;
  tail = system_state.q.tail<12>().data();
  msg.joint_positions.assign(tail, tail + 12);

  tail = system_state.q_dot.tail<12>().data();
  msg.joint_velocities.assign(tail, tail + 12);

  tail = optimized.q_ddot.tail<12>().data();
  msg.joint_accelerations.assign(tail, tail + 12);

  for (int i = 0; i < 4; ++i) {
    wbc_control_msgs::msg::FootState fs;
    Eigen::Map<Eigen::Vector3d>(&fs.position.x) = 
        system_state.foot_state[i].foot_pos_world;
    Eigen::Map<Eigen::Vector3d>(&fs.velocity.x) =
        system_state.foot_state[i].foot_vel_lwa;
    Eigen::Map<Eigen::Vector3d>(&fs.acceleration.x) = Eigen::Vector3d::Zero();
    Eigen::Map<Eigen::Vector3d>(&fs.contact_force.x) =
        optimized.contact_forces.segment<3>(3 * i);
    fs.is_contact = system_state.foot_state[i].contact;
    msg.feet.push_back(fs);
  }

  state_pub->publish(msg);
}
controller_interface::return_type
WBC::update(const rclcpp::Time &time, const rclcpp::Duration & /*period*/) {

  if (!urdf_received_) {
    return controller_interface::return_type::OK;
  }
  update_system_state();
  if (__first_update) {
    // clang-format off
    InterpolatrorConfig config = {
      40,
         40*30, 
   100,
      200
    };
    // clang-format on
    double fz_target = dynamic_system_->get_total_mass() * 9.81 / 4.;
    TargetState default_target;
    default_target.contact_forces_odom.setZero();
    default_target.contact_forces_odom << 0.0, 0.0, fz_target, 0.0, 0.0,
        fz_target, 0.0, 0.0, fz_target, 0.0, 0.0, fz_target;
    // default_target.fs = system_state.fs;
    default_target.q.setZero();
    default_target.q[2] = 0.3;
    default_target.q[6] = 1.0;
    default_target.q_dot.setZero();
    default_target.q_ddot.setZero();

    for (int i = 0; i < 4; ++i) {
      system_state.foot_state[i].contact = true;
      default_target.foot_state[i].contact = true;
      default_target.foot_state[i].foot_vel_lwa.setZero();
    }
    default_target.foot_state[0].foot_pos_world << 0.19, 0.142, 0.0;
    default_target.foot_state[1].foot_pos_world << 0.19, -0.142, 0.0;
    default_target.foot_state[2].foot_pos_world << -0.19, 0.142, 0.0;
    default_target.foot_state[3].foot_pos_world << -0.19, -0.142, 0.0;

    default_target.j_kp.setIdentity();
    default_target.j_kd.setIdentity();
    default_target.j_kp *= 30.;
    default_target.j_kd *= 1.4;

    dynamic_system_->inverse_kinematics(default_target);
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "DEFAULT TARGET:\n"
                                                     << default_target);
    interpolator_->init(config, default_target);
    __first_update = false;
  }
  const auto &target = interpolator_->get_next_target(time);
  publish_target_state(target);
  // update foot contacts for state estimation:
  wbc_control_msgs::msg::FootContactFlags msg;
  msg.fl = system_state.foot_state[0].contact;
  msg.fr = system_state.foot_state[1].contact;
  msg.rl = system_state.foot_state[2].contact;
  msg.rr = system_state.foot_state[3].contact;
  fc_pub->publish(msg);

  ControlOutput output;
  if (!optimizer_->solve(system_state, target, output)) {
    RCLCPP_FATAL_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          100, "Solver failed!");
    return controller_interface::return_type::ERROR;
  }
  dynamic_system_->inverse_dynamics(system_state, output);
  send_system_state(target, output);
  publish_robot_state(output);

  // RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
  //                             *get_node()->get_clock(), 100,
  //                             "\n" << system_state << "\n -- -- -- --- \n"
  //                                  << target << "\n -- -- -- --- \n"
  //                                  << output);
  return controller_interface::return_type::OK;
}

} // namespace wbc
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wbc::WBC, controller_interface::ControllerInterface)