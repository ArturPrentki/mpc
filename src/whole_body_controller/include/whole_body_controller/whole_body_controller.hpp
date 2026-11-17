#pragma once
// ros2_control wrapper around the ROS-independent WbcCore.
//
// Everything robot-specific comes from the URDF and the parameters
// (config/whole_body_params.yaml documents all of them); see
// go2_description/config/go2_controllers.yaml and
// gardo_description/config/gardo_controllers.yaml for complete examples.

#include "whole_body_controller/mpc/locomotion_planner.hpp"
#include "whole_body_controller/wbc_core.hpp"
#include "whole_body_controller/wbc_parameters.hpp"

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include "wbc_control_msgs/msg/foot_contact_flags.hpp"
#include "wbc_control_msgs/msg/mpc_trajectory.hpp"
#include "wbc_control_msgs/msg/robot_state.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace wbc {

class WBC : public controller_interface::ControllerInterface {
public:
  ~WBC() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

private:
  struct JointCommandInterfaces {
    hardware_interface::LoanedCommandInterface *effort = nullptr;
    hardware_interface::LoanedCommandInterface *position = nullptr;
    hardware_interface::LoanedCommandInterface *velocity = nullptr;
    hardware_interface::LoanedCommandInterface *kp = nullptr;
    hardware_interface::LoanedCommandInterface *kd = nullptr;
  };

  CoreConfig makeCoreConfig() const;
  LocomotionConfig makeLocomotionConfig() const;
  void startPlanner();
  void stopPlanner();
  void plannerLoop();
  void storeSnapshot(double time);
  bool buildCore(const std::string &urdf);
  void onTrajectory(const wbc_control_msgs::msg::MpcTrajectory &msg);
  bool readState();
  void writeCommand(const JointCommand &cmd);
  void writeDamping();
  void publishDiagnostics(const rclcpp::Time &time);
  bool hasCommandInterface(const std::string &name) const;

  std::shared_ptr<whole_body_controller::ParamListener> param_listener_;
  whole_body_controller::Params params_;

  std::mutex core_mutex_;  // guards construction only
  std::shared_ptr<WbcCore> core_;
  std::atomic<bool> core_ready_{false};

  SystemState state_;
  Eigen::VectorXd joint_pos_, joint_vel_, joint_eff_, tau_out_;
  std::vector<hardware_interface::LoanedStateInterface *> base_ifs_;
  std::vector<hardware_interface::LoanedStateInterface *> pos_ifs_, vel_ifs_,
      eff_ifs_;
  std::vector<JointCommandInterfaces> cmd_ifs_;
  bool hardware_pd_ = false;  // drives run the joint PD themselves

  // Built-in locomotion planner (MPC), running in planner_thread_
  std::unique_ptr<LocomotionPlanner> planner_;
  std::thread planner_thread_;
  std::atomic<bool> planner_running_{false};
  std::mutex snapshot_mutex_;
  PlannerInput snapshot_;
  bool snapshot_fresh_ = false;
  std::mutex command_mutex_;
  VelocityCommand command_;
  std::chrono::steady_clock::time_point command_stamp_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<wbc_control_msgs::msg::MpcTrajectory>::SharedPtr
      trajectory_sub_;
  rclcpp::Publisher<wbc_control_msgs::msg::FootContactFlags>::SharedPtr
      contacts_pub_;
  rclcpp::Publisher<wbc_control_msgs::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::Publisher<wbc_control_msgs::msg::RobotState>::SharedPtr target_pub_;

  uint64_t tick_ = 0;
  SafetySupervisor::State last_safety_ = SafetySupervisor::State::OK;
  TrajectoryInterpolator::Mode last_ref_mode_ =
      TrajectoryInterpolator::Mode::EXPIRED;
};

}  // namespace wbc
