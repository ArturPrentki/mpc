#pragma once
#include "wbc_control_msgs/msg/foot_contact_flags.hpp"
#include "wbc_control_msgs/msg/mpc_state_step.hpp"
#include "wbc_control_msgs/msg/robot_state.hpp"

#include "whole_body_controller/control_interpolator.hpp"
#include "whole_body_controller/dynamic_system.hpp"
#include "whole_body_controller/optimizer.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>
#include <unordered_map>

#include "std_msgs/msg/string.hpp"
#include "wbc_control_msgs/msg/mpc_trajectory.hpp"
#include "whole_body_controller/system_state.hpp"

#include <realtime_tools/realtime_buffer.hpp>
#include <vector>
#include "whole_body_controller/wbc_parameters.hpp"

namespace wbc {

class WBC : public controller_interface::ControllerInterface {
  enum class BaseIfs : int {
    POS_X = 0,
    POS_Y,
    POS_Z,
    Q_X,
    Q_Y,
    Q_Z,
    Q_W,
    VEL_X,
    VEL_Y,
    VEL_Z,
    RVEL_X,
    RVEL_Y,
    RVEL_Z,
  };
  enum class ActIfs : int {
    POS = 0,
    VEL,
    TRQ,
    SKP,
    SKD,
  };

  enum class Actuator : int { HIP = 0, THIGH, CALF };
  enum class Legs : int { LF = 0, RF, LR, RR };
  using map_actuators_names = std::unordered_map<ActIfs, std::string>;
  using map_leg_names = std::unordered_map<Actuator, map_actuators_names>;
  using map_legs_names = std::unordered_map<Legs, map_leg_names>;

  using map_actuators_state =
      std::unordered_map<ActIfs, hardware_interface::LoanedStateInterface *>;
  using map_leg_state = std::unordered_map<Actuator, map_actuators_state>;
  using map_legs_states = std::unordered_map<Legs, map_leg_state>;

  using map_actuators_command =
      std::unordered_map<ActIfs, hardware_interface::LoanedCommandInterface *>;
  using map_leg_command = std::unordered_map<Actuator, map_actuators_command>;
  using map_legs_commands = std::unordered_map<Legs, map_leg_command>;

public:
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  bool urdf_received_ = false;

  rclcpp::Subscription<wbc_control_msgs::msg::MpcTrajectory>::SharedPtr
      mpc_trj_sub;
  void trajectory_update(const wbc_control_msgs::msg::MpcTrajectory &msg);

  rclcpp::Publisher<wbc_control_msgs::msg::FootContactFlags>::SharedPtr fc_pub;
  rclcpp::Publisher<wbc_control_msgs::msg::RobotState>::SharedPtr state_pub;
  rclcpp::Publisher<wbc_control_msgs::msg::RobotState>::SharedPtr target_state_pub;

  std::unordered_map<BaseIfs, std::string> base_interfaces_names;
  std::unordered_map<BaseIfs, hardware_interface::LoanedStateInterface *>
      base_interfaces_map;

  map_legs_names leg_map_names_;
  map_legs_states leg_state_interfaces_map;
  map_legs_commands leg_command_interfaces_map;

  std::unique_ptr<DynamicSystem> dynamic_system_;
  std::unique_ptr<Optimizer> optimizer_;
  std::unique_ptr<ControlInterpolator> interpolator_;

  SystemState system_state;

  void publish_robot_state(const ControlOutput &optimized);
  void publish_target_state(const TargetState & target);

  bool __first_update;

  void update_system_state();
  void send_system_state(const TargetState &target,
                         const ControlOutput &output);
  template <int size>
  Eigen::Vector<double, size> as_e_vector(std::vector<double>&parameter) {
    assert(parameter.size() == size);
    return Eigen::Map<Eigen::Vector<double, size>>(parameter.data(), parameter.size());
  }

  std::shared_ptr<whole_body_controller::ParamListener> param_listener_;
  whole_body_controller::Params params_;
};
} // namespace wbc