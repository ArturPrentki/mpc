#include "hardware_unitree_sdk2/hardware_unitree_sdk2.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#include <chrono>
#include <cstdint>
#include <thread>

static uint32_t crc32_core(const uint32_t *ptr, uint32_t len) {
  unsigned int xbit = 0;
  unsigned int data = 0;
  unsigned int CRC32 = 0xFFFFFFFF;
  const unsigned int dwPolynomial = 0x04c11db7;

  for (unsigned int i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (unsigned int bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else {
        CRC32 <<= 1;
      }

      if (data & xbit)
        CRC32 ^= dwPolynomial;
      xbit >>= 1;
    }
  }

  return CRC32;
}
using namespace unitree::robot;
using hardware_interface::return_type;

bool HardwareUnitree::releaseSportMode() {
  // Same sequence as Unitree's SDK examples: while a motion control service
  // (sport mode, AI mode, ...) is active, ask the motion switcher to release
  // it.
  unitree::robot::b2::MotionSwitcherClient switcher;
  switcher.SetTimeout(5.0f);
  switcher.Init();

  constexpr int kMaxAttempts = 5;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    std::string form, name;
    const int32_t ret = switcher.CheckMode(form, name);
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(),
                   "Cannot reach the motion switcher service (error %d). Is "
                   "the robot connected on '%s', domain %d?",
                   ret, network_interface_.c_str(), domain_);
      return false;
    }
    if (name.empty()) {
      RCLCPP_INFO(get_logger(), "Sport mode is off, low-level control enabled");
      return true;
    }
    RCLCPP_WARN(get_logger(), "Motion service '%s' is active, releasing it "
                              "(attempt %d/%d)",
                name.c_str(), attempt, kMaxAttempts);
    if (switcher.ReleaseMode() != 0) {
      RCLCPP_WARN(get_logger(), "ReleaseMode failed, retrying");
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  RCLCPP_ERROR(get_logger(), "Could not switch off sport mode; refusing to "
                             "send low-level commands");
  return false;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HardwareUnitree::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joint_torque_command_.assign(12, 0);
  joint_position_command_.assign(12, 0);
  joint_velocities_command_.assign(12, 0);
  joint_kp_command_.assign(12, 0);
  joint_kd_command_.assign(12, 0);

  joint_position_.assign(12, 0);
  joint_velocities_.assign(12, 0);
  joint_effort_.assign(12, 0);

  imu_states_.assign(10, 0);
  foot_force_.assign(4, 0);
  high_states_.assign(6, 0);

  for (const auto &joint : info_.joints) {
    for (const auto &interface : joint.state_interfaces) {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  if (const auto network_interface_param =
          info.hardware_info.hardware_parameters.find("network_interface");
      network_interface_param != info.hardware_info.hardware_parameters.end()) {
    network_interface_ = network_interface_param->second;
  }
  if (const auto domain_param =
          info.hardware_info.hardware_parameters.find("domain");
      domain_param != info.hardware_info.hardware_parameters.end()) {
    domain_ = std::stoi(domain_param->second);
  }

  if (const auto show_foot_force_param =
          info.hardware_info.hardware_parameters.find("show_foot_force");
      show_foot_force_param != info.hardware_info.hardware_parameters.end()) {
    show_foot_force_ = show_foot_force_param->second == "true";
  }

  if (const auto release_param =
          info.hardware_info.hardware_parameters.find("release_sport_mode");
      release_param != info.hardware_info.hardware_parameters.end()) {
    release_sport_mode_ = release_param->second == "true";
  }

  RCLCPP_INFO(get_logger(),
              " network_interface: %s, domain: %d, release_sport_mode: %s",
              network_interface_.c_str(), domain_,
              release_sport_mode_ ? "true" : "false");
  ChannelFactory::Instance()->Init(domain_, network_interface_);

  // Must happen before the first LowCmd is published.
  if (release_sport_mode_ && !releaseSportMode()) {
    return CallbackReturn::ERROR;
  }

  low_cmd_publisher_ =
      std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
          TOPIC_LOWCMD);
  low_cmd_publisher_->InitChannel();

  lows_tate_subscriber_ =
      std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
          TOPIC_LOWSTATE);
  lows_tate_subscriber_->InitChannel(
      [this](auto &&PH1) {
        lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
      },
      1);
  initLowCmd();

  high_state_subscriber_ = std::make_shared<
      ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
      TOPIC_HIGHSTATE);
  high_state_subscriber_->InitChannel(
      [this](auto &&PH1) {
        highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
      },
      1);

  return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface>
HardwareUnitree::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    state_interfaces.emplace_back(joint_name, "position",
                                  &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    state_interfaces.emplace_back(joint_name, "velocity",
                                  &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["effort"]) {
    state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
  }

  // export imu sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(info_.sensors[0].name,
                                  info_.sensors[0].state_interfaces[i].name,
                                  &imu_states_[i]);
  }

  // export foot force sensor state interface
  if (info_.sensors.size() > 1) {
    for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(info_.sensors[1].name,
                                    info_.sensors[1].state_interfaces[i].name,
                                    &foot_force_[i]);
    }
  }

  // export odometer state interface
  if (info_.sensors.size() > 2) {
    // export high state interface
    for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(info_.sensors[2].name,
                                    info_.sensors[2].state_interfaces[i].name,
                                    &high_states_[i]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
HardwareUnitree::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto &joint_name : joint_interfaces["position"]) {
    command_interfaces.emplace_back(joint_name,
                                    hardware_interface::HW_IF_POSITION,
                                    &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["velocity"]) {
    command_interfaces.emplace_back(joint_name,
                                    hardware_interface::HW_IF_VELOCITY,
                                    &joint_velocities_command_[ind++]);
  }

  ind = 0;
  for (const auto &joint_name : joint_interfaces["effort"]) {
    command_interfaces.emplace_back(joint_name,
                                    hardware_interface::HW_IF_EFFORT,
                                    &joint_torque_command_[ind]);
    command_interfaces.emplace_back(joint_name,
                                    hardware_interface::HW_IF_PROPORTIONAL_GAIN,
                                    &joint_kp_command_[ind]);
    command_interfaces.emplace_back(joint_name,
                                    hardware_interface::HW_IF_DERIVATIVE_GAIN,
                                    &joint_kd_command_[ind]);
    ind++;
  }
  return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
  // joint states
  int unitree_indexes[12] = {
      3, 4,  5,  // FL
      0, 1,  2,  // FR
      9, 10, 11, // RL
      6, 7,  8   // RR
  };
  for (int i(0); i < 12; ++i) {
    joint_position_[i] = low_state_.motor_state()[unitree_indexes[i]].q();
    joint_velocities_[i] = low_state_.motor_state()[unitree_indexes[i]].dq();
    joint_effort_[i] = low_state_.motor_state()[unitree_indexes[i]].tau_est();
  }

  // imu states
  imu_states_[0] = low_state_.imu_state().quaternion()[0]; // w
  imu_states_[1] = low_state_.imu_state().quaternion()[1]; // x
  imu_states_[2] = low_state_.imu_state().quaternion()[2]; // y
  imu_states_[3] = low_state_.imu_state().quaternion()[3]; // z

  imu_states_[4] = low_state_.imu_state().gyroscope()[0];
  imu_states_[5] = low_state_.imu_state().gyroscope()[1];
  imu_states_[6] = low_state_.imu_state().gyroscope()[2];

  imu_states_[7] = low_state_.imu_state().accelerometer()[0];
  imu_states_[8] = low_state_.imu_state().accelerometer()[1];
  imu_states_[9] = low_state_.imu_state().accelerometer()[2];

  // contact states
  foot_force_[0] = low_state_.foot_force()[1];
  foot_force_[1] = low_state_.foot_force()[0];
  foot_force_[2] = low_state_.foot_force()[3];
  foot_force_[3] = low_state_.foot_force()[2];

  if (show_foot_force_) {
    RCLCPP_INFO(get_logger(), "foot_force(): %f, %f, %f, %f", foot_force_[0],
                foot_force_[1], foot_force_[2], foot_force_[3]);
  }

  // high states
  high_states_[0] = high_state_.position()[0];
  high_states_[1] = high_state_.position()[1];
  high_states_[2] = high_state_.position()[2];
  high_states_[3] = high_state_.velocity()[0];
  high_states_[4] = high_state_.velocity()[1];
  high_states_[5] = high_state_.velocity()[2];

  return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {
  // send command
  int unitree_indexes[12] = {
      3, 4,  5,  // FL
      0, 1,  2,  // FR
      9, 10, 11, // RL
      6, 7,  8   // RR
  };
  for (int i(0); i < 12; ++i) {
    auto *cmd = &low_cmd_.motor_cmd()[unitree_indexes[i]];
    cmd->mode() = 0x01;
    cmd->q() = static_cast<float>(joint_position_command_[i]);
    cmd->dq() = static_cast<float>(joint_velocities_command_[i]);
    cmd->kp() = static_cast<float>(joint_kp_command_[i]);
    cmd->kd() = static_cast<float>(joint_kd_command_[i]);
    cmd->tau() = static_cast<float>(joint_torque_command_[i]);
  }

  low_cmd_.crc() =
      crc32_core(reinterpret_cast<uint32_t *>(&low_cmd_),
                 (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
  low_cmd_publisher_->Write(low_cmd_);
  return return_type::OK;
}

void HardwareUnitree::initLowCmd() {
  low_cmd_.head()[0] = 0xFE;
  low_cmd_.head()[1] = 0xEF;
  low_cmd_.level_flag() = 0xFF;
  low_cmd_.gpio() = 0;

  for (int i = 0; i < 20; i++) {
    low_cmd_.motor_cmd()[i].mode() = 0x01; // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd()[i].q() = 0;
    low_cmd_.motor_cmd()[i].kp() = 0;
    low_cmd_.motor_cmd()[i].dq() = 0;
    low_cmd_.motor_cmd()[i].kd() = 0;
    low_cmd_.motor_cmd()[i].tau() = 0;
  }
}

void HardwareUnitree::lowStateMessageHandle(const void *messages) {
  low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_ *>(messages);
}

void HardwareUnitree::highStateMessageHandle(const void *messages) {
  high_state_ =
      *static_cast<const unitree_go::msg::dds_::SportModeState_ *>(messages);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(HardwareUnitree, hardware_interface::SystemInterface)