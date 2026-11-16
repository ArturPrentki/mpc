#pragma once

#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "wbc_control_msgs/msg/foot_contact_flags.hpp"
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
// Include the InEKF library headers
#include "InEKF.h"

#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
namespace quadruped_estimators {

class ContactInEKFEstimator
    : public controller_interface::ChainableControllerInterface {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface>
  on_export_state_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

private:
  nav_msgs::msg::Odometry get_odom_msg(const Eigen::Vector3d &pos,
                                       const Eigen::Quaterniond &quat,
                                       const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &ang_vel);

  // Forward kinematics of the 4 feet in the IMU frame (one FK per cycle)
  void compute_feet_fk(const std::array<double, 12> &joint_positions,
                       std::array<Eigen::Matrix4d, 4> &T_imu_foot);

  // Initial filter state: roll/pitch from gravity (yaw ~ 0 for the base),
  // base xy = 0 and height such that the feet in contact are at z = 0.
  inekf::RobotState
  make_initial_state(const Eigen::Vector3d &acc_imu,
                     const Eigen::Vector3d &gyro_bias,
                     const std::array<Eigen::Matrix4d, 4> &T_imu_foot,
                     const std::array<bool, 4> &contact_flags);

  // Convert the IMU-frame filter state to the base frame, fill base_state_,
  // publish /odom and the odom -> base transform
  void publish_state(inekf::RobotState &state,
                     const Eigen::Vector3d &ang_vel_imu,
                     const rclcpp::Time &time);

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<wbc_control_msgs::msg::FootContactFlags>::SharedPtr
      foot_contacts_sub;

  realtime_tools::RealtimeBuffer<std::array<bool, 4>> command_contacts;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  std::vector<double> base_state_;
  std::shared_ptr<inekf::InEKF> filter_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  std::shared_ptr<pinocchio::Data> pin_data_;
  std::atomic<bool> urdf_received_{false};

  // ---- cached from the URDF (written before urdf_received_ is set) --------
  std::string base_frame_{"base"};
  std::string imu_frame_{"imu_link"};
  pinocchio::FrameIndex imu_frame_id_{0};
  std::array<pinocchio::FrameIndex, 4> foot_frame_ids_{};
  std::array<int, 12> joint_q_idx_{};
  Eigen::VectorXd q_;                                  // pinocchio configuration
  Eigen::Matrix3d R_base_imu_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_base_imu_{Eigen::Vector3d::Zero()}; // IMU in base frame

  // ---- IMU input -------------------------------------------------------------
  // imu_source = "interface": imu_sensor/* state interfaces (simulation, Unitree)
  // imu_source = "topic":     sensor_msgs/Imu (e.g. MicroStrain /imu/data_raw),
  //                           only gyro + accel are used
  struct ImuSample {
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acc{Eigen::Vector3d::Zero()};
    double stamp{0.0};  // header stamp [s], only differences are used
    uint64_t seq{0};    // incremented for every received message
    bool valid{false};
  };
  bool use_imu_topic_{false};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  realtime_tools::RealtimeBuffer<ImuSample> imu_buffer_;
  uint64_t imu_seq_{0};   // written by the subscription callback only
  uint64_t last_seq_{0};  // control loop only
  double last_imu_stamp_{0.0};
  double last_new_sample_time_s_{0.0};  // controller clock [s]
  double imu_timeout_{0.2};             // [s] without a new sample -> hold
  double imu_max_dt_{0.05};             // [s] larger stamp gaps use the period

  // ---- initialisation at (re)activation ---------------------------------------
  // calibrate_on_start_: average calib_target_ samples with the robot standing
  // still (gyro bias + gravity direction). Otherwise a single sample is used
  // for the initial roll/pitch and the gyro bias starts at zero.
  bool calibrate_on_start_{false};
  int calib_target_{1};
  int calib_count_{0};
  bool calibrated_{false};
  Eigen::Vector3d calib_gyro_sum_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d calib_acc_sum_{Eigen::Vector3d::Zero()};
  bool auto_initial_pose_{true};
  double initial_height_{0.095};  // IMU height if auto_initial_pose_ is false

  // ---- InEKF noise -----------------------------------------------------------
  double noise_gyro_{0.01};
  double noise_accel_{0.1};
  double noise_gyro_bias_{0.00001};
  double noise_accel_bias_{0.0001};
  double noise_contact_{0.01};
  double noise_kinematics_{0.01};
};

} // namespace quadruped_estimators
