#include "quadruped_estimators/cheater_estimator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/logging.hpp>

namespace quadruped_estimators {

controller_interface::InterfaceConfiguration
CheaterEstimator::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
CheaterEstimator::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back("odometer/position.x");
  config.names.push_back("odometer/position.y");
  config.names.push_back("odometer/position.z");

  config.names.push_back("imu_sensor/orientation.x");
  config.names.push_back("imu_sensor/orientation.y");
  config.names.push_back("imu_sensor/orientation.z");
  config.names.push_back("imu_sensor/orientation.w");

  config.names.push_back("odometer/velocity.x");
  config.names.push_back("odometer/velocity.y");
  config.names.push_back("odometer/velocity.z");

  config.names.push_back("imu_sensor/angular_velocity.x");
  config.names.push_back("imu_sensor/angular_velocity.y");
  config.names.push_back("imu_sensor/angular_velocity.z");

  return config;
}

controller_interface::CallbackReturn CheaterEstimator::on_init() {
  base_state_.resize(13, 0.0);
  base_state_[6] = 1.0; // quat_w default
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
CheaterEstimator::on_configure(const rclcpp_lifecycle::State &) {
  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(1).transient_local());
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CheaterEstimator::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;

  // clang-format off
  std::vector<std::string> names = { 
      "pos_x", "pos_y", "pos_z",
      "quat_x", "quat_y", "quat_z", "quat_w",
      "lin_vel_x", "lin_vel_y", "lin_vel_z",
      "ang_vel_x", "ang_vel_y", "ang_vel_z"
    };
  // clang-format on

  for (size_t i = 0; i < names.size(); ++i) {
    interfaces.emplace_back(hardware_interface::StateInterface(
        get_node()->get_name(), names[i], &base_state_[i]));
  }
  return interfaces;
}

bool CheaterEstimator::on_set_chained_mode(bool /*chained_mode*/) {
  return true;
}

controller_interface::return_type
CheaterEstimator::update_reference_from_subscribers(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {

  // Write directly into reference_interfaces_ using standard pointer access
  base_state_[0] = state_interfaces_[0].get_optional().value_or(0);
  base_state_[1] = state_interfaces_[1].get_optional().value_or(0);
  base_state_[2] = state_interfaces_[2].get_optional().value_or(0);

  base_state_[3] = state_interfaces_[3].get_optional().value_or(0);
  base_state_[4] = state_interfaces_[4].get_optional().value_or(0);
  base_state_[5] = state_interfaces_[5].get_optional().value_or(0);
  base_state_[6] = state_interfaces_[6].get_optional().value_or(1.);

  base_state_[7] = state_interfaces_[7].get_optional().value_or(0);
  base_state_[8] = state_interfaces_[8].get_optional().value_or(0);
  base_state_[9] = state_interfaces_[9].get_optional().value_or(0);

  base_state_[10] = state_interfaces_[10].get_optional().value_or(0);
  base_state_[11] = state_interfaces_[11].get_optional().value_or(0);
  base_state_[12] = state_interfaces_[12].get_optional().value_or(0);

  return controller_interface::return_type::OK;
}

controller_interface::return_type
CheaterEstimator::update_and_write_commands(const rclcpp::Time &time,
                                            const rclcpp::Duration &) {

  base_state_[0] = state_interfaces_[0].get_optional().value_or(0);
  base_state_[1] = state_interfaces_[1].get_optional().value_or(0);
  base_state_[2] = state_interfaces_[2].get_optional().value_or(0);

  base_state_[3] = state_interfaces_[3].get_optional().value_or(0);
  base_state_[4] = state_interfaces_[4].get_optional().value_or(0);
  base_state_[5] = state_interfaces_[5].get_optional().value_or(0);
  base_state_[6] = state_interfaces_[6].get_optional().value_or(1.);

  base_state_[7] = state_interfaces_[7].get_optional().value_or(0);
  base_state_[8] = state_interfaces_[8].get_optional().value_or(0);
  base_state_[9] = state_interfaces_[9].get_optional().value_or(0);

  base_state_[10] = state_interfaces_[10].get_optional().value_or(0);
  base_state_[11] = state_interfaces_[11].get_optional().value_or(0);
  base_state_[12] = state_interfaces_[12].get_optional().value_or(0);

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = time;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base";
  msg.pose.pose.position.x = base_state_[0];
  msg.pose.pose.position.y = base_state_[1];
  msg.pose.pose.position.z = base_state_[2];

  msg.pose.pose.orientation.x = base_state_[3];
  msg.pose.pose.orientation.y = base_state_[4];
  msg.pose.pose.orientation.z = base_state_[5];
  msg.pose.pose.orientation.w = base_state_[6];

  msg.twist.twist.linear.x = base_state_[7];
  msg.twist.twist.linear.y = base_state_[8];
  msg.twist.twist.linear.z = base_state_[9];

  msg.twist.twist.angular.x = base_state_[10];
  msg.twist.twist.angular.y = base_state_[11];
  msg.twist.twist.angular.z = base_state_[12];
  odom_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = time;
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base";

  tf_msg.transform.translation.x = base_state_[0];
  tf_msg.transform.translation.y = base_state_[1];
  tf_msg.transform.translation.z = base_state_[2];

  tf_msg.transform.rotation.w = base_state_[6];
  tf_msg.transform.rotation.x = base_state_[3];
  tf_msg.transform.rotation.y = base_state_[4];
  tf_msg.transform.rotation.z = base_state_[5];
  tf_broadcaster_->sendTransform(tf_msg);

  return controller_interface::return_type::OK;
}

} // namespace quadruped_estimators

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_estimators::CheaterEstimator,
                       controller_interface::ChainableControllerInterface)