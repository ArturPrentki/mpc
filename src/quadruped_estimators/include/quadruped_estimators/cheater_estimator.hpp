#include "controller_interface/chainable_controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <rclcpp/publisher.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace quadruped_estimators {

class CheaterEstimator
    : public controller_interface::ChainableControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::InterfaceConfiguration
  
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  // This is where we expose variables to the WBC
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> odom_buffer_;

  // [x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]

  std::vector<double> base_state_;
};

} // namespace quadruped_estimators