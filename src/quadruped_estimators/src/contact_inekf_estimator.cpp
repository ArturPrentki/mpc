
#include "quadruped_estimators/contact_inekf_estimator.hpp"
#include "InEKF.h"
#include <algorithm>
#include <array>
#include <controller_interface/controller_interface_base.hpp>
#include <memory>
#include <rclcpp/logging.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <utility>
namespace quadruped_estimators {

namespace {
const std::array<const char *, 12> kJointNames = {
    "FL_hip_joint",  "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint",  "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint",  "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint",  "RR_thigh_joint", "RR_calf_joint"};
const std::array<const char *, 4> kFootFrameNames = {"FL_foot", "FR_foot",
                                                     "RL_foot", "RR_foot"};
} // namespace

controller_interface::InterfaceConfiguration
ContactInEKFEstimator::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
ContactInEKFEstimator::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request IMU data from hardware, unless it comes from a ROS topic
  const bool topic_mode =
      get_node()->get_parameter("imu_source").as_string() == "topic";
  if (!topic_mode) {
    config.names.push_back("imu_sensor/linear_acceleration.x");
    config.names.push_back("imu_sensor/linear_acceleration.y");
    config.names.push_back("imu_sensor/linear_acceleration.z");
    config.names.push_back("imu_sensor/angular_velocity.x");
    config.names.push_back("imu_sensor/angular_velocity.y");
    config.names.push_back("imu_sensor/angular_velocity.z");
  }

  for (const auto &joint : kJointNames) {
    config.names.push_back(std::string(joint) + "/position");
  }

  return config;
}

controller_interface::CallbackReturn ContactInEKFEstimator::on_init() {

  urdf_received_ = false;
  base_state_.resize(13, 0.0);
  base_state_[6] = 1.0; // quat_w default

  // IMU input
  auto_declare<std::string>("imu_source", "interface"); // "interface" | "topic"
  auto_declare<std::string>("imu_topic", "/imu/data_raw");
  auto_declare<double>("imu_timeout", 0.2);
  auto_declare<double>("imu_max_dt", 0.05);
  auto_declare<bool>("imu_calibrate_on_start", false);
  auto_declare<int>("imu_calibration_samples", 500);

  // initial pose / frames
  auto_declare<bool>("auto_initial_pose", true);
  auto_declare<double>("initial_height", 0.095);
  auto_declare<std::string>("base_frame", "base");
  auto_declare<std::string>("imu_frame", "imu_link");

  // InEKF noise (tune for your IMU / robot)
  auto_declare<double>("noise_gyro", 0.01);
  auto_declare<double>("noise_accel", 0.1);
  auto_declare<double>("noise_gyro_bias", 0.00001);
  auto_declare<double>("noise_accel_bias", 0.0001);
  auto_declare<double>("noise_contact", 0.01);
  auto_declare<double>("noise_kinematics", 0.01);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ContactInEKFEstimator::on_configure(const rclcpp_lifecycle::State &) {

  auto node = get_node();

  // ---- parameters -----------------------------------------------------------
  const std::string imu_source = node->get_parameter("imu_source").as_string();
  if (imu_source != "topic" && imu_source != "interface") {
    RCLCPP_ERROR(node->get_logger(),
                 "imu_source must be 'topic' or 'interface', got '%s'",
                 imu_source.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  use_imu_topic_ = (imu_source == "topic");
  imu_timeout_ = node->get_parameter("imu_timeout").as_double();
  imu_max_dt_ = node->get_parameter("imu_max_dt").as_double();
  calibrate_on_start_ = node->get_parameter("imu_calibrate_on_start").as_bool();
  calib_target_ =
      calibrate_on_start_
          ? std::max<int>(10, static_cast<int>(node->get_parameter(
                                                   "imu_calibration_samples")
                                                   .as_int()))
          : 1;
  auto_initial_pose_ = node->get_parameter("auto_initial_pose").as_bool();
  initial_height_ = node->get_parameter("initial_height").as_double();
  base_frame_ = node->get_parameter("base_frame").as_string();
  imu_frame_ = node->get_parameter("imu_frame").as_string();

  noise_gyro_ = node->get_parameter("noise_gyro").as_double();
  noise_accel_ = node->get_parameter("noise_accel").as_double();
  noise_gyro_bias_ = node->get_parameter("noise_gyro_bias").as_double();
  noise_accel_bias_ = node->get_parameter("noise_accel_bias").as_double();
  noise_contact_ = node->get_parameter("noise_contact").as_double();
  noise_kinematics_ = node->get_parameter("noise_kinematics").as_double();

  // ---- IMU topic -----------------------------------------------------------
  imu_sub_.reset();
  imu_seq_ = 0;
  imu_buffer_.writeFromNonRT(ImuSample{});
  if (use_imu_topic_) {
    const std::string imu_topic = node->get_parameter("imu_topic").as_string();
    auto imu_qos = rclcpp::SensorDataQoS();
    imu_qos.keep_last(5);
    imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          ImuSample sample;
          sample.gyro << msg->angular_velocity.x, msg->angular_velocity.y,
              msg->angular_velocity.z;
          sample.acc << msg->linear_acceleration.x, msg->linear_acceleration.y,
              msg->linear_acceleration.z;
          sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
          sample.seq = ++imu_seq_;
          sample.valid = true;
          imu_buffer_.writeFromNonRT(sample);
        });
    RCLCPP_INFO(node->get_logger(), "Using IMU topic %s", imu_topic.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "Using imu_sensor state interfaces");
  }

  std::array<bool, 4> contacts = {true, true, true, true};

  command_contacts.writeFromNonRT(contacts);

  auto wbc_command_qos = rclcpp::SensorDataQoS();
  wbc_command_qos.keep_last(1);
  foot_contacts_sub =
      get_node()->create_subscription<wbc_control_msgs::msg::FootContactFlags>(
          "/foot_contacts", wbc_command_qos,
          [this](const wbc_control_msgs::msg::FootContactFlags::SharedPtr msg) {
            std::array<bool, 4> contacts = {msg->fl, msg->fr, msg->rl, msg->rr};
            command_contacts.writeFromNonRT(contacts);
          });

  urdf_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (urdf_received_.load(std::memory_order_acquire)) {
          return;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Got URDF file!");
        auto model = std::make_shared<pinocchio::Model>();
        // Build Pinocchio model from URDF XML string
        pinocchio::urdf::buildModelFromXML(
            msg->data, pinocchio::JointModelFreeFlyer(), *model);
        auto data = std::make_shared<pinocchio::Data>(*model);

        // Validate and cache everything the real-time loop needs, so it does
        // no string lookups
        bool ok = true;
        for (const std::string &f : {base_frame_, imu_frame_}) {
          if (!model->existFrame(f)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Frame '%s' not found in the URDF", f.c_str());
            ok = false;
          }
        }
        for (size_t i = 0; i < kFootFrameNames.size(); ++i) {
          if (!model->existFrame(kFootFrameNames[i])) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Frame '%s' not found in the URDF",
                         kFootFrameNames[i]);
            ok = false;
          }
        }
        for (size_t i = 0; i < kJointNames.size(); ++i) {
          if (!model->existJointName(kJointNames[i])) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Joint '%s' not found in the URDF", kJointNames[i]);
            ok = false;
          }
        }
        if (!ok) {
          return; // flag stays false: the estimator keeps waiting
        }

        imu_frame_id_ = model->getFrameId(imu_frame_);
        for (size_t i = 0; i < kFootFrameNames.size(); ++i) {
          foot_frame_ids_[i] = model->getFrameId(kFootFrameNames[i]);
        }
        for (size_t i = 0; i < kJointNames.size(); ++i) {
          joint_q_idx_[i] =
              model->joints[model->getJointId(kJointNames[i])].idx_q();
        }
        q_ = pinocchio::neutral(*model);

        // IMU pose in the base frame (fixed, independent of the joints)
        pinocchio::forwardKinematics(*model, *data, q_);
        pinocchio::updateFramePlacements(*model, *data);
        const pinocchio::FrameIndex base_id = model->getFrameId(base_frame_);
        const pinocchio::SE3 T_base_imu =
            data->oMf[base_id].inverse() * data->oMf[imu_frame_id_];
        R_base_imu_ = T_base_imu.rotation();
        p_base_imu_ = T_base_imu.translation();

        pin_model_ = model;
        pin_data_ = data;
        urdf_received_.store(true, std::memory_order_release);
      });

  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(1).transient_local());
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn ContactInEKFEstimator::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  filter_ = std::shared_ptr<inekf::InEKF>(new inekf::InEKF());
  // Initialize InEKF Noise Parameters (Tune these for your specific robot/IMU)
  inekf::NoiseParams noise_params;
  noise_params.setGyroscopeNoise(noise_gyro_);
  noise_params.setAccelerometerNoise(noise_accel_);
  noise_params.setGyroscopeBiasNoise(noise_gyro_bias_);
  noise_params.setAccelerometerBiasNoise(noise_accel_bias_);
  noise_params.setContactNoise(noise_contact_);

  // Provisional state: replaced by the gravity/FK based one on the first
  // update (see make_initial_state)
  inekf::RobotState initial_state;
  Eigen::Vector3d init_pos(0.0, 0.0, initial_height_);
  initial_state.setPosition(init_pos);
  filter_->setState(initial_state);
  filter_->setNoiseParams(noise_params);

  // restart the initialisation phase
  calibrated_ = false;
  calib_count_ = 0;
  calib_gyro_sum_.setZero();
  calib_acc_sum_.setZero();
  last_seq_ = 0;
  last_imu_stamp_ = 0.0;
  last_new_sample_time_s_ = 0.0;

  std::fill(base_state_.begin(), base_state_.end(), 0.0);
  base_state_[2] = initial_height_;
  base_state_[6] = 1.0;
  return controller_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
ContactInEKFEstimator::on_export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  std::vector<std::string> names = {
      "pos_x",     "pos_y",     "pos_z",     "quat_x",    "quat_y",
      "quat_z",    "quat_w",    "lin_vel_x", "lin_vel_y", "lin_vel_z",
      "ang_vel_x", "ang_vel_y", "ang_vel_z"};
  for (size_t i = 0; i < names.size(); ++i) {
    interfaces.emplace_back(hardware_interface::StateInterface(
        get_node()->get_name(), names[i], &base_state_[i]));
  }
  return interfaces;
}
controller_interface::CallbackReturn ContactInEKFEstimator::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  filter_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}
bool ContactInEKFEstimator::on_set_chained_mode(bool /*chained_mode*/) {
  return true;
}

controller_interface::return_type
ContactInEKFEstimator::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  // Can be left empty, as we handle RT logic in update_and_write_commands
  return controller_interface::return_type::OK;
}

controller_interface::return_type
ContactInEKFEstimator::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {

  // the pinocchio model arrives asynchronously on /robot_description
  if (!urdf_received_.load(std::memory_order_acquire) || !filter_) {
    return controller_interface::return_type::OK;
  }

  double dt = period.seconds();
  Eigen::Vector3d acc, gyro;
  bool new_sample = true; // propagate only once per IMU sample
  size_t joint_offset = 6;

  // 1. Read the IMU
  if (use_imu_topic_) {
    const ImuSample *sample = imu_buffer_.readFromRT();
    if (!sample || !sample->valid) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           2000, "Waiting for the first IMU message");
      return controller_interface::return_type::OK;
    }
    gyro = sample->gyro;
    acc = sample->acc;
    new_sample = (sample->seq != last_seq_);
    if (new_sample) {
      if (last_seq_ != 0) {
        // dt from the IMU's own stamps: independent of the controller rate and
        // of any offset between the IMU clock and the ROS clock
        const double d = sample->stamp - last_imu_stamp_;
        dt = (d > 0.0 && d < imu_max_dt_) ? d : period.seconds();
      }
      last_seq_ = sample->seq;
      last_imu_stamp_ = sample->stamp;
      last_new_sample_time_s_ = time.seconds();
    } else if (time.seconds() - last_new_sample_time_s_ > imu_timeout_) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000, "IMU stale (> %.2f s without a new sample), "
                                  "holding the estimate", imu_timeout_);
      return controller_interface::return_type::OK;
    }
    joint_offset = 0; // joints follow directly, no IMU interfaces
  } else {
    std::array<double, 6> v{};
    for (size_t i = 0; i < 6; ++i) {
      const auto value = state_interfaces_[i].get_optional();
      if (!value) {
        return controller_interface::return_type::OK;
      }
      v[i] = *value;
    }
    acc << v[0], v[1], v[2];
    gyro << v[3], v[4], v[5];
  }

  // 2. Joint positions and forward kinematics of the feet (once per cycle)
  std::array<double, 12> joint_positions{};
  for (size_t i = 0; i < 12; ++i) {
    const auto value = state_interfaces_[joint_offset + i].get_optional();
    if (!value) {
      return controller_interface::return_type::OK;
    }
    joint_positions[i] = *value;
  }
  std::array<Eigen::Matrix4d, 4> T_imu_foot;
  compute_feet_fk(joint_positions, T_imu_foot);

  auto contacts_ptr = command_contacts.readFromRT();
  const std::array<bool, 4> contact_flags =
      contacts_ptr ? *contacts_ptr
                   : std::array<bool, 4>{true, true, true, true};

  // 3. Initialisation phase (robot standing). The provisional state is already
  // published so that the controllers never see a zero pose.
  if (!calibrated_) {
    if (new_sample) {
      calib_acc_sum_ += acc;
      calib_gyro_sum_ += gyro;
      ++calib_count_;
    }
    if (calib_count_ == 0) {
      return controller_interface::return_type::OK;
    }
    const Eigen::Vector3d acc_mean = calib_acc_sum_ / calib_count_;
    const Eigen::Vector3d gyro_mean = calib_gyro_sum_ / calib_count_;
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    if (calibrate_on_start_ && gyro_mean.norm() < 0.2) {
      bias = gyro_mean;
    }
    inekf::RobotState state =
        make_initial_state(acc_mean, bias, T_imu_foot, contact_flags);
    if (calib_count_ >= calib_target_) {
      filter_->setState(state);
      calibrated_ = true;
      auto logger = get_node()->get_logger();
      RCLCPP_INFO(logger,
                  "InEKF initialised (%d samples): gyro bias = [%.4f %.4f "
                  "%.4f] rad/s, |accel| = %.3f m/s^2",
                  calib_count_, bias.x(), bias.y(), bias.z(), acc_mean.norm());
      if (calibrate_on_start_ && gyro_mean.norm() >= 0.2) {
        RCLCPP_WARN(logger,
                    "The robot was moving during calibration (mean |gyro| = "
                    "%.3f rad/s), gyro bias not estimated",
                    gyro_mean.norm());
      }
      if (acc_mean.norm() < 7.0 || acc_mean.norm() > 12.0) {
        RCLCPP_ERROR(logger,
                     "|accel| = %.2f m/s^2 (expected about 9.8): check the IMU "
                     "units, topic and mounting",
                     acc_mean.norm());
      }
    }
    publish_state(state, gyro - bias, time);
    return controller_interface::return_type::OK;
  }

  // 4. Propagation step (IMU), once per new sample
  if (new_sample) {
    Eigen::Matrix<double, 6, 1> imu_meas;
    imu_meas << gyro, acc;
    filter_->Propagate(imu_meas, dt);
  }

  // 5. Kinematics correction. Every leg is passed with its contact flag: the
  // InEKF only forgets the estimated foot position when it is told that the
  // contact is false. Passing only the feet in contact keeps a stale foot
  // position that is then used again at the next touchdown.
  inekf::vectorKinematics kinematics_list;
  std::vector<std::pair<int, bool>> contacts;
  contacts.reserve(4);
  Eigen::Matrix<double, 6, 6> contact_cov =
      Eigen::Matrix<double, 6, 6>::Identity() * noise_kinematics_;
  for (int leg_id = 0; leg_id < 4; leg_id++) {
    inekf::Kinematics kin_meas(leg_id, T_imu_foot[leg_id], contact_cov);
    kinematics_list.push_back(kin_meas);
    contacts.push_back(std::pair<int, bool>(leg_id, contact_flags[leg_id]));
  }
  filter_->setContacts(contacts);

  // Apply right-invariant update
  filter_->CorrectKinematics(kinematics_list);

  inekf::RobotState state = filter_->getState();
  const Eigen::Vector3d gyro_bias = state.getGyroscopeBias();
  publish_state(state, gyro - gyro_bias, time);

  return controller_interface::return_type::OK;
}

void ContactInEKFEstimator::compute_feet_fk(
    const std::array<double, 12> &joint_positions,
    std::array<Eigen::Matrix4d, 4> &T_imu_foot) {

  // joints are mapped by name (indices cached when the URDF arrived)
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    q_[joint_q_idx_[i]] = joint_positions[i];
  }

  pinocchio::forwardKinematics(*pin_model_, *pin_data_, q_);
  pinocchio::updateFramePlacements(*pin_model_, *pin_data_);

  const pinocchio::SE3 T_imu_world = pin_data_->oMf[imu_frame_id_].inverse();
  for (size_t leg = 0; leg < 4; ++leg) {
    const pinocchio::SE3 T = T_imu_world * pin_data_->oMf[foot_frame_ids_[leg]];
    T_imu_foot[leg] = T.toHomogeneousMatrix();
  }
}

inekf::RobotState ContactInEKFEstimator::make_initial_state(
    const Eigen::Vector3d &acc_imu, const Eigen::Vector3d &gyro_bias,
    const std::array<Eigen::Matrix4d, 4> &T_imu_foot,
    const std::array<bool, 4> &contact_flags) {

  // Roll/pitch from gravity, expressed for the BASE frame so that a rotated
  // IMU mounting (R_base_imu_ from the URDF) does not change the base yaw.
  // At rest the accelerometer measures the reaction to gravity, which points up.
  const double n = acc_imu.norm();
  Eigen::Matrix3d R_world_base = Eigen::Matrix3d::Identity();
  if (n > 5.0 && n < 15.0) {
    const Eigen::Vector3d up_base = (R_base_imu_ * acc_imu) / n;
    R_world_base = Eigen::Quaterniond::FromTwoVectors(
                       up_base, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();
  }
  const Eigen::Matrix3d R_world_imu = R_world_base * R_base_imu_;

  inekf::RobotState state;
  state.setRotation(R_world_imu);
  state.setGyroscopeBias(gyro_bias);

  Eigen::Vector3d p_imu;
  if (auto_initial_pose_) {
    // Base at x = y = 0; the IMU height puts the feet in contact at z = 0
    double sum_z = 0.0;
    int count = 0;
    for (int leg = 0; leg < 4; ++leg) {
      if (contact_flags[leg]) {
        sum_z += (R_world_imu * T_imu_foot[leg].block<3, 1>(0, 3)).z();
        ++count;
      }
    }
    if (count == 0) { // no contact information: use all feet
      for (int leg = 0; leg < 4; ++leg) {
        sum_z += (R_world_imu * T_imu_foot[leg].block<3, 1>(0, 3)).z();
      }
      count = 4;
    }
    const Eigen::Vector3d offset = R_world_base * p_base_imu_;
    p_imu << offset.x(), offset.y(), -sum_z / count;
  } else {
    p_imu << 0.0, 0.0, initial_height_;
  }
  state.setPosition(p_imu);
  return state;
}

void ContactInEKFEstimator::publish_state(inekf::RobotState &state,
                                          const Eigen::Vector3d &ang_vel_imu,
                                          const rclcpp::Time &time) {
  // A. Raw IMU state (world frame for pos/vel, IMU frame for the gyro)
  Eigen::Vector3d pos_imu_world = state.getPosition();
  Eigen::Quaterniond quat_imu_world(state.getRotation());
  Eigen::Vector3d vel_imu_world = state.getVelocity();

  // B. IMU pose relative to the base (cached from the URDF)
  const Eigen::Vector3d &p_offset_base = p_base_imu_;
  const Eigen::Matrix3d &R_base_imu = R_base_imu_;

  Eigen::Matrix3d R_world_base =
      quat_imu_world.toRotationMatrix() * R_base_imu.transpose();
  Eigen::Quaterniond quat_world_base(R_world_base);

  // Angular velocity in base frame
  Eigen::Vector3d ang_vel_base = R_base_imu * ang_vel_imu;

  // Position of the base in the world frame
  Eigen::Vector3d pos_base_world = pos_imu_world - R_world_base * p_offset_base;

  // Linear velocity of the base in the base frame (body velocity for the ROS
  // odometry twist)
  Eigen::Vector3d vel_base_body = R_world_base.transpose() * vel_imu_world -
                                  ang_vel_base.cross(p_offset_base);

  // C. Populate the exported state
  base_state_[0] = pos_base_world.x();
  base_state_[1] = pos_base_world.y();
  base_state_[2] = pos_base_world.z();

  base_state_[3] = quat_world_base.x();
  base_state_[4] = quat_world_base.y();
  base_state_[5] = quat_world_base.z();
  base_state_[6] = quat_world_base.w();

  base_state_[7] = vel_base_body.x();
  base_state_[8] = vel_base_body.y();
  base_state_[9] = vel_base_body.z();

  base_state_[10] = ang_vel_base.x();
  base_state_[11] = ang_vel_base.y();
  base_state_[12] = ang_vel_base.z();

  // D. Publish odometry
  nav_msgs::msg::Odometry odom = get_odom_msg(pos_base_world, quat_world_base,
                                              vel_base_body, ang_vel_base);

  odom.header.stamp = time;
  odom.header.frame_id = "odom"; // world frame for position/orientation
  odom.child_frame_id = base_frame_; // body frame for the twist

  odom_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = time;
  t.header.frame_id = "odom";
  t.child_frame_id = base_frame_;

  t.transform.translation.x = pos_base_world.x();
  t.transform.translation.y = pos_base_world.y();
  t.transform.translation.z = pos_base_world.z();

  t.transform.rotation.x = quat_world_base.x();
  t.transform.rotation.y = quat_world_base.y();
  t.transform.rotation.z = quat_world_base.z();
  t.transform.rotation.w = quat_world_base.w();

  tf_broadcaster_->sendTransform(t);
}

nav_msgs::msg::Odometry ContactInEKFEstimator::get_odom_msg(
    const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat,
    const Eigen::Vector3d &vel, const Eigen::Vector3d &ang_vel) {

  nav_msgs::msg::Odometry odom;

  // Pose (position and orientation in the 'odom' frame)
  odom.pose.pose.position.x = pos.x();
  odom.pose.pose.position.y = pos.y();
  odom.pose.pose.position.z = pos.z();

  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();

  // Twist (in the child frame)
  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();

  odom.twist.twist.angular.x = ang_vel.x();
  odom.twist.twist.angular.y = ang_vel.y();
  odom.twist.twist.angular.z = ang_vel.z();

  // Covariances: 36-element row-major arrays, constants for now. Replace with
  // the filter covariance if you need real uncertainties.
  odom.pose.covariance[0] = 0.01;   // x
  odom.pose.covariance[7] = 0.01;   // y
  odom.pose.covariance[14] = 0.01;  // z
  odom.pose.covariance[21] = 0.001; // roll
  odom.pose.covariance[28] = 0.001; // pitch
  odom.pose.covariance[35] = 0.001; // yaw

  odom.twist.covariance[0] = 0.01;   // vx
  odom.twist.covariance[7] = 0.01;   // vy
  odom.twist.covariance[14] = 0.01;  // vz
  odom.twist.covariance[21] = 0.001; // v_roll
  odom.twist.covariance[28] = 0.001; // v_pitch
  odom.twist.covariance[35] = 0.001; // v_yaw

  return odom;
}

} // namespace quadruped_estimators

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(quadruped_estimators::ContactInEKFEstimator,
                       controller_interface::ChainableControllerInterface)
