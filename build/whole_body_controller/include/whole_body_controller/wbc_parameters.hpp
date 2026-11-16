// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

// silence deprecation warnings for parameter_traits, needed for backwards compatibility
#define SILENCE_DEPRECATION_WARNINGS
#include <parameter_traits/parameter_traits.hpp>
#undef SILENCE_DEPRECATION_WARNINGS

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace whole_body_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::vector<std::string> joint_suffixes = {"_hip_joint", "_thigh_joint", "_calf_joint"};
        std::vector<std::string> legs_prefixes = {"FL", "FR", "RL", "RR"};
        std::string foot_suffix = "_foot";
        std::vector<std::string> base_state_int = {};
        std::vector<double> base_weights = {20.0, 20.0, 60.0, 100.0, 100.0, 40.0};
        std::vector<double> joint_weights = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
        std::vector<double> force_weights = {0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02};
        std::vector<double> foot_weights = {150.0, 150.0, 150.0};
        double kp_foot = 350.0;
        double kd_foot = 37.0;
        double kp_base_lin = 80.0;
        double kd_base_lin = 18.0;
        double kp_base_ang = 100.0;
        double kd_base_ang = 20.0;
        double mu = 0.6;
        double max_fz = 500.0;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double kp_foot = 350.0;
        double kd_foot = 37.0;
        double kp_base_lin = 80.0;
        double kd_base_lin = 18.0;
        double kp_base_ang = 100.0;
        double kd_base_ang = 20.0;
        double mu = 0.6;
        double max_fz = 500.0;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    template <typename NodeT>
    ParamListener(NodeT node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("whole_body_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.kp_foot = params.kp_foot;
      output.kd_foot = params.kd_foot;
      output.kp_base_lin = params.kp_base_lin;
      output.kd_base_lin = params.kd_base_lin;
      output.kp_base_ang = params.kp_base_ang;
      output.kd_base_ang = params.kd_base_ang;
      output.mu = params.mu;
      output.max_fz = params.max_fz;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "joint_suffixes")) {
            updated_params.joint_suffixes = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "legs_prefixes")) {
            updated_params.legs_prefixes = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "foot_suffix")) {
            updated_params.foot_suffix = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_state_int")) {
            updated_params.base_state_int = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_weights")) {
            updated_params.base_weights = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "joint_weights")) {
            updated_params.joint_weights = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "force_weights")) {
            updated_params.force_weights = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "foot_weights")) {
            updated_params.foot_weights = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kp_foot")) {
            updated_params.kp_foot = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kd_foot")) {
            updated_params.kd_foot = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kp_base_lin")) {
            updated_params.kp_base_lin = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kd_base_lin")) {
            updated_params.kd_base_lin = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kp_base_ang")) {
            updated_params.kp_base_ang = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "kd_base_ang")) {
            updated_params.kd_base_ang = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "mu")) {
            updated_params.mu = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_fz")) {
            updated_params.max_fz = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "joint_suffixes")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joint_suffixes);
          parameters_interface_->declare_parameter(prefix_ + "joint_suffixes", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "legs_prefixes")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.legs_prefixes);
          parameters_interface_->declare_parameter(prefix_ + "legs_prefixes", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "foot_suffix")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.foot_suffix);
          parameters_interface_->declare_parameter(prefix_ + "foot_suffix", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_state_int")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_state_int);
          parameters_interface_->declare_parameter(prefix_ + "base_state_int", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_weights")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_weights);
          parameters_interface_->declare_parameter(prefix_ + "base_weights", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "joint_weights")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.joint_weights);
          parameters_interface_->declare_parameter(prefix_ + "joint_weights", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "force_weights")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.force_weights);
          parameters_interface_->declare_parameter(prefix_ + "force_weights", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "foot_weights")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.foot_weights);
          parameters_interface_->declare_parameter(prefix_ + "foot_weights", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kp_foot")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kp_foot);
          parameters_interface_->declare_parameter(prefix_ + "kp_foot", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kd_foot")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kd_foot);
          parameters_interface_->declare_parameter(prefix_ + "kd_foot", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kp_base_lin")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kp_base_lin);
          parameters_interface_->declare_parameter(prefix_ + "kp_base_lin", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kd_base_lin")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kd_base_lin);
          parameters_interface_->declare_parameter(prefix_ + "kd_base_lin", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kp_base_ang")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kp_base_ang);
          parameters_interface_->declare_parameter(prefix_ + "kp_base_ang", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "kd_base_ang")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.kd_base_ang);
          parameters_interface_->declare_parameter(prefix_ + "kd_base_ang", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "mu")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.mu);
          parameters_interface_->declare_parameter(prefix_ + "mu", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_fz")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_fz);
          parameters_interface_->declare_parameter(prefix_ + "max_fz", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "joint_suffixes");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "joint_suffixes") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_suffixes = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "legs_prefixes");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "legs_prefixes") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.legs_prefixes = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "foot_suffix");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "foot_suffix") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.foot_suffix = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "base_state_int");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "base_state_int") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_state_int = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "base_weights");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "base_weights") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_weights = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "joint_weights");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "joint_weights") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.joint_weights = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "force_weights");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "force_weights") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.force_weights = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "foot_weights");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "foot_weights") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.foot_weights = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "kp_foot");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kp_foot") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kp_foot = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "kd_foot");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kd_foot") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kd_foot = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "kp_base_lin");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kp_base_lin") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kp_base_lin = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "kd_base_lin");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kd_base_lin") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kd_base_lin = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "kp_base_ang");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kp_base_ang") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kp_base_ang = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "kd_base_ang");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "kd_base_ang") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.kd_base_ang = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "mu");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "mu") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.mu = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "max_fz");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "max_fz") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_fz = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace whole_body_controller
