#include "whole_body_controller/control_interpolator.hpp"
#include "whole_body_controller/system_state.hpp"
#include <cstddef>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/time.hpp>

namespace wbc {

const TargetState &
ControlInterpolator::get_next_target(const rclcpp::Time &time) {
  std::vector<TargetState> *mpc_trajectory = targets_.readFromRT();
  rclcpp::Time *timestamp = timestamp_.readFromRT();

  if (!mpc_trajectory || mpc_trajectory->empty() || !timestamp) {
    current_target_ = default_state_;
    current_target_.q_dot.setZero();
    current_target_.q_ddot.setZero();
    return current_target_;
  }

  new_data_flag_.store(false, std::memory_order_release);

  int64_t elapsed_ns = (time - (*timestamp)).nanoseconds();
  int tick_ = static_cast<int>(elapsed_ns / 1'000'000);
  tick_ = std::max(0, tick_);

  size_t max_idx = mpc_trajectory->size() - 1;

  if (tick_ < config_.horizon_ticks) {
    size_t target_id_a = std::min(
        static_cast<size_t>(tick_ / config_.ticks_per_sample), max_idx);
    size_t target_id_b = std::min(target_id_a + 1, max_idx);

    const TargetState &ref_a = (*mpc_trajectory)[target_id_a];
    const TargetState &ref_b = (*mpc_trajectory)[target_id_b];

    int tick_in_this_inter = tick_ % config_.ticks_per_sample;
    double alpha =
        static_cast<double>(tick_in_this_inter) / config_.ticks_per_sample;
    alpha = std::min(alpha, 1.0);

    // SLERP for base orientation + LERP for position & joints
    pinocchio::interpolate(model_, ref_a.q, ref_b.q, alpha, current_target_.q);

    // LERP velocities and accelerations
    current_target_.q_dot = ref_a.q_dot * (1.0 - alpha) + ref_b.q_dot * alpha;
    current_target_.q_ddot =
        ref_a.q_ddot * (1.0 - alpha) + ref_b.q_ddot * alpha;

    for (size_t f = 0; f < 4; ++f) {
      // bool is_in_contact = (alpha < 0.1) ? ref_a.foot_state[f].contact
      //                                    : ref_b.foot_state[f].contact;
      bool is_in_contact = ref_b.foot_state[f].contact;
      current_target_.foot_state[f].contact = is_in_contact;

      // 2. Interpolate Forces Safely
      for (size_t axis = 0; axis < 3; ++axis) {
        size_t force_idx = f * 3 + axis;
        current_target_.contact_forces_odom[force_idx] =
            ref_a.contact_forces_odom[force_idx] * (1.0 - alpha) +
            ref_b.contact_forces_odom[force_idx] * alpha;
      }

      // 3. Interpolate Foot Kinematics
      current_target_.foot_state[f].foot_pos_world =
          ref_a.foot_state[f].foot_pos_world * (1.0 - alpha) +
          ref_b.foot_state[f].foot_pos_world * alpha;

      current_target_.foot_state[f].foot_vel_lwa =
          ref_a.foot_state[f].foot_vel_lwa * (1.0 - alpha) +
          ref_b.foot_state[f].foot_vel_lwa * alpha;
          
      current_target_.foot_state[f].foot_acc_lwa =
          ref_a.foot_state[f].foot_acc_lwa * (1.0 - alpha) +
          ref_b.foot_state[f].foot_acc_lwa * alpha;
    }

  } else if (tick_ < config_.horizon_ticks + config_.extrapolation_ticks) {
    // --- 2. EXTRAPOLATION / DECAY PHASE ---
    int extra_ticks = tick_ - config_.horizon_ticks;
    double decay =
        1.0 - (static_cast<double>(extra_ticks) / config_.extrapolation_ticks);

    // BUGFIX: Always interpolate between the LAST MPC NODE and DEFAULT STATE
    // Do NOT use current_target_ as the source, or it decays recursively!
    const TargetState &last_node = (*mpc_trajectory)[max_idx];

    current_target_.q_dot = last_node.q_dot * decay;
    current_target_.q_ddot = last_node.q_ddot * decay;

    // Decay joint positions smoothly from last known MPC node to default
    pinocchio::interpolate(model_, default_state_.q, last_node.q, decay,
                           current_target_.q);

    for (size_t f = 0; f < 4; ++f) {
      current_target_.foot_state[f].contact =
          true; // Assume default is standing
      current_target_.foot_state[f].foot_vel_lwa =
          last_node.foot_state[f].foot_vel_lwa * decay;

      current_target_.foot_state[f].foot_pos_world =
          last_node.foot_state[f].foot_pos_world * decay +
          default_state_.foot_state[f].foot_pos_world * (1.0 - decay);
    }

    for (size_t i = 0; i < 12; ++i) {
      current_target_.contact_forces_odom[i] =
          (last_node.contact_forces_odom[i] -
           default_state_.contact_forces_odom[i]) *
              decay +
          default_state_.contact_forces_odom[i];
    }

  } else {
    // --- 3. RECOVERY PHASE ---
    current_target_ = default_state_; // Ensure it rests safely
    current_target_.q_dot.setZero();
    current_target_.q_ddot.setZero();
  }

  return current_target_;
}
} // namespace wbc