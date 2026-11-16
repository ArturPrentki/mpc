#pragma once
#include "system_state.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <vector>

namespace wbc {
// WBC ticks   |----------------------------------
// MPC samples |-          -          -          -
// MPC updates |-                                -
//
struct InterpolatrorConfig {
  int ticks_per_sample; // e.g. 10
  int horizon_ticks; // e.g. 100 (10 MPC samples per 100 ticks per 1 MPC update)
  int extrapolation_ticks; // e.g 10 (extrapolate max 10 samples if MPC is late)
  int recovery_samples;    // e.g 10 take 10 (MPC) samples to get to safe pose
};
class ControlInterpolator {
  // this class is supposed to take in current state and ultimate target state
  // and interpolate N in-beetween states. in case source misses it's deadline
  // it should gracefully extrapolate for M next states. after that set target
  // state to nominal and interpolate K states inbeetween.
public:
  ControlInterpolator(const pinocchio::Model &model):model_(model){}
  ~ControlInterpolator() = default;
  void init(InterpolatrorConfig config,
            const TargetState &default_state) {

    config_ = config;
    default_state_ = default_state;
    new_data_flag_.store(false);
    tick_ = 0;
    targets_.reset();
  }
  void set_target_from_non_rt(const std::vector<TargetState> &ss, const rclcpp::Time& ts) {

    targets_.writeFromNonRT(ss);
    timestamp_.writeFromNonRT(ts);
    new_data_flag_.store(true, std::memory_order_release);
  }

  const TargetState &get_next_target(const rclcpp::Time& time);

private:
  int tick_;
  // State tracker
  InterpolatrorConfig config_;
  TargetState default_state_;

  std::atomic<bool> new_data_flag_{false};
  realtime_tools::RealtimeBuffer<std::vector<TargetState>> targets_;
  realtime_tools::RealtimeBuffer<rclcpp::Time> timestamp_;
  TargetState current_target_;
  // TargetState start_tracking;
  const pinocchio::Model model_;
};
} // namespace wbc