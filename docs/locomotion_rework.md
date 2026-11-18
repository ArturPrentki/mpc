### 1. Contact force estimate read garbage memory

`dynamic_system.cpp`:

```cpp
system_state.hu = pin_data_->nle.head<Consts::base_dof>();   // 6 entries
...
Eigen::Vector3d tau_leg_active =
    tau_leg - system_state.hu.segment<3>(6 + i * 3);          // entries 6..17!
Eigen::Vector3d force_lwa =
    -J_leg_linear.transpose().colPivHouseholderQr().solve(tau_leg_active);
```

`hu` holds only the 6 floating-base entries of `nle`, but the code reads the leg
entries 6–17 from it. Eigen does not range-check in release builds, so it read
whatever memory followed `hu` in `SystemState` (the `mass_matrix` object's
pointer and size, reinterpreted as doubles). The 15 N touchdown and 9.81 N
liftoff thresholds were compared against `J⁻ᵀ(τ − junk)`, so contact flags were
partly noise. Even with the full vector, `6 + i*3` assumes the Pinocchio joint
order, which is wrong on Gardo.

Fix: take each joint's term from the full 18-entry `nle` at its real index:

```diff
     Eigen::Matrix3d J_leg_linear;
     Eigen::Vector3d tau_leg;
+    Eigen::Vector3d h_leg;  // gravity/Coriolis torque of this leg's joints
 
     for (int j = 0; j < 3; ++j) {
       int joint_idx = i * 3 + j;
+      int v_idx = joint_v_indices_[joint_idx];   // index in Pinocchio's v (0..17)
-      J_leg_linear.col(j) = system_state.foot_jac_lin_lwa.block<3, 1>(
-          3 * i, joint_v_indices_[joint_idx]);
+      J_leg_linear.col(j) = system_state.foot_jac_lin_lwa.block<3, 1>(3 * i, v_idx);
       tau_leg(j) = system_state.j_torq(joint_idx);
+      h_leg(j) = pin_data_->nle[v_idx];          // full vector, not the 6-entry hu
     }
 
-    Eigen::Vector3d tau_leg_active =
-        tau_leg - system_state.hu.segment<3>(6 + i * 3);
+    Eigen::Vector3d tau_leg_active = tau_leg - h_leg;
     Eigen::Vector3d force_lwa =
         -J_leg_linear.transpose().colPivHouseholderQr().solve(tau_leg_active);
```

The formula itself, `f = −J⁻ᵀ(τ − h)`, was right. The thresholds are now the
parameters `contact_detection.touchdown_force` (15 N) and `liftoff_force`
(10 N), and a foot that stalls while reaching for the ground counts as landed
even below 15 N.

### 2. Joint order assumed to match Pinocchio

`update_system_state()` (and `send_system_state()` the same way):

```cpp
system_state.q[7 + i * num_of_act + j] = actuator.at(ActIfs::POS)->get_optional().value_or(0.);
system_state.q_dot[6 + i * num_of_act + j] = ...
```

This writes leg `i`, joint `j` of the interfaces into slot `i*3 + j` of
Pinocchio's state, which only works if Pinocchio orders the joints the same
way. On Go2 it does; on Gardo Pinocchio's order is LF, LH, RF, RH, so the WBC
saw the legs swapped.

Fix: use each joint's real index, as the Jacobian code already did:

```diff
-      system_state.q[7 + i * num_of_act + j] = ...POS...;
-      system_state.q_dot[6 + i * num_of_act + j] = ...VEL...;
+      const int k = i * num_of_act + j;                // interface order
+      system_state.q[model.joints[joint_ids_[k]].idx_q()] = ...POS...;
+      system_state.q_dot[joint_v_indices_[k]] = ...VEL...;
```

The same mapping applies when reading `target.q` / `target.q_dot` in
`send_system_state()`.

### 3. Joint PD fighting the WBC on stance legs

`trajectory_update()` and `send_system_state()`:

```cpp
ts.j_kp.segment<3>(3 * id).setConstant(100.0);
ts.j_kd.segment<3>(3 * id).setConstant(25.0);
...
coms.at(ActIfs::POS)->set_value(q[i * n_o_a + j]);       // MPC joint angles
coms.at(ActIfs::VEL)->set_value(q_dot[i * n_o_a + j]);   // MPC joint velocities
```

Every joint, stance legs included, was pulled hard towards the MPC's joint
angles and velocities while the WBC torque was doing something else. Even
small damping against the motion the WBC wanted caused about 6 rad/s² roll
errors on Go2. The fix was already in the old code, commented out:

```diff
-      ts.j_kp.segment<3>(3 * id).setConstant(100.0);
-      ts.j_kd.segment<3>(3 * id).setConstant(25.0);
+      if (foot.is_contact) {           // stance: torque does the work
+        ts.j_kp.segment<3>(3 * id).setZero();
+        ts.j_kd.segment<3>(3 * id).setConstant(1.0);
+      } else {                         // swing: track the leg
+        ts.j_kp.segment<3>(3 * id).setConstant(20.0);
+        ts.j_kd.segment<3>(3 * id).setConstant(1.0);
+      }
```

and, for stance legs, send the velocity the WBC is about to produce:

```diff
-      (void)coms.at(ActIfs::VEL)->set_value(q_dot[i * n_o_a + j]);
+      // stance: v_des = v + qdd * dt, so damping only corrects deviations
+      (void)coms.at(ActIfs::VEL)->set_value(v_meas[k] + output.q_ddot[v_idx] * dt);
```

The new code also gives swing legs joint targets from leg IK of the foot
target, so their PD helps the swing task.

### 4. Solver failure turned the controller off

`update()`:

```cpp
if (!optimizer_->solve(system_state, target, output)) {
  RCLCPP_FATAL_THROTTLE(..., "Solver failed!");
  return controller_interface::return_type::ERROR;
}
```

Returning `ERROR` makes ros2_control deactivate the controller, so one bad QP
left the robot limp mid-step.

Fix: keep commanding the drives:

```diff
 if (!optimizer_->solve(system_state, target, output)) {
-  return controller_interface::return_type::ERROR;
+  if (time - last_good_time_ < 0.05) output = last_good_output_;  // hold 50 ms
+  else { damp_joints(); return controller_interface::return_type::OK; }
+} else {
+  last_good_output_ = output; last_good_time_ = time;
 }
```

In the new code this is the `SafetySupervisor` (hold for `safety.hold_time`,
then latched damping; tilting past `safety.max_tilt` also damps).

### 5. Zero torque when the dynamics residual exceeded 1

`inverse_dynamics()`:

```cpp
if (tau_full.head<6>().norm() > 1.0) {
  std::cout << "WARNING: WBC violated floating base dynamics! ...";
  target.torque.setZero();
}
```

OSQP solves to a tolerance, so a residual above 1 N can happen when a heavy
robot moves fast. Zero torque on all joints drops the robot.

Fix: warn, keep the torques and clamp them to the limits:

```diff
 if (tau_full.head<6>().norm() > 1.0) {
   std::cout << "WARNING: ...";
-  target.torque.setZero();
-} else {
-  for (int i = 0; i < 12; ++i) target.torque[i] = tau_full[joint_v_indices_[i]];
 }
+for (int i = 0; i < 12; ++i)
+  target.torque[i] = std::clamp(tau_full[joint_v_indices_[i]], -tau_max[i], tau_max[i]);
```

### 6. Interpolator timing and contact switching

`control_interpolator.cpp`, with config `{40, 40*30, 100, 200}`:

```cpp
int tick_ = static_cast<int>(elapsed_ns / 1'000'000);   // "1 tick = 1 ms"
...
bool is_in_contact = ref_b.foot_state[f].contact;        // next point's contact
```

- It counted milliseconds as ticks and assumed 40 ms between MPC points, so it
  was only right at 1 kHz with a 40 ms MPC step; the message's `time_offset`
  was ignored.
- It took the contact flag of the next point, so feet switched to stance or
  swing up to 40 ms early.

Fix: work in seconds from `time_offset`, and use the contact of the current
interval:

```diff
-int tick_ = static_cast<int>(elapsed_ns / 1'000'000);
-size_t target_id_a = tick_ / config_.ticks_per_sample;
+double t = elapsed_ns * 1e-9;
+// a = last point with time_offset <= t, b = a + 1
+double alpha = (t - ref_a.t) / (ref_b.t - ref_a.t);
 ...
-bool is_in_contact = ref_b.foot_state[f].contact;
+bool is_in_contact = ref_a.foot_state[f].contact;   // flag holds until the next point
```

### 7. Hard-coded stand target at the world origin

Default target in `update()`:

```cpp
default_target.q[2] = 0.3;                                       // Go2 height
default_target.foot_state[0].foot_pos_world << 0.19, 0.142, 0.0; // Go2 feet at (0, 0)
```

- Wrong for Gardo, which stands at about 0.43 m with different feet.
- If the MPC stopped after the robot had walked away, the fallback pulled it
  back towards (0, 0).

Fix: build it at the current pose, with height and feet from the URDF, and
rebuild it whenever the planner stops:

```diff
-default_target.q[2] = 0.3;
-default_target.foot_state[0].foot_pos_world << 0.19, 0.142, 0.0;  ...
+// base: current x, y, yaw; z = ground + nominal height from forward kinematics
+// feet: nominal stance from FK, rotated by yaw, placed on the ground
+default_target = robot.standTarget(current_base_pos, current_yaw, ground_z);
```

### 8. Old Python MPC (now in `legacy/`)

Only the first of these was fixed; the rest are documented in
`legacy/README.md`.

- **Fixed: time offsets counted impact nodes.**
  ```diff
  -sec_offset = int(i * self.dt)          # i counts dropped impact nodes too
  +t_offset += self.dt                    # count only published nodes
  ```
- **Not fixed: delay compensation thrown away.** The predicted state was
  computed and then overwritten on the next line
  (`predicted_state = self.current_state.copy()`). Fix: delete that line.
- **Not fixed: `RobotState.copy()` drops `dt`.** Impact nodes were never used,
  and the plan was integrated with 0.02 s steps but published 0.04 s apart.
  Fix: `return RobotState(..., dt=self.dt)`.

### 9. Build and configuration

| File | Bug | Fix |
|---|---|---|
| `hardware_unitree_sdk2/CMakeLists.txt` | `ARCHIVE DESTINATION ib` | `lib` |
| `gardo_description/launch/launch.py` | spawned `optimal_controller` and `gait_controller`, which do not exist in this repository | spawn `whole_body_controller` |
| `quadruped_teleop/teleop.py` | published `/cmd_vel`, the stack listens on `/vel_cmd` | `/vel_cmd` |
| `quadruped_teleop/package.xml` | imports `controller_manager_msgs` without declaring it | add `<depend>` |

## How it was verified

`whole_body_controller` has 30 tests that run on both robots' URDFs, most of
them closed-loop with a small rigid-contact simulator in which feet leave the
ground only when they stop pushing and land with an impact:

| Test | Go2 | Gardo |
|---|---|---|
| Stand still, push recovery (3 cm, 0.11 rad) | recovers to < 1 µm | recovers to < 1 µm |
| Walk forward (commanded 0.3 / 0.4 m/s) | 0.294 m/s | 0.403 m/s |
| Walk sideways (0.15 m/s) | 0.16 m/s | 0.18 m/s |
| Turn in place (0.6 rad/s) | 0.598 rad/s | 0.598 rad/s |
| Trot in place, stop after walking | stable, height within 0.5 mm | stable |

Not yet verified: Gazebo/Mujoco runs and hardware. The test world is flat
ground with ideal point contacts and no motor or sensor models, so expect
retuning (joint gains, gait timing) in simulation and on the robot.

## Tuning entry points

All parameters are documented in
`src/whole_body_controller/config/whole_body_params.yaml`. The ones most
likely to need tuning per robot:

- `joint_gains.*`: joint PD per stance/swing (start low on hardware)
- `gait.cycle_time`, `gait.step_height`, `foothold.max_offset`
- `mpc.state_weights`, `mpc.force_weight`
- `command.timeout`: set e.g. 0.5 s on hardware with a teleop that publishes
  continuously (`teleop` does; `keyboard_teleop` only publishes on key presses)
- `safety.damping_kd`, `safety.max_tilt`
