# Locomotion rework: what changed and why

Three steps, each its own pull request:

1. A new whole-body controller (WBC) that works on any robot (PR #1)
2. A built-in convex MPC that plans the walking (PR #2)
3. Cleanup: old planners moved to `legacy/`, dead files and build outputs removed (PR #3)

Each bug fixed along the way is shown with the old code and the fix in
[Bug fixes in detail](#bug-fixes-in-detail).

## Starting point: what was wrong with the old stack

The old pipeline was a **Python full-body MPC** (Crocoddyl) publishing
trajectories on `/mpc_trajectory` at 10 Hz, tracked by a **WBC** in a
ros2_control controller at 1 kHz.

- **The MPC was too slow and too weak.** At 10 Hz the robot ran on plans up to
  100 ms old, and friction and joint limits were only penalties, so the plan
  could ask for forces the ground cannot give. It also had timing bugs: plans
  were integrated with 0.02 s steps but published as if 0.04 s apart, impact
  nodes were never active, and the delay compensation was overwritten.
- **The WBC mixed all goals in one weighted problem.** Body, swing-foot and
  joint goals fought each other; the joint PD pulled stance legs towards zero
  velocity against the body task.
- **Contact detection read past the end of an array** (garbage memory).
- **Solver failures stopped the robot unsafely:** the controller returned
  `ERROR` (ros2_control deactivates it) or sent zero torque.
- **Everything was Go2-specific:** frame and joint names, the standing pose,
  foot positions and the 0.3 m height were hard-coded, and the joint order was
  assumed to match Pinocchio's. On Gardo that order differs, so its legs would
  have been mixed up.

## Step 1: new whole-body controller (PR #1)

The WBC was rewritten as a ROS-independent core (`wbc_core`) wrapped by a thin
ros2_control plugin.

1. **Robot description from the URDF.** Mass, inertia, kinematics and torque
   limits come from the URDF through Pinocchio. Names and the standing pose come
   from the robot's YAML (`joint_names`, `foot_frames`, `base_frame`,
   `nominal_joint_positions`). The joint order is mapped explicitly, and the
   controller refuses to start with a clear message if the URDF and the YAML
   disagree.
2. **Strict task priorities (WBIC).** Instead of one weighted blend, goals are
   solved in order: stance feet stay put → body orientation → body position →
   swing feet → joint posture. A lower goal can only use the freedom left by
   the higher ones. A small QP then picks contact forces within the friction
   pyramid and the joint torque limits.
3. **Contact handling.** Planned contacts are combined with detection from
   joint torques (the out-of-bounds read is fixed). A foot that should be down
   but is not reaches down; if it is blocked it counts as landed. Contacts are
   assumed for 0.2 s after activation so the stance cannot drop at start-up.
4. **Joint PD that helps instead of fighting.** Stance-leg joint targets follow
   the WBC's own solution one step ahead; swing-leg targets come from leg IK of
   the foot target. On Go2 the old damping alone produced roll-acceleration
   errors of about 6 rad/s².
5. **Safety supervisor.** If the solver fails, the last good command is
   repeated for 50 ms, then the joints are damped (latched). Tilting over also
   damps. The controller never returns `ERROR` and never sends zero torque
   because of a solver failure.
6. **Trajectory timing.** Trajectories are interpolated on their timestamps.
   If the planner stops, the robot settles into a stand where it is (not at
   the origin).
7. **Two drive modes.** If the drives take gains (Go2: position, velocity,
   proportional, derivative), the joint PD runs in the drives; if they take
   effort only (Gardo in Gazebo), it runs inside the controller.

Bug fix in the old MPC node: published time offsets now skip impact nodes.

## Step 2: built-in convex MPC (PR #2)

The Python MPC is replaced by a C++ planner running in its own 50 Hz thread
inside the controller, so no ROS messages sit between MPC and WBC. Input is a
velocity command (`geometry_msgs/Twist` on `/vel_cmd`).

1. **Command ramp:** velocity commands are limited and ramped (acceleration
   limits) so steps in the command do not jolt the robot.
2. **Gait scheduler:** periodic gait from cycle time, duty factor and per-foot
   phase (trot by default). It starts when the command is non-zero, with every
   foot in stance until its first liftoff. It stops 0.5 s after the command
   goes to zero, letting feet already in the air land first.
3. **Footholds:** under the hip at touchdown, plus half a stance of travel,
   plus capture-point feedback on the velocity error, plus a centrifugal term
   when turning, limited to `foothold.max_offset`. Assumes flat ground at the
   height of the stance feet.
4. **Swing trajectories:** minimum-jerk curves from liftoff to foothold with a
   smooth apex at `gait.step_height`, replanned every cycle.
5. **Convex MPC:** the robot is modelled as one rigid body (mass, inertia and
   centre of mass from the URDF at the standing pose). The horizon (14 steps of
   30 ms) is one quadratic program solved with OSQP, with hard friction-pyramid
   and normal-force limits and zero force on swing feet. Forces are regularized
   towards an equal share of the weight and normalized by `m g`, so the same
   weights work on the 16 kg Go2 and the 40 kg Gardo. One update takes
   2.6–4.8 ms on average, under 10 ms worst case.
6. **Output to the WBC:** a dense trajectory (5 ms spacing, 0.25 s) with body
   pose and twist, foot positions/velocities/accelerations, contact flags and
   contact forces.

`mpc.enabled: false` makes the controller follow an external planner on
`/mpc_trajectory` again.

## Step 3: cleanup (PR #3)

- **Old planners moved to `legacy/`** (not built): `model_predictive_control`
  and `walk_pattern_generator`. See `legacy/README.md`.
- **Build outputs removed from git:** `build/`, `install/`, `log/` (about
  150 MB, tracked since the first commit) and the InEKF files the estimator
  build writes into its source folder. A `.gitignore` keeps them out.
- **Gardo launch fixed:** it spawned `optimal_controller` and `gait_controller`,
  which do not exist in this repository; it now loads `whole_body_controller`
  after stand-up, like Go2. Their entries were removed from
  `gardo_controllers.yaml`.
- **Teleop updated:** `teleop` publishes on `/vel_cmd` (it used `/cmd_vel`),
  switches `stand_up` → `whole_body_controller` (it switched the two missing
  controllers), and its height/roll/pitch keys were removed (the planner takes
  forward, sideways and turn). Its missing `controller_manager_msgs`
  dependency was added.
- **Dead or duplicate files deleted:** a broken Gardo URDF copy in both
  description packages, a copy of the Gardo xacro and an old Gardo launch file
  inside `go2_description`, `go2_controllers copy.yaml`, `launch_old.py`, and a
  hardware launch file that pointed to a config that does not exist.
- **Fixes:** `hardware_unitree_sdk2` installed its archive to `ib` instead of
  `lib`; the Mujoco launch no longer starts the "ghost" robot that only showed
  the old MPC's plan; commented-out code and unused functions were removed.

## Bug fixes in detail

Each bug in the old code as **old code → what went wrong → minimal fix in the
old code's terms**. The new code fixes them the same way, just restructured.
Bugs 1–7 made the old stack unusable on Gardo and fragile on Go2: bugs 2 and 7
alone broke Gardo, and bugs 4 and 5 made any solver hiccup drop the robot.

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
