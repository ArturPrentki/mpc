# Locomotion rework: what changed and why

Three steps, each its own pull request:

1. A new whole-body controller (WBC) that works on any robot (PR #1)
2. A built-in convex MPC that plans the walking (PR #2)
3. Cleanup: old planners moved to `legacy/`, dead files and build outputs removed (PR #3)

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
