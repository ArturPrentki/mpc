# whole_body_controller

Robot-agnostic whole-body controller for legged robots, packaged as a
ros2_control controller (`whole_body_controller/WBC`).

Nothing about a specific robot is hard-coded: sizes, kinematics, mass and
torque limits come from the URDF (through Pinocchio), and names plus the
standing pose come from parameters. The same build runs Go2 (test platform)
and Gardo, and any other floating-base robot with point feet.

## Pipeline (1 kHz)

```
state interfaces ──► DynamicSystem ──► TrajectoryInterpolator ──► contact modes
 (estimator +         M, h, Jacobians,   MPC trajectory, or              │
  joints)             foot kinematics,   stand-in-place fallback          ▼
                      contact detection                              Wbic (WBIC)
                                                                          │
command interfaces ◄── joint PD (drive or software) ◄── SafetySupervisor ◄┘
```

* **Wbic**: strict task priorities resolved with dynamically consistent
  null-space projection: (0) stance feet fixed, (1) base orientation,
  (2) base position, (3) swing feet, (4) joint posture. A small QP then
  finds contact forces inside the friction pyramid and the torque limits, and
  relaxes the floating-base acceleration as little as possible (Kim et al.
  2019). Task gains are accelerations, so they carry over between robots of
  different mass.
* **Contact modes** combine the plan with detection: `STANCE` (planned and
  detected), `SWING` (planned swing), `LATE_TOUCHDOWN` (planned contact, not
  detected yet: the foot reaches down). A reaching foot that stalls is declared
  in contact, so a lightly loaded foot cannot get stuck.
* **TrajectoryInterpolator** uses each step's `time_offset`. When the planner
  is late it blends into standing over `fade_time`, at the robot's current
  pose (not where it was activated).
* **SafetySupervisor**: on a WBC failure it repeats the last good command for
  `safety.hold_time`, then latches joint damping. Excessive tilt or a
  non-finite state also latches damping. The controller never returns `ERROR`
  and never sends zero torque because the solver failed.

`wbc_core` (everything above except the ROS wrapper) does not depend on ROS.

## Conventions

* Joint arrays in `MpcTrajectory` / `RobotState` and all command/state
  interfaces use the **`joint_names` order**. Internally the controller uses
  Pinocchio's order and maps between the two (for Gardo they differ).
* Feet use the **`foot_frames` order**. `FootContactFlags` (fl, fr, rl, rr) is
  published every tick when there are 4 feet; the InEKF estimator uses it.
* Base pose in the world frame, base twist/acceleration in the local base
  frame (Pinocchio free-flyer convention), foot quantities in the world frame.

## Adding a robot

Add a `whole_body_controller` section to the robot's controllers yaml. See
`go2_description/config/go2_controllers.yaml` and
`gardo_description/config/gardo_controllers.yaml`. Required:

```yaml
base_frame: "TORSO"
joint_names: [...]               # every actuated joint
foot_frames: [...]               # contact frames
nominal_joint_positions: [...]   # standing pose, joint_names order
command_interfaces: [effort]     # add position, velocity, proportional,
                                 # derivative if the drives run the PD
```

All parameters, with defaults and descriptions, are in
`config/whole_body_params.yaml`. The controller refuses to configure, with a
message, if the URDF and the parameters do not match (unknown frame or joint,
a joint missing, no effort limit in the URDF, ...).

## Tests

`test/test_wbc_core.cpp` runs on both robots (URDFs in `test/data/`) and
closes the loop with a small rigid-contact simulator:

* joint order mapping, config validation, stand pose IK
* standing still with forces summing to the weight
* recovering from a 3 cm / 0.11 rad base perturbation
* lifting one foot 6 cm on three legs and touching down again
* safety hold → damping latch, tilt detection
* trajectory timing, fading and expiry

```bash
colcon build --packages-select whole_body_controller
colcon test --packages-select whole_body_controller --event-handlers console_direct+
```

`test/data/gardo.urdf` is generated from `gardo_description` with xacro;
regenerate it when the robot description changes.
