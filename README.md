# Legged locomotion stack (ROS 2 Jazzy)

Locomotion for quadrupeds with ros2_control: Unitree Go2 (test platform) and
Gardo (our robot). The same controller runs both; robot-specific settings live
in each robot's `config/*_controllers.yaml`.

```
/vel_cmd (Twist)
     │
     ▼
whole_body_controller  ── one ros2_control controller ──────────────────────
  │  50 Hz thread : gait ─► footholds ─► swing ─► convex MPC (single rigid body)
  │                                                    │ body + feet + forces
  │  1 kHz loop   : contact modes ─► prioritized WBC (WBIC) ─► safety ─► joint torques
  ▼
hardware (Unitree SDK2 / Mujoco / Gazebo)      state_estimator (InEKF / cheater)
```

## Packages (`src/`)

| Package | Purpose |
|---|---|
| `whole_body_controller` | Locomotion controller: convex MPC planner + prioritized whole-body control. See its README |
| `wbc_control_msgs` | Messages: robot/target state, foot contact flags, external trajectories |
| `quadruped_estimators` | Base state estimation (contact-aided InEKF, cheater estimator for simulation) |
| `stand_up_pkg` | Stand-up controllers used before walking |
| `hardware_unitree_sdk2` | ros2_control hardware interface for Unitree robots |
| `go2_description`, `gardo_description` | URDF/xacro, controller configs and launch files |
| `quadruped_teleop` | Keyboard teleops publishing `/vel_cmd` |

Old planners are kept in `legacy/` (not built).

## Build and test

```bash
colcon build
colcon test --packages-select whole_body_controller --event-handlers console_direct+
```

## Run

Launch a robot; it stands up, and `whole_body_controller` is loaded inactive:

```bash
ros2 launch go2_description launch.mujoco.py   # Go2, Mujoco / real robot
ros2 launch go2_description launch.py          # Go2, Gazebo (set command_interfaces: [effort])
ros2 launch gardo_description launch.py        # Gardo, Gazebo
```

Then start the teleop. It switches `stand_up` → `whole_body_controller` and
back on Ctrl-C; hold w/a/s/d/q/e to walk:

```bash
ros2 run quadruped_teleop teleop
```

## Real Go2

The same launch talks to the real robot through the Unitree SDK2 (DDS domain
0 on the Ethernet port wired to the Go2) and switches off the built-in sport
mode before sending any low-level command. `config/go2_controllers_real.yaml`
overrides a few settings for hardware (IMU calibration at start, command
timeout, lower speed limits).

**1. Network (once).** Connect the PC to the Go2 by Ethernet and give that
port a static address on the robot's subnet (the Go2 is `192.168.123.161`):

```bash
sudo ip addr add 192.168.123.99/24 dev enx08beac225241
ping 192.168.123.161
```

The port name depends on the PC: `ip a` lists them. USB Ethernet adapters
are named `enx<MAC>` (like `enx08beac225241` above), built-in ports usually
`enp…` or `eth0`. Use the same name in the steps below.

In Docker, run the container with `--network host` so DDS reaches the robot.

**2. Prepare the robot.** Hang it on a stand, or lay it down with the remote
first: releasing sport mode makes the motors go limp, so a standing Go2 drops.
Keep the area clear: the stand-up controller starts as soon as the launch is
up.

**3. Launch**, with the robot still for the first second (IMU calibration):

```bash
ros2 launch go2_description launch.mujoco.py real_robot:=true network_interface:=enx08beac225241
```

or set the port and domain once in the environment (e.g. the container's
config) and launch without arguments; `UNITREE_DOMAIN=0` selects the real
robot:

```bash
export UNITREE_NETWORK_INTERFACE=enx08beac225241
export UNITREE_DOMAIN=0
ros2 launch go2_description launch.mujoco.py
```

Launch arguments override the variables (`real_robot:=false` always runs
Mujoco on `lo`). The launch prints which mode it picked.

The log should show `Sport mode is off, low-level control enabled`. If the
motion switcher cannot be reached, the hardware refuses to start.

**4. Walk** with the teleop (switches `stand_up` → `whole_body_controller`;
releasing the keys stops the robot, and the controller stops on its own if
commands stop for 0.5 s):

```bash
ros2 run quadruped_teleop teleop
```

Ctrl-C in the teleop switches back to `stand_up`. To stop the motors
completely, deactivate the controllers or stop the launch; the
`whole_body_controller` damps the joints when deactivated.

Sport mode stays off until the robot is restarted (or re-enabled from the
Unitree app).

