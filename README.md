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

Two steps: switch off the Go2's built-in sport mode, then run the same launch
as in Mujoco (it stands the robot up). The launch talks to the robot through
the Unitree SDK2 (DDS domain 0 on the Ethernet port wired to the Go2) and
refuses to start while sport mode is still on, so Unitree's controller and
ours never fight over the motors. `config/go2_controllers_real.yaml` overrides
a few settings for hardware (IMU calibration at start, command timeout, lower
speed limits).

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

**2. Switch off sport mode.** Put the robot on a stand or lay it down with the
remote first: the motors go limp when sport mode is released, so a standing
Go2 drops.

```bash
ros2 run hardware_unitree_sdk2 release_sport_mode enx08beac225241
```

It retries for up to about 25 s and exits once the robot reports
`Sport mode is off`. Wait for the legs to be fully limp before the next step.
Sport mode stays off until the robot is restarted.

**3. Launch**, with the robot still for the first second (IMU calibration).
Keep the area clear: `stand_up` starts as soon as the controllers are up.

```bash
ros2 launch go2_description launch.mujoco.py real_robot:=true network_interface:=enx08beac225241
```

The log should show `Sport mode is off, low-level control enabled`. If sport
mode is still on, or the robot cannot be reached, the hardware refuses to
start and says why.

The port and mode can also come from the environment
(`UNITREE_NETWORK_INTERFACE=enx08beac225241`, `UNITREE_DOMAIN=0` selects the
real robot); launch arguments override them, `real_robot:=false` always runs
Mujoco on `lo`, and the launch prints which mode it picked.

**4. Walk** with the teleop (switches `stand_up` → `whole_body_controller`;
releasing the keys stops the robot, and the controller stops on its own if
commands stop for 0.5 s):

```bash
ros2 run quadruped_teleop teleop
```

Ctrl-C in the teleop switches back to `stand_up`. To stop the motors
completely, deactivate the controllers or stop the launch; the
`whole_body_controller` damps the joints when deactivated.
