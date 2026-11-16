import rclpy
from rclpy.node import Node
import numpy as np
import math

# ROS 2 Standard Messages
from geometry_msgs.msg import Twist, Pose, Vector3, Point, Accel
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# IMPORTANT: Import your custom messages here.
# We alias FootState to FootStateMsg to avoid colliding with your dataclass.
from wbc_control_msgs.msg import MpcTrajectory, MpcStateStep

from wbc_control_msgs.msg import FootState as FootStateMsg
from wbc_control_msgs.msg import RobotState as RobotStateMsg

# Import your classes here (assuming they are in a module named 'quadruped_control')
from model_predictive_control.kinematic_model import (
    VelocityCommand,
    Base,
    Foot,
    RobotState,
    Model,
)
from model_predictive_control.heuristic import (
    Heuristic,
    SingleLegCircleHeuristic,
    StandHeuristic,
)
from model_predictive_control.mpc import MPC
import os
import xacro
from ament_index_python.packages import get_package_share_directory


class MPCNode(Node):
    def __init__(self):
        super().__init__("heuristic_trajectory_generator")

        # --- Parameters ---
        self.declare_parameter("dt", 0.04)  # Time step for horizon
        self.declare_parameter("horizon_length", 30)
        self.declare_parameter("control_rate", 10)  # Hz to run the loop

        self.dt = self.get_parameter("dt").get_parameter_value().double_value
        self.horizon_length = (
            self.get_parameter("horizon_length").get_parameter_value().integer_value
        )
        self.control_rate = (
            self.get_parameter("control_rate").get_parameter_value().integer_value
        )

        self.declare_parameter(
            "cycle_time", 0.60
        )  # Standard trot cycle is usually 0.5s to 0.7s
        self.declare_parameter("duty_factor", 0.50)  # 60% stance, 40% swing

        cycle_time = self.get_parameter("cycle_time").get_parameter_value().double_value
        duty_factor = (
            self.get_parameter("duty_factor").get_parameter_value().double_value
        )

        # --- Initialize Data Structures ---
        self.cmd = VelocityCommand()
        self.current_state = RobotState()
        self.state_received = False

        # --- Initialize Kinematic Model ---
        self.model = Model()

        try:
            # 1. Locate the package directory
            pkg_path = get_package_share_directory("go2_description")
            xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")

            # 2. Process the xacro file into an XML DOM
            self.get_logger().info(f"Parsing xacro file: {xacro_file}")
            doc = xacro.process_file(xacro_file)

            # 3. Convert DOM to an XML string
            urdf_xml_string = doc.toprettyxml(indent="  ")

            # 4. Load the XML string into Pinocchio
            if not self.model.load_from_xml(urdf_xml_string):
                self.get_logger().error(
                    "Pinocchio failed to build model from parsed URDF."
                )
                return
            self.model.inspect_pinocchio_ordering()
            self.mpc = MPC(self.model, self.dt, self.horizon_length)
            self.get_logger().info("Pinocchio Model loaded successfully from Xacro.")

        except Exception as e:
            self.get_logger().error(f"Failed to parse xacro file: {e}")
            return

        self.single_leg_heuristic = SingleLegCircleHeuristic(
            nominal_positions=self.model.foot_nom
        )
        self.stand_heuristic = StandHeuristic()
        # --- ROS 2 Interfaces ---
        self.cmd_sub = self.create_subscription(Twist, "/vel_cmd", self.cmd_vel_cb, 10)
        self.state_sub = self.create_subscription(
            RobotStateMsg, "/robot_state", self.state_cb, 10
        )
        self.traj_pub = self.create_publisher(MpcTrajectory, "/mpc_trajectory", 10)
        self.mpc_feet_pub = self.create_publisher(MarkerArray, "ghost/foot_pos", 1)
        self.mpc_path_pub = self.create_publisher(Path, "ghost/mpc_path", 1)
        self.ghost_joint_pub = self.create_publisher(
            JointState, "ghost/joint_states", 1
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Control Loop ---
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info("Heuristic Trajectory Generator Initialized.")
        self.iters_run = 0

    def joints_cb(self, msg: JointState):
        if not self.model.is_loaded:
            return
        joint_pos_dict = dict(zip(msg.name, msg.position))
        joint_vel_dict = dict(zip(msg.name, msg.velocity))

        try:
            q_mapped, v_mapped = self.model.map_joints(joint_pos_dict, joint_vel_dict)
            self.current_state.q_joints = q_mapped
            self.current_state.v_joints = (
                0.8 * v_mapped + 0.2 * self.current_state.v_joints
            )
            # self.joints_received = True
        except KeyError:
            pass

    def cmd_vel_cb(self, msg: Twist):
        """Updates the target velocity command."""
        self.cmd.v_x = msg.linear.x
        self.cmd.v_y = msg.linear.y
        self.cmd.v_z = msg.linear.z
        self.cmd.w_yaw = msg.angular.z

    def state_cb(self, msg: RobotStateMsg):
        """Updates the current estimate of the robot state."""
        self.current_state.from_msg(msg)
        self.state_received = True

    def create_reference_trajectory(
        self, sim_state: RobotState, time: float, cmd: VelocityCommand
    ) -> list[RobotState]:
        reference_states = []
        init_samples = max(0, self.horizon_length - self.iters_run)

        # Track actual initial contacts without mutating sim_state
        prev_contacts = [f.is_contact for f in sim_state.fs]
        t_curr = time

        self.stand_heuristic.reset_horizon(t_curr, sim_state, cmd)
        heuristic = None
        while len(reference_states) < self.horizon_length:
            if len(reference_states) < init_samples:
                heuristic = self.stand_heuristic
                if any( not foot.is_contact for foot in sim_state.fs):
                    self.single_leg_heuristic.reset_horizon(t_curr, sim_state, cmd)
                    heuristic = self.single_leg_heuristic
            else:
                if not heuristic or heuristic is not self.single_leg_heuristic:
                    self.single_leg_heuristic.reset_horizon(t_curr, sim_state, cmd)
                heuristic = self.single_leg_heuristic
            t_curr += self.dt
            heuristic.update_phase(t_curr, sim_state, cmd)
            target_positions, target_velocities, contacts = heuristic.get_targets()
            reference_force = self.model.compute_dynamic_force(sum(contacts))

            # Detect any swing -> stance transition (touchdown)
            is_touchdown = any(
                should_contact and not was_contact
                for should_contact, was_contact in zip(contacts, prev_contacts)
            )

            # 1. Insert instantaneous impulse node (dt = 0.0) on touchdown
            if is_touchdown:
                impulse_state = sim_state.copy()
                impulse_state.dt = 0.0
                print(f"heuristic used: {type(heuristic)}")
                # exit()
                for idx, foot_state in enumerate(impulse_state.fs):
                    foot_state.is_contact = contacts[idx]
                    foot_state.position = target_positions[idx]
                    foot_state.velocity = (
                        np.zeros(3) if contacts[idx] else target_velocities[idx]
                    )
                    foot_state.acceleration = np.zeros(3)
                    foot_state.contact_force = np.zeros(3)

                reference_states.append(impulse_state)
                prev_contacts = list(contacts)

                # Stop if the impulse node filled the final slot of the horizon
                if len(reference_states) >= self.horizon_length:
                    break

            # 2. Insert standard continuous node (dt = self.dt)
            ref_state = sim_state.copy()
            ref_state.dt = self.dt
            for idx, foot_state in enumerate(ref_state.fs):
                foot_state.is_contact = contacts[idx]
                foot_state.position = target_positions[idx]
                foot_state.velocity = target_velocities[idx]
                foot_state.acceleration = np.zeros(3)
                foot_state.contact_force = (
                    reference_force.linear if contacts[idx] else np.zeros(3)
                )

            reference_states.append(ref_state)
            prev_contacts = list(contacts)

        return reference_states

    def control_loop(self):
        if not self.model.is_loaded or not self.state_received:
            return

        current_time_sec = self.get_clock().now().nanoseconds / 1e9

        delay_dt = 0.03
        if self.iters_run is not 0:
            last_opt = self.mpc.get_optimized_states()[1]
            predicted_state = self.current_state.predict_forward(
                self.model.rmodel, delay_dt, last_opt
            )
        else:
            predicted_state = self.current_state.copy()
        predicted_state = self.current_state.copy()
        self.model.compute_kinematics(predicted_state)
        predicted_time_sec = current_time_sec + delay_dt

        stamp = rclpy.time.Time(seconds=predicted_time_sec).to_msg()

        sim_state = predicted_state.copy()
        sim_state.q_joints = self.model.q_nominal.copy()
        sim_state.v_joints = np.zeros(12)

        reference_states = self.create_reference_trajectory(
            sim_state, predicted_time_sec, self.cmd
        )

        optimization_count = self.mpc.optimize_trajectory(
            predicted_state, reference_states
        )
        optimal_states = self.mpc.get_optimized_states()
        # if self.iters_run == 0:
        # first iteration. force zero delay
        # stamp = self.get_clock().now().to_msg()
        self._publish_trajectory(optimal_states, stamp)
        self._publish_mpc_foot_markers(optimal_states)
        self._publish_mpc_path(optimal_states, stamp)
        self._publish_terminal_ghost(optimal_states[-1])
        
        self.iters_run += 1
        print(
            f"{optimization_count} iterations, time: {self.get_clock().now().nanoseconds/1e9 - current_time_sec}"
        )

    def _publish_trajectory(self, trajectory: list[RobotState], stamp: Time):
        traj_msg = MpcTrajectory()
        traj_msg.header = Header()
        traj_msg.header.stamp = stamp
        traj_msg.header.frame_id = "odom"
        # Offsets count only continuous nodes: impulse nodes (dt == 0) take no
        # time and are not published. The WBC interpolates on these offsets.
        t_offset = 0.0
        for state in trajectory:
            if state.dt == 0.0:
                continue
            step_msg = state.as_msg
            sec_offset = int(t_offset)
            nanosec_offset = int(round((t_offset - sec_offset) * 1e9))
            step_msg.time_offset = Time(sec=sec_offset, nanosec=nanosec_offset)
            traj_msg.steps.append(step_msg)
            t_offset += self.dt

        self.traj_pub.publish(traj_msg)

    def _publish_mpc_foot_markers(self, trajectory: list[RobotState]):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        marker_id = 0
        for node_idx, state in enumerate(trajectory):
            # print(state.fs[0].position)

            for idx, foot in enumerate(state.fs):
                marker = Marker()
                marker.header.stamp = now
                marker.header.frame_id = "odom"
                marker.ns = f"mpc_foot_targets_{idx}"
                marker.id = marker_id
                marker_id += 1

                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = float(foot.position[0])
                marker.pose.position.y = float(foot.position[1])
                marker.pose.position.z = float(foot.position[2])
                marker.pose.orientation.w = 1.0

                # Make spheres smaller further down the horizon
                scale = max(0.005, 0.02 * (1.0 - node_idx / len(trajectory)))
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

                # Color code: Stance vs Swing or just distinct colors per foot
                if foot.is_contact:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.6  # Transparent

                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.6  # Transparent
                marker_array.markers.append(marker)

        self.mpc_feet_pub.publish(marker_array)

    def _publish_mpc_path(self, trajectory: list[RobotState], stamp):
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = "odom"
        for state in trajectory:
            q = state.q
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(q[0])
            pose.pose.position.y = float(q[1])
            pose.pose.position.z = float(q[2])
            pose.pose.orientation.x = float(q[3])
            pose.pose.orientation.y = float(q[4])
            pose.pose.orientation.z = float(q[5])
            pose.pose.orientation.w = float(q[6])

            path_msg.poses.append(pose)

        self.mpc_path_pub.publish(path_msg)

    def _publish_terminal_ghost(self, terminal_state: RobotState):

        # 1. Extract terminal configuration from the MPC solution
        q_term = terminal_state.q

        # Pinocchio floating base order: [x, y, z, qx, qy, qz, qw]
        base_pos = terminal_state.bs.pos
        base_quat = terminal_state.bs.quat
        joint_angles = terminal_state.q_joints

        now = self.get_clock().now().to_msg()

        # 2. Broadcast a TF frame for the ghost's base
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "ghost/base"

        t.transform.translation.x = float(base_pos[0])
        t.transform.translation.y = float(base_pos[1])
        t.transform.translation.z = float(base_pos[2])

        t.transform.rotation.x = float(base_quat[0])
        t.transform.rotation.y = float(base_quat[1])
        t.transform.rotation.z = float(base_quat[2])
        t.transform.rotation.w = float(base_quat[3])

        self.tf_broadcaster.sendTransform(t)

        # 3. Publish joint states for the ghost legs
        js = JointState()
        js.header.stamp = now
        js.name = (
            self.model.joint_names
        )  # Ensure your RobotDescription provides this list
        js.position = joint_angles.tolist()
        self.ghost_joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
