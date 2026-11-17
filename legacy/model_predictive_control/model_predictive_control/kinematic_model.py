import numpy as np
import pinocchio as pin
import math
from dataclasses import dataclass, field
from wbc_control_msgs.msg import MpcStateStep, FootState

from wbc_control_msgs.msg import RobotState as RobotStateMsg

from geometry_msgs.msg import Pose, Twist, Accel, Point, Vector3
import tf2_ros


@dataclass
class VelocityCommand:
    """Stores high-level velocity commands sent from teleop/planner."""

    v_x: float = 0.0
    v_y: float = 0.0
    v_z: float = 0.0
    w_yaw: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.v_x == 0.0 and self.v_y == 0.0 and self.w_yaw == 0.0 and False

    @property
    def arr(self) -> np.array:
        return np.array([self.v_x, self.v_y, self.w_yaw])

    @property
    def planar(self) -> np.array:
        return np.array([self.v_x, self.v_y])

    @property
    def linear(self) -> np.ndarray:
        return np.array([self.v_x, self.v_y, self.v_z])


@dataclass
class Base:
    # Base Velocities (body aligned as used by estimation)
    pose: np.ndarray = field(default_factory=lambda: np.zeros(7))
    twist: np.ndarray = field(default_factory=lambda: np.zeros(6))
    accel: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def from_msg(self, pose: Pose, twist: Twist, accel: Accel):
        self.pose[0] = pose.position.x
        self.pose[1] = pose.position.y
        self.pose[2] = pose.position.z

        self.pose[3] = pose.orientation.x
        self.pose[4] = pose.orientation.y
        self.pose[5] = pose.orientation.z
        self.pose[6] = pose.orientation.w

        self.twist[0] = twist.linear.x
        self.twist[1] = twist.linear.y
        self.twist[2] = twist.linear.z

        self.twist[3] = twist.angular.x
        self.twist[4] = twist.angular.y
        self.twist[5] = twist.angular.z

        self.accel[0] = accel.linear.x
        self.accel[1] = accel.linear.y
        self.accel[2] = accel.linear.z

        self.accel[3] = accel.angular.x
        self.accel[4] = accel.angular.y
        self.accel[5] = accel.angular.z

    def as_msg(self) -> tuple[Pose, Twist, Accel]:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.pos
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.quat
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = self.vel_lin
        twist.angular.x, twist.angular.y, twist.angular.z = self.vel_ang
        accel = Accel()
        accel.linear.x, accel.linear.y, accel.linear.z = self.acc_lin
        accel.angular.x, accel.angular.y, accel.angular.z = self.acc_ang
        return pose, twist, accel

    def copy(self) -> "RobotState":
        """Returns a deep copy of the robot state."""
        return Base(
            pose=self.pose.copy(), twist=self.twist.copy(), accel=self.accel.copy()
        )

    @property
    def pos(self) -> np.ndarray:
        return self.pose[:3]

    @property
    def quat(self) -> np.ndarray:
        return self.pose[3:]

    @property
    def vel_lin(self) -> np.ndarray:
        return self.twist[:3]

    @property
    def vel_ang(self) -> np.ndarray:
        return self.twist[3:]

    @property
    def acc_lin(self) -> np.ndarray:
        return self.accel[:3]

    @property
    def acc_ang(self) -> np.ndarray:
        return self.accel[3:]

    """base position integration"""

    def forward(self, cmd: VelocityCommand, dt: float, nominal_z: float = 0.3) -> None:
        # 1. Smooth velocity commands
        tau_linear = 0.25
        tau_angular = 0.5
        alpha_lin = min(dt / tau_linear, 1.0)
        alpha_ang = min(dt / tau_angular, 1.0)

        self.vel_lin[:] += (cmd.linear - self.vel_lin) * alpha_lin
        target_vel_ang = np.array([0.0, 0.0, float(cmd.w_yaw)])
        self.vel_ang[:] += (target_vel_ang - self.vel_ang) * alpha_ang

        # 2. Extract current Rotation Matrix and Euler Angles using Pinocchio
        # Pinocchio's Quaternion expects [x, y, z, w] which matches your setup
        quat = pin.Quaternion(self.quat)
        R = quat.matrix()

        # matrixToRpy returns [roll, pitch, yaw] using the standard convention
        roll, pitch, yaw = pin.rpy.matrixToRpy(R)
        self.yaw = yaw

        # 3. KINEMATICALLY CONSISTENT POSTURE DECAY
        # Proportional gain to recover posture (kp = 2.5 roughly matches tau = 0.4)
        kp_posture = 2.5

        # Inject restorative velocities directly into the tracked velocity state
        self.vel_lin[2] += kp_posture * (nominal_z - self.pos[2]) * dt
        self.vel_ang[0] += kp_posture * (0.0 - roll) * dt
        self.vel_ang[1] += kp_posture * (0.0 - pitch) * dt

        # 4. Integrate Position (Assuming vel_lin is in the local base frame)
        self.pos[:] += R @ self.vel_lin * dt

        # 5. Integrate Quaternion using Pinocchio's exponential map (exp3)
        # exp3 accurately maps an angular velocity vector to a delta Rotation Matrix
        dR = pin.exp3(self.vel_ang * dt)
        new_R = R @ dR

        # Convert back to quaternion, normalize, and save
        new_quat = pin.Quaternion(new_R)
        new_quat.normalize()
        self.quat[:] = new_quat.coeffs()  # coeffs() outputs [x, y, z, w]


@dataclass
class Foot:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))
    contact_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    is_contact: bool = field(default=False)

    def from_msg(self, fs: FootState):
        self.position[0] = fs.position.x
        self.position[1] = fs.position.y
        self.position[2] = fs.position.z

        self.velocity[0] = fs.velocity.x
        self.velocity[1] = fs.velocity.y
        self.velocity[2] = fs.velocity.z

        self.acceleration[0] = fs.acceleration.x
        self.acceleration[1] = fs.acceleration.y
        self.acceleration[2] = fs.acceleration.z

        self.contact_force[0] = fs.contact_force.x
        self.contact_force[1] = fs.contact_force.y
        self.contact_force[2] = fs.contact_force.z
        self.is_contact = fs.is_contact

    @property
    def as_msg(self) -> FootState:
        fs = FootState()
        fs.position.x, fs.position.y, fs.position.z = self.position.copy()
        fs.velocity.x, fs.velocity.y, fs.velocity.z = self.velocity.copy()
        fs.acceleration.x, fs.acceleration.y, fs.acceleration.z = self.acceleration.copy()
        fs.contact_force.x, fs.contact_force.y, fs.contact_force.z = self.contact_force.copy()
        fs.is_contact = self.is_contact

        return fs

    def copy(self) -> "Foot":
        return Foot(
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            acceleration=self.acceleration.copy(),
            contact_force=self.contact_force.copy(),
            is_contact=self.is_contact,
        )


@dataclass
class RobotState:
    """Encapsulates full floating-base + joint state for estimation and MPC."""

    # Base Position & Orientation
    bs: Base = field(default_factory=lambda: Base())
    fs: list[Foot] = field(default_factory=lambda: [Foot(), Foot(), Foot(), Foot()])
    # Joint States
    q_joints: np.ndarray = field(default_factory=lambda: np.zeros(12))
    v_joints: np.ndarray = field(default_factory=lambda: np.zeros(12))
    a_joints: np.ndarray = field(default_factory=lambda: np.zeros(12))
    dt: float = 0.02

    @property
    def q(self) -> np.ndarray:
        return np.concatenate([self.bs.pose, self.q_joints])

    @property
    def v(self) -> np.ndarray:
        return np.concatenate([self.bs.twist, self.v_joints])

    @property
    def a(self) -> np.ndarray:
        return np.concatenate([self.bs.accel, self.a_joints])

    @property
    def as_crocoddyl(self) -> np.ndarray:
        """Packages full state vector x0 = [q, v] (dimension nq + nv)."""
        return np.concatenate([self.q, self.v])

    def copy(self) -> "RobotState":
        """Returns a deep copy of the robot state."""
        return RobotState(
            bs=self.bs.copy(),
            fs=[foot.copy() for foot in self.fs],
            q_joints=self.q_joints.copy(),
            v_joints=self.v_joints.copy(),
            a_joints=self.a_joints.copy(),
        )

    def from_crocoddyl(self, x_vec: np.ndarray) -> None:
        """
        Unpacks a Crocoddyl state vector x = [q, v] back into the RobotState attributes.
        Doesn't call forward kinematics. foot state is not affected
        """
        n_joints = len(self.q_joints)
        nq = 7 + n_joints  # 3 pos + 4 quat + n_joints

        # Unpack configuration vector q
        q = x_vec[:nq]
        self.bs.pose = q[:7].copy()
        self.q_joints = q[7:].copy()

        v = x_vec[nq:]
        self.bs.twist = v[:6].copy()
        self.v_joints = v[6:].copy()

    def from_msg(self, msg: RobotStateMsg):
        self.bs.from_msg(msg.body_pose, msg.body_velocity, msg.body_acceleration)
        for foot_id, foot_msg in enumerate(msg.feet):
            self.fs[foot_id].from_msg(foot_msg)
            self.q_joints[:] = msg.joint_positions
            self.v_joints[:] = msg.joint_velocities
            self.a_joints[:] = msg.joint_accelerations

    @property
    def as_msg(self) -> MpcStateStep:
        msg = MpcStateStep()
        msg.joint_positions = self.q_joints
        msg.joint_velocities = self.v_joints
        msg.joint_accelerations = self.a_joints
        msg.body_pose, msg.body_velocity, msg.body_acceleration = self.bs.as_msg()
        for foot in self.fs:
            msg.feet.append(foot.as_msg)
        return msg

    def predict_forward(
        self, rmodel: pin.Model, dt: float, last_prediction: "RobotState" = None
    ) -> "RobotState":
        """
        Extrapolates the state forward in time by dt using current or last predicted velocities.
        This compensates for MPC computation delay.
        """
        pred = self.copy()
        v_used = self.v if last_prediction is None else last_prediction.v

        # 1. Integrate configuration by dt (CRITICAL: must multiply by dt)
        pred_q = pin.integrate(rmodel, self.q, v_used * dt)
        pred.bs.pose = pred_q[:7]
        pred.q_joints = pred_q[7:]

        pred.bs.twist = v_used[:6]
        pred.v_joints = v_used[6:]

        # 2. Extrapolate swing foot positions in task space
        for idx, foot in enumerate(pred.fs):
            if not foot.is_contact:
                foot_vel = (
                    foot.velocity
                    if last_prediction is None
                    else last_prediction.fs[idx].velocity
                )
                foot.position += foot_vel * dt
                foot.velocity = foot_vel.copy()

        return pred

class Model:
    """Handles Pinocchio model creation, joint state mapping, and frame kinematics."""

    def __init__(self, urdf: str = None, foot_names=None, hip_names=None):
        self.foot_names = foot_names or ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        self.hip_names = hip_names or ["FL_hip", "FR_hip", "RL_hip", "RR_hip"]
        self.foot_nom = None
        self.rmodel = None
        self.rdata = None
        self.q_nominal = None
        self.is_loaded = False
        self.pin_foot_ids = None
        self.pin_hip_ids = None

        self.foot_range = range(len(self.foot_names))
        if urdf is not None:
            self.load_from_xml(urdf)

    @property
    def joint_names(self):
        """Returns the names of the actuated joints (skipping universe and root joint)."""
        if self.rmodel is None:
            return []
        return list(self.rmodel.names)[2:]

    @property
    def total_mass(self) -> float:
        """Returns the sum of all link masses in the robot model."""
        return sum([i.mass for i in self.rmodel.inertias]) if self.rmodel else 0.0

    def inspect_pinocchio_ordering(self):
        print("=== 1. PINOCCHIO JOINT ORDERING ===")
        # Joint 0 is 'universe', Joint 1 is typically 'root_joint' (floating base)
        actuated_joint_names = [
            name for name in self.rmodel.names if name not in ["universe", "root_joint"]
        ]

        for idx, name in enumerate(actuated_joint_names):
            print(f"Joint index {idx:2d} (q[{7+idx}]): {name}")

        print("\n=== 2. PROJECT VS PINOCCHIO FOOT FRAME IDs ===")
        # Replace these strings with your exact URDF foot frame names if different
        expected_feet = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

        for foot_name in expected_feet:
            if self.rmodel.existFrame(foot_name):
                frame_id = self.rmodel.getFrameId(foot_name)
                print(f"{foot_name:10s} -> Frame ID: {frame_id}")
            else:
                print(f"WARNING: {foot_name} not found in model frames!")

        print("\n=== 3. ALL MODEL FRAMES (Operational Space) ===")
        for i, frame in enumerate(self.rmodel.frames):
            if frame.type in [pin.FrameType.BODY, pin.FrameType.OP_FRAME]:
                print(f"Frame {i:2d}: {frame.name}")

    def compute_dynamic_force(self, num_stance_legs: int) -> pin.Force:
        """Computes reference vertical ground reaction force per stance leg."""
        fz = (self.total_mass * 9.81) / max(1, num_stance_legs)
        return pin.Force(np.array([0.0, 0.0, fz]), np.zeros(3))

    def load_from_xml(self, urdf_xml: str) -> bool:
        """Builds Pinocchio model from URDF XML and sets up nominal posture/height."""
        try:
            self.rmodel = pin.buildModelFromXML(urdf_xml, pin.JointModelFreeFlyer())
            self.rdata = self.rmodel.createData()
            self._init_nominal_joints()
            self.is_loaded = True
            return True
        except Exception as e:
            print(f"[RobotDescription] Failed to build Pinocchio model: {e}")
            return False

    def _init_nominal_joints(self):
        default_angles = {"hip": 0.0, "thigh": 0.8, "calf": -1.5}
        q_nom = []
        for i in range(2, self.rmodel.njoints):
            jname = self.rmodel.names[i].lower()
            val = 0.0
            for key, angle in default_angles.items():
                if key in jname:
                    val = angle
                    break
            q_nom.append(val)

        self.q_nominal = np.array(q_nom)

        # Compute exact standing height from nominal posture
        q_test = np.concatenate([[0, 0, 0], [0, 0, 0, 1], self.q_nominal])
        pin.forwardKinematics(self.rmodel, self.rdata, q_test)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        self.pin_foot_ids = [self.rmodel.getFrameId(f) for f in self.foot_names]
        self.pin_hip_ids = [self.rmodel.getFrameId(f) for f in self.hip_names]

        self.foot_nom = [self.rdata.oMf[foot].translation for foot in self.pin_foot_ids]
        self.z_height = -self.foot_nom[0][2]
        print(f"Nominal foot positions: {self.foot_nom}\n height: {self.z_height}")

    def map_joints(self, joint_pos_dict: dict, joint_vel_dict: dict):
        """Maps incoming ROS joint dictionaries to Pinocchio order."""
        q_mapped, v_mapped = [], []
        for i in range(2, self.rmodel.njoints):
            name = self.rmodel.names[i]
            q_mapped.append(joint_pos_dict[name])
            v_mapped.append(joint_vel_dict[name])
        return np.array(q_mapped), np.array(v_mapped)

    def compute_kinematics(self, state: RobotState):
        """Computes forward kinematics and current foot positions."""
        # q0 = np.concatenate([state.q, state.bs.quat, state.q_joints])
        # v0 = np.concatenate([state.bs.vel, state.bs.w_ang, state.v_joints])

        pin.forwardKinematics(self.rmodel, self.rdata, state.q, state.v)
        pin.computeAllTerms(self.rmodel, self.rdata, state.q, state.v)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        pin.centerOfMass(self.rmodel, self.rdata, state.q, False)
        # for i, foot in enumerate(state.fs):
        #     foot.position = self.rdata.oMf[self.pin_foot_ids[i]].translation.copy()
        # foot.velocity = ...

    @property
    def com(self):
        return self.rdata.com[0]

    def compute_inverse_kinematics(
        self, base: Base, foot_positions_world: list, foot_velocities_world: list
    ) -> np.ndarray:
        """
        Computes Analytical Inverse Kinematics for 4 quadruped legs.
        base_quat expected in [x, y, z, w] format.
        foot_positions_world: list of 4 np.arrays (3D) in the world/odom frame.
        """
        q_joints = np.zeros(12)
        v_joints = np.zeros(12)
        # Base pose in world frame
        R_base = pin.Quaternion(base.quat).toRotationMatrix()
        base_to_odom = pin.SE3(R_base, base.pos)

        for i, foot_id in enumerate(self.pin_foot_ids):
            # 1. Dynamically trace the kinematic tree: Foot -> Calf -> Thigh -> Hip
            calf_joint = self.rmodel.frames[foot_id].parentJoint
            thigh_joint = self.rmodel.parents[calf_joint]
            hip_joint = self.rmodel.parents[thigh_joint]

            # 2. Extract exact link lengths from Pinocchio model
            hip_to_thigh_trans = self.rmodel.jointPlacements[thigh_joint].translation
            mul = 1.0 if hip_to_thigh_trans[1] > 0 else -1.0
            l_hip = np.linalg.norm(hip_to_thigh_trans) * mul

            l_thigh = np.linalg.norm(
                self.rmodel.jointPlacements[calf_joint].translation
            )
            l_calf = np.linalg.norm(self.rmodel.frames[foot_id].placement.translation)

            # 3. Position IK
            base_to_hip = self.rmodel.jointPlacements[hip_joint]
            hip_to_odom = base_to_odom * base_to_hip

            # Transform target foot position from world down to local hip frame
            foot_in_hip = hip_to_odom.actInv(
                foot_positions_world[i].reshape(3, 1)
            ).flatten()
            x, y, z = foot_in_hip[0], foot_in_hip[1], foot_in_hip[2]

            L2 = x**2 + y**2 + z**2 - l_hip**2
            L = math.sqrt(max(0.0, L2))

            q1 = math.atan2(y, -z) - math.atan2(l_hip, L)

            cos_q3 = (L2 - l_thigh**2 - l_calf**2) / (2.0 * l_thigh * l_calf)
            cos_q3 = np.clip(cos_q3, -1.0, 1.0)
            q3 = -math.acos(cos_q3)

            q2 = math.atan2(-x, L) - math.atan2(
                l_calf * math.sin(q3), l_thigh + l_calf * math.cos(q3)
            )

            q_joints[i * 3 : i * 3 + 3] = [q1, q2, q3]

            r_foot_world = foot_positions_world[i] - base.pos
            v_foot_rel_world = (
                foot_velocities_world[i]
                - base.vel_lin
                - np.cross(base.vel_ang, r_foot_world)
            )

            # Rotate into local hip frame
            v_hip = hip_to_odom.rotation.T @ v_foot_rel_world

            s1, c1 = math.sin(q1), math.cos(q1)
            s2, c2 = math.sin(q2), math.cos(q2)
            s23, c23 = math.sin(q2 + q3), math.cos(q2 + q3)

            J_leg = np.zeros((3, 3))
            J_leg[0, 0] = 0.0
            J_leg[1, 0] = -l_hip * s1 + c1 * (l_thigh * c2 + l_calf * c23)
            J_leg[2, 0] = l_hip * c1 + s1 * (l_thigh * c2 + l_calf * c23)

            J_leg[0, 1] = -l_thigh * c2 - l_calf * c23
            J_leg[1, 1] = -s1 * (l_thigh * s2 + l_calf * s23)
            J_leg[2, 1] = c1 * (l_thigh * s2 + l_calf * s23)

            J_leg[0, 2] = -l_calf * c23
            J_leg[1, 2] = -l_calf * s1 * s23
            J_leg[2, 2] = l_calf * c1 * s23

            # Use pseudo-inverse for robust solving near singularities (fully extended leg)
            qdot = np.linalg.pinv(J_leg) @ v_hip
            v_joints[i * 3 : i * 3 + 3] = qdot

        return q_joints, v_joints
