import math
import numpy as np
import pinocchio as pin
import crocoddyl
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from wbc_control_msgs.msg import MpcTrajectory, MpcStateStep, FootState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class CrocoddylMPCNode(Node):
    def __init__(self):
        super().__init__('crocoddyl_mpc_node')
        
        # Model placeholders
        self.rmodel = None
        self.rdata = None
        self.robot_model_received = False
        
        # Foot mapping matching standard URDF frame names
        self.foot_names = ["FR_foot", "FL_foot", "RR_foot", "RL_foot"]
        self.foot_nom_x = [ 0.19,  0.19, -0.19, -0.19]
        self.foot_nom_y = [-0.142, 0.142, -0.142, 0.142]
        
        # Track ground stance anchors and liftoff positions
        self.last_in_stance = {foot: True for foot in self.foot_names}
        self.is_first_run = True
        self.liftoff_positions = {foot: np.zeros(3) for foot in self.foot_names}
        self.stance_anchor_positions = {foot: np.zeros(3) for foot in self.foot_names}
        
        # Gait Parameters
        self.z_height = 0.3
        self.gait_period = 0.35
        self.duty_factor = 0.5
        self.swing_height = 0.1
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0]
        self.k_vel_feedback = 0.05
        
        # State tracking & filtering
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_z = 0.3
        self.actual_yaw = 0.0
        self.base_quat = [0.0, 0.0, 0.0, 1.0]
        
        self.v_actual = np.zeros(3)
        self.w_actual = np.zeros(3)
        self.alpha_vel = 0.2

        self.v_cmd_x = 0.0
        self.v_cmd_y = 0.0
        self.w_cmd_yaw = 0.0
        
        self.q_nominal = None
        self.q_joints = None
        self.v_joints = np.zeros(12)
        
        self.start_time = None
        self.last_xs = None
        self.last_us = None
        self.odom_received = False
        self.joints_received = False
        
        self.dt = 0.040
        self.mpc_horizon = 20
        
        # Solver & Action Models persistent references 
        self.running_models = []
        self.terminal_model = None
        self.problem = None
        self.solver = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.sub_robot_desc = self.create_subscription(
            String, '/robot_description', self.robot_description_callback, latched_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_command = self.create_subscription(Twist, '/vel_cmd', self.command_callback, 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.trajectory_pub = self.create_publisher(MpcTrajectory, '/mpc_trajectory', 10)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Crocoddyl MPC Node Initialized. Waiting for /robot_description...")
        
    def robot_description_callback(self, msg):
        if not self.robot_model_received:
            try:
                self.rmodel = pin.buildModelFromXML(msg.data, pin.JointModelFreeFlyer())
                self.rdata = self.rmodel.createData()
                
                self._init_nominal_joints()
                self._init_crocoddyl_problem()
                
                self.robot_model_received = True
                self.get_logger().info(f"Loaded Pinocchio model! NV: {self.rmodel.nv}")
            except Exception as e:
                self.get_logger().error(f"Failed to build Pinocchio model: {e}")

    def _init_nominal_joints(self):
        default_angles = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5, 'knee': -1.5}
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
        self.q_joints = self.q_nominal.copy()

        # Calculate exact standing height corresponding to q_nominal
        q_test = np.concatenate([[0, 0, 0], [0, 0, 0, 1], self.q_nominal])
        pin.forwardKinematics(self.rmodel, self.rdata, q_test)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        
        foot_z = [self.rdata.oMf[self.rmodel.getFrameId(f)].translation[2] for f in self.foot_names]
        self.z_height = -float(np.mean(foot_z))

    def _create_action_model(self):
        state = crocoddyl.StateMultibody(self.rmodel)
        actuation = crocoddyl.ActuationModelFloatingBase(state)
        nu = actuation.nu

        stateWeights = np.concatenate([
            np.array([0.0, 0.0, 10.0]),                # X, Y low/zero, Z height moderate
            np.array([5.0, 5.0, 5.0]),               # Roll, Pitch, Yaw moderate
            np.full(self.rmodel.nv - 6, 0.01),         # Low joint posture weight
            np.array([10.0, 10.0, 5.0, 5.0, 5.0, 5.0]),# Base linear/angular velocity
            np.full(self.rmodel.nv - 6, 0.001)         # Joint velocity damping
        ])
        mu = 1.0
        Rsurf = np.eye(3)
        cone = crocoddyl.FrictionCone(Rsurf, mu, 4, False)
        coneActivation = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(cone.lb, cone.ub))

        contactModel = crocoddyl.ContactModelMultiple(state, nu)
        costModel = crocoddyl.CostModelSum(state, nu)

        x_ref = np.zeros(self.rmodel.nq + self.rmodel.nv)
        x_ref[2] = 0.3
        x_ref[6] = 1.0

        
        stateResidual = crocoddyl.ResidualModelState(state, x_ref, nu)
        stateActivation = crocoddyl.ActivationModelWeightedQuad(stateWeights)
        stateCost = crocoddyl.CostModelResidual(state, stateActivation, stateResidual)
        costModel.addCost("state_tracking", stateCost, 5.0)

        ctrlResidual = crocoddyl.ResidualModelControl(state, nu)
        ctrlCost = crocoddyl.CostModelResidual(state, ctrlResidual)
        costModel.addCost("ctrl_reg", ctrlCost, 0.1)

        # Calculate ideal resting weight per leg once
        robot_mass = sum([inertia.mass for inertia in self.rmodel.inertias])
        f_ideal = np.array([0.0, 0.0, (robot_mass * 9.81) / 4.0])
        f_ref = pin.Force(f_ideal, np.zeros(3))

        
        for foot in self.foot_names:
            frame_id = self.rmodel.getFrameId(foot)
            p_zero = np.zeros(3)
            
            contact3d = crocoddyl.ContactModel3D(state, frame_id, p_zero, pin.LOCAL_WORLD_ALIGNED, nu, np.array([10.0, 10.0]))
            contactModel.addContact(foot + "_contact", contact3d)

            coneResidual = crocoddyl.ResidualModelContactFrictionCone(state, frame_id, cone, nu)
            coneCost = crocoddyl.CostModelResidual(state, coneActivation, coneResidual)
            costModel.addCost(foot + "_friction", coneCost, 100.0)

            footResidual = crocoddyl.ResidualModelFrameTranslation(state, frame_id, p_zero, nu)
            footCost = crocoddyl.CostModelResidual(state, footResidual)
            costModel.addCost(foot + "_swing", footCost, 2000.0)
            # Create a force residual: penalize deviation from f_ideal
            
            forceResidual = crocoddyl.ResidualModelContactForce(state, frame_id, f_ref, 3, nu)
            forceCost = crocoddyl.CostModelResidual(state, forceResidual)
    
            # Add it to the model (Tune the weight! 0.1 to 1.0 is usually a good start)
            costModel.addCost(foot + "_force_reg", forceCost, 0.5)
    

        dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(
            state, actuation, contactModel, costModel, 0.0, True
        )
        return crocoddyl.IntegratedActionModelEuler(dmodel, self.dt)

    def _init_crocoddyl_problem(self):
        self.running_models = [self._create_action_model() for _ in range(self.mpc_horizon - 1)]
        self.terminal_model = self._create_action_model()
        
        x0 = np.zeros(self.rmodel.nq + self.rmodel.nv)
        self.problem = crocoddyl.ShootingProblem(x0, self.running_models, self.terminal_model)
        self.solver = crocoddyl.SolverFDDP(self.problem)

    def bezier_curve(self, p0, p1, p2, p3, t):
        return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

    def integrate_pose(self, x, y, yaw, vx, vy, omega, dt):
        if abs(omega) < 1e-3:
            nx = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            ny = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            nyaw = yaw
        else:
            nyaw = yaw + omega * dt
            nx = x + (vx * (math.sin(nyaw) - math.sin(yaw)) + vy * (math.cos(nyaw) - math.cos(yaw))) / omega
            ny = y + (vx * (math.cos(yaw) - math.cos(nyaw)) + vy * (math.sin(nyaw) - math.sin(yaw))) / omega
        return nx, ny, nyaw

    def odom_callback(self, msg):
        """ msg odometry comes in odom (world) frame """
        self.odom_received = True
        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        self.actual_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.base_quat = [q.x, q.y, q.z, q.w]
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.actual_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp_cosp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp_cosp) >= 1:
            self.actual_pitch = math.copysign(math.pi / 2, sinp_cosp)
        else:
            self.actual_pitch = math.asin(sinp_cosp)
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.actual_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        v_raw = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        w_raw = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        self.v_actual = (1.0 - self.alpha_vel) * self.v_actual + self.alpha_vel * v_raw
        self.w_actual = (1.0 - self.alpha_vel) * self.w_actual + self.alpha_vel * w_raw

    def joint_callback(self, msg):
        if self.rmodel is None:
            return
        joint_pos_dict = dict(zip(msg.name, msg.position))
        joint_vel_dict = dict(zip(msg.name, msg.velocity))
        
        try:
            q_mapped = []
            v_mapped = []
            for i in range(2, self.rmodel.njoints):
                name = self.rmodel.names[i]
                q_mapped.append(joint_pos_dict[name])
                v_mapped.append(joint_vel_dict[name])
                
            self.q_joints = np.array(q_mapped)
            v_new = np.array(v_mapped)
            self.v_joints = 0.3 * v_new + 0.7 * self.v_joints
            self.joints_received = True
        except KeyError:
            pass

    def command_callback(self, msg):
        self.v_cmd_x = msg.linear.x if abs(msg.linear.x) > 0.02 else 0.0
        self.v_cmd_y = msg.linear.y if abs(msg.linear.y) > 0.02 else 0.0
        self.w_cmd_yaw = msg.angular.z if abs(msg.angular.z) > 0.02 else 0.0

    def control_loop(self):
        if not self._is_ready_to_compute():
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            return 
        
        t_current_abs = (now - self.start_time).nanoseconds / 1e9
        
        x0, current_foot_pos, current_cycle_nums, is_leg_in_stance_t0 = self._prepare_current_state(t_current_abs)
        self._update_mpc_horizon(t_current_abs, current_foot_pos, current_cycle_nums, is_leg_in_stance_t0)
        self._solve_mpc(x0, t_current_abs)
        
        traj_msg = self._build_trajectory_msg()
        self.trajectory_pub.publish(traj_msg)
        print(f"Took {(now - self.get_clock().now()).nanoseconds/1.e9} seconds",end="\r")

    def _is_ready_to_compute(self):
        return self.robot_model_received and self.odom_received and self.joints_received

    def _prepare_current_state(self, t_current_abs):
        q0 = np.concatenate([
            [self.actual_x, self.actual_y, self.actual_z], 
            self.base_quat, 
            self.q_joints
        ])
        
        # 2. Read real body-frame velocities from your InEKF
        v0 = np.concatenate([
            self.v_actual, 
            self.w_actual, 
            self.v_joints
        ])
        x0 = np.concatenate([q0, v0])
        
        pin.forwardKinematics(self.rmodel, self.rdata, q0[:self.rmodel.nq], v0[:self.rmodel.nv])
        pin.updateFramePlacements(self.rmodel, self.rdata)
        
        current_foot_pos = {}
        current_cycle_nums = {}
        is_leg_in_stance_t0 = {}
        
        is_standing = (self.v_cmd_x == 0.0 and self.v_cmd_y == 0.0 and self.w_cmd_yaw == 0.0)
        active_duty = 1.0 if is_standing else self.duty_factor

        for i, foot in enumerate(self.foot_names):
            foot_id = self.rmodel.getFrameId(foot)
            pos = self.rdata.oMf[foot_id].translation.copy()
            current_foot_pos[foot] = pos
            
            phi_t0 = (t_current_abs / self.gait_period) + self.phase_offsets[i]
            current_cycle_nums[foot] = math.floor(phi_t0)
            phase_t0 = phi_t0 - current_cycle_nums[foot]
            in_stance = (phase_t0 < active_duty)
            is_leg_in_stance_t0[foot] = in_stance
            
            if self.is_first_run:
                self.stance_anchor_positions[foot] = pos.copy()
                self.liftoff_positions[foot] = pos.copy()
            elif in_stance and not self.last_in_stance[foot]:
                self.stance_anchor_positions[foot] = pos.copy()
            elif not in_stance and self.last_in_stance[foot]:
                self.liftoff_positions[foot] = pos.copy()
            
            self.last_in_stance[foot] = in_stance

        self.is_first_run = False
        return x0, current_foot_pos, current_cycle_nums, is_leg_in_stance_t0

    def _get_raibert_target(self, foot_idx, foot_name, c_num, is_touchdown, t_current_abs, current_cycle_nums, is_leg_in_stance_t0): 
        if c_num == current_cycle_nums[foot_name] and is_leg_in_stance_t0[foot_name]:
            return tuple(self.stance_anchor_positions[foot_name])
        
        T_stance = self.gait_period * self.duty_factor
        t_mid = (c_num - self.phase_offsets[foot_idx]) * self.gait_period + (0.5 * T_stance)
        dt_mid = t_mid - t_current_abs
        b_x_mid, b_y_mid, b_yaw_mid = self.integrate_pose(
            self.actual_x, self.actual_y, self.actual_yaw, self.v_cmd_x, self.v_cmd_y, self.w_cmd_yaw, dt_mid)
        if is_touchdown:
            v_foot_x = self.v_cmd_x - self.w_cmd_yaw * self.foot_nom_y[foot_idx]
            v_foot_y = self.v_cmd_y + self.w_cmd_yaw * self.foot_nom_x[foot_idx]
            r_x = self.foot_nom_x[foot_idx] + (0.5 * T_stance * v_foot_x) + self.k_vel_feedback * (self.v_actual[0] - self.v_cmd_x)
            r_y = self.foot_nom_y[foot_idx] + (0.5 * T_stance * v_foot_y) + self.k_vel_feedback * (self.v_actual[1] - self.v_cmd_y)
            if foot_idx in [0, 2]:
                r_y = min(-0.05, r_y)
            else:
                r_y = max(0.05, r_y)
        else:
            v_foot_x = self.v_cmd_x - self.w_cmd_yaw * self.foot_nom_y[foot_idx]
            v_foot_y = self.v_cmd_y + self.w_cmd_yaw * self.foot_nom_x[foot_idx]
            r_x = self.foot_nom_x[foot_idx] + (0.5 * T_stance * v_foot_x)
            r_y = self.foot_nom_y[foot_idx] + (0.5 * T_stance * v_foot_y)
        
        f_x = b_x_mid + (r_x * math.cos(b_yaw_mid) - r_y * math.sin(b_yaw_mid))
        f_y = b_y_mid + (r_x * math.sin(b_yaw_mid) + r_y * math.cos(b_yaw_mid))
        z_ground = float(self.stance_anchor_positions[foot_name][2])
        return (f_x, f_y, z_ground)

    def _update_mpc_horizon(self, t_current_abs, current_foot_pos, current_cycle_nums, is_leg_in_stance_t0):
        is_standing = (self.v_cmd_x == 0.0 and self.v_cmd_y == 0.0 and self.w_cmd_yaw == 0.0)
        active_duty = 1.0 if is_standing else self.duty_factor
        all_models = self.running_models + [self.terminal_model]

        for k, model in enumerate(all_models):
            dt_k = k * self.dt
            t_k = t_current_abs + dt_k
            
            base_x_k, base_y_k, base_yaw_k = self.integrate_pose(
                self.actual_x, self.actual_y, self.actual_yaw, self.v_cmd_x, self.v_cmd_y, self.w_cmd_yaw, dt_k)
            
            ref_quat = [0.0, 0.0, math.sin(base_yaw_k / 2.0), math.cos(base_yaw_k / 2.0)]
            q_ref = np.concatenate([[base_x_k, base_y_k, self.z_height], ref_quat, self.q_nominal])
            v_ref = np.concatenate([[self.v_cmd_x, self.v_cmd_y, 0.0], [0.0, 0.0, self.w_cmd_yaw], np.zeros(12)])
            x_ref = np.concatenate([q_ref, v_ref])

            diff = model.differential
            diff.costs.costs["state_tracking"].cost.residual.reference = x_ref
            # diff.costs.costs["state_tracking"].cost.residual.xref = x_ref
            
            for i, foot_name in enumerate(self.foot_names):
                phi_total = (t_k / self.gait_period) + self.phase_offsets[i]
                cycle_num = math.floor(phi_total)
                phase = phi_total - cycle_num
                
                c_name = foot_name + "_contact"
                f_cost_name = foot_name + "_friction"
                s_cost_name = foot_name + "_swing"

                if phase < active_duty:
                    diff.contacts.changeContactStatus(c_name, True)
                    diff.costs.changeCostStatus(f_cost_name, True)
                    diff.costs.changeCostStatus(s_cost_name, False)
                    
                    if cycle_num == current_cycle_nums[foot_name] and is_leg_in_stance_t0[foot_name]:
                        p_stance = self.stance_anchor_positions[foot_name]
                    else:
                        p_stance = np.array(self._get_raibert_target(
                            i, foot_name, cycle_num, True, t_current_abs, current_cycle_nums, is_leg_in_stance_t0))
                    
                    diff.contacts.contacts[c_name].contact.xref = p_stance
                else:
                    diff.contacts.changeContactStatus(c_name, False)
                    diff.costs.changeCostStatus(f_cost_name, False)
                    diff.costs.changeCostStatus(s_cost_name, True)
                    
                    s = (phase - active_duty) / (1.0 - active_duty)
                    
                    if cycle_num == current_cycle_nums[foot_name] and not is_leg_in_stance_t0[foot_name]:
                        p_start = self.liftoff_positions[foot_name]
                    else:
                        p_start = np.array(self._get_raibert_target(
                            i, foot_name, cycle_num, True, t_current_abs, current_cycle_nums, is_leg_in_stance_t0))
                    
                    p_end = np.array(self._get_raibert_target(
                        i, foot_name, cycle_num + 1, True, t_current_abs, current_cycle_nums, is_leg_in_stance_t0))
                    
                    control_h = self.swing_height * (4.0 / 3.0)
                    sx = self.bezier_curve(p_start[0], p_start[0], p_end[0], p_end[0], s)
                    sy = self.bezier_curve(p_start[1], p_start[1], p_end[1], p_end[1], s)
                    sz = self.bezier_curve(p_start[2], p_start[2] + control_h, p_end[2] + control_h, p_end[2], s)
                    
                    diff.costs.costs[s_cost_name].cost.residual.xref = np.array([sx, sy, sz])

    def _solve_mpc(self, x0, t_current_abs):
        num_intervals = self.mpc_horizon - 1
        
        if self.last_xs is not None and len(self.last_xs) == self.mpc_horizon:
            init_xs = self.last_xs[1:] + [self.last_xs[-1]]
            init_us = self.last_us[1:] + [self.last_us[-1]]
            init_xs[0] = x0
        else:
            init_xs = [x0] * self.mpc_horizon
            init_us = [np.zeros(12)] * num_intervals
            
        try:
            self.problem.x0 = x0
            self.solver.solve(init_xs, init_us, 15, is_feasible=False)
            self.last_xs = list(self.solver.xs)
            self.last_us = list(self.solver.us)
        except Exception as e:
            self.get_logger().error(f"Crocoddyl Solver exception: {e}")
            self.last_xs = None
            self.last_us = None

    def _build_trajectory_msg(self):
        traj_msg = MpcTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = "odom"
        
        last_rdata = self.solver.problem.runningDatas[-1]
        for i in range(self.mpc_horizon):
            step_msg = MpcStateStep()
            time_offset_sec = float(i * self.dt)
            step_msg.time_offset.sec = int(time_offset_sec)
            step_msg.time_offset.nanosec = int((time_offset_sec - int(time_offset_sec)) * 1e9)
            
            x = self.solver.xs[i]
            q, v = x[:self.rmodel.nq], x[self.rmodel.nq:]
            
            step_msg.body_pose.position.x = float(q[0])
            step_msg.body_pose.position.y = float(q[1])
            step_msg.body_pose.position.z = float(q[2])
            step_msg.body_pose.orientation.x = float(q[3])
            step_msg.body_pose.orientation.y = float(q[4])
            step_msg.body_pose.orientation.z = float(q[5])
            step_msg.body_pose.orientation.w = float(q[6])

            q_quat = pin.Quaternion(float(q[6]), float(q[3]), float(q[4]), float(q[5]))
            R_w_b = q_quat.toRotationMatrix()
            
            v_lin_world = R_w_b @ v[0:3]
            v_ang_world = R_w_b @ v[3:6]

            step_msg.body_velocity.linear.x = float(v_lin_world[0])
            step_msg.body_velocity.linear.y = float(v_lin_world[1])
            step_msg.body_velocity.linear.z = float(v_lin_world[2])
            step_msg.body_velocity.angular.x = float(v_ang_world[0])
            step_msg.body_velocity.angular.y = float(v_ang_world[1])
            step_msg.body_velocity.angular.z = float(v_ang_world[2])

            if i < self.mpc_horizon - 1:
                model = self.running_models[i]
                rdata = self.solver.problem.runningDatas[i]
            else:
                model = self.terminal_model
                rdata = last_rdata

            active_contacts = list(model.differential.contacts.active_set)
            a_base_local = rdata.differential.xout[:6]
            contact_forces = rdata.differential.multibody.contacts.contacts

            a_lin_world = R_w_b @ a_base_local[0:3]
            a_ang_world = R_w_b @ a_base_local[3:6]

            step_msg.body_acceleration.linear.x = float(a_lin_world[0])
            step_msg.body_acceleration.linear.y = float(a_lin_world[1])
            step_msg.body_acceleration.linear.z = float(a_lin_world[2])
            step_msg.body_acceleration.angular.x = float(a_ang_world[0])
            step_msg.body_acceleration.angular.y = float(a_ang_world[1])
            step_msg.body_acceleration.angular.z = float(a_ang_world[2])

            pin.forwardKinematics(self.rmodel, self.rdata, q, v)
            pin.updateFramePlacements(self.rmodel, self.rdata)
            
            for foot in self.foot_names:
                f_msg = FootState()
                f_msg.name = foot
                foot_id = self.rmodel.getFrameId(foot)
                
                contact_name = foot + "_contact"
                is_contact = contact_name in active_contacts
                f_msg.is_contact = is_contact

                if is_contact:
                    foot_pos_global = self.rdata.oMf[foot_id].translation
                    f_msg.position.x = float(foot_pos_global[0])
                    f_msg.position.y = float(foot_pos_global[1])
                    f_msg.position.z = float(foot_pos_global[2])

                    if contact_name in contact_forces:
                        force_3d = contact_forces[contact_name].f.linear
                        f_msg.contact_force.x = float(force_3d[0])
                        f_msg.contact_force.y = float(force_3d[1])
                        f_msg.contact_force.z = float(force_3d[2])
                else:
                    ref_swing_pos = model.differential.costs.costs[foot + "_swing"].cost.residual.xref
                    f_msg.position.x = float(ref_swing_pos[0])
                    f_msg.position.y = float(ref_swing_pos[1])
                    f_msg.position.z = float(ref_swing_pos[2])

                    f_msg.contact_force.x = 0.0
                    f_msg.contact_force.y = 0.0
                    f_msg.contact_force.z = 0.0

                foot_vel_global = pin.getFrameVelocity(
                    self.rmodel, self.rdata, foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear
                f_msg.velocity.x = float(foot_vel_global[0])
                f_msg.velocity.y = float(foot_vel_global[1])
                f_msg.velocity.z = float(foot_vel_global[2])
                
                step_msg.feet.append(f_msg)
            traj_msg.steps.append(step_msg)
        return traj_msg


def main(args=None):
    rclpy.init(args=args)
    node = CrocoddylMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()