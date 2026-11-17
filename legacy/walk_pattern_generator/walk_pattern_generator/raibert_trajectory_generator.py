import rclpy
from rclpy.node import Node
import math

from wbc_control_msgs.msg import MpcTrajectory, MpcStateStep, FootState 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MpcRaibertTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('mpc_raibert_trajectory_generator')
        
        self.traj_pub = self.create_publisher(MpcTrajectory, '/mpc_trajectory', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_command = self.create_subscription(Twist, '/vel_cmd', self.command_callback, 10)
        
        self.horizon_length = 30
        self.dt = 0.04
        
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_yaw = 0.0

        # Virtual Reference State to prevent Odometry Yielding
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0
        self.last_loop_time = None

        # Filtered velocities to prevent feedback instability
        self.v_actual_x = 0.0
        self.v_actual_y = 0.0
        self.w_actual_yaw = 0.0
        self.alpha_vel = 0.1
        
        self.robot_mass = 12.0 
        self.gravity = 9.81

        self.v_cmd_x = 0.0
        self.v_cmd_y = 0.0
        self.w_cmd_yaw = 0.0
        
        self.start_time = None
        self.z_height = 0.3
        self.foot_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        self.foot_nom_x = [ 0.19,   0.19,  -0.19,  -0.19]
        self.foot_nom_y = [0.142,  -0.142, 0.142,  -0.142]
        self.phase_offsets = [0.0, 0.5, 0.5, 0.0] 
        
        self.gait_period = 0.45
        self.duty_factor = 0.50
        self.swing_height = 0.08
        self.k_vel_feedback = 0.1

        self.stance_anchor_world = [None] * 4
        self.last_phase = [0.0] * 4

        self.timer = self.create_timer(0.04, self.publish_trajectory)
        self.get_logger().info("Raibert MPC Trajectory Generator - Drift Fixed.")

    def bezier_curve(self, p0, p1, p2, p3, t):
        return (1 - t)**3 * p0 + 3 * (1 - t)**2 * t * p1 + 3 * (1 - t) * t**2 * p2 + t**3 * p3

    def odom_callback(self, msg):
        vx_raw = msg.twist.twist.linear.x
        vy_raw = msg.twist.twist.linear.y
        wz_raw = msg.twist.twist.angular.z

        self.v_actual_x = (1 - self.alpha_vel) * self.v_actual_x + self.alpha_vel * vx_raw
        self.v_actual_y = (1 - self.alpha_vel) * self.v_actual_y + self.alpha_vel * vy_raw
        self.w_actual_yaw = (1 - self.alpha_vel) * self.w_actual_yaw + self.alpha_vel * wz_raw
        
        self.actual_x = msg.pose.pose.position.x
        self.actual_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.actual_yaw = math.atan2(siny_cosp, cosy_cosp)

    def command_callback(self, msg):
        self.v_cmd_x = msg.linear.x if abs(msg.linear.x) > 0.02 else 0.0
        self.v_cmd_y = msg.linear.y if abs(msg.linear.y) > 0.02 else 0.0
        self.w_cmd_yaw = msg.angular.z if abs(msg.angular.z) > 0.02 else 0.0

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

    def compute_touchdown_target(self, i, base_x, base_y, base_yaw, dt_mid):
        T_stance = self.gait_period * self.duty_factor
        
        # Integrate from the passed base (now the virtual reference)
        b_x_mid, b_y_mid, b_yaw_mid = self.integrate_pose(
            base_x, base_y, base_yaw, 
            self.v_cmd_x, self.v_cmd_y, self.w_cmd_yaw, dt_mid
        )
        
        v_foot_x = self.v_cmd_x - self.w_cmd_yaw * self.foot_nom_y[i]
        v_foot_y = self.v_cmd_y + self.w_cmd_yaw * self.foot_nom_x[i]

        # Raibert Foot Placement still uses ACTUAL velocity to catch falls
        v_err_x = self.v_actual_x - self.v_cmd_x
        v_err_y = self.v_actual_y - self.v_cmd_y
        
        r_x = self.foot_nom_x[i] + (0.5 * T_stance * v_foot_x) + self.k_vel_feedback * v_err_x
        r_y = self.foot_nom_y[i] + (0.5 * T_stance * v_foot_y) + self.k_vel_feedback * v_err_y
        
        if i in [1, 3]:
            r_y = min(-0.05, r_y)
        else:
            r_y = max(0.05, r_y)

        f_x = b_x_mid + (r_x * math.cos(b_yaw_mid) - r_y * math.sin(b_yaw_mid))
        f_y = b_y_mid + (r_x * math.sin(b_yaw_mid) + r_y * math.cos(b_yaw_mid))
        return (f_x, f_y)

    def publish_trajectory(self):
        now = self.get_clock().now()
        
        if self.start_time is None:
            self.start_time = now
            self.last_loop_time = now
            
            # Snap virtual reference to actual odometry once on boot
            self.ref_x = self.actual_x
            self.ref_y = self.actual_y
            self.ref_yaw = self.actual_yaw
            return

        dt_loop = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        # --- THE FIX: Integrate the virtual base state ---
        self.ref_x, self.ref_y, self.ref_yaw = self.integrate_pose(
            self.ref_x, self.ref_y, self.ref_yaw,
            self.v_cmd_x, self.v_cmd_y, self.w_cmd_yaw, dt_loop
        )

        t_current_abs = (now - self.start_time).nanoseconds / 1e9
        # is_standing = (self.v_cmd_x == 0.0 and self.v_cmd_y == 0.0 and self.w_cmd_yaw == 0.0)
        is_standing = False
        active_duty = 1.0 if is_standing else self.duty_factor
        
        T_stance = self.gait_period * active_duty
        T_swing = self.gait_period * (1.0 - active_duty)

        msg = MpcTrajectory()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"

        # Update Current (k=0) Stance Anchors using REFERENCE STATE
        c_num_now = [0] * 4
        for i in range(4):
            phi_total_now = (t_current_abs / self.gait_period) + self.phase_offsets[i]
            c_num_now[i] = math.floor(phi_total_now)
            phase = phi_total_now - c_num_now[i]
            
            if phase < active_duty:
                if self.stance_anchor_world[i] is None or self.last_phase[i] >= active_duty:
                    # Anchor cleanly using the Raibert target perfectly matched to the virtual base
                    self.stance_anchor_world[i] = self.compute_touchdown_target(
                        i, self.ref_x, self.ref_y, self.ref_yaw, 0.0
                    )
            
            self.last_phase[i] = phase

        # Generate Horizon Steps
        for k in range(self.horizon_length):
            dt_k = k * self.dt
            t_k = t_current_abs + dt_k
            step = MpcStateStep()

            step.time_offset.sec = int(dt_k)
            step.time_offset.nanosec = int((dt_k - int(dt_k)) * 1e9)

            # Base prediction from REFERENCE STATE, not actual
            base_x_k, base_y_k, base_yaw_k = self.integrate_pose(
                self.ref_x, self.ref_y, self.ref_yaw,
                self.v_cmd_x, self.v_cmd_y, self.w_cmd_yaw, dt_k
            )

            step.body_pose.position.x = float(base_x_k)
            step.body_pose.position.y = float(base_y_k)
            step.body_pose.position.z = float(self.z_height)
            step.body_pose.orientation.x = 0.0
            step.body_pose.orientation.y = 0.0
            step.body_pose.orientation.z = float(math.sin(base_yaw_k / 2.0))
            step.body_pose.orientation.w = float(math.cos(base_yaw_k / 2.0))

            step.body_velocity.linear.x = float(self.v_cmd_x)
            step.body_velocity.linear.y = float(self.v_cmd_y)
            step.body_velocity.angular.z = float(self.w_cmd_yaw)

            legs_in_stance = 0
            phases, cycle_nums = [], []

            for i in range(4):
                phi_total = (t_k / self.gait_period) + self.phase_offsets[i]
                c_num = math.floor(phi_total)
                phase = phi_total - c_num
                phases.append(phase)
                cycle_nums.append(c_num)
                if phase < active_duty:
                    legs_in_stance += 1

            fz_target = (self.robot_mass * self.gravity) / max(1, legs_in_stance)

            for i in range(4):
                foot = FootState()
                foot.name = self.foot_names[i]
                phase = phases[i]
                c_num = cycle_nums[i]

                if c_num == c_num_now[i]:
                    active_anchor = self.stance_anchor_world[i]
                    if active_anchor is None:
                        active_anchor = (base_x_k + self.foot_nom_x[i], base_y_k + self.foot_nom_y[i])
                else:
                    t_mid_stance = (c_num - self.phase_offsets[i]) * self.gait_period + (0.5 * T_stance)
                    dt_mid_stance = t_mid_stance - t_current_abs
                    active_anchor = self.compute_touchdown_target(i, self.ref_x, self.ref_y, self.ref_yaw, dt_mid_stance)

                if phase < active_duty:
                    foot.position.x = float(active_anchor[0])
                    foot.position.y = float(active_anchor[1])
                    foot.position.z = 0.0
                    foot.velocity.x = 0.0
                    foot.velocity.y = 0.0
                    foot.velocity.z = 0.0
                    foot.contact_force.x = 0.0
                    foot.contact_force.y = 0.0
                    foot.contact_force.z = float(fz_target)
                    foot.is_contact = True
                else:
                    s = (phase - active_duty) / (1.0 - active_duty)
                    p_start = active_anchor
                    
                    t_mid_next = ((c_num + 1) - self.phase_offsets[i]) * self.gait_period + (0.5 * T_stance)
                    dt_mid_next = t_mid_next - t_current_abs
                    p_end = self.compute_touchdown_target(i, self.ref_x, self.ref_y, self.ref_yaw, dt_mid_next)

                    foot.position.x = float(self.bezier_curve(p_start[0], p_start[0], p_end[0], p_end[0], s))
                    foot.position.y = float(self.bezier_curve(p_start[1], p_start[1], p_end[1], p_end[1], s))
                    foot.position.z = float(self.bezier_curve(0.0, self.swing_height, self.swing_height, 0.0, s))

                    dx_ds = 6.0 * s * (1.0 - s) * (p_end[0] - p_start[0])
                    dy_ds = 6.0 * s * (1.0 - s) * (p_end[1] - p_start[1])
                    dz_ds = 3.0 * self.swing_height * (1.0 - 2.0 * s)

                    foot.velocity.x = float(dx_ds / max(1e-3, T_swing))
                    foot.velocity.y = float(dy_ds / max(1e-3, T_swing))
                    foot.velocity.z = float(dz_ds / max(1e-3, T_swing))
                    foot.is_contact = False

                step.feet.append(foot)
            
            msg.steps.append(step)

        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MpcRaibertTrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()