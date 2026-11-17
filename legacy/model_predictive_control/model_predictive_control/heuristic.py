import numpy as np
from model_predictive_control.kinematic_model import VelocityCommand, Base, RobotState
import pinocchio as pin
GRAVITY = 9.81

class StandHeuristic:
    def __init__(self):
        self.NUM_FEET = 4
        self.locked_stance_pos = np.zeros((self.NUM_FEET, 3))

    def reset_horizon(self, current_time: float, state: RobotState, cmd: VelocityCommand):
        for i in range(self.NUM_FEET):
            self.locked_stance_pos[i] = state.fs[i].position.copy()

    def update_phase(self, time: float, state: RobotState, cmd: VelocityCommand):
        pass  # Pure static stance; foot targets do not move

    def get_targets(self):
        target_positions = [self.locked_stance_pos[i].copy() for i in range(self.NUM_FEET)]
        target_velocities = [np.zeros(3) for _ in range(self.NUM_FEET)]
        contacts = [True] * self.NUM_FEET
        return target_positions, target_velocities, contacts
    

class Heuristic:
    def __init__(
        self,
        nominal_positions: list[np.ndarray],
        duty_factor: float,
        cycle_time: float,
        step_height: float,
        phase_offsets: list[float] = [0.0, 0.5, 0.5, 0.0],
        kp_multiplier: float = 1.0,
    ):
        self.CYCLE_TIME = cycle_time
        self.DUTY_FACTOR = duty_factor
        self.PHASE_OFFSETS = phase_offsets
        self.LOCAL_NOM_FOOT_POS = nominal_positions  # 4x3
        self.KP_MULTIPLIER = kp_multiplier
        self.STEP_HEIGHT = step_height
        self.NUM_FEET = 4
        self.STANCE_DURATION = self.CYCLE_TIME * self.DUTY_FACTOR
        self.SWING_DURATION = self.CYCLE_TIME * (1 - self.DUTY_FACTOR)

        # Real state memory
        self.real_stance_pos = np.zeros((self.NUM_FEET, 3))
        self.real_swing_start = np.zeros((self.NUM_FEET, 3))
        # self.phase_adaptations = np.zeros(self.NUM_FEET)

        # Prediction state memory
        self.pred_time = 0.0
        self.pred_contacts = np.ones(self.NUM_FEET, dtype=bool)
        self.pred_stance_pos = np.zeros((self.NUM_FEET, 3))
        self.pred_swing_start = np.zeros((self.NUM_FEET, 3))
        self.pred_swing_target = np.zeros(
            (self.NUM_FEET, 3)
        )  # FIXED: Anchored landing target
        self.liftoff_base_state = [
            None
        ] * self.NUM_FEET  # Snapshot of base state at lift-off

    def _get_phase(self, time: float, foot_id: int):
        """Calculates phase including dynamic adaptations for early/late contacts."""
        base_phi = (time % self.CYCLE_TIME) / self.CYCLE_TIME
        return (base_phi + self.PHASE_OFFSETS[foot_id] ) % 1.0

    # def sync_clock_to_reality(self, current_time: float, state: RobotState):
    #     """Adjusts the timeline for each leg based on actual physical contact events."""
    #     for i in range(self.NUM_FEET):
    #         current_phase = self._get_phase(current_time, i)
    #         scheduled_stance = current_phase < self.DUTY_FACTOR
    #         is_contact = state.fs[i].is_contact

    #         # 1. EARLY TOUCHDOWN (Stepped on a rock)
    #         if not scheduled_stance and is_contact:
    #             # Fast-forward this leg's clock to the start of the Stance phase (0.0)
    #             target_phase = 0.0
    #             base_phi = (current_time % self.CYCLE_TIME) / self.CYCLE_TIME
    #             self.phase_adaptations[i] = target_phase - (base_phi + self.PHASE_OFFSETS[i])
            
    #         # 2. LATE TOUCHDOWN (Stepped in a hole / physics hasn't landed yet)
    #         elif scheduled_stance and not is_contact:
    #             # Rewind the clock to the very end of the Swing phase (0.999).
    #             # This perpetually holds the leg in "Reach Down" mode and PREVENTS
    #             # the MPC from activating the contact constraint until the foot physically lands.
    #             target_phase = 0.999
    #             base_phi = (current_time % self.CYCLE_TIME) / self.CYCLE_TIME
    #             self.phase_adaptations[i] = target_phase - (base_phi + self.PHASE_OFFSETS[i])

    def reset_horizon(
        self,
        current_time: float,
        state: RobotState,
        cmd: VelocityCommand,
    ):
        # self.phase_adaptations *= 0.95
        # self.sync_clock_to_reality(current_time, state)
        for i in range(self.NUM_FEET):
            phase = self._get_phase(current_time, i)
            scheduled_stance = phase < self.DUTY_FACTOR
            if scheduled_stance:
                # CLOCK SAYS STANCE: Command stance. 
                # If physical contact is false, WBC reflex will kick it down to find the floor.
                self.real_stance_pos[i] = state.fs[i].position.copy()
                # Ensure the target is actually at ground level so the WBC has a correct anchor
                self.real_stance_pos[i][2] = -0.01 
                self.pred_contacts[i] = True
            else:
                # CLOCK SAYS SWING: Command swing.
                if state.fs[i].is_contact:
                    # EARLY TOUCHDOWN: Physical contact happened early. 
                    # Latch the start position from exactly where it hit.
                    self.real_swing_start[i] = state.fs[i].position.copy()
                else:
                    # NORMAL SWING
                    self.real_swing_start[i] = state.fs[i].position.copy()
                self.pred_contacts[i] = False

        self.pred_time = current_time
        self.pred_stance_pos = self.real_stance_pos.copy()
        self.pred_swing_start = self.real_swing_start.copy()

        # Pre-compute initial swing targets
        for i in range(self.NUM_FEET):
            if not self.pred_contacts[i]:
                phase = self._get_phase(current_time, i)
                progress = np.clip((phase - self.DUTY_FACTOR) / (1.0 - self.DUTY_FACTOR), 0.0, 1.0)
                t_remain = (1.0 - progress) * self.SWING_DURATION
                self.pred_swing_target[i] = self._calculate_swing_target(
                    state.bs, cmd, self.LOCAL_NOM_FOOT_POS[i], 
                    self.STANCE_DURATION, t_remain, self.KP_MULTIPLIER
                )
            else:
                self.pred_swing_target[i] = self.real_stance_pos[i].copy()
                
    def update_phase(self, time: float, state: RobotState, cmd: VelocityCommand):
        """Called for each horizon node."""
        self.pred_time = time

        for i in range(self.NUM_FEET):
            # 1. Get the default state from the trot clock
            phase = self._get_phase(time, i)
            clocked_contact = phase < self.DUTY_FACTOR
            is_contact = clocked_contact# and state.fs[i].is_contact
            # Apply Contact Logic
            if clocked_contact and not self.pred_contacts[i]:
                # TOUCHDOWN: Lock ground position to the target
                # print("Touchdown update")
                self.pred_stance_pos[i] = self.pred_swing_target[i].copy()

            elif not clocked_contact and self.pred_contacts[i]:
                # LIFT-OFF: Latch start position AND compute static landing target
                self.pred_swing_start[i] = self.pred_stance_pos[i].copy()
                self.pred_swing_target[i] = self._calculate_swing_target(
                    state.bs,
                    cmd,
                    self.LOCAL_NOM_FOOT_POS[i],
                    self.STANCE_DURATION,
                    self.SWING_DURATION,
                    self.KP_MULTIPLIER,
                )

            self.pred_contacts[i] = is_contact

    def get_targets(self):
        """Returns stable target positions for the current node."""
        target_positions = []
        target_velocities = []
        contacts = []
        swing_duration = self.CYCLE_TIME * (1.0 - self.DUTY_FACTOR)
        for i in range(self.NUM_FEET):
            contacts.append(bool(self.pred_contacts[i]))

            if self.pred_contacts[i]:
                stance_pos = self.pred_stance_pos[i].copy()
                target_positions.append(stance_pos)
                target_velocities.append(np.zeros(3))
            else:
                phase = self._get_phase(self.pred_time, i)
                # progress = (phase - self.DUTY_FACTOR) / (1.0 - self.DUTY_FACTOR)
                progress = max(0.0, (phase - self.DUTY_FACTOR) / (1.0 - self.DUTY_FACTOR))
                # progress = np.clip(progress, 0.0, 1.0)
                start_pos = self.pred_swing_start[i].copy()
                end_pos = self.pred_swing_target[i].copy()
                ref_position, ref_velocity = self._generate_swing_trajectory(
                    start_pos, end_pos, progress, swing_duration
                )
                target_positions.append(ref_position)
                target_velocities.append(ref_velocity)

        return target_positions, target_velocities, contacts

    def _generate_swing_trajectory(
        self,
        start_pos: np.ndarray,
        end_pos: np.ndarray,
        progress: float,
        swing_duration: float,
    ):
        """Generates position and velocity using Cubic (XY) and Quartic (Z) Bezier curves."""

        if progress <= 0.0:
            return start_pos.copy(), np.zeros(3)
        if progress >= 1.0:
            over_time = (progress - 1.0) * swing_duration
            pos = end_pos.copy()
            vel = np.zeros(3)
            vel[2] = -0.5 # Reach down at 0.5 m/s
            pos[2] = end_pos[2] + (vel[2] * over_time)
            return pos, vel
        
        d_progress_dt = 1.0 / max(swing_duration, 1e-4)

        pos = np.zeros(3)
        vel = np.zeros(3)

        s = 3 * progress**2 - 2 * progress**3
        ds_dt = (6 * progress - 6 * progress**2) * d_progress_dt

        # XY interpolation
        pos[0:2] = start_pos[0:2] + (end_pos[0:2] - start_pos[0:2]) * s
        vel[0:2] = (end_pos[0:2] - start_pos[0:2]) * ds_dt

        # Z interpolation (Fixed: now interpolates from start Z to end Z)
        pos[2] = (
            start_pos[2]
            + (end_pos[2] - start_pos[2]) * s
            + 16 * self.STEP_HEIGHT * (progress**2) * ((1 - progress) ** 2)
        )
        vel[2] = (
            end_pos[2] - start_pos[2]
        ) * ds_dt + 32 * self.STEP_HEIGHT * progress * (1 - progress) * (
            1 - 2 * progress
        ) * d_progress_dt

        return pos, vel

    @staticmethod
    def _calculate_swing_target(
        state: Base,
        cmd: VelocityCommand,
        local_nominal_position_3d: np.ndarray, # Pass full [x, y, z] hip offset
        t_stance: float,
        t_remain_swing: float,
        kp_multiplier: float,
    ):
        h_actual = max(state.pos[2], 0.1)
        time_constant = np.sqrt(h_actual / 9.81)
        kp = time_constant * kp_multiplier

        v_act_local = state.vel_lin[:2]
        v_cmd_local = cmd.planar

        # 1. Raibert Heuristics (Calculated in the 2D local ground plane)
        feedforward_term = (t_remain_swing + t_stance / 2.0) * v_cmd_local
        feedback_term = kp * (v_act_local - v_cmd_local)
        
        cross_prod = np.cross([v_cmd_local[0], v_cmd_local[1], 0.0], [0.0, 0.0, cmd.w_yaw])
        centrifugal_term = 0.5 * (h_actual / 9.81) * cross_prod[:2]

        raibert_offset_local_xy = feedforward_term + feedback_term + centrifugal_term

        # 2. Kinematics: Rotate the physical hip by FULL 3D orientation
        # (Assuming you have a function or scipy rotation for RPY)
        # from scipy.spatial.transform import Rotation as R
        from pinocchio import Quaternion
        # R_full = R.from_euler('xyz', [state.roll, state.pitch, state.yaw]).as_matrix()
        quat = Quaternion(state.quat)
        R_full = quat.toRotationMatrix()
        # This calculates exactly where the hip socket is in world space right now
        hip_offset_world = R_full @ local_nominal_position_3d

        # 3. Ground Step: Rotate only the Raibert offset by Yaw
        yaw = np.arctan2(R_full[1, 0], R_full[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)

        R_yaw_2d = np.array([[c, -s], [s, c]])
        raibert_offset_world_xy = R_yaw_2d @ raibert_offset_local_xy

        # 4. Combine CoM + True Hip + Raibert Offset
        target_x = state.pos[0] + hip_offset_world[0] + raibert_offset_world_xy[0]
        target_y = state.pos[1] + hip_offset_world[1] + raibert_offset_world_xy[1]
        
        # For perfectly flat ground, Z remains 0.0. For uneven terrain, you would 
        # add the terrain height estimate here.
        target_z = -0.01 

        return np.array([target_x, target_y, target_z])


class SingleLegCircleHeuristic:
    def __init__(
        self,
        nominal_positions: list[np.ndarray],
        swing_leg_id: int = 0,
        radius: float = 0.04,
        frequency: float = 1.0,
        z_lift: float = 0.1,
        k_converge: float = 15.0
    ):
        self.NUM_FEET = 4
        self.nominal_positions = [p.copy() for p in nominal_positions]
        self.swing_leg_id = swing_leg_id
        self.radius = radius
        self.omega = 2.0 * np.pi * frequency
        self.v_tangent = self.omega * self.radius
        self.z_lift = z_lift
        self.k_converge = k_converge

        self.pred_time = 0.0
        self.locked_stance_pos = np.zeros((self.NUM_FEET, 3))
        self.sim_pos = np.zeros(3)
        self.sim_vel = np.zeros(3)
        self.world_circle_center = np.zeros(3)

    def reset_horizon(
        self,
        current_time: float,
        state: RobotState,
        cmd: VelocityCommand,
    ):
        self.pred_time = current_time

        stance_curr = []
        stance_nom = []

        # 1. Anchor stance legs in WORLD frame
        for i in range(self.NUM_FEET):
            if i != self.swing_leg_id:
                self.locked_stance_pos[i] = state.fs[i].position.copy()
                stance_curr.append(self.locked_stance_pos[i])
                stance_nom.append(self.nominal_positions[i])

        # 2. Seed the swing trajectory at the current physical WORLD state
        self.sim_pos = state.fs[self.swing_leg_id].position.copy()

        # 3. Anchor circle center relative to the stationary stance feet (time-invariant)
        stance_centroid_curr = np.mean(stance_curr, axis=0)
        stance_centroid_nom = np.mean(stance_nom, axis=0)
        nom_offset = self.nominal_positions[self.swing_leg_id] - stance_centroid_nom

        # Rotate nominal XY offset by current base yaw
        R = pin.Quaternion(state.bs.quat).toRotationMatrix()
        yaw = pin.rpy.matrixToRpy(R)[2]
        R_yaw = pin.rpy.rpyToMatrix(0.0, 0.0, yaw)

        self.world_circle_center = stance_centroid_curr + R_yaw @ nom_offset
        self.world_circle_center[2] += self.z_lift

        # Seed initial velocity
        self.sim_vel = self._compute_vector_field(self.sim_pos)

    def update_phase(self, time: float, state: RobotState, cmd: VelocityCommand):
        """Integrates the physical position forward along the vector field."""
        dt = time - self.pred_time
        if dt > 0:
            self.sim_vel = self._compute_vector_field(self.sim_pos)
            self.sim_pos = self.sim_pos + self.sim_vel * dt
        self.pred_time = time

    def _compute_vector_field(self, pos: np.ndarray) -> np.ndarray:
        """Returns the target velocity based on distance from the World center."""
        dx = pos[0] - self.world_circle_center[0]
        dz = pos[2] - self.world_circle_center[2]
        dy = pos[1] - self.world_circle_center[1]

        r = np.sqrt(dx**2 + dz**2)
        if r < 1e-4:
            dx, dz, r = 1e-4, 0.0, 1e-4

        nr_x = dx / r
        nr_z = dz / r
        nt_x = -nr_z
        nt_z = nr_x

        v_radial = self.k_converge * (self.radius - r)

        vel = np.zeros(3)
        vel[0] = v_radial * nr_x + self.v_tangent * nt_x
        vel[2] = v_radial * nr_z + self.v_tangent * nt_z
        vel[1] = self.k_converge * (0.0 - dy)
        return vel

    def get_targets(self):
        target_positions = []
        target_velocities = []
        contacts = []

        for i in range(self.NUM_FEET):
            if i == self.swing_leg_id:
                contacts.append(False)
                target_positions.append(self.sim_pos.copy())
                target_velocities.append(self.sim_vel.copy())
            else:
                contacts.append(True)
                target_positions.append(self.locked_stance_pos[i].copy())
                target_velocities.append(np.zeros(3))

        return target_positions, target_velocities, contacts