import pytest
import numpy as np
from pathlib import Path
import xacro
from ament_index_python.packages import get_package_share_directory
# Adjust imports to match your package structure
from model_predictive_control.description import RobotDescription, VelocityCommand, RobotState
from model_predictive_control.heuristic import GaitHeuristic
from model_predictive_control.predictive import PredictiveMPC

@pytest.fixture
def robot_desc():
    xacro_path = (
        Path(get_package_share_directory("go2_description"))
        / "xacro"
        / "robot.xacro"
    )

    # Convert xacro to in-memory URDF string (with optional xacro mappings/args)
    # mappings = {'robot_name': 'quadruped'}  # Pass xacro arguments here if needed
    doc = xacro.process_file(str(xacro_path))  # , mappings=mappings)
    urdf_xml_string = doc.toxml()
    desc = RobotDescription()
        
    loaded = desc.load_from_xml(urdf_xml_string)
    assert loaded, f"Failed to parse URDF from {xacro_path}"
    return desc

@pytest.fixture
def initial_state(robot_desc):
    state = RobotState()
    # Match what your callback does:
    state.q_joints = robot_desc.q_nominal.copy()
    state.z = robot_desc.z_height if hasattr(robot_desc, "z_height") else 0.3
    return state

@pytest.fixture
def heuristic(robot_desc):
    return GaitHeuristic(robot_desc)

@pytest.fixture
def mpc(robot_desc):
    return PredictiveMPC(robot_desc, dt=0.040, mpc_horizon=20)

def simulate_mpc_sequence(mpc, state, heuristic, duration, command_profile, disturbance_profile=None):
    """
    Helper function to run the MPC in a closed-loop fashion (assuming perfect physics).
    Returns telemetry data for assertions.
    """
    dt = mpc.dt
    steps = int(duration / dt)
    
    history = {
        'time': [],
        'success': [],
        'iterations': [],
        'cost': [],
        'control_norm': []
    }
    
    for step in range(steps):
        t_sim = step * dt
        
        # 1. Apply command profile
        cmd = command_profile(t_sim)
        
        # 2. Apply disturbances if defined
        if disturbance_profile:
            state = disturbance_profile(t_sim, state)
            
        # 3. Prepare inputs
        x0 = state.to_crocoddyl_x0()
        current_foot_pos = mpc.desc.compute_kinematics(state)
        
        # 4. Update and Solve
        current_cycle_nums, is_leg_in_stance_t0 = heuristic.update_gait_phase(
            t_sim, current_foot_pos, cmd.is_zero)
            
        mpc.update_horizon(t_sim, state, cmd, heuristic, 
                           current_cycle_nums, is_leg_in_stance_t0)
        
        success = mpc.solve(x0)
        
        # 5. Log metrics
        history['time'].append(t_sim)
        history['success'].append(success)
        history['iterations'].append(mpc.solver.iter)
        history['cost'].append(mpc.solver.cost)
        
        if success:
            u0 = mpc.last_us[0]
            history['control_norm'].append(np.linalg.norm(u0))
            # Perfect physics assumption: state becomes the next predicted state
            state.update_from_crocoddyl_x(mpc.last_xs[1])
        else:
            history['control_norm'].append(np.nan)
            
    return history


def test_steady_state_standing( mpc, initial_state, heuristic):
    """Validates that the MPC can hold a stable standing position indefinitely."""
    
    def cmd_stand(t):
        return VelocityCommand(v_x=0.0, v_y=0.0, w_yaw=0.0)
        
    history = simulate_mpc_sequence(mpc, initial_state, heuristic, 1.0, cmd_stand)
    
    # Assert the solver never failed
    assert all(history['success']), "Solver failed during steady-state standing."
    
    # After initial warm-start (first 2 steps), iterations should be minimal
    assert max(history['iterations'][2:]) <= 3, "Solver is struggling to maintain steady state (high iterations)."
    
    # Cost should be strictly non-increasing or stable
    costs = history['cost'][2:]
    assert np.var(costs) < 1e-3, "Cost is fluctuating during steady-state standing."


def test_command_transient_stand_to_walk(mpc, initial_state, heuristic):
    """Validates solver recovery when switching from standing to a trot/walk."""
    
    def cmd_walk_at_0_5(t):
        if t < 0.5:
            return VelocityCommand(v_x=0.0, v_y=0.0, w_yaw=0.0)
        return VelocityCommand(v_x=0.3, v_y=0.0, w_yaw=0.0)
        
    history = simulate_mpc_sequence(mpc, initial_state, heuristic, 1.5, cmd_walk_at_0_5)
    
    assert all(history['success']), "Solver failed during stand-to-walk transition."
    
    # Find the index where the command changed
    transition_idx = int(0.5 / mpc.dt)
    
    # The transient might cause iterations to hit the max (15), which is acceptable 
    # for a step or two, but it MUST recover to < 5 within 4 timesteps (160ms).
    recovery_idx = transition_idx + 4
    assert max(history['iterations'][recovery_idx:]) < 5, \
        "Solver failed to recover low-iteration convergence after command transient."
        
    # Ensure control commands didn't explode to extreme values during the shock
    max_u = np.nanmax(history['control_norm'])
    assert max_u < 500.0, f"Control effort exploded during transition: {max_u}"


def test_disturbance_transient_z_drop( mpc, initial_state, heuristic):
    """Validates solver stability when a large, unexpected state error is introduced."""
    
    def cmd_walk(t):
        return VelocityCommand(v_x=0.3, v_y=0.0, w_yaw=0.0)
        
    def z_drop_disturbance(t, state):
        # Introduce a 5cm drop exactly at t=0.5s
        if abs(t - 0.5) < 1e-5:
            state.z -= 0.05 
        return state
        
    history = simulate_mpc_sequence(
        mpc, initial_state, heuristic, 1.5, cmd_walk, z_drop_disturbance
    )
    
    assert all(history['success']), "Solver failed to handle a 5cm z-drop disturbance."
    
    transition_idx = int(0.5 / mpc.dt)
    
    # Verify the cost spiked as expected (confirming the disturbance was registered)
    pre_drop_cost = history['cost'][transition_idx - 1]
    post_drop_cost = history['cost'][transition_idx]
    assert post_drop_cost > pre_drop_cost * 2, "Disturbance did not trigger expected cost penalty."
    
    # Verify recovery
    recovery_idx = transition_idx + 6
    assert max(history['iterations'][recovery_idx:]) < 5, \
        "Solver failed to stabilize after z-drop disturbance."