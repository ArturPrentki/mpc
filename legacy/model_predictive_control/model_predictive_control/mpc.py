import numpy as np
import pinocchio as pin
import crocoddyl as croc

from model_predictive_control.kinematic_model import (
    RobotState,
    Model,
)

FL, FR, RL, RR = 0, 1, 2, 3

class MPC:
    def __init__(self, model: Model, dt: float, horizon_length: int):
        self.DT = dt
        self.horizon_length = horizon_length
        self.model = model
        self.croc_state = croc.StateMultibody(self.model.rmodel)
        self.actuation = croc.ActuationModelFloatingBase(self.croc_state)
        self.mu = 0.7
        self.Rsurf = np.eye(3)

        # Pre-allocate lists for status toggling
        self.contacts = [f"{name}_contact" for name in self.model.foot_names]
        self.frictions = [f"{name}_friction" for name in self.model.foot_names]
        self.force_regs = [f"{name}_force_reg" for name in self.model.foot_names]
        self.swings_pos = [f"{name}_swing_pos" for name in self.model.foot_names]
        self.swings_vel = [f"{name}_swing_vel" for name in self.model.foot_names]
        self.impacts = [f"{name}_impact" for name in self.model.foot_names]

        # Build Universal Models
        self.continuous_models = [self._create_universal_model() for _ in range(self.horizon_length)]
        self.impulse_models = [self._create_universal_impulse_model() for _ in range(self.horizon_length)]
        self.terminal_node = self._create_universal_model(is_terminal=True)
        
        x0 = np.concatenate([
            [0.0, 0.0, self.model.z_height, 0.0, 0.0, 0.0, 1.0], 
            self.model.q_nominal,
            np.zeros(self.model.rmodel.nv)
        ])
        
        self.problem = croc.ShootingProblem(x0, self.continuous_models, self.terminal_node)
        self.solver = croc.SolverFDDP(self.problem)
        self.solver.th_stop = 1e-4
        self.xs = None
        self.us = None

    def _create_universal_model(self, is_terminal=False) -> croc.IntegratedActionModelEuler:
        """Builds a model containing ALL possible constraints and costs. We toggle them later."""
        nu = self.actuation.nu
        contact_model = croc.ContactModelMultiple(self.croc_state, nu)
        cost_model = croc.CostModelSum(self.croc_state, nu)

        # 1. State & Control Regularization (Using optimal decoupled weights)
        state_weights = np.concatenate([
            np.array([0.0, 0.0, 40.0]),            # Base drift
            np.array([10.0, 10.0, 10.0]),            # Base RPY
            np.full(self.model.rmodel.nv - 6, 0.1), # Joints 
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), # Base Vel
            np.full(self.model.rmodel.nv - 6, 0.1) # Joint Vel
        ])
        state_residual = croc.ResidualModelState(self.croc_state, self.croc_state.zero(), nu)
        state_activation = croc.ActivationModelWeightedQuad(state_weights)

        cost_model.addCost("state_reg", croc.CostModelResidual(self.croc_state, state_activation, state_residual), 10.0)



        if not is_terminal:
            ctrl_residual = croc.ResidualModelControl(self.croc_state, nu)
            cost_model.addCost("ctrl_reg", croc.CostModelResidual(self.croc_state, ctrl_residual), 1e-3)

        cone = croc.FrictionCone(self.Rsurf, self.mu, 4, False)
        cone_activation = croc.ActivationModelQuadraticBarrier(croc.ActivationBounds(cone.lb, cone.ub))
        swing_activation = croc.ActivationModelWeightedQuad(np.array([1.0, 1.0, 1.0]))
        
        for i, frame_id in enumerate(self.model.pin_foot_ids):
            # Contact Constraint
            contact3d = croc.ContactModel3D(self.croc_state, frame_id, np.zeros(3), pin.LOCAL_WORLD_ALIGNED, nu, np.array([20.0, 50.0]))
            contact_model.addContact(self.contacts[i], contact3d)

            if not is_terminal:
                # Friction Cone
                cone_residual = croc.ResidualModelContactFrictionCone(self.croc_state, frame_id, cone, nu)
                cost_model.addCost(self.frictions[i], croc.CostModelResidual(self.croc_state, cone_activation, cone_residual), 1e3)
                
                # Dynamic Force Regularization (Pulls toward mg/N)
                force_residual = croc.ResidualModelContactForce(self.croc_state, frame_id, pin.Force.Zero(), 3, nu)
                cost_model.addCost(self.force_regs[i], croc.CostModelResidual(self.croc_state, force_residual), 0.001)

            # Swing Position Tracking
            pos_residual = croc.ResidualModelFrameTranslation(self.croc_state, frame_id, np.zeros(3), nu)
            cost_model.addCost(self.swings_pos[i], croc.CostModelResidual(self.croc_state, swing_activation, pos_residual), 1e4)

            # Swing Velocity Tracking
            vel_residual = croc.ResidualModelFrameVelocity(self.croc_state, frame_id, pin.Motion.Zero(), pin.LOCAL_WORLD_ALIGNED, nu)
            cost_model.addCost(self.swings_vel[i], croc.CostModelResidual(self.croc_state, vel_residual), 1e-1)

            # # IMPACT Velocity Penalty (Triggered only on dt=0 switch nodes)
            # cost_model.addCost(self.impacts_vel[i], croc.CostModelResidual(self.croc_state, vel_residual), 1e3)


        ndx = self.croc_state.ndx  # Tangent space dimension (36)
        nv = self.model.rmodel.nv  # Velocity space dimension (18)

        # 1. Initialize bounds to infinity for the whole 36D state
        lb = np.full(ndx, -np.inf)
        ub = np.full(ndx, np.inf)

        # 2. Map only the joint position limits
        # In Pinocchio (nq=19), base is [0:7], joints are [7:]
        # In tangent space (ndx=36), base pos diff is [0:6], joint pos diff is [6:18]
        lb[6:nv] = self.model.rmodel.lowerPositionLimit[7:]
        ub[6:nv] = self.model.rmodel.upperPositionLimit[7:]

        state_bounds = croc.ActivationBounds(lb, ub)
        limit_activation = croc.ActivationModelQuadraticBarrier(state_bounds)
        
        # state.zero() evaluates to q=0. So state diff (q - 0) maps exactly to actual joint angles
        limit_residual = croc.ResidualModelState(self.croc_state, self.croc_state.zero(), nu)

        # Add with a very high weight so it acts as a hard boundary
        cost_model.addCost("joint_limits", 
                           croc.CostModelResidual(self.croc_state, limit_activation, limit_residual), 
                           1e2)

        dmodel = croc.DifferentialActionModelContactFwdDynamics(self.croc_state, self.actuation, contact_model, cost_model, 0.0, True)
        
        # Start all nodes at standard dt (will be overwritten to 0.0 dynamically during impact)
        return croc.IntegratedActionModelEuler(dmodel, 0.0 if is_terminal else self.DT)

    def _create_universal_impulse_model(self) -> croc.ActionModelImpulseFwdDynamics:
        """Builds a model for instantaneous impact. Control dimension nu=0."""
        nu = 0  # No control torques during instantaneous impact
        
        impulse_model = croc.ImpulseModelMultiple(self.croc_state)
        cost_model = croc.CostModelSum(self.croc_state, nu)

        # State Regularization (keeps the solver grounded during the jump)
        state_residual = croc.ResidualModelState(self.croc_state, self.croc_state.zero(), nu)
        cost_model.addCost("state_reg", croc.CostModelResidual(self.croc_state, state_residual), 10.0)

        # Add impulses for all 4 feet (we will toggle these on/off)
        for i, frame_id in enumerate(self.model.pin_foot_ids):
            # Impulse physics handles the velocity penalty implicitly, so we just define the kinematic constraint
            impulse3d = croc.ImpulseModel3D(self.croc_state, frame_id, pin.LOCAL_WORLD_ALIGNED)
            
            # Note: Assuming you have a list of names for impulses, e.g., self.impacts = ["FL_impact", ...]
            impulse_model.addImpulse(self.impacts[i], impulse3d)

        return croc.ActionModelImpulseFwdDynamics(self.croc_state, impulse_model, cost_model)

    def optimize_trajectory(self, current_state: RobotState, ref_trajectory: list[RobotState]):
        """Updates the horizon dynamically, detects impacts, and solves."""
        self.problem.x0 = current_state.as_crocoddyl
        num_intervals = self.problem.T
        
        prev_contacts = [f.is_contact for f in current_state.fs]
        pin.centerOfMass(self.model.rmodel, self.model.rdata, current_state.q, False)
        v_zero = np.zeros(self.model.rmodel.nv)
        a_zero = np.zeros(self.model.rmodel.nv)
        tau_grav = pin.rnea(self.model.rmodel, self.model.rdata, current_state.q, v_zero, a_zero)
        u_grav = tau_grav[6:]
        # 1. Update the running horizon (Toggle Strategy)
        for k in range(num_intervals):
            ref_state = ref_trajectory[k].copy()
            is_impact_node = (ref_state.dt == 0.0)

            # --- MODEL SWAPPING & TOGGLING ---
            if is_impact_node:
                if not isinstance(self.problem.runningModels[k], croc.ActionModelImpulseFwdDynamics):
                    self.problem.updateModel(k, self.impulse_models[k])
                
                model = self.problem.runningModels[k]
                model.dt = 0
                for i, foot in enumerate(ref_state.fs):
                    is_stance = foot.is_contact
                    was_stance = prev_contacts[i]
                    is_touchdown = is_stance and not was_stance
                    
                    # Activate impulse ONLY on the exact touchdown node
                    model.impulses.changeImpulseStatus(self.impacts[i], is_touchdown)
                    
            else:
                if not isinstance(self.problem.runningModels[k], croc.IntegratedActionModelEuler):
                    self.problem.updateModel(k, self.continuous_models[k])

                model = self.problem.runningModels[k]
                diff = model.differential
                model.dt = ref_state.dt # Update dt on the active continuous model

                # Count active contacts to calculate ideal feedforward force
                num_stance = sum(f.is_contact for f in ref_state.fs)
                f_ref_dynamic = self.model.compute_dynamic_force(num_stance) if num_stance > 0 else pin.Force.Zero()

                # Toggle Foot Statuses
                for i, foot in enumerate(ref_state.fs):
                    is_stance = foot.is_contact
                    
                    # Toggle the math matrices on/off!
                    diff.contacts.changeContactStatus(self.contacts[i], is_stance)
                    diff.costs.changeCostStatus(self.frictions[i], is_stance)
                    diff.costs.changeCostStatus(self.force_regs[i], is_stance)
                    diff.costs.changeCostStatus(self.swings_pos[i], not is_stance)
                    diff.costs.changeCostStatus(self.swings_vel[i], not is_stance)

                    # Update References
                    if is_stance:
                        diff.contacts.contacts[self.contacts[i]].contact.reference = foot.position
                        diff.costs.costs[self.force_regs[i]].cost.residual.reference = f_ref_dynamic
                    else:
                        diff.costs.costs[self.swings_pos[i]].cost.residual.reference = foot.position
                        diff.costs.costs[self.swings_vel[i]].cost.residual.reference = pin.Motion(foot.velocity, np.zeros(3))
                
            # --- UPDATE STATE & COM REFERENCES (Handling both Model Types) ---
            # Determine where the costs live based on model type
            costs_container = model.costs if is_impact_node else model.differential.costs
            
            # State Update
            costs_container.costs["state_reg"].cost.residual.reference = ref_state.as_crocoddyl

            # Save for impact detection on next knot
            prev_contacts = [f.is_contact for f in ref_state.fs]

        terminal_diff = self.terminal_node.differential
        term_state = ref_trajectory[-1]
        
        terminal_diff.costs.costs["state_reg"].cost.residual.reference = term_state.as_crocoddyl

        for i, foot in enumerate(term_state.fs):
            is_stance = foot.is_contact
            terminal_diff.contacts.changeContactStatus(self.contacts[i], is_stance)
            terminal_diff.costs.changeCostStatus(self.swings_pos[i], not is_stance)
            terminal_diff.costs.changeCostStatus(self.swings_vel[i], not is_stance)

            if is_stance:
                terminal_diff.contacts.contacts[self.contacts[i]].contact.reference = foot.position
            else:
                terminal_diff.costs.costs[self.swings_pos[i]].cost.residual.reference = foot.position
                terminal_diff.costs.costs[self.swings_vel[i]].cost.residual.reference = pin.Motion(foot.velocity, np.zeros(3))
            
        # 3. Shift Warm-Start and Solve
        if self.xs is not None and self.us is not None:
            init_xs = self.xs[1:] + [self.xs[-1]]
            init_us = []
            for k in range(num_intervals):
                expected_nu = self.problem.runningModels[k].nu
                if expected_nu > 0:
                    # Try to use previous warm start, otherwise fallback to gravity
                    if k + 1 < len(self.us) and len(self.us[k+1]) > 0:
                        init_us.append(self.us[k+1])
                    else:
                        init_us.append(u_grav.copy())
                else:
                    init_us.append(np.array([])) # Empty control array for impulse node
            max_iters = 50
            init_xs[0] = current_state.as_crocoddyl.copy()
        else:
            init_xs = [state.as_crocoddyl.copy() for state in ref_trajectory]
            if len(init_xs) < num_intervals + 1:
                init_xs.append(init_xs[-1])
                


                init_us = [u_grav.copy()] * num_intervals
                
                # Give the cold start more iterations to find the true minimum
                max_iters = 300

        self.solver.solve(init_xs, init_us, max_iters, is_feasible=False)
        self.xs = list(self.solver.xs)
        self.us = list(self.solver.us)
        return self.solver.iter

    def get_optimized_states(self) -> list[RobotState]:
        num_intervals = self.problem.T
        states = []

        for i in range(num_intervals):
            state = RobotState()
            running_model = self.solver.problem.runningModels[i]
            running_data = self.solver.problem.runningDatas[i]

            state.from_crocoddyl(self.xs[i])
            if isinstance(running_model, croc.ActionModelImpulseFwdDynamics):
                # IMPULSE NODE
                pin_running_data = running_data.multibody.pinocchio
                state.bs.accel[:] = 0.0  # Instantaneous jump, no continuous acceleration
                state.a_joints[:] = 0.0
                
                active_contacts = running_model.impulses.active_set
                contact_forces_local = running_data.multibody.impulses.impulses
            else:
                # CONTINUOUS NODE
                pin_running_data = running_data.differential.multibody.pinocchio
                xout = running_data.differential.xout
                state.bs.accel[:] = xout[:6]
                state.a_joints[:] = xout[6:]
                
                active_contacts = running_model.differential.contacts.active_set
                contact_forces_local = running_data.differential.multibody.contacts.contacts

            for id, foot in enumerate(state.fs):
                pin_id = self.model.pin_foot_ids[id]
                contact_name = self.contacts[id]
                impact_name = self.impacts[id]
                is_active = (contact_name in active_contacts) or (impact_name in active_contacts)
                if is_active:
                    if isinstance(running_model, croc.ActionModelImpulseFwdDynamics):
                         foot.contact_force = np.zeros(3)
                    else:
                         foot.contact_force = contact_forces_local[contact_name].f.linear
                    foot.is_contact = True
                else:
                    foot.contact_force = np.zeros(3)
                    foot.is_contact = False

                foot.position = pin_running_data.oMf[pin_id].translation
                foot.velocity = pin.getFrameVelocity(
                    self.model.rmodel,
                    pin_running_data,
                    pin_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                ).linear
                foot.acceleration = pin.getFrameAcceleration(
                    self.model.rmodel,
                    pin_running_data,
                    pin_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                ).linear

                # if contact_name in active_contacts:
                #     foot.contact_force = contact_forces_local[contact_name].f.linear
                #     foot.is_contact = True
                # else:
                #     foot.contact_force = np.zeros(3)
                #     foot.is_contact = False

                # print(f"Node: {i}, Fz: {foot.contact_force[2]}, {foot.is_contact}")
                # FORCE_THRESHOLD = 10.0
                # foot.is_contact = (
                #     True if foot.contact_force[2] > FORCE_THRESHOLD else False
                # )
            states.append(state.copy())

        return states
