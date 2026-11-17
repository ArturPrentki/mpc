# legacy

Old planning code, kept for reference. **Not built** (`COLCON_IGNORE`).

Both packages are replaced by the convex MPC built into
`src/whole_body_controller` (velocity commands on `/vel_cmd`).

| Package | What it was | Why it was retired |
|---|---|---|
| `model_predictive_control` | Python full-body MPC (Crocoddyl FDDP) publishing `/mpc_trajectory` at 10 Hz | Too slow for reactive walking, friction/limits only as penalties. Known bugs: `RobotState.copy()` drops `dt` (impact nodes never used, plan integrated at 0.02 s but published at 0.04 s spacing), delay compensation overwritten |
| `walk_pattern_generator` | Python trajectory generators (Raibert heuristic, circle, test trajectories, a Crocoddyl variant) and an ONNX/RL policy node (`nets/model_1499.pt`) | Superseded by the built-in planner. Several nodes use message types (`WbcCommand`, `WbcVelocityCommand`) that no longer exist in `wbc_control_msgs` |

To use one again, move it back into `src/`. The whole-body controller still
follows `wbc_control_msgs/MpcTrajectory` on `/mpc_trajectory` when
`mpc.enabled: false`; joint arrays must be in `joint_names` order and feet in
`foot_frames` order.
