# msg/RobotState.msg

# Body SE3 Pose (Translation + Quaternion Orientation)
geometry_msgs/Pose body_pose

# Pinocchio::Motion Spatial Velocity (Linear + Angular)
geometry_msgs/Twist body_velocity

# Pinocchio::Motion Spatial Acceleration (Linear + Angular)
geometry_msgs/Accel body_acceleration

float64[] joint_positions             # q[7:19]
float64[] joint_velocities            # v[6:18]
float64[] joint_accelerations         # a[6:18]
# Foot target states and optimized contact forces
FootState[] feet