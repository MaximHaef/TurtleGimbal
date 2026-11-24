# Physical-model prediction of the base motion over a horizon (predictor node,
# ARCHITECTURE.md 4.2). Base frame: x forward, y left, z up.

std_msgs/Header header                         # stamp = t_k: time of the command used (robot Pi clock); frame_id = base_footprint
float64 dt                                     # s, spacing of the horizon samples
geometry_msgs/Twist command                    # u_k used: linear.x = v_cmd [m/s], angular.z = w_cmd [rad/s]
geometry_msgs/Twist initial_state              # measured at t_k: linear.x = v [m/s] (/odom), angular.z = w_z [rad/s] (Kobuki gyro)
geometry_msgs/Vector3[] linear_acceleration    # F samples at t_k + j*dt, j = 1..F, m/s^2 (forward dv/dt, centripetal v*w_z, 0)
geometry_msgs/Vector3[] angular_velocity       # F samples at t_k + j*dt, j = 1..F, rad/s
