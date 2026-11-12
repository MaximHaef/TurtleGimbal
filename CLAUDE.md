# TurtleGimbal

Prediction-assisted 3-axis gimbal stabilization on a differential-drive TurtleBot.
Full spec: [ARCHITECTURE.md](ARCHITECTURE.md). Method and equation numbers: [Bravo_Team.md](Bravo_Team.md) (the PDF `../Bravo_Team.pdf` is authoritative).

## System

- **Robot:** Kobuki (TurtleBot 2) with its own Raspberry Pi, ROS2 Humble, driver `kobuki_node` (listens on `/commands/velocity`; publishes `/joint_states`, `/odom`, and its built-in gyro on `/sensors/imu_data(_raw)`). Never modify `kobuki_node` or the firmware. The only project node on the robot Pi is `base_command`.
- **Gimbal Pi** (Raspberry Pi 4, Ubuntu Server, ROS2 Humble): runs all gimbal software and normally sends the robot's commands. Wired Ethernet to the robot Pi, chrony time sync.
- **Gimbal:** yaw (outer, base z) → pitch (middle, local y) → roll (inner, local x). `q = [ψ, θ, φ]`.
- **Motors:** DC motors with encoders, velocity mode. Where the velocity PI loop runs is not decided.
- **IMU A:** on the gimbal base, measures base angular velocity ω_gyr. **IMU B:** in the camera frame, measures residual camera motion. Both are read directly by the gimbal Pi over I2C/SPI, not through ROS topics.
- **Camera:** visual orientation feedback e_R.

## Nodes

| Node | Job | Rules |
|---|---|---|
| `predictor` | Context C_k = (u_k, last H samples of IMU A) → physical model (eqs. 4–6) + flow-matching residuals (eq. 7) → M candidate trajectories, published on `/gimbal/prediction` with timestamp | Separate process, slower clock, all M samples in one batch, ONNX Runtime on CPU |
| `gimbal_control` | Weight candidates with IMU A (eq. 8, log domain), take weighted mean at t + T_a (eq. 9), compute q̇_ff + q̇_fb via damped Jacobian pseudo-inverse (eqs. 2, 10), apply `Limit`, send q̇_cmd to motor driver | Fixed-rate real-time thread on a reserved core. Falls back to feedback-only without a valid prediction. Publishes all internal signals |
| `vision` | e_R = Log(ᶜR_Cd)^∨ on `/gimbal/orientation_error`, stamped with the image time | |
| `base_command` (robot Pi) | `/cmd_vel` → hold + `request_timeout` → wheel-speed saturation → `/commands/velocity` (driver) and `/base_command` (`TwistStamped`, executed command) at a fixed rate | Only publisher to `/commands/velocity`. Sends zero when requests stop |
| `keyboard`, `excitation` (gimbal Pi, `base_teleop`) | Command sources publishing `/cmd_vel` | Fixed rate; send zero on exit |

u_k = (v_cmd, ω_z,cmd) comes from `/base_command` (`twist.linear.x`, `twist.angular.z`), stamped on the robot Pi. It is used directly, without converting to wheel speeds; eq. (5) is only for turning measured wheel speeds into v, ω_z. Deviations from the paper are listed in [Paper_changes.md](Paper_changes.md).

## Must hold

- Inference never blocks the control loop.
- Prediction horizon F·Δt ≥ predictor period + T_a + `/base_command` latency.
- The robot stops when the command source or the link fails (`base_command` timeout).
- Never run, deploy to, or edit anything on the robot Pi. Hardware tests and deployment are done by the user; write them as checklists (see `src/base_command/README.md`). Local tests that start ROS nodes use `ROS_LOCALHOST_ONLY=1` and a non-default `ROS_DOMAIN_ID`, so they cannot reach the robot.
- Runtime switch between feedback-only and prediction-assisted control (for comparison).
- Log every input, internal signal and output to rosbag2 (MCAP).
- Stop the motors safely if the control loop, an IMU or the motor driver fails.
- Respect joint-angle, velocity and acceleration limits; keep the mechanical range away from cos θ = 0 (Jacobian singularity).

## Workspace

The repo root is the colcon workspace. Packages in `src/` (all `ament_python`): `gimbal_predictor`, `gimbal_control`, `gimbal_vision`, `gimbal_bringup` (launch file + `config/gimbal.yaml`), `base_teleop` (command sources + `data_collection.launch.py`), and `base_command` (deployed to and built on the robot Pi). Build with `colcon build`, run with `ros2 launch gimbal_bringup gimbal.launch.py`. Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test` (a `typeguard` plugin in `~/.local` breaks pytest otherwise).

## Stack

Python for predictor, vision and tools. Control loop in Python first, moved to C++ if jitter is too high. Training in PyTorch off the Pi, exported to ONNX.

## Conventions

- Units: rad, rad/s, SI everywhere.
- `ᴬR_B` maps vectors from frame B to frame A. Frames: W (world), B (base), C (camera).
- Values still **TBD** (Δt, T_a, H, F, M, N, τ_v, τ_ω, r, b, hardware models) live in ARCHITECTURE.md §5. Do not invent them; update ARCHITECTURE.md when they are measured or chosen.
