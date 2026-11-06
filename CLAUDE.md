# TurtleGimbal

Prediction-assisted 3-axis gimbal stabilization on a differential-drive TurtleBot.
Full spec: [ARCHITECTURE.md](ARCHITECTURE.md). Method and equation numbers: [Bravo_Team.md](Bravo_Team.md) (the PDF `../Bravo_Team.pdf` is authoritative).

## System

- **TurtleBot** (ROS2): publishes `/cmd_vel`, `/joint_states`, `/odom`. It has **no IMU**. Never modify the TurtleBot software.
- **Gimbal Pi** (Raspberry Pi 4, Ubuntu Server, same ROS2 distro as the TurtleBot): runs all gimbal software. Wired Ethernet to the TurtleBot, chrony time sync.
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

`/cmd_vel` → wheel angular-speed commands u_k via the inverse of eq. (5). If `/cmd_vel` has no header, stamp it on arrival.

## Must hold

- Inference never blocks the control loop.
- Prediction horizon F·Δt ≥ predictor period + T_a + `/cmd_vel` latency.
- Runtime switch between feedback-only and prediction-assisted control (for comparison).
- Log every input, internal signal and output to rosbag2 (MCAP).
- Stop the motors safely if the control loop, an IMU or the motor driver fails.
- Respect joint-angle, velocity and acceleration limits; keep the mechanical range away from cos θ = 0 (Jacobian singularity).

## Workspace

The repo root is the colcon workspace. Packages in `src/` (all `ament_python`): `gimbal_predictor`, `gimbal_control`, `gimbal_vision`, `gimbal_bringup` (launch file + `config/gimbal.yaml`). Build with `colcon build`, run with `ros2 launch gimbal_bringup gimbal.launch.py`.

## Stack

Python for predictor, vision and tools. Control loop in Python first, moved to C++ if jitter is too high. Training in PyTorch off the Pi, exported to ONNX.

## Conventions

- Units: rad, rad/s, SI everywhere.
- `ᴬR_B` maps vectors from frame B to frame A. Frames: W (world), B (base), C (camera).
- Values still **TBD** (Δt, T_a, H, F, M, N, τ_v, τ_ω, r, b, hardware models) live in ARCHITECTURE.md §5. Do not invent them; update ARCHITECTURE.md when they are measured or chosen.
