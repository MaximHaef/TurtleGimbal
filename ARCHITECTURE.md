# TurtleGimbal: System Architecture and Requirements

Implementation of prediction-assisted three-axis gimbal stabilization on a TurtleBot, following the method in `../Bravo_Team.pdf`. Equation numbers below refer to that paper. Where this implementation deviates from the paper, the deviation is listed in [Paper_changes.md](Paper_changes.md).

Values marked **TBD** are not decided yet. Fill them in as they are measured or chosen.

## 1. Overview

A three-axis gimbal (yaw, pitch, roll) is mounted on a differential-drive TurtleBot. A Raspberry Pi 4 Model B on the robot runs the gimbal software. It receives the robot's velocity commands over ROS2 and uses them to predict the base's angular velocity before the motion happens. The predicted motion is turned into feedforward joint-rate commands. Visual and IMU feedback correct what the prediction misses.

The robot base is a Kobuki (TurtleBot 2). Its velocity command is the body velocity u_k = (v_cmd, ω_z,cmd), not wheel speeds (Sec. 4.1). Commands are normally sent from the gimbal Pi. A relay node on the robot Pi forwards them to the driver and echoes back the command actually executed, with a timestamp, so the predictor works whichever machine sends the commands. The Kobuki has a built-in gyro, but the method uses the gimbal's own IMUs: IMU A on the gimbal base and IMU B in the camera frame. The gimbal Pi reads both directly.

```
┌──────────── Robot Pi (Kobuki, ROS2) ──────────────────────────────────┐
│ /cmd_vel ──► base_command ──/commands/velocity──► kobuki_node ──► base │
│   ▲            │ (hold, timeout, saturate,      /joint_states, /odom,  │
│   │            │  fixed rate)                   /sensors/imu_data      │
└───┼────────────┼───────────────────────────────────────────────────────┘
    │            │ /base_command (TwistStamped, executed u_k)
    │            │ Ethernet (wired), same ROS2 distro, chrony time sync
┌───┼────────────▼────── Gimbal Pi 4 (Ubuntu + ROS2) ────────────────────┐
│ base_teleop: keyboard / excitation (or any other command source)      │
│                                                                       │
│  ┌─ predictor node (separate process, slow clock) ──────────────────┐ │
│  │ context C_k = (u_k, ω_{k-H+1:k})                                 │ │
│  │ physics rollout (4)–(6) + flow-matching residuals (7)            │ │
│  │ → M candidate trajectories X̂_k with timestamp t_k                │ │
│  └───────────────┬──────────────────────────────────────────────────┘ │
│                  │ /gimbal/prediction                                 │
│  ┌───────────────▼─ gimbal_control node (real-time thread, Δt) ─────┐ │
│  │ IMU A (base gyro) ──► proposal weights (8)                       │ │
│  │                      weighted mean at t + T_a (9)                │ │
│  │ IMU B, e_R ─────────► q̇_ff, q̇_fb, Limit (10) ──► motor driver    │ │
│  │ publishes internal signals for logging                           │ │
│  └───────────────▲──────────────────────────────────────────────────┘ │
│                  │ /gimbal/orientation_error (e_R + image stamp)      │
│  ┌───────────────┴─ camera driver + vision node ────────────────────┐ │
│  └──────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────────┘
```

## 2. Hardware

| Component | Choice | Notes |
|---|---|---|
| Robot base | Kobuki (TurtleBot 2) with its own Raspberry Pi | ROS2 Humble, `kobuki_ros` driver (`kobuki_core` 1.4.1). The firmware runs the wheel velocity loops |
| Gimbal computer | Raspberry Pi 4 Model B, RAM **TBD** | 4 GB or more recommended |
| Gimbal | 3 axes: yaw (outer), pitch (middle), roll (inner) | Axis order must be verified on the hardware (Sec. II-A) |
| Gimbal motors | DC motors with encoders, model **TBD** | Driven in velocity mode. The motor-level PI velocity loop needs encoder feedback; where it runs (motor driver or Pi) is **TBD** |
| IMU A | On the gimbal base, model and bus **TBD** | Measures base angular velocity ω_gyr. Read by the gimbal Pi |
| IMU B | In the camera frame of the gimbal, model and bus **TBD** | Measures residual camera motion. Read by the gimbal Pi |
| Camera | USB or Pi Camera Module, **TBD** | Used for visual orientation feedback |
| Link to TurtleBot | Short Ethernet cable | Not Wi-Fi |
| Power | 5 V / 3 A from robot battery via buck converter | Separate supply for the gimbal motors |
| Cooling | Heatsink and fan on the Pi | The Pi slows itself down around 80 °C |

## 3. Software Platform

- **OS:** Ubuntu Server, version matching the TurtleBot's ROS2 distro (22.04 for Humble, 24.04 for Jazzy). No desktop.
- **Middleware:** ROS2, same distro as the TurtleBot (`ros-<distro>-ros-base`).
- **Languages:** Python for the predictor, vision and tooling. The control loop starts in Python and moves to C++ if timing requirements are not met.
- **Inference:** PyTorch for training (off the Pi), ONNX Runtime on the CPU of the Pi.
- **Logging:** rosbag2 (MCAP storage), viewable in Foxglove or PlotJuggler.

## 4. Components

### 4.1 Robot base interface

#### Topics

| Topic | Type | Use |
|---|---|---|
| `/cmd_vel` | `Twist` | Command requests from any source (gimbal Pi teleop/excitation, later navigation or another machine). Only `base_command` subscribes |
| `/commands/velocity` | `Twist` | `base_command` → `kobuki_node`. No other node publishes here |
| `/base_command` | `TwistStamped` | **Input u_k** = (v_cmd, ω_z,cmd) = (`twist.linear.x`, `twist.angular.z`), in m/s and rad/s: the command actually sent to the driver (after timeout and saturation), stamped on the robot Pi when it was sent. Used directly; no conversion to wheel speeds |
| `/joint_states` | `JointState` | Measured wheel speeds. Converted to measured v and ω_z with r and b (eq. 5) for system identification and residual targets |
| `/odom` | `Odometry` | Logging, evaluation |
| `/debug/raw_control_command` | `Int16MultiArray` | The (speed [mm/s], radius [mm]) pair actually sent to the firmware. Logging; checks for saturation |
| `/sensors/imu_data`, `/sensors/imu_data_raw` | `Imu` | Kobuki gyro (yaw rate; raw 3-axis). Logging; possible reference or extra context |

All inertial data used by the controller comes from the two gimbal IMUs (Sec. 4.3).

#### How the Kobuki executes a command

- `kobuki_core` converts (v_cmd, ω_z,cmd) into (speed, radius), the only drive command of the Kobuki protocol. Speed is the outer-wheel speed; radius is an int16 in mm, so |radius| ≤ 32.767 m, and |radius| ≤ 1 mm becomes a turn in place.
- The firmware limits wheel speed (about 0.7 m/s, to be verified). An over-limit command is scaled down along the same radius, so both v and ω_z come out smaller than commanded. `base_command` applies the same limit before sending, so `/base_command` already contains the scaled command.
- `kobuki_node` zeroes the command if no message arrives for `cmd_vel_timeout_sec` (0.6 s).

#### base_command node (robot Pi, package `base_command`)

The only project node on the robot Pi. `kobuki_node` and the firmware are not modified.

- Subscribes to `/cmd_vel` and holds the latest request.
- Every 1/`publish_rate` s: uses the held request, or zero if none arrived within `request_timeout` (the robot stops if the source or the network fails), applies the wheel-speed limit, and publishes the result to `/commands/velocity` and, stamped, to `/base_command`.
- Sends zero on shutdown.
- Started with `kobuki_node` by `base_command.launch.py`, optionally at boot as a systemd service. Deployment and hardware checklist: `src/base_command/README.md`.

#### Command sources (gimbal Pi, package `base_teleop`)

- `keyboard`: hold-to-drive keyboard teleop, publishes `/cmd_vel` at a fixed rate.
- `excitation`: plays a YAML sequence of (duration, v, w) segments on `/cmd_vel` for data collection (F9), publishes the active segment index on `/excitation/segment`, sends zero at the end and on Ctrl-C.
- `data_collection.launch.py`: excitation + `ros2 bag record` (MCAP), optionally with the gimbal nodes.

Any other machine can drive the robot by publishing `/cmd_vel`; only one source should publish at a time.

### 4.2 Predictor node

- Runs in its own process, so that inference never blocks the control loop.
- Subscribes to `/base_command` and builds the context C_k from the latest command u_k = (v_cmd, ω_z,cmd), normalized, and the last H samples of IMU A. A command older than `command_timeout` is not used; the predictor then publishes no prediction and gimbal_control falls back to feedback-only.
- Rolls out the physical model and adds residuals sampled by flow matching (eq. 7, N Euler steps). The physical model is eq. (6) driven directly by u_k. The wheel-speed limit is already applied by `base_command`, and eq. (5) is not used here. Time constants τ_v and τ_ω.
- Runs all M samples as one batch.
- Publishes the M candidate trajectories X̂_k with the timestamp of step k.
- Runs slower than the control loop. The horizon covers the time until the next inference plus T_a.

### 4.3 gimbal_control node

- Runs a fixed-rate loop with period Δt in a real-time thread pinned to a reserved CPU core.
- Reads IMU A (gimbal base) and IMU B (camera frame) directly over I2C or SPI on the gimbal Pi, not through ROS topics.
- Updates the proposal weights with each IMU A sample (eq. 8, log domain).
- Takes the weighted-mean prediction at t + T_a, chosen by timestamp (eq. 9).
- Computes q̇_ff and q̇_fb with the damped pseudo-inverse of the Jacobian (eqs. 2, 10).
- Applies `Limit` (joint-angle, velocity and acceleration limits) and sends q̇_cmd to the motor driver.
- Falls back to feedback-only control when no valid prediction is available.
- Publishes ω_gyr, ω̂, q, q̇_ff, q̇_fb, q̇_cmd and the proposal weights for logging.

### 4.4 Vision node

- Takes images from the camera driver (`v4l2_camera` / `usb_cam`, or `camera_ros` for the Pi Camera Module).
- Computes the orientation error e_R = Log(ᶜR_Cd)^∨ and publishes it with the image timestamp.
- Starts with static, distant scenes or rotation-dominant motion, as in the paper.

### 4.5 Offline tools (not on the Pi)

- Extract training data from rosbags: contexts C^(i) and residual trajectories R^(i) relative to the physical model.
- Identify τ_v and τ_ω from step responses of v_cmd and ω_z,cmd, using v and ω_z measured from `/joint_states` (eq. 5) or the gyros.
- Measure the Kobuki wheel-speed limit.
- Train the flow-matching model and export it to ONNX.
- Evaluate prediction error and compare controller configurations.

## 5. Requirements

### 5.1 Functional

| ID | Requirement |
|---|---|
| F1 | Drive the base from the gimbal Pi (or any other ROS2 machine) and receive the executed command, stamped, on the gimbal Pi. The Kobuki driver and firmware are not modified; `base_command` is the only project node on the robot Pi. |
| F2 | Read the angular velocity of IMU A (gimbal base) and IMU B (camera frame) on the gimbal Pi at the control rate. |
| F3 | Predict the base angular velocity over a horizon of F steps, as M candidate trajectories (physical model plus learned residual). |
| F4 | Weight the candidates with IMU A measurements (eq. 8) and provide the weighted prediction at t + T_a (eq. 9). |
| F5 | Compute feedforward and feedback joint-rate commands (eq. 10) and send the limited command to the motors. |
| F6 | Compute the visual orientation error and use it as feedback. |
| F7 | Switch at runtime between feedback-only and prediction-assisted control, so the two can be compared. |
| F8 | Record all inputs, internal signals and outputs with timestamps in a rosbag. |
| F9 | Support a data-collection mode that runs excitation trajectories and records training data. |
| F10 | Stop the gimbal motors safely when the control loop, the IMUs or the motor driver fail. |

### 5.2 Timing and performance

| ID | Requirement | Value |
|---|---|---|
| T1 | Control period Δt | **TBD** |
| T2 | Control-loop jitter | Small compared to Δt; target **TBD** |
| T3 | Predictor inference time (M samples, N steps) | Shorter than the predictor period; checked offline before integration (Fig. 3) |
| T4 | Prediction horizon F·Δt | At least the predictor period + T_a + one-way latency of `/base_command` |
| T5 | Clock offset between the TurtleBot and the gimbal Pi | Below 1 ms (chrony) |
| T6 | Latency of `/base_command` from robot Pi to gimbal Pi | Measured (receive time − header stamp); much smaller than the motor time constants of the base |
| T7 | Actuation delay T_a | Measured after tuning the motor PI gains (Fig. 3) |
| T8 | CPU temperature during operation | Below the throttling threshold |
| T9 | Rate of `/commands/velocity` and `/base_command` | `publish_rate`, 50 Hz (design choice) |
| T10 | Robot stops after the command source or the link fails | Within `request_timeout`, 0.5 s (design choice, below `kobuki_node`'s 0.6 s) |

### 5.3 Parameters to identify (Table II)

| Parameter | Symbol | Value | Source |
|---|---|---|---|
| Wheel radius | r | 0.035 m (to verify) | Kobuki specification. Only for converting `/joint_states` |
| Track width | b | 0.230 m (to verify) | Kobuki specification. Only for converting `/joint_states` and the wheel-speed limit (`base_command` parameter `track_width`) |
| Wheel-speed limit | v_max | about 0.7 m/s (to verify) | Kobuki firmware; measured (`base_command` parameter `wheel_speed_limit`) |
| Speed time constant | τ_v | **TBD** | Step response |
| Yaw-rate time constant | τ_ω | **TBD** | Step response |
| Control period | Δt | **TBD** | Design choice |
| Actuation delay | T_a | **TBD** | Measured |
| History length | H | **TBD** | Design choice |
| Prediction horizon | F | **TBD** | From T4 |
| Trajectory samples | M | **TBD** | Limited by Pi 4 compute |
| Flow integration steps | N | **TBD** | Limited by Pi 4 compute |

## 6. Constraints and Risks

- **Compute:** The Pi 4 limits the size of the flow-matching network and M × N. Measure inference time early.
- **ROS2 distro:** Communication between different ROS2 distros is not officially supported. Both machines must run the same distro.
- **Network:** Wi-Fi gives latency spikes and discovery problems with DDS. Use the wired link. Both Pis need the same `ROS_DOMAIN_ID` (robot Pi: default 0).
- **Python timing:** The global interpreter lock (GIL) and garbage collection can cause jitter. Keep inference in a separate process; move the control loop to C++ if T2 is not met.
- **Jacobian singularity:** The Jacobian is singular at cos θ = 0. Damping limits command amplification, but the mechanical range should stay away from it.
- **Translation:** Rotational compensation does not remove image motion caused by translation and scene depth.
- **Command saturation:** Commands beyond the Kobuki wheel-speed limit are scaled down by the firmware. If `wheel_speed_limit` in `base_command` is wrong, `/base_command` no longer matches what the base executes and the prediction is off. Keep commands within the limit where possible.

## 7. Open Questions

- Use the Kobuki gyro (`/sensors/imu_data_raw`) as extra context or as a reference for IMU A?
- Which DC motors, encoders and motor driver? Does the driver close the velocity loop itself, or must the Pi do it?
- Which IMUs, and on which buses?
- Which camera?
- How much RAM does the Pi 4 have?
