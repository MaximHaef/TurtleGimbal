# TurtleGimbal: System Architecture and Requirements

Implementation of prediction-assisted three-axis gimbal stabilization on a TurtleBot, following the method in `../Bravo_Team.pdf`. Equation numbers below refer to that paper.

Values marked **TBD** are not decided yet. Fill them in as they are measured or chosen.

## 1. Overview

A three-axis gimbal (yaw, pitch, roll) is mounted on a differential-drive TurtleBot. A Raspberry Pi 4 Model B on the robot runs the gimbal software. It receives the robot's velocity commands over ROS2 and uses them to predict the base's angular velocity before the motion happens. The predicted motion is turned into feedforward joint-rate commands. Visual and IMU feedback correct what the prediction misses.

The TurtleBot itself has no IMU. Both IMUs are part of the gimbal: IMU A on the gimbal base and IMU B in the camera frame. The gimbal Pi reads both directly.

```
┌──────────── TurtleBot computer (ROS2) ────────────┐
│ teleop / nav ──/cmd_vel──► base driver ──► wheels │
│                 │            /joint_states, /odom │
└─────────────────┼─────────────────────────────────┘
                  │ Ethernet (wired), same ROS2 distro, chrony time sync
┌─────────────────▼──── Gimbal Pi 4 (Ubuntu + ROS2) ────────────────────┐
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
| Robot base | TurtleBot, model **TBD** (3 or 4) | Runs ROS2, distro **TBD** |
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

### 4.1 Interfaces from the TurtleBot

| Topic | Type | Use |
|---|---|---|
| `/cmd_vel` | `Twist` or `TwistStamped` | Input u_k. Converted to wheel angular-speed commands using the inverse of eq. (5) |
| `/joint_states` | `JointState` | Measured wheel speeds: system identification, optional extra context |
| `/odom` | `Odometry` | Logging, evaluation |

If `/cmd_vel` has no header, each message is stamped when it arrives at the gimbal Pi.

The TurtleBot has no IMU, so no IMU topic comes from the robot. All inertial data comes from the two gimbal IMUs (Sec. 4.3).

### 4.2 Predictor node

- Runs in its own process, so that inference never blocks the control loop.
- Builds the context C_k from the latest command and the last H samples of IMU A.
- Rolls out the physical model (eqs. 4–6, time constants τ_v and τ_ω) and adds residuals sampled by flow matching (eq. 7, N Euler steps).
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
- Identify τ_v and τ_ω from step responses.
- Train the flow-matching model and export it to ONNX.
- Evaluate prediction error and compare controller configurations.

## 5. Requirements

### 5.1 Functional

| ID | Requirement |
|---|---|
| F1 | Receive the TurtleBot's velocity commands over ROS2 without modifying the TurtleBot software. |
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
| T4 | Prediction horizon F·Δt | At least the predictor period + T_a + one-way latency of `/cmd_vel` |
| T5 | Clock offset between the TurtleBot and the gimbal Pi | Below 1 ms (chrony) |
| T6 | Latency of `/cmd_vel` from TurtleBot to gimbal Pi | Measured; much smaller than the motor time constants of the base |
| T7 | Actuation delay T_a | Measured after tuning the motor PI gains (Fig. 3) |
| T8 | CPU temperature during operation | Below the throttling threshold |

### 5.3 Parameters to identify (Table II)

| Parameter | Symbol | Value | Source |
|---|---|---|---|
| Wheel radius | r | **TBD** | TurtleBot specification |
| Track width | b | **TBD** | TurtleBot specification |
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
- **Network:** Wi-Fi gives latency spikes and discovery problems with DDS. Use the wired link.
- **Python timing:** The global interpreter lock (GIL) and garbage collection can cause jitter. Keep inference in a separate process; move the control loop to C++ if T2 is not met.
- **Jacobian singularity:** The Jacobian is singular at cos θ = 0. Damping limits command amplification, but the mechanical range should stay away from it.
- **Translation:** Rotational compensation does not remove image motion caused by translation and scene depth.

## 7. Open Questions

- Which TurtleBot model and ROS2 distro?
- Which DC motors, encoders and motor driver? Does the driver close the velocity loop itself, or must the Pi do it?
- Which IMUs, and on which buses?
- Which camera?
- How much RAM does the Pi 4 have?
