# TurtleGimbal: System Architecture and Requirements

Implementation of prediction-assisted three-axis gimbal stabilization on a TurtleBot, following the method in `../Bravo_Team.pdf`. Equation numbers below refer to that paper. Where this implementation deviates from the paper, the deviation is listed in [Paper_changes.md](Paper_changes.md).

Values marked **TBD** are not decided yet. Fill them in as they are measured or chosen.

## 1. Overview

A three-axis gimbal (yaw, pitch, roll) is mounted on a differential-drive TurtleBot. A Raspberry Pi 4 Model B on the robot runs the gimbal software. It receives the robot's velocity commands over ROS2 and uses them to predict the base's angular velocity and acceleration before the motion happens. The predicted angular velocity is turned into feedforward joint-rate commands. The predicted acceleration, as the IMU A accelerometer will measure it (specific force, gravity included), is turned into feedforward joint torques that compensate the inertial and gravity loads on the gimbal links ahead of time. Visual and IMU feedback correct what the prediction misses. This acceleration path goes beyond the paper (Paper_changes.md §6).

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
│  │ context C_k = (u_k, ω_{k-H+1:k}, a_{k-H+1:k})                    │ │
│  │ physics rollout (4)–(6) + flow-matching residuals (7)            │ │
│  │ → M candidate trajectories X̂_k with timestamp t_k                │ │
│  └───────────────┬──────────────────────────────────────────────────┘ │
│                  │ /gimbal/prediction                                 │
│  ┌───────────────▼─ gimbal_control node (real-time thread, Δt) ─────┐ │
│  │ IMU A (gyro, accel) ► proposal weights (8)                       │ │
│  │                      weighted mean at t + T_a, t + T_τ (9)       │ │
│  │ IMU B, e_R ─────────► q̇_ff, q̇_fb, Limit (10) ──► motor driver    │ │
│  │ â(t + T_τ) ─────────► τ_ff = G(q) â (11) ───────► motor driver   │ │
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
| IMU A | On the gimbal base. CEVA/Bosch **BNO08x** (board labelled HW-921 / GY-BNO055, but the chip is a BNO08x: SHTP protocol, address 0x4B). Software I2C bus 3 (`i2c-gpio` overlay, SDA GPIO23 / pin 16, SCL GPIO24 / pin 18), 3.3 V | Measures base angular velocity ω_gyr. Read by the gimbal Pi. Software I2C because the BNO08x stretches the clock, which the Pi's hardware I2C handles badly. PS0/PS1 are not broken out, so the board is I2C-only. INT not wired yet (data-ready; worth wiring for the control loop) |
| IMU B | In the camera frame of the gimbal, model and bus **TBD** (if also a BNO08x: other address 0x4A, or its own `i2c-gpio` bus) | Measures residual camera motion. Read by the gimbal Pi |
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
- Subscribes to `/base_command` and builds the context C_k from the latest command u_k = (v_cmd, ω_z,cmd), normalized, and the last H samples of IMU A: gyro ω and accelerometer a, both calibrated and rotated into the base frame B. A command older than `command_timeout` is not used; the predictor then publishes no prediction and gimbal_control falls back to feedback-only.
- Rolls out the physical model and adds residuals sampled by flow matching (eq. 7, N Euler steps). The predicted state is x = [v, ω, a] (d_x = 7) and the residual covers ω and a (d_r = 6), see Paper_changes.md §6. The physical model is eq. (6) driven directly by u_k. The wheel-speed limit is already applied by `base_command`, and eq. (5) is not used here. Time constants τ_v and τ_ω.
- **Physical model (implemented, `physical_model.py`):**
  - **Start state:** v from `/odom` and ω_z from the Kobuki gyro (`/sensors/imu_data`), each the newest sample at or before the command stamp t_k (at most `state_timeout` older).
  - **Rollout:** eq. (6) solved exactly with u_k held, for F samples at t_k + j·Δt.
  - **Output** on `/gimbal/physical_prediction` (`gimbal_interfaces/BaseMotionPrediction`, stamp t_k), in the base frame:
    - angular velocity [0, 0, ω_z]
    - translational acceleration [dv/dt, v·ω_z, 0] (forward + centripetal, without gravity)
  - **TODO (accelerometer prediction):** the target quantity is the specific force at IMU A, as the accelerometer measures it: a = [dv/dt, v·ω_z, 0] − ᴮg + ω̇ × r_A + ω × (ω × r_A), with ᴮg = [0, 0, −g] (level ground) and the lever arm r_A from the base's center of rotation to IMU A. The current output lacks the gravity and lever-arm terms; `BaseMotionPrediction.linear_acceleration` changes meaning with them.
  - **Clocks:** all inputs are stamped on the robot Pi and only compared with each other. Whether the command is current is judged by its arrival time on the gimbal Pi, so a clock offset between the Pis does not stop the predictor.
  - **Parameters** in `gimbal.yaml`: `tau_v`, `tau_omega` (measured); `horizon_dt` (= Δt) and `horizon_steps` (= F) are TBD, and the node does not start without them.
- Runs all M samples as one batch.
- Publishes the M candidate trajectories X̂_k with the timestamp of step k.
- Runs slower than the control loop. The horizon covers the time until the next inference plus T_a.

### 4.3 gimbal_control node

- Runs a fixed-rate loop with period Δt in a real-time thread pinned to a reserved CPU core.
- Reads IMU A (gimbal base) and IMU B (camera frame) directly over I2C or SPI on the gimbal Pi, not through ROS topics. `gimbal_control/imu.py`: `ImuReader` polls the sensor in a background thread so a slow read never blocks the loop; `open_bno08x()` is the IMU A driver (gyro + accelerometer). Until the control loop reads IMU A itself, the standalone `imu_node` publishes it for data collection (`sensor_msgs/Imu`, angular velocity and linear acceleration incl. gravity, no orientation, sensor frame `imu_a`): `/gimbal/imu_a` with the static calibration applied, `/gimbal/imu_a/raw` with the sensor reports, so bags can be calibrated again later. Never run both on the same sensor. Tested on the Pi: gyro reports at 100 and 200 Hz arrive at the requested rate without read errors; at 250 Hz and above the stream stalls silently (no errors, the watchdog catches it), so 200 Hz is the current limit for IMU A. Static calibration of IMU A (`imu_calibrate`, `imu_calibration.py`, see `src/gimbal_control/README.md`): accelerometer matrix K (upper triangular) and bias fitted to ‖a‖ = g over still poses, gyro bias = mean at rest; `imu_node` applies it when `calibration_file` is set.
- **IMU A ground truth:** the gyro and the accelerometer are both ground truth: for the weights (eq. 8), the context and the residual training targets. Before use, each sample is corrected with the static calibration (accelerometer K and b_a, gyro b_g) and rotated into B with the mounting rotation ᴮR_A. ᴮR_A and the lever arm r_A still have to be measured (§5.3). Keep the BNO08x configuration (reports, dynamic calibration on/off) the same for calibration, data collection and operation.
- Updates the proposal weights with each IMU A sample (eq. 8, log domain). First version: gyro innovation only.
- Takes the weighted-mean prediction at t + T_a, chosen by timestamp (eq. 9).
- Computes q̇_ff and q̇_fb with the damped pseudo-inverse of the Jacobian (eqs. 2, 10).
- Applies `Limit` (joint-angle, velocity and acceleration limits) and sends q̇_cmd to the motor driver.
- **Inertial feedforward:** takes the weighted-mean specific force â at t + T_τ (eq. 9, by timestamp) and computes τ_ff = G(q) â (Paper_changes.md §6, eq. 11). G(q) comes from the link masses and centers of mass; a perfectly balanced gimbal has G = 0. How τ_ff reaches the motors depends on the driver (current/torque feedforward, or added to the output of the velocity PI loop if the Pi runs it); see §7. Limited to the motor torque/current limits.
- Falls back to feedback-only control when no valid prediction is available.
- Publishes ω_gyr, a_acc, ω̂, â, q, q̇_ff, q̇_fb, q̇_cmd, τ_ff and the proposal weights for logging.

### 4.4 Vision node

- Takes images from the camera driver (`v4l2_camera` / `usb_cam`, or `camera_ros` for the Pi Camera Module).
- Computes the orientation error e_R = Log(ᶜR_Cd)^∨ and publishes it with the image timestamp.
- Starts with static, distant scenes or rotation-dominant motion, as in the paper.

### 4.5 Offline tools (not on the Pi)

- Extract training data from rosbags: contexts C^(i) and residual trajectories R^(i) relative to the physical model, for ω and a. The accelerometer targets come from the calibrated IMU A accelerometer in B. Wheel and floor vibration make it much noisier than the gyro, so the targets probably need low-pass filtering to the bandwidth the gimbal motors can follow.
- Calibrate IMU A: static accelerometer/gyro calibration (`imu_calibrate`, `src/gimbal_control/README.md`), then mounting rotation ᴮR_A and lever arm r_A (not implemented yet).
- Identify the gimbal mass properties for G(q) (CAD, or static joint torques at several base tilts) and the torque-path delay T_τ.
- Identify τ_v and τ_ω (and the base dead time T_d) from step responses: `tools/sysid/fit_base_dynamics.py` fits a first-order-plus-dead-time model to `/base_command` against the Kobuki gyro (ω_z) and wheel odometry (v). Sequence `base_teleop/config/sysid_steps.yaml`; procedure in `tools/sysid/README.md`.
- Training-data runs (F9), run from the gimbal Pi (IMU A, excitation and recording there; only the relay on the robot Pi; chrony must sync the Pis because IMU A and the robot topics are stamped on different clocks): `tools/excitation/make_sequence.py` generates the excitation sequence (designed blocks + random segments, commanded path kept inside the free area, wheel speed below 0.9 × limit); `tools/excitation/check_bag.py` checks each bag right after recording. Procedure in `tools/excitation/README.md`.
- Measure the Kobuki wheel-speed limit.
- Train the flow-matching model and export it to ONNX.
- Evaluate prediction error and compare controller configurations.

## 5. Requirements

### 5.1 Functional

| ID | Requirement |
|---|---|
| F1 | Drive the base from the gimbal Pi (or any other ROS2 machine) and receive the executed command, stamped, on the gimbal Pi. The Kobuki driver and firmware are not modified; `base_command` is the only project node on the robot Pi. |
| F2 | Read the angular velocity of IMU A (gimbal base) and IMU B (camera frame), and the acceleration of IMU A, on the gimbal Pi at the control rate. |
| F3 | Predict the base angular velocity and the specific force at IMU A over a horizon of F steps, as M candidate trajectories (physical model plus learned residual). |
| F4 | Weight the candidates with IMU A measurements (eq. 8) and provide the weighted prediction at t + T_a (eq. 9). |
| F5 | Compute feedforward and feedback joint-rate commands (eq. 10) and send the limited command to the motors. |
| F6 | Compute the visual orientation error and use it as feedback. |
| F7 | Switch at runtime between feedback-only and prediction-assisted control, and turn the inertial feedforward on and off independently, so the configurations can be compared. |
| F8 | Record all inputs, internal signals and outputs with timestamps in a rosbag. |
| F9 | Support a data-collection mode that runs excitation trajectories and records training data. |
| F10 | Stop the gimbal motors safely when the control loop, the IMUs or the motor driver fail. |
| F11 | Compensate the predicted inertial and gravity loads on the gimbal joints with a torque feedforward from the predicted specific force at t + T_τ (Paper_changes.md §6). |

### 5.2 Timing and performance

| ID | Requirement | Value |
|---|---|---|
| T1 | Control period Δt | **TBD** |
| T2 | Control-loop jitter | Small compared to Δt; target **TBD** |
| T3 | Predictor inference time (M samples, N steps) | Shorter than the predictor period; checked offline before integration (Fig. 3) |
| T4 | Prediction horizon F·Δt | At least the predictor period + max(T_a, T_τ) + one-way latency of `/base_command` |
| T5 | Clock offset between the TurtleBot and the gimbal Pi | Below 1 ms (chrony) |
| T6 | Latency of `/base_command` from robot Pi to gimbal Pi | Measured (receive time − header stamp); much smaller than the motor time constants of the base |
| T7 | Actuation delay T_a | Measured after tuning the motor PI gains (Fig. 3) |
| T7b | Torque-path delay T_τ | Measured, once the driver's torque/current interface is known |
| T8 | CPU temperature during operation | Below the throttling threshold |
| T9 | Rate of `/commands/velocity` and `/base_command` | `publish_rate`, 50 Hz (design choice) |
| T10 | Robot stops after the command source or the link fails | Within `request_timeout`, 0.5 s (design choice, below `kobuki_node`'s 0.6 s) |

### 5.3 Parameters to identify (Table II)

| Parameter | Symbol | Value | Source |
|---|---|---|---|
| Wheel radius | r | 0.035 m (to verify) | Kobuki specification. Only for converting `/joint_states` |
| Track width | b | 0.230 m (to verify) | Kobuki specification. Only for converting `/joint_states` and the wheel-speed limit (`base_command` parameter `track_width`) |
| Wheel-speed limit | v_max | about 0.7 m/s (to verify) | Kobuki firmware; measured (`base_command` parameter `wheel_speed_limit`) |
| Speed time constant | τ_v | 94 ms | Step response, `tools/sysid`, run `sysid_20260314_0116` (one run, gimbal mounted). First-order fit (R² 0.96); gain K = 0.74, T_d ≈ 0 |
| Yaw-rate time constant | τ_ω | 120 ms | Step response, same run. First-order fit (R² 0.94); gain K = 0.66, T_d = 38 ms |
| Base dead time (command → motion) | T_d | 38 ms (yaw), 0 ms (forward) | Same fit. Not in eq. (6); the physical model keeps K = 1 and T_d = 0, and the learned residual covers the rest (see note below) |
| IMU A mounting rotation | ᴮR_A | **TBD** | Measured (gravity direction at rest + forward drive) |
| IMU A lever arm | r_A | **TBD** | Measured from the base's center of rotation (midpoint between the wheels) |
| IMU A accelerometer calibration | K, b_a | **TBD** | `imu_calibrate` |
| Gimbal mass properties | m_ℓ, r_iℓ | **TBD** | CAD or static torque measurement; give G(q) |
| Torque-path delay | T_τ | **TBD** | Measured |
| Control period | Δt | **TBD** | Design choice |
| Actuation delay | T_a | **TBD** | Measured |
| History length | H | **TBD** | Design choice |
| Prediction horizon | F | **TBD** | From T4 |
| Trajectory samples | M | **TBD** | Limited by Pi 4 compute |
| Flow integration steps | N | **TBD** | Limited by Pi 4 compute |

**Note on the base fit (`tools/sysid`):** The Kobuki is only roughly first-order. The values above are a compromise over all steps. What the learned residual has to cover:
- **Speed shortfall:** the base reaches only 65–80% of the commanded speed, less at low wheel speeds (46% at 0.035 m/s). The wheel encoders and the gyro agree, so this is real.
- **Starts vs. stops:** starts have ~130 ms (forward) / ~190 ms (yaw) dead time followed by a jump with overshoot; stops react within ~20–60 ms.
- **Jolts:** at starts and stops, yaw-rate spikes reach up to 1.5× the final value.

Repeat the run with a full battery before relying on the numbers.

## 6. Constraints and Risks

- **Compute:** The Pi 4 limits the size of the flow-matching network and M × N. Measure inference time early.
- **ROS2 distro:** Communication between different ROS2 distros is not officially supported. Both machines must run the same distro.
- **Network:** Wi-Fi gives latency spikes and discovery problems with DDS. Use the wired link. Both Pis need the same `ROS_DOMAIN_ID` (robot Pi: default 0). Setup (static cable addresses, chrony, Fast DDS restricted to the cable, laptop access): [NETWORK.md](NETWORK.md).
- **Python timing:** The global interpreter lock (GIL) and garbage collection can cause jitter. Keep inference in a separate process; move the control loop to C++ if T2 is not met.
- **Jacobian singularity:** The Jacobian is singular at cos θ = 0. Damping limits command amplification, but the mechanical range should stay away from it.
- **Translation:** Rotational compensation does not remove image motion caused by translation and scene depth.
- **Accelerometer noise:** Wheel and floor vibration put high-frequency content on IMU A's accelerometer that no command-based model can predict. It enters the context, the residual targets and possibly the weights; filter it to the useful bandwidth.
- **Inertial feedforward and balance:** τ_ff only acts through imbalance (G ≠ 0). A well-balanced gimbal gains little from it, and a wrong G(q) adds a disturbance instead of removing one. Measure G before relying on it.
- **Command saturation:** Commands beyond the Kobuki wheel-speed limit are scaled down by the firmware. If `wheel_speed_limit` in `base_command` is wrong, `/base_command` no longer matches what the base executes and the prediction is off. Keep commands within the limit where possible.

## 7. Open Questions

- Use the Kobuki gyro (`/sensors/imu_data_raw`) as extra context or as a reference for IMU A?
- The residual now has to cover acceleration (d_r = 6); `tools/flow_predictor` still has 3 channels. Should the proposal weights (eq. 8) also use the accelerometer innovation, and with which Σ^acc and P_j?
- How does τ_ff reach the motors: does the driver accept a current/torque feedforward, or must the Pi close the velocity loop and add τ_ff to its output?
- Add the base angular-acceleration load (−I ω̇, from the predicted ω trajectory) to τ_ff?
- Which DC motors, encoders and motor driver? Does the driver close the velocity loop itself, or must the Pi do it?
- Which IMUs, and on which buses?
- Which camera?
- How much RAM does the Pi 4 have?
