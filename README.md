# TurtleGimbal

Prediction-assisted stabilization of a 3-axis camera gimbal on a TurtleBot.

When the robot turns, its base rotates and shakes the camera. A gimbal that only reacts to measured motion is always late, because its motors need time to respond. This project predicts the base motion from the velocity commands sent to the robot, so the gimbal can start compensating before the motion happens. Camera and IMU feedback then correct whatever the prediction got wrong.

## Documents in this repo

| File | What it is | Read it when |
|---|---|---|
| [Bravo_Team.md](Bravo_Team.md) | Our paper (Markdown version of `Bravo_Team.pdf`). Describes the **method**: gimbal kinematics, the physical robot model, the flow-matching predictor, how predictions are weighted with the gyroscope, and the control law. All equation numbers in the code and docs refer to this paper. Figures are in [paper_figures/](paper_figures/). | You want to understand *why* something is computed the way it is, or need an equation. |
| [ARCHITECTURE.md](ARCHITECTURE.md) | The **engineering spec**: hardware, software platform, ROS2 nodes and interfaces, requirements (functional F1–F10, timing T1–T8), parameters still to be identified, risks and open questions. | You want to know *what* to build, how the parts connect, or what a requirement says. |
| [CLAUDE.md](CLAUDE.md) | A condensed summary of the system and rules for the Claude Code AI assistant. Also a good one-page cheat sheet. | You use Claude Code in this repo, or want the short version. |

Values marked **TBD** in ARCHITECTURE.md are not decided yet. When you measure or choose one, update ARCHITECTURE.md in the same commit.

## System at a glance

- **Robot:** Kobuki (TurtleBot 2, differential drive) with its own Raspberry Pi running ROS2 Humble and the `kobuki_node` driver. It publishes wheel speeds, odometry and the data of its built-in gyro. We do not modify the driver; our only node there is the `base_command` relay.
- **Gimbal Pi** (Raspberry Pi 4, Ubuntu + ROS2 Humble): runs all other code and normally sends the robot's velocity commands. Connected to the robot Pi by Ethernet, clocks synced with chrony.
- **Gimbal:** yaw (outer) → pitch (middle) → roll (inner), driven by DC motors with encoders in velocity mode.
- **IMU A** on the gimbal base and **IMU B** in the camera frame. Both are read directly by the gimbal Pi over I2C/SPI, not through ROS.
- **Camera** for visual orientation feedback.

## Node structure

The robot is driven by publishing `/cmd_vel` (from the gimbal Pi or any other machine). The `base_command` relay on the robot Pi forwards it to the driver and echoes the executed command, stamped, on `/base_command`, which is the predictor's input. The gimbal software runs on the gimbal Pi as three ROS2 nodes, each in its own process:

```
keyboard / excitation ──/cmd_vel──► base_command (robot Pi) ──► kobuki_node
                                         │
                                         └─/base_command─►  predictor  ──M candidate trajectories──►  gimbal_control ──► motor driver
                                                           (slow clock)                              (fast, fixed rate)
                                                                                                       ▲   ▲
                                                                                 IMU A, IMU B (I2C/SPI)┘   │
                                                                       camera ──► vision ──orientation error┘
```

| Node | Package | Where | What it does |
|---|---|---|---|
| `base_command` | `base_command` | robot Pi | Relay: `/cmd_vel` → `/commands/velocity` at 50 Hz, with a 0.5 s timeout to zero and wheel-speed saturation; echoes the executed command on `/base_command`. See [src/base_command/README.md](src/base_command/README.md) for deployment and the hardware checklist. |
| `keyboard` | `base_teleop` | gimbal Pi | Hold-to-drive keyboard teleop: `ros2 run base_teleop keyboard`. |
| `excitation` | `base_teleop` | gimbal Pi | Plays a command sequence for data collection: `ros2 launch base_teleop data_collection.launch.py sequence_file:=...` (records an MCAP bag). |

| Node | Package | What it does | Timing |
|---|---|---|---|
| `predictor` | `gimbal_predictor` | Takes the latest command and recent IMU A history, rolls out the physical model of the robot (paper eqs. 4–6) and adds learned residuals from the flow-matching network (eq. 7). Publishes M candidate future trajectories of the base angular velocity. | Slower than the control loop. Separate process so inference never blocks control. |
| `gimbal_control` | `gimbal_control` | Weights the candidates by how well they match IMU A (eq. 8), takes the weighted prediction one actuation delay ahead (eq. 9), turns it into feedforward joint rates, adds feedback from vision and IMU B (eq. 10), limits the command and sends it to the motors. Falls back to feedback-only if no prediction is available. Stops the motors safely on any fault. | Fixed rate with period Δt. |
| `vision` | `gimbal_vision` | Computes the camera orientation error from images and publishes it with the image timestamp. | Once per camera frame. |

The `mode` parameter of `gimbal_control` switches between `feedback_only` and `prediction_assisted` at runtime, so we can compare the two controllers on the same run:

```bash
ros2 param set /gimbal_control mode prediction_assisted
```

Training the network, system identification and evaluation run **offline on a laptop**, not on the Pi (ARCHITECTURE.md 4.5).

## Repository layout

The repo root is a colcon workspace.

```
src/
  gimbal_predictor/   predictor node
  gimbal_control/     control node
  gimbal_vision/      vision node
  gimbal_bringup/     launch file and parameters (config/gimbal.yaml)
  base_teleop/        command sources for the robot (keyboard, excitation), data-collection launch
  base_command/       relay node for the robot Pi (deployed there, not run on the gimbal Pi)
ARCHITECTURE.md       engineering spec
Bravo_Team.md         method paper
Paper_changes.md      where the implementation deviates from the paper
paper_figures/        figures for the paper
```

## Build and run

Requires ROS2 (Humble or Jazzy; must match the TurtleBot). Write Python that runs on 3.10, so it works on both.

```bash
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
ros2 launch gimbal_bringup gimbal.launch.py
```

`predictor_period` and `control_period` in [src/gimbal_bringup/config/gimbal.yaml](src/gimbal_bringup/config/gimbal.yaml) are still TBD. The nodes refuse to start until they are set. For a local test, put in any value (for example 0.05 and 0.005).

## Current status

- Node framework is in place: the three gimbal nodes start, run their timers and shut down cleanly. The algorithms are `TODO`s marked with the paper's equation numbers.
- Robot command path is implemented (`base_command`, `base_teleop`) and tested locally; the hardware checklist in `src/base_command/README.md` is still open. The predictor subscribes to `/base_command`.
- No custom messages are defined yet.
- Open hardware decisions (motors and driver, IMUs, camera) are listed in ARCHITECTURE.md §7.

Run the unit tests with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test` if a user-installed pytest plugin (e.g. `typeguard`) breaks collection.
