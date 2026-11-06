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

- **TurtleBot** (differential drive, ROS2): drives the robot and publishes its velocity commands (`/cmd_vel`), wheel speeds and odometry. It has **no IMU**, and we do not modify its software.
- **Gimbal Pi** (Raspberry Pi 4, Ubuntu + same ROS2 distro as the TurtleBot): runs all our code. Connected to the TurtleBot by Ethernet, clocks synced with chrony.
- **Gimbal:** yaw (outer) → pitch (middle) → roll (inner), driven by DC motors with encoders in velocity mode.
- **IMU A** on the gimbal base and **IMU B** in the camera frame. Both are read directly by the gimbal Pi over I2C/SPI, not through ROS.
- **Camera** for visual orientation feedback.

## Node structure

All gimbal software runs on the gimbal Pi as three ROS2 nodes, each in its own process:

```
TurtleBot ──/cmd_vel──►  predictor  ──M candidate trajectories──►  gimbal_control ──► motor driver
                        (slow clock)                              (fast, fixed rate)
                                                                    ▲   ▲
                                              IMU A, IMU B (I2C/SPI)┘   │
                                    camera ──► vision ──orientation error┘
```

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
ARCHITECTURE.md       engineering spec
Bravo_Team.md         method paper
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

- Node framework is in place: the three nodes start, run their timers and shut down cleanly. The algorithms are `TODO`s marked with the paper's equation numbers.
- No topics or custom messages are defined yet.
- Open hardware decisions (TurtleBot model, ROS2 distro, motors and driver, IMUs, camera) are listed in ARCHITECTURE.md §7.
