# Base system identification

Finds the time constants of the Kobuki base for the physical model (paper eq. 6) and the dead time between a command and the base reacting. For each channel it fits

```
tau * dy/dt = K * u(t - T_d) - y
```

| Channel | Command u | Measured y | Gives |
|---|---|---|---|
| Yaw rate | `/base_command` `twist.angular.z` | Kobuki gyro, `/sensors/imu_data` `angular_velocity.z` | **τ_ω**, T_d,ω |
| Forward speed | `/base_command` `twist.linear.x` | Wheel odometry, `/odom` `twist.twist.linear.x` | **τ_v**, T_d,v |

The gyro only measures rotation, so τ_v comes from the wheel encoders.

- The gain K should be close to 1. If it isn't, the base doesn't reach the commanded speed (gyro scale, wheel slip, saturation).
- T_d is not in eq. (6). If it turns out clearly above zero, it has to be added to the model and to the horizon requirement T4 (ARCHITECTURE.md).

All timestamps come from the robot Pi (`/base_command` is stamped by `base_command`, the gyro and odometry by `kobuki_node`). The clock offset between the two Pis therefore plays no part.

## 1. Record (robot Pi, done by you)

The robot moves during this run, so the **wheels must be on the floor**: the gyro only sees real rotation. You need a free area of about 1 m around the robot. Mount the gimbal (or an equal mass) the way it will be used, because the load changes the dynamics.

Prerequisites on the robot Pi: `base_command` is deployed (see `src/base_command/README.md`), and `base_teleop` is deployed the same way so the excitation can run there until the gimbal Pi has ROS2:

```bash
rsync -av src/base_teleop hungchun@192.168.50.175:~/kobuki_ws/src/      # from the repo root on the laptop
# on the robot Pi:
cd ~/kobuki_ws && colcon build --packages-select base_command base_teleop
```

Three terminals on the robot Pi, each with `source ~/kobuki_ws/install/setup.bash`:

```bash
# 1: driver + relay
ros2 launch base_command base_command.launch.py

# 2: recording (default sqlite3 storage; no extra plugin needed)
ros2 bag record -o $HOME/sysid_$(date +%Y%m%d_%H%M) \
  /base_command /cmd_vel /sensors/imu_data /sensors/imu_data_raw /odom /joint_states \
  /debug/raw_control_command /excitation/segment

# 3: the step sequence (about 70 s); starts 2 s after it finds the relay
ros2 run base_teleop excitation --ros-args \
  -p sequence_file:=$HOME/kobuki_ws/install/base_teleop/share/base_teleop/config/sysid_steps.yaml
```

Stop the recording with Ctrl-C after the excitation prints `sequence finished`. Keep your hand near the robot's power button or Ctrl-C in terminal 3: the relay stops the robot 0.5 s after the excitation stops publishing. Repeat the run 2–3 times to see how repeatable the numbers are.

Copy the bag to the laptop:

```bash
scp -r hungchun@192.168.50.175:~/sysid_YYYYMMDD_HHMM .
```

## 2. Analyse (laptop)

```bash
source /opt/ros/humble/setup.bash
python3 tools/sysid/fit_base_dynamics.py sysid_YYYYMMDD_HHMM --plot sysid.png --json sysid.json
```

It prints the fit over the whole run and one fit per step, with median and spread. The plot shows:
- **Left:** command, measurement and fitted model over time.
- **Right:** every step response normalized and laid on top of each other, with the model.

How to read the result:
- **Per-step values agree, R² close to 1, and the overlaid steps follow the black curve:** the first-order model is good. Enter τ_ω and τ_v (and T_d) in ARCHITECTURE.md §5.3.
- **τ grows with the step size, and the rise looks straight rather than curved:** the Kobuki limits acceleration. A first-order model is then only an approximation, and a rate limit should be added to the model.
- **Step up and step down differ, or + and − directions differ:** asymmetric dynamics. Note it, and use the median.
- **K clearly off 1 for yaw:** check the gyro scale against the odometry yaw rate before trusting τ.

Bags recorded as MCAP can be read too if `ros-humble-rosbag2-storage-mcap` is installed on the laptop.

## Tests

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider tools/sysid
```

`test_fopdt.py` checks that the fit recovers known parameters from noisy 50 Hz data. `test_end_to_end.py` writes a synthetic bag with the real message types and runs the script on it.
