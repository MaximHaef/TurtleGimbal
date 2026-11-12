# base_command: command relay on the robot Pi

The only project node that runs on the robot (Kobuki) Pi. It lets any machine on the network drive the base, and tells the gimbal Pi exactly which command the base is executing.

```
gimbal Pi                                   robot Pi
base_teleop keyboard / excitation ─/cmd_vel─► base_command ─/commands/velocity─► kobuki_node
gimbal_predictor ◄─────────────────/base_command (TwistStamped)──┘
```

Every `1/publish_rate` s the node:
1. takes the latest `/cmd_vel` request, or zero if none arrived within `request_timeout` (so the robot stops if the command source or the network goes away),
2. scales it down if a wheel would exceed `wheel_speed_limit` (same radius, like the Kobuki firmware),
3. publishes it to `/commands/velocity` (`kobuki_node`) and, with a timestamp, to `/base_command` (u_k for the predictor).

On shutdown it sends one zero command.

| Parameter | Default | Meaning |
|---|---|---|
| `publish_rate` | 50 Hz | Output rate |
| `request_timeout` | 0.5 s | Zero output after this long without a request. Must stay below `kobuki_node`'s `cmd_vel_timeout_sec` (0.6 s) |
| `track_width` | 0.230 m | b, Kobuki spec, to verify |
| `wheel_speed_limit` | 0.7 m/s | v_max, Kobuki spec, to verify |

## Deployment (robot Pi)

1. Copy the package (from the repo root on your laptop):
   ```bash
   rsync -av src/base_command hungchun@192.168.50.175:~/kobuki_ws/src/
   ```
2. Build on the robot Pi:
   ```bash
   cd ~/kobuki_ws && source /opt/ros/humble/setup.bash
   colcon build --packages-select base_command
   ```
3. Change the teleop in `~/kobuki_ws/src/cpp_pkg/src/helloworld.cpp` to publish to `/cmd_vel` instead of `/commands/velocity`, so the relay is the only publisher to the driver. Then `colcon build --packages-select cpp_pkg`.
4. Start the driver and the relay:
   ```bash
   source ~/kobuki_ws/install/setup.bash
   ros2 launch base_command base_command.launch.py            # starts kobuki_node too
   ros2 launch base_command base_command.launch.py start_kobuki:=false   # driver already running
   ```
5. Optional, start at boot so no SSH is needed for runs:
   ```bash
   sudo cp ~/kobuki_ws/src/base_command/systemd/base_command.service /etc/systemd/system/
   sudo systemctl daemon-reload && sudo systemctl enable --now base_command
   journalctl -u base_command -f      # logs
   ```

Both Pis must use the same `ROS_DOMAIN_ID` (the robot Pi uses the default, 0) and the same RMW. Check from the gimbal Pi with `ros2 topic list`: `/base_command` and `/commands/velocity` should appear.

## Hardware checklist

Robot on blocks, **wheels lifted**, until steps 1–3 pass.

1. Gimbal Pi: `ros2 run base_teleop keyboard`. While a key is held, `ros2 topic hz /commands/velocity` shows about 50 Hz and the wheels turn.
2. Release the key, or kill the keyboard node: the wheels stop within about 0.6 s (`key_hold`), or 0.5 s after the node dies (`request_timeout`). Unplug the Ethernet cable while driving: the wheels stop.
3. `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.5}}"`: `/base_command` shows the scaled command (v ≈ 0.6, w ≈ 0.9), and `/debug/raw_control_command` matches it. Stop with Ctrl-C.
4. Gimbal Pi: `ros2 topic echo /base_command --field header.stamp` is close to the local clock (chrony), and the predictor no longer warns about a missing `/base_command`.
5. Excitation run on the floor with the example sequence, recorded to MCAP:
   `ros2 launch base_teleop data_collection.launch.py`. Plot `/base_command` against `/joint_states` and IMU A.
