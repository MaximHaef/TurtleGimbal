# Excitation sequences for training-data collection

Training data for the flow-matching residual (F9, ARCHITECTURE.md §4.2, §4.5). The robot drives a generated command sequence while the gimbal Pi records everything. The sequence has to make the residual learn what the physical model (eq. 6) misses: the speed shortfall (sysid K = 0.66–0.74), start/stop overshoot, dead time, and the floor vibration seen by IMU A.

| File | Job |
|---|---|
| `make_sequence.py` | Writes a sequence YAML for the `excitation` node (format of `base_teleop/sequence.py`) |
| `check_bag.py` | Post-run checks of a recorded bag, ends with PASS or FAIL (needs ROS2 and IMU A in the bag) |
| `export_bag.py` | Exports a bag to one CSV per topic plus `overview.png` and `path.png`. Needs no ROS (`pip install rosbags matplotlib`) |
| `sequences/` | Sequence files that were played on the robot (`excitation_box07_s1.yaml`: `--box 0.7 --seed 1`, 7.3 min) |

## Quick start: one data-collection run

What we run today: robot Pi only, no IMU A (details in §2.0). The robot Pi runs ROS2 Humble; the laptop needs no ROS. Run the commands from the repo root in a laptop PowerShell terminal.

**0. Once (and after changing `base_teleop` or the sequence): copy to the robot Pi and build there.**

```powershell
scp -r src/base_teleop hungchun@192.168.50.175:~/kobuki_ws/src/
scp tools/excitation/sequences/excitation_box07_s1.yaml hungchun@192.168.50.175:~/
ssh hungchun@192.168.50.175 "source /opt/ros/humble/setup.bash; cd ~/kobuki_ws; rm -rf build/base_teleop install/base_teleop; colcon build --packages-select base_teleop"
```

**1. Before each run:** robot switched on, laptop on the robot's network (192.168.50.x), robot in the middle of the free area (1 m on every side) facing straight ahead, battery voltage noted, hand near the power button.

**2. Run (about 7.5 min).** Starts the driver and relay, records the bag, plays the sequence, and cleans up afterwards. **Press space (or Ctrl-C) to stop the robot.**

```powershell
ssh -t hungchun@192.168.50.175 "trap true INT; source ~/kobuki_ws/install/setup.bash; mkdir -p ~/bags; B=~/bags/floor_`$(date +%Y%m%d_%H%M); ros2 launch base_command base_command.launch.py > /tmp/base_command.log 2>&1 & sleep 5; ros2 bag record -o `$B /base_command /cmd_vel /sensors/imu_data /sensors/imu_data_raw /odom /joint_states /debug/raw_control_command /excitation/segment /events/bumper /events/button > /tmp/bag.log 2>&1 & sleep 2; ros2 run base_teleop excitation --ros-args -p sequence_file:=`$HOME/excitation_box07_s1.yaml -p stop_on_kobuki_events:=false; sleep 2; kill -INT %2; sleep 2; kill %1; echo bag: `$B"
```

It ends with `bag: /home/hungchun/bags/floor_<date>_<time>`.

**3. Copy the bag to the laptop and plot it** (replace `<bag>` with the name printed in step 2):

```powershell
scp -r hungchun@192.168.50.175:~/bags/<bag> bags/
python tools/excitation/export_bag.py bags/<bag> --sequence tools/excitation/sequences/excitation_box07_s1.yaml
```

CSV files and `overview.png` / `path.png` are written to `bags/<bag>/export/`.

**4. Put the robot back in the middle** before the next run; it ends up to about 0.6 m away from its start.

## 1. Generate a sequence (laptop)

Measure the free area around the start pose first. `--box` is the half-width of the free square around the robot's start position (x forward, y left), **minus a safety margin of at least 0.3 m**. The generator checks the commanded path against it. The real robot drifts off the commanded path (it reaches only part of the commanded speed and slips), so the margin is needed.

```bash
python3 tools/excitation/make_sequence.py --box 1.2 --seed 1 --out excitation_s1.yaml
```

It prints the segment count, the total duration and how far the commanded path goes from the start. With the defaults, one round takes about 7.5 min. The same arguments always give the same file, and the full command is written into the file header.

| Argument | Default | |
|---|---|---|
| `--box` | required | m, see above |
| `--seed` | 0 | random block; use a new seed for each run |
| `--rounds` | 1 | each round adds about 7.5 min |
| `--vmax`, `--wmax` | 0.2 m/s, 1.0 rad/s | the largest amplitudes of `sysid_steps.yaml`. **Raise them in steps only after the Kobuki wheel-speed limit has been measured** (ARCHITECTURE.md §4.5) |
| `--track-width`, `--wheel-speed-limit` | 0.23 m, 0.7 m/s | Kobuki spec, unverified, same as `base_command.yaml`. The outer wheel speed stays below 0.9 × limit |

One round contains:

| Block | Content | Why |
|---|---|---|
| Rest | 5 s at (0, 0) | gyro bias, floor-vibration baseline |
| Straight | v = ±0.3, ±0.6, ±1.0 vmax, up to 10 s each, back and forth | vibration vs. speed |
| Turn in place | w = ±0.3, ±0.6, ±1.0 wmax, 4 s each | clean yaw response |
| Turn while driving | v = 0.5 vmax with w = ±0.3, ±0.6 wmax, 8 s, each arc driven back along itself | the real operating case |
| Sysid steps | the steps of `sysid_steps.yaml` | τ_v, τ_ω stay identifiable from the same bag |
| Ramps | staircases of 0.25 s steps up to vmax / wmax and back down, 10 s each | acceleration limits |
| Random | 120 s of 0.5–2.0 s segments, (v, w) uniform within the limits, a 0.5–1.0 s rest after a segment with probability 0.3 | combinations the designed blocks miss; many starts and stops |
| Return | turn towards the start, drive to it, turn back to the start heading | each round starts where the first did |

Every segment is a multiple of 0.05 s (the excitation node runs at 20 Hz) and at least 0.25 s long. The first and last segment are rests (the sequence starts and ends with 5 s at rest). If the box is too small, straight segments and the speed ramp are shortened and a note is printed; if even that does not fit, the generator stops with an error.

### Following a run

While it runs, the excitation node logs every new segment as `segment N: v=... m/s, w=... rad/s`:

- `N`: segment number, from 0.
- `v`: forward speed in m/s; positive = forward, negative = backward.
- `w`: yaw rate in rad/s; positive = turn left (counter-clockwise seen from above), negative = turn right.

Which segment numbers belong to which block depends on `--box` and `--seed`. `make_sequence.py` prints the table and writes it into the header of the YAML file. For `excitation_box07_s1.yaml` (`--box 0.7 --seed 1`, 332 segments, 438.4 s):

| Segments | Time [s] | Block | What the robot does |
|---|---|---|---|
| 0 | 0–5 | rest | stands still |
| 1–12 | 5–58 | straight | slow, medium, fast; forward then back to the start, 2.5 s rest in between |
| 13–24 | 58–97 | turn in place | three yaw rates, left then right |
| 25–40 | 97–181 | turn while driving | drives an arc, then back along the same arc; left and right |
| 41–64 | 181–245 | sysid steps | the steps of `sysid_steps.yaml`: sudden turns, drives and stops, 3 s each |
| 65–224 | 245–294 | ramps | speed up in 0.25 s steps and back down, straight then turning; segment numbers run fast here |
| 225–324 | 294–414 | random | a new random (v, w) every 0.5–2 s, many stops and starts |
| 325–330 | 414–433 | return to start | turns towards the start, drives there, turns back to the start heading |
| 331 | 433–438 | rest | stands still, end |

The robot does not end exactly at its start: it reaches only part of the commanded speed, so the commanded return misses (about 0.6 m and 100° in the first run).

## 2. Record (done by you, at the robot)

### Stopping the robot

The `excitation` node stops at once (sends zero, exits, never resumes) on:

- **any key** in its terminal (only with `ros2 run` in an interactive terminal, e.g. `ssh -t`; not with `ros2 launch`),
- **Ctrl-C**,
- Kobuki **bumper, cliff, wheel drop, or button B0–B2** events, if `stop_on_kobuki_events` is true (default). On our robot the right bumper reports presses while the robot stands still or when a cable touches it, so the runs below set it to **false**.

If the node dies or the link drops, the relay stops the robot after `request_timeout` (0.5 s). Keep a hand near the power button anyway.

### 2.0 Current setup: robot Pi only, without IMU A

No IMU A and no gimbal Pi yet, so the run uses the packages already deployed on the robot Pi (`base_command`, `base_teleop`) and records the Kobuki topics only. `data_collection.launch.py` does not work there (it needs `gimbal_bringup`). The bag still contains the commands, wheel odometry and the Kobuki gyro (speed shortfall, dead time, yaw response), but no IMU A data for training.

Deploy `base_teleop` and the sequence (laptop, repo root), then build on the robot Pi:

```bash
scp -r src/base_teleop hungchun@192.168.50.175:~/kobuki_ws/src/
scp tools/excitation/sequences/excitation_box07_s1.yaml hungchun@192.168.50.175:~/
# robot Pi:
cd ~/kobuki_ws && rm -rf build/base_teleop install/base_teleop && colcon build --packages-select base_teleop
```

**Remove `build/` and `install/` of the package before building** as long as the robot Pi's clock is wrong (it was 8 months behind): files copied from the laptop then look older than the previous build, and colcon silently keeps the old code. Check that the node logs `press ANY KEY` at startup.

Run from a laptop terminal (PowerShell; one command, one terminal). It starts the driver and relay, the recording and the excitation, and cleans up afterwards, also after Ctrl-C:

```powershell
ssh -t hungchun@192.168.50.175 "trap true INT; source ~/kobuki_ws/install/setup.bash; mkdir -p ~/bags; B=~/bags/floor_`$(date +%Y%m%d_%H%M); ros2 launch base_command base_command.launch.py > /tmp/base_command.log 2>&1 & sleep 5; ros2 bag record -o `$B /base_command /cmd_vel /sensors/imu_data /sensors/imu_data_raw /odom /joint_states /debug/raw_control_command /excitation/segment /events/bumper /events/button > /tmp/bag.log 2>&1 & sleep 2; ros2 run base_teleop excitation --ros-args -p sequence_file:=`$HOME/excitation_box07_s1.yaml -p stop_on_kobuki_events:=false; sleep 2; kill -INT %2; sleep 2; kill %1; echo bag: `$B"
```

Put the robot back at the centre and straighten it before every run: the real robot ends up to about 0.6 m and 100° away from its start (it reaches only part of the commanded speed), so the next run would otherwise start off-centre.

Look at the result on the laptop:

```bash
scp -r hungchun@192.168.50.175:~/bags/<bag> bags/
python tools/excitation/export_bag.py bags/<bag> --sequence tools/excitation/sequences/excitation_box07_s1.yaml
```

`bags/` is in `.gitignore`. The bag names carry the robot Pi's date, which may be wrong; write the real date into meta.yaml.

### 2.1 With IMU A, from the gimbal Pi: set up (once, and after every code change)

IMU A is wired to the gimbal Pi (`/dev/i2c-3`). The gimbal Pi runs the IMU A publisher, the excitation and the recording with `data_collection.launch.py`. The robot Pi runs only the driver and the `base_command` relay, as in the project rule (`base_command` is the only project node on the robot Pi). `base_teleop` on the robot Pi is only needed for the setup of §2.0.

Copy the code to the gimbal Pi (from the repo root on the laptop; `<gimbal-pi>` is its user@address):

```bash
scp -r src/base_teleop src/gimbal_interfaces src/gimbal_control src/gimbal_predictor src/gimbal_vision src/gimbal_bringup <gimbal-pi>:~/gimbal_ws/src/
scp -r tools <gimbal-pi>:~/
scp excitation_s1.yaml <gimbal-pi>:~/
```

On the gimbal Pi:

```bash
cd ~/gimbal_ws && colcon build --packages-up-to base_teleop gimbal_bringup
source ~/gimbal_ws/install/setup.bash
mkdir -p ~/bags
```

One-time checks (only the first time):

- [ ] **Same ROS network:** both Pis use the same `ROS_DOMAIN_ID` (the robot Pi uses the default, 0), and neither sets `ROS_LOCALHOST_ONLY`. With the relay running on the robot Pi, `ros2 topic hz /base_command` **on the gimbal Pi** shows about 50 Hz.
- [ ] **Clocks synced (chrony):** `/base_command`, `/odom` and the Kobuki gyro are stamped by the robot Pi, IMU A by the gimbal Pi. Training needs them on one time base (ARCHITECTURE.md T5, below 1 ms). Set up as in [NETWORK.md](../../NETWORK.md) §2: the gimbal Pi serves time to the robot Pi over the cable. ROS must also run only over the cable (§3). On both Pis, `chronyc tracking` must show `Leap status: Normal` and a `System time` offset below 1 ms. Check it again before each recording day; `check_bag.py` also fails a bag with clocks more than 20 ms apart.
- [ ] **Kobuki messages:** the excitation node needs `kobuki_ros_interfaces` to stop on bumper/cliff/wheel-drop/button events: `sudo apt install ros-humble-kobuki-ros-interfaces`. Without it, run with `stop_on_kobuki_events:=false`, or the node exits at start.
- [ ] **MCAP storage plugin** (`data_collection.launch.py` records with `-s mcap`): `dpkg -l ros-humble-rosbag2-storage-mcap`. If missing: `sudo apt install ros-humble-rosbag2-storage-mcap`.
- [ ] **IMU A** on `/dev/i2c-3` (i2c-gpio overlay, GPIO23/24): `ls /dev/i2c-3` exists and `i2cdetect -y 3` shows `4b`.
- [ ] **IMU driver libraries:** `python3 -c "import adafruit_bno08x, adafruit_extended_bus"` prints nothing.
- [ ] **IMU A calibrated** on the gimbal Pi: `ros2 run gimbal_control imu_calibrate`, procedure in `src/gimbal_control/README.md`. Set `calibration_file` of `imu_a` in `src/gimbal_bringup/config/gimbal.yaml` to the YAML's **absolute path on the gimbal Pi**, keep `accel: true` and `dynamic_calibration: false`, then rebuild `gimbal_bringup`. Repeat the calibration if IMU A is remounted or replaced.
- [ ] **Standalone test** without motion: `ros2 run gimbal_control imu_node --ros-args --params-file ~/gimbal_ws/install/gimbal_bringup/share/gimbal_bringup/config/gimbal.yaml` logs `IMU A: 100.0 Hz, 0 read errors in 5 s`, `calibration <path>` and `BNO dynamic calibration off`. It publishes `/gimbal/imu_a` (calibrated) and `/gimbal/imu_a/raw` (sensor reports). Stop it with Ctrl-C before the run (only one process may read the sensor).

### 2.2 With IMU A, from the gimbal Pi: run

The robot moves. **Keep a hand near the robot's power button.** Ctrl-C in the gimbal Pi terminal stops the excitation (under `ros2 launch` the key stop is not available).

Before each run:

- [ ] Gimbal mounted as it will be used (the load changes the dynamics).
- [ ] IMU warmed up for about 5 min after power-on (gyro bias depends on temperature).
- [ ] `chronyc tracking` on both Pis: offset below 1 ms.
- [ ] Battery voltage noted.
- [ ] Robot at the start pose, facing the x direction you used for `--box`.

```bash
# robot Pi (ssh), terminal 1: driver + relay
source ~/kobuki_ws/install/setup.bash
ros2 launch base_command base_command.launch.py

# gimbal Pi (ssh), terminal 2: first check the relay over the network: about 50 Hz, then Ctrl-C
source ~/gimbal_ws/install/setup.bash
ros2 topic hz /base_command
# gimbal Pi, terminal 2: then IMU A + recording + excitation
ros2 launch base_teleop data_collection.launch.py \
  sequence_file:=$HOME/excitation_s1.yaml bag:=$HOME/bags/<terrain>_<YYYYMMDD>_<n> \
  stop_on_kobuki_events:=false
```

`stop_on_kobuki_events:=false` because of the faulty right bumper (see "Stopping the robot"); drop it once the bumper is fixed.

**Always start terminal 1 first and check `/base_command`.** `ros2 bag record -a` itself subscribes to `/cmd_vel`, so the excitation node starts even without the relay, and a relay started late makes the robot begin in the middle of the sequence.

The excitation starts 2 s after it finds a `/cmd_vel` subscriber. While it runs, `imu_a` logs its rate every 5 s; it should stay at 100 Hz with 0 read errors. When the excitation prints `sequence finished`, wait 2 s and stop terminal 2 with Ctrl-C (this also stops the recording).

Right after the run, copy the IMU A calibration used into the bag and write `<bag>/meta.yaml` next to `metadata.yaml`:

```bash
cp <calibration_file from gimbal.yaml> ~/bags/<bag>/imu_a_calibration.yaml
```

```yaml
terrain: smooth_floor        # smooth_floor | tile_joints | carpet | threshold | outdoor_<surface>
date: 2026-11-25
battery_start: 16.4          # V
battery_end: 16.1            # V
gimbal_mounted: true
seed: 1
sequence_file: excitation_s1.yaml
git_commit: 6c5dadc          # git rev-parse --short HEAD of the laptop repo the code was copied from
imu_rate: 100                # Hz, 1 / report_interval in gimbal.yaml
imu_calibration: imu_a_calibration.yaml   # copy in the bag directory; 'none' if calibration_file was empty
dynamic_calibration: false   # gimbal.yaml imu_a dynamic_calibration
operator: Tanya
notes: ''                    # anything unusual: bumps, slip, interruptions
```

The bag has IMU A twice: `/gimbal/imu_a` with the calibration applied, and `/gimbal/imu_a/raw` with the sensor reports. With the raw topic and the copied YAML, a bag can be calibrated again later, e.g. with a better calibration.

## 3. Check (gimbal Pi, right after the run)

```bash
source ~/gimbal_ws/install/setup.bash
python3 ~/tools/excitation/check_bag.py ~/bags/<bag> --sequence ~/excitation_s1.yaml --imu-rate 100
```

Reading MCAP bags needs `ros-humble-rosbag2-storage-mcap`. The check only reads the bag, so it can also run later on any machine with ROS2 Humble. The script prints one line per check and ends with `PASS` or `FAIL: <checks>`. **Record again if it fails** (except for `meta`: then fix meta.yaml).

| Check | Fails when |
|---|---|
| `topics` | one of `/base_command`, `/cmd_vel`, `/gimbal/imu_a`, `/gimbal/imu_a/raw`, `/excitation/segment`, `/odom`, `/sensors/imu_data`, `/debug/raw_control_command` is missing |
| `segments` | a segment was not played, played out of order, or played more than 0.15 s longer or shorter than in the YAML (interrupted run, stalled excitation) |
| per topic | `/base_command` not 50 Hz ±5 %, `/cmd_vel` not 20 Hz, `/gimbal/imu_a` or `/gimbal/imu_a/raw` not `--imu-rate`, a gap longer than 3 periods (0.2 s for the other topics), stamps going backwards, or the topic does not cover the whole run |
| `IMU A accel` | IMU A published the gyro only |
| `clock sync` | the robot Pi's stamps (`/base_command`) and IMU A's stamps differ by more than `--max-clock-offset` (20 ms, includes the network latency) relative to the recording time: chrony not synced. Catches gross errors only; the 1 ms requirement is checked with `chronyc tracking` |
| `relay saturation` | `base_command` scaled the requests down (its wheel-speed limit is lower than the generator's) |
| `firmware speed` | `/debug/raw_control_command` above 0.9 × wheel-speed limit |
| `first/last rest IMU A gyro` | gyro mean at rest above 0.02 rad/s on an axis (calibration missing or wrong). Also prints the noise; compare first and last rest for drift |
| `meta` | meta.yaml missing or incomplete, or the `imu_calibration` file is not in the bag directory |

The Kobuki gyro bias at rest is printed for information only.

Copy the bags to the laptop: `scp -r <gimbal-pi>:~/bags/<bag> .`

## 4. What to record

- **Start with one pilot run** (1 round, about 7.5 min) on a smooth floor, check it, and look at the data before recording more.
- **Repeat** each sequence 3–5 times per condition, with a new `--seed` each time.
- **Battery:** full and about half.
- **Terrain:** smooth floor (baseline), tile joints, carpet, a threshold or ramp, outdoor if safe. The terrain set is not specified in the repo yet; agree on it with Maxim before relying on it for the paper.
- **How much data:** not specified anywhere. Decide later with a learning curve (train on 25 %, 50 %, 100 % of the bags and see whether the error still drops).

## Tests

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider tools/excitation
```

`test_make_sequence.py` checks every constraint above on generated files (including an independent integration of the path), that the files load with `load_sequence`/`command_at`, determinism, and that the steps agree with `sysid_steps.yaml`. `test_check_bag.py` runs the checks on synthetic data (clean run passes; gaps, wrong rate, missing accelerometer, scaling, late relay, interruption, bias and missing meta fail). `test_script_on_synthetic_bag` writes a real bag and needs a sourced ROS2 environment; it is skipped otherwise.
