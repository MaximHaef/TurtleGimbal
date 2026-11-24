# gimbal_control

## IMU A static calibration

`imu_calibrate` fits, in the sensor frame:

```
accel = K (accel_raw - b_a)     K upper triangular: 3 scales + 3 axis non-orthogonalities
gyro  = gyro_raw - b_g
```

K and b_a are fitted so that the calibrated acceleration has length g in every still pose (Levenberg–Marquardt, 9 parameters, at least 9 poses). b_g is the mean gyro reading over all still poses. A full 3×3 K can't be found this way, because a rotation of K leaves ‖K d‖ unchanged. The mounting rotation from sensor to base frame B is a separate step. The gyro scale can't be found at rest.

The fit is applied on top of the BNO08x reports that `open_bno08x` enables (the sensor's calibrated accelerometer and gyroscope, with factory scale). The BNO08x's own dynamic calibration would keep changing its internal bias correction while it runs, so `open_bno08x` turns it off by default (`imu_calibrate` and `imu_node` parameter `dynamic_calibration: false`). Calibration and operation must use the same setting.

### Checklist (gimbal Pi, done by you)

1. Stop everything else that reads IMU A (`imu_node`, `gimbal_control`). Only one process may talk to the sensor.
2. Let the IMU warm up for about 5 min after power-on. Gyro bias depends on temperature.
3. `source install/setup.bash`, then `ros2 run gimbal_control imu_calibrate`. It must not print `sensor did not acknowledge turning off dynamic calibration`; if it does, stop and report it. Options: `--bus 3 --address 0x4B --report-interval 0.01` (the defaults match `gimbal.yaml`), `--gravity <local g>`, `--duration 3`, `--out DIR`.
4. For each pose: put the IMU (or the whole gimbal base) down still, in a new orientation, and press Enter. The script waits 1 s, records 3 s, and rejects the pose if it moved or is within 15° of an earlier pose.
   - First the 6 poses with each axis up and down, then 8–12 tilted ones (cube edges and corners).
   - Resting it on a surface or against a block is better than holding it by hand.
   - Watch `coverage`. It should end well above 0.1; 1/3 is a perfectly even spread.
5. Type `f` to fit (`u` undoes the last pose, `q` quits). Check the report:
   - `| |a| - g | rms` should drop clearly, e.g. from ~0.1 to ~0.01 m/s². A large calibrated rms means a pose wasn't truly still, or the sensor isn't linear.
   - The diagonal of K should be close to 1, and off-diagonals ≪ 0.1.
   - Gyro `spread over poses` shows how stable b_g is. It should be small compared with b_g.
6. Output: `DIR/raw.npz` (all windows) and `DIR/imu_a_calibration.yaml`. To fit again later without the sensor (e.g. with another g): `ros2 run gimbal_control imu_calibrate --refit DIR/raw.npz`.
7. To use it, copy the YAML to `src/gimbal_bringup/config/imu_a_calibration.yaml` and set `calibration_file` of `imu_a` in `gimbal.yaml` to its absolute path. `imu_node` then logs `calibration <path>` at startup.

Gyro bias changes with every power-up and with temperature, so the stored b_g is only a starting value. Re-estimating it while the robot stands still at startup is the usual next step.
