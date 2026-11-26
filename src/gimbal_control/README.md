# gimbal_control

## IMU A wiring and Pi setup

IMU A is a board labelled **HW-921 / GY-BNO055**. The chip on it is a **BNO08x**, not a BNO055: it answers at **0x4B** and speaks the BNO08x packet protocol (SHTP). PS0/PS1 are not broken out, so the board only works over I2C.

It runs on a **software I2C bus** (`/dev/i2c-3`), not the Pi's hardware I2C. The BNO08x holds the clock line low while it is busy (clock stretching), and the Pi's hardware I2C handles that badly: random read errors and corrupt values.

### Wiring (Pi powered off)

| IMU pin | Pi 4 header pin | |
|---|---|---|
| VIN | 1 | 3.3 V. Not 5 V: some boards pull SDA/SCL up to VIN |
| GND | 9 | Any GND pin works |
| SDA | 16 | GPIO23 |
| SCL | 18 | GPIO24 |
| ADR / ADD | open | The board pulls it high → 0x4B. Low → 0x4A |
| INT | open | Data-ready output. Optional, for reading on data-ready instead of polling |
| BOOT | open | **Never to GND**: low at reset starts the firmware-update mode |
| RST | open | Touching it (or BOOT) resets the sensor. Insulate the open pins, or better tie RST to 3.3 V via ~10 kΩ |

A second BNO08x (IMU B) can't share the bus at the same address. Either pull its ADR low (0x4A), or better, give it its own software bus (e.g. `bus=4` on GPIO5/6, pins 29/31), so one stalled sensor can't block the other.

### Pi setup (once)

1. Add the software I2C bus to `/boot/firmware/config.txt` and reboot:
   ```
   dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2
   ```
2. Give the user access to I2C without sudo: `sudo usermod -aG dialout $USER`, then log in again.
3. Install the tools and Python libraries:
   ```bash
   sudo apt install -y i2c-tools python3-pip
   python3 -m pip install --user adafruit-circuitpython-bno08x adafruit-extended-bus
   ```
4. Check: `i2cdetect -y 3` shows `4b`.

### Troubleshooting

| `i2cdetect -y 3` shows | Likely cause |
|---|---|
| Nothing | No power (VIN/GND), SDA/SCL swapped, a bad solder joint, or the chip doesn't start (e.g. the 32.768 kHz crystal next to the chip is missing) |
| Every address 0x08–0x77 | SDA held low: an unpowered board connected to SDA, or SDA shorted to GND |
| `4b` | OK |

`imu_node` and `imu_calibrate` use bus 3 and 0x4B by default (`gimbal.yaml`). Tested: gyro reports at 100 and 200 Hz arrive at the requested rate; at 250 Hz and above the stream stalls without errors.

**Sensor resets.** The BNO08x resets itself when RST/BOOT is touched, a wire is loose, or the supply glitches. Seen on the bench (2026-11-26): a corrupt read, ~3 s without data, ~3 s of `[Errno 6] No such device or address`, then a "reset complete" packet. After a reset it has no reports enabled and its own dynamic calibration is on again. `open_bno08x` (`Bno08xPoller`) catches the reset-complete packet and enables the reports and the `dynamic_calibration` setting again, retrying while the sensor boots; it does the same after 1 s without a sample (a stall), and if that brings no sample within another second (the sensor hangs, e.g. reads only 0xFF) it sends a soft reset first, as at startup. All-0xFF reads are counted, not printed. `imu_node` logs a warning with the count in its 5 s report; `imu_calibrate` discards a pose recorded across a reset or stall. The data around a reset is missing (a few seconds), so a reset during a recording shows up as a gap in the bag.

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
4. For each pose: put the IMU (or the whole gimbal base) down still, in a new orientation, and press Enter. The script waits 1 s, records 3 s, and rejects the pose if it moved, is within 15° of an earlier pose, or the sensor reset meanwhile.
   - Hold it by the board edges or the mount, not by the pins: a finger on RST/BOOT resets the sensor.
   - First the 6 poses with each axis up and down, then 8–12 tilted ones (cube edges and corners).
   - Resting it on a surface or against a block is better than holding it by hand.
   - Watch `coverage`. It should end well above 0.1; 1/3 is a perfectly even spread.
5. Type `f` to fit (`u` undoes the last pose, `q` quits). Check the report:
   - `| |a| - g | rms` should drop clearly, e.g. from ~0.1 to ~0.01 m/s². A large calibrated rms means a pose wasn't truly still, or the sensor isn't linear.
   - The diagonal of K should be close to 1, and off-diagonals ≪ 0.1.
   - Gyro `spread over poses` shows how stable b_g is. It should be small compared with b_g.
6. Output: `DIR/raw.npz` (all windows) and `DIR/imu_a_calibration.yaml`. To fit again later without the sensor (e.g. with another g): `ros2 run gimbal_control imu_calibrate --refit DIR/raw.npz`.
7. To use it, copy the YAML to `src/gimbal_bringup/config/imu_a_calibration.yaml` and set `calibration_file` of `imu_a` in `gimbal.yaml` to its absolute path. `imu_node` then logs `calibration <path>` at startup and publishes the calibrated values on `/gimbal/imu_a`; `/gimbal/imu_a/raw` always carries the uncalibrated sensor reports. For data collection, also copy the YAML into each bag (`tools/excitation/README.md` §2.2).

Gyro bias changes with every power-up and with temperature, so the stored b_g is only a starting value. Re-estimating it while the robot stands still at startup is the usual next step.
