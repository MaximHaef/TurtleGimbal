"""imu_node: publishes IMU A (gimbal base gyro + accelerometer) on /gimbal/imu_a.

For data collection and testing while the control loop is not running. It uses
the same ImuReader as gimbal_control. Do not run it together with a
gimbal_control that reads IMU A itself: both would talk to the same sensor.

Publishes sensor_msgs/Imu with angular velocity and linear acceleration
(incl. gravity, REP-145), no orientation, in the sensor frame
(frame_id `imu_a`). The mounting rotation to the base frame B is not measured
yet. Stamped with the gimbal Pi's ROS clock when the sample is read.

If `calibration_file` is set (YAML from `imu_calibrate`), the accelerometer
matrix/bias and the gyro bias are applied before publishing; otherwise the
sensor reports are published unchanged.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from gimbal_control.imu import ImuReader, open_bno08x
from gimbal_control.imu_calibration import ImuCalibration


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_a')
        self.declare_parameter('bus', 3)                  # /dev/i2c-3, i2c-gpio overlay
        self.declare_parameter('address', 0x4B)
        self.declare_parameter('report_interval', 0.01)   # s, gyro and accel report period
        self.declare_parameter('accel', True)             # also read the accelerometer
        self.declare_parameter('timeout', 0.1)            # s without a sample -> error
        self.declare_parameter('frame_id', 'imu_a')
        self.declare_parameter('dynamic_calibration', False)  # BNO08x's own running bias estimation
        self.declare_parameter('calibration_file', '')     # imu_calibrate YAML; '' = uncalibrated
        g = lambda n: self.get_parameter(n).value

        bus, address, interval = g('bus'), g('address'), g('report_interval')
        self.timeout = g('timeout')
        self.frame_id = g('frame_id')
        cal_file = g('calibration_file')
        self.calibration = ImuCalibration.load(cal_file) if cal_file else None

        self.pub = self.create_publisher(Imu, '/gimbal/imu_a', 50)
        accel = g('accel')
        dyn_cal = g('dynamic_calibration')
        poll = open_bno08x(bus, address, interval, accel, dyn_cal)
        if poll.dynamic_calibration_ack is False:
            self.get_logger().warn('IMU A: sensor did not acknowledge turning off dynamic calibration')
        self.reader = ImuReader(poll, stamp=lambda: self.get_clock().now(),
                                on_sample=self.publish)
        self.reader.start()

        self._stale = False
        self._last_count = (0, 0)
        self.create_timer(self.timeout / 2, self.watchdog)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'IMU A: BNO08x on i2c-{bus} @0x{address:02X}, '
            f'{"gyro + accel" if accel else "gyro"} every {interval * 1e3:.1f} ms, '
            f'{"calibration " + cal_file if cal_file else "uncalibrated"}, '
            f'BNO dynamic calibration {"on" if dyn_cal else "off" if poll.dynamic_calibration_ack else "off (not acknowledged)"}')

    def publish(self, stamp, sample):
        # Called from the reader thread; rclpy publishers are thread-safe.
        if self.calibration is not None:
            sample = self.calibration.apply(sample)
        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.gyro
        msg.orientation_covariance[0] = -1.0          # not provided
        if sample.accel is None:
            msg.linear_acceleration_covariance[0] = -1.0  # not provided
        else:
            a = msg.linear_acceleration
            a.x, a.y, a.z = sample.accel
        self.pub.publish(msg)

    def watchdog(self):
        age = self.reader.age()
        stale = age is None or age > self.timeout
        if stale and not self._stale:
            self.get_logger().error(
                f'IMU A: no sample for more than {self.timeout} s '
                f'(last error: {self.reader.last_error!r})')
        elif self._stale and not stale:
            self.get_logger().info('IMU A: samples again')
        self._stale = stale

    def report(self):
        n, e = self.reader.samples, self.reader.errors
        n0, e0 = self._last_count
        self._last_count = (n, e)
        self.get_logger().info(f'IMU A: {(n - n0) / 5.0:.1f} Hz, {e - e0} read errors in 5 s')

    def destroy_node(self):
        self.reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
