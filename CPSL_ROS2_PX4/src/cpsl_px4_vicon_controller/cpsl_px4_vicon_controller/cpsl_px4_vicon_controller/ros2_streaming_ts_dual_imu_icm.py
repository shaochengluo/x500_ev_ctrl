import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import serial
import json
import numpy as np
import time


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # Parameters
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_8553931393135131D121-if00')
        self.declare_parameter('topic_name', 'imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.baudrate = int(self.get_parameter('baudrate').get_parameter_value().integer_value)

        self.deg2rad = float(np.pi / 180.0)

        self.pub = self.create_publisher(Imu, topic_name, 10)
        self.get_logger().info(
            f"IMU Publisher started: port={serial_port}, topic={topic_name}, frame_id={self.frame_id}"
        )

        # Timestamp alignment
        self.first_arduino_ts = None
        self.first_ros_ts = None  # rclpy.time.Time

        # Serial state
        self.serial_port = serial_port
        self.ser = None
        self.last_good_data_wall = time.time()
        self.last_log_wall = 0.0

        # Open serial initially
        self._open_serial()

        # Timer: 200 Hz is plenty even if IMU is 100 Hz
        self.timer = self.create_timer(0.005, self.timer_callback)  # 5ms

    def _open_serial(self):
        # Close old if needed
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        # Retry loop (non-blocking for ROS; short sleep)
        for attempt in range(1, 1000000):
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baudrate,
                    timeout=0.2,        # KEY: prevents "freeze"
                    write_timeout=0.2,
                )
                # Clear any partial line garbage
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

                self.get_logger().info(f"Serial connected: {self.serial_port}")
                self.first_arduino_ts = None
                self.first_ros_ts = None
                self.last_good_data_wall = time.time()
                return
            except Exception as e:
                # Throttle logs
                now = time.time()
                if now - self.last_log_wall > 1.0:
                    self.get_logger().warn(f"Serial open failed ({self.serial_port}): {e} (retrying...)")
                    self.last_log_wall = now
                time.sleep(0.5)

    def _maybe_reconnect_on_stale(self):
        # If no valid t_us samples in 2 seconds, reopen the port
        if time.time() - self.last_good_data_wall > 2.0:
            self.get_logger().warn("No valid IMU data for >2s; reconnecting serial...")
            self._open_serial()

    def _build_stamp_from_arduino_us(self, t_us: int) -> Time:
        # Capture first sample for alignment
        if self.first_arduino_ts is None:
            self.first_arduino_ts = t_us
            self.first_ros_ts = self.get_clock().now()

        dt_ns = int((t_us - self.first_arduino_ts) * 1000)  # us -> ns

        base = self.first_ros_ts.to_msg()
        sec = base.sec
        nsec = base.nanosec + dt_ns

        # Normalize nanoseconds with full carry
        if nsec >= 1_000_000_000:
            carry = nsec // 1_000_000_000
            sec += int(carry)
            nsec = int(nsec % 1_000_000_000)

        stamp = Time()
        stamp.sec = int(sec)
        stamp.nanosec = int(nsec)
        return stamp

    def timer_callback(self):
        if self.ser is None:
            self._open_serial()
            return

        try:
            raw = self.ser.readline()  # returns b'' on timeout
            if not raw:
                self._maybe_reconnect_on_stale()
                return

            line = raw.decode('utf-8', errors='replace').strip()
            if not line:
                self._maybe_reconnect_on_stale()
                return

            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                # Ignore partial/garbage lines quietly (optional: throttle warn)
                return

            # Ignore log lines without spamming warnings
            if 't_us' not in data:
                if 'log' in data:
                    # Optional: throttle-print log messages
                    # now = time.time()
                    # if now - self.last_log_wall > 1.0:
                    #     self.get_logger().info(f"MCU: {data.get('log')}")
                    #     self.last_log_wall = now
                    return
                # Only warn on truly unexpected JSON
                self.get_logger().warn(f"Missing key 't_us' in: {line}")
                return

            # Valid data packet
            t_us = int(data['t_us'])
            imu_stamp = self._build_stamp_from_arduino_us(t_us)

            imu_msg = Imu()
            imu_msg.header.stamp = imu_stamp
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance[0] = -1.0

            # Angular velocity: deg/s -> rad/s
            imu_msg.angular_velocity.x = float(data['gx']) * self.deg2rad
            imu_msg.angular_velocity.y = float(data['gy']) * self.deg2rad
            imu_msg.angular_velocity.z = float(data['gz']) * self.deg2rad
            imu_msg.angular_velocity_covariance[0] = -1.0

            # Linear acceleration: m/s^2
            imu_msg.linear_acceleration.x = float(data['ax'])
            imu_msg.linear_acceleration.y = float(data['ay'])
            imu_msg.linear_acceleration.z = float(data['az'])
            imu_msg.linear_acceleration_covariance[0] = -1.0

            self.pub.publish(imu_msg)
            self.last_good_data_wall = time.time()

        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial read error: {e}; reconnecting...")
            self._open_serial()
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            # If something weird happens repeatedly, reconnect as a safety net
            self._maybe_reconnect_on_stale()


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
