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

        # Declare ROS 2 parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('topic_name', 'imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('baudrate', 115200)

        # Get parameter values
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.deg2rad = np.pi / 180.0

        self.ser = serial.Serial(serial_port, baudrate, timeout=1)

        self.pub = self.create_publisher(Imu, topic_name, 10)
        
        self.get_logger().info(f"IMU Publisher started: port={serial_port}, topic={topic_name}, frame_id={self.frame_id}")

        # Timestamp alignment variables
        self.first_arduino_ts = None
        self.first_ros_ts = None

        # Timer just triggers reading; frequency doesn’t matter nows
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            data = json.loads(line)

            # REQUIRED: Arduino timestamp in microseconds
            t_us = int(data['t_us'])

            # Capture first timestamp for alignment
            if self.first_arduino_ts is None:
                self.first_arduino_ts = t_us
                self.first_ros_ts = self.get_clock().now()
            
            # Compute delta time relative to first sample
            dt = (t_us - self.first_arduino_ts) / 1e6  # seconds

            # Build adjusted ROS timestamp
            imu_stamp = Time()
            imu_stamp.sec = self.first_ros_ts.to_msg().sec + int(dt)
            imu_stamp.nanosec = self.first_ros_ts.to_msg().nanosec + int((dt % 1) * 1e9)

            # Normalize nanoseconds
            if imu_stamp.nanosec >= 1e9:
                imu_stamp.sec += 1
                imu_stamp.nanosec -= int(1e9)

            # Build IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = imu_stamp
            imu_msg.header.frame_id = self.frame_id

            # No orientation provided
            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance[0] = -1

            # Angular velocity (deg/s → rad/s)
            imu_msg.angular_velocity.x = float(data['gx']) * self.deg2rad
            imu_msg.angular_velocity.y = float(data['gy']) * self.deg2rad
            imu_msg.angular_velocity.z = float(data['gz']) * self.deg2rad
            imu_msg.angular_velocity_covariance[0] = -1

            # Linear acceleration (m/s^2)
            imu_msg.linear_acceleration.x = float(data['ax'])
            imu_msg.linear_acceleration.y = float(data['ay'])
            imu_msg.linear_acceleration.z = float(data['az'])
            imu_msg.linear_acceleration_covariance[0] = -1

            self.pub.publish(imu_msg)

        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON decode error: {line}")
        except KeyError as e:
            self.get_logger().warn(f"Missing key {e} in: {line}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
