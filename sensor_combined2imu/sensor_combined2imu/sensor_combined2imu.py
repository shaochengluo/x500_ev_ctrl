#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class PX4SensorCombinedToImu(Node):
    def __init__(self):
        super().__init__('px4_sensor_combined_to_imu')

        # Parameters
        self.declare_parameter('frame_id', 'base_link')   # ROS FLU body frame
        self.declare_parameter('keep_px4_frd', False)     # if True, no axis conversion
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.keep_frd = self.get_parameter('keep_px4_frd').get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.pub = self.create_publisher(Imu, 'imu/data', qos)
        self.sub = self.create_subscription(SensorCombined, '/cpsl_uav_7/fmu/out/sensor_combined', self.cb, qos)

    def frd_to_flu(self, x, y, z):
        # FRD (PX4) -> FLU (ROS): [x, y, z] -> [ +x, -y, -z ]
        return float(x), -float(y), -float(z)

    def cb(self, msg: SensorCombined):
        imu = Imu()

        # Timestamp: use ROS time; if you need strict PX4 timing, add a time-sync later
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        gx, gy, gz = msg.gyro_rad
        ax, ay, az = msg.accelerometer_m_s2

        if self.keep_frd:
            # Publish in PX4 FRD axes (not typical for ROS consumers)
            imu.angular_velocity.x = float(gx)
            imu.angular_velocity.y = float(gy)
            imu.angular_velocity.z = float(gz)
            imu.linear_acceleration.x = float(ax)
            imu.linear_acceleration.y = float(ay)
            imu.linear_acceleration.z = float(az)
        else:
            # Convert to ROS FLU (recommended)
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = self.frd_to_flu(gx, gy, gz)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = self.frd_to_flu(ax, ay, az)

        # Orientation unknown (you can fill this from another source if available)
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(imu)

def main():
    rclpy.init()
    node = PX4SensorCombinedToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
