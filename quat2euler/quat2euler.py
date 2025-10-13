#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from tf_transformations import euler_from_quaternion
from math import degrees

from px4_msgs.msg import VehicleOdometry


class PoseToEuler(Node):
    def __init__(self):
        super().__init__('pose_to_euler')

        # --- QoS ---
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # --- Parameters ---
        self.declare_parameter('pose_topic', '/vicon/x500_7/x500_7')      # geometry_msgs/PoseStamped
        self.declare_parameter('tf_topic',   '/vicon/x500_7/x500_7')      # geometry_msgs/TransformStamped (only if present)
        self.declare_parameter('vo_topic',   '/cpsl_uav_7/fmu/out/vehicle_odometry')  # px4_msgs/VehicleOdometry

        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        tf_topic   = self.get_parameter('tf_topic').get_parameter_value().string_value
        vo_topic   = self.get_parameter('vo_topic').get_parameter_value().string_value

        # --- Subscribers ---
        self.sub_pose = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, qos)
        self.sub_tf   = self.create_subscription(TransformStamped, tf_topic, self.tf_callback, qos)
        self.sub_vo   = self.create_subscription(VehicleOdometry, vo_topic, self.vehicle_odometry_callback, qos)

        # --- Publishers (distinct per source) ---
        self.pose_pub_rad = self.create_publisher(Vector3, '/euler/pose/rad', 10)
        self.pose_pub_deg = self.create_publisher(Vector3, '/euler/pose/deg', 10)

        self.tf_pub_rad = self.create_publisher(Vector3, '/euler/tf/rad', 10)
        self.tf_pub_deg = self.create_publisher(Vector3, '/euler/tf/deg', 10)

        self.vo_pub_rad = self.create_publisher(Vector3, '/euler/vo/rad', 10)
        self.vo_pub_deg = self.create_publisher(Vector3, '/euler/vo/deg', 10)

        self.get_logger().info(f"Subscribing PoseStamped from: {pose_topic}")
        self.get_logger().info(f"Subscribing TransformStamped from: {tf_topic}")
        self.get_logger().info(f"Subscribing VehicleOdometry from: {vo_topic}")
        self.get_logger().info("Publishing to /euler/(pose|tf|vo)/(rad|deg)")

    # ---------------- PoseStamped path ----------------
    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.pose_pub_rad.publish(Vector3(x=roll, y=pitch, z=yaw))
        self.pose_pub_deg.publish(Vector3(x=degrees(roll), y=degrees(pitch), z=degrees(yaw)))

        self.get_logger().info(
            f"[PoseStamped] Euler (deg)  Roll: {degrees(roll):6.2f}  "
            f"Pitch: {degrees(pitch):6.2f}  Yaw: {degrees(yaw):6.2f}"
        )

    # ---------------- TransformStamped path ----------------
    def tf_callback(self, msg: TransformStamped):
        q = msg.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.tf_pub_rad.publish(Vector3(x=roll, y=pitch, z=yaw))
        self.tf_pub_deg.publish(Vector3(x=degrees(roll), y=degrees(pitch), z=degrees(yaw)))

        self.get_logger().info(
            f"[Transform {msg.header.frame_id}->{msg.child_frame_id}] Euler (deg)  "
            f"Roll: {degrees(roll):6.2f}  Pitch: {degrees(pitch):6.2f}  Yaw: {degrees(yaw):6.2f}"
        )

    # ---------------- VehicleOdometry (PX4) path ----------------
    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        """
        Convert px4_msgs/VehicleOdometry quaternion to Euler angles.
        PX4 order: q = [w, x, y, z]; tf expects [x, y, z, w].
        """
        q = list(msg.q)  # [w, x, y, z]
        if len(q) != 4:
            self.get_logger().warn("VehicleOdometry.q does not have length 4.")
            return

        q_tf = [q[1], q[2], q[3], q[0]]  # -> [x, y, z, w]
        roll, pitch, yaw = euler_from_quaternion(q_tf)

        self.vo_pub_rad.publish(Vector3(x=roll, y=pitch, z=yaw))
        self.vo_pub_deg.publish(Vector3(x=degrees(roll), y=degrees(pitch), z=degrees(yaw)))

        self.get_logger().info(
            f"[VehicleOdometry] Euler (deg)  Roll: {degrees(roll):6.2f}  "
            f"Pitch: {degrees(pitch):6.2f}  Yaw: {degrees(yaw):6.2f}"
        )


def main():
    rclpy.init()
    node = PoseToEuler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
