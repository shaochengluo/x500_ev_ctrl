#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
import numpy as np
from time import time_ns

# ------------ Small quaternion helpers ------------
def quat_to_rotmat(x, y, z, w):
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz),     2*(xy-wz),       2*(xz+wy)],
        [2*(xy+wz),         1 - 2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),         2*(yz+wx),       1 - 2*(xx+yy)]
    ], dtype=float)

def rotmat_to_quat(R):
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t+1.0)*2.0
        w = 0.25*s
        x = (R[2,1]-R[1,2])/s
        y = (R[0,2]-R[2,0])/s
        z = (R[1,0]-R[0,1])/s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])*2.0
            w = (R[2,1]-R[1,2])/s; x = 0.25*s
            y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])*2.0
            w = (R[0,2]-R[2,0])/s
            x = (R[0,1]+R[1,0])/s; y = 0.25*s
            z = (R[1,2]+R[2,1])/s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])*2.0
            w = (R[1,0]-R[0,1])/s
            x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s
            z = 0.25*s
    return np.array([x, y, z, w], dtype=float)

# ------------ ENU -> NED helpers ------------
C_ENU2NED = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0,-1]], dtype=float)

def enu_pos_to_ned(p_enu):
    x_e, y_e, z_e = p_enu
    return np.array([y_e, x_e, -z_e], dtype=float)

def enu_quat_to_ned(q_enu_xyzw):
    x, y, z, w = q_enu_xyzw
    R_enu = quat_to_rotmat(x, y, z, w)
    R_ned = C_ENU2NED @ R_enu @ C_ENU2NED.T
    q_ned_xyzw = rotmat_to_quat(R_ned)
    x, y, z, w = q_ned_xyzw
    # PX4 expects [w, x, y, z]
    return [float(w), float(x), float(y), float(z)]

class ViconToPX4EV(Node):
    def __init__(self):
        super().__init__('vicon_to_px4_ev')

        # -------- Parameters --------
        self.declare_parameter('vicon_topic', '/vicon/x500_3/x500_3')
        self.declare_parameter('use_fd_velocity', True)
        self.declare_parameter('pos_var', 0.0004)   # m^2 (σ ~ 2 cm)
        self.declare_parameter('vel_var', 0.0025)   # (m/s)^2 (σ ~ 5 cm/s)
        self.declare_parameter('ori_var', 1e-4)     # rad^2
        self.declare_parameter('rate_hz', 30.0)     # <-- downsample rate

        vicon_topic = self.get_parameter('vicon_topic').get_parameter_value().string_value
        self.use_fd_velocity = self.get_parameter('use_fd_velocity').get_parameter_value().bool_value
        self.pos_var = float(self.get_parameter('pos_var').value)
        self.vel_var = float(self.get_parameter('vel_var').value)
        self.ori_var = float(self.get_parameter('ori_var').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # -------- QoS --------
        # Ingest: keep only the latest (drop-friendly)
        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        # Output: also Best Effort + depth=1 so we don't queue and create lag
        out_qos = sensor_qos

        # Sub: cache latest Vicon frame
        self.sub = self.create_subscription(TransformStamped, vicon_topic, self.cb, sensor_qos)

        # Pub: downsampled EV to PX4
        self.pub = self.create_publisher(VehicleOdometry, 'fmu/in/vehicle_visual_odometry', out_qos)

        # Cached frames (latest & previous) for FD velocity
        self.last_tf = None
        self.prev_tf = None

        # 30 Hz (or param) timer publisher
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_ev)

        self.get_logger().info(f'Listening to Vicon (TransformStamped): {vicon_topic}')
        self.get_logger().info(f'Publishing EV to /fmu/in/vehicle_visual_odometry at {self.rate_hz:.1f} Hz (BestEffort depth=1)')
        self.get_logger().info('Origin: using Vicon world origin (no rebasing)')

        # Prebuild 6x6 upper-tri covariance (21 elems)
        self.pose_cov_21 = self._compact_cov(self.pos_var, self.ori_var)
        self.vel_cov_21  = self._compact_cov(self.vel_var, 0.0)

    def _compact_cov(self, lin_var, ang_var):
        diag = [lin_var, lin_var, lin_var, ang_var, ang_var, ang_var]
        out = []
        for i in range(6):
            for j in range(i, 6):
                out.append(diag[i] if i == j else 0.0)
        return out

    # -------- Ingest: just cache latest frame --------
    def cb(self, msg: TransformStamped):
        self.prev_tf = self.last_tf
        self.last_tf = msg

    # -------- 30 Hz publisher --------
    def publish_ev(self):
        if self.last_tf is None:
            return

        # Extract latest pose (Vicon ENU)
        tf = self.last_tf
        t = tf.transform.translation
        q = tf.transform.rotation
        p_enu = np.array([t.x, t.y, t.z], dtype=float)
        q_enu = [q.x, q.y, q.z, q.w]

        # ROS capture time from header; fallback to node clock if zero
        t_ros_ns = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
        if t_ros_ns == 0:
            t_ros_ns = int(self.get_clock().now().nanoseconds)
        t_ros_us = int(t_ros_ns // 1000)  # PX4 expects microseconds

        # ENU -> NED (no origin subtraction)
        p_ned = enu_pos_to_ned(p_enu)
        qw, qx, qy, qz = enu_quat_to_ned(q_enu)

        # Finite-difference linear velocity in NED (from two cached frames)
        vx = vy = vz = 0.0
        if self.use_fd_velocity and (self.prev_tf is not None):
            # prev
            pt = self.prev_tf.transform.translation
            p_prev_enu = np.array([pt.x, pt.y, pt.z], dtype=float)
            p_prev_ned = enu_pos_to_ned(p_prev_enu)

            t_prev_ns = self.prev_tf.header.stamp.sec * 10**9 + self.prev_tf.header.stamp.nanosec
            if t_prev_ns == 0:
                t_prev_ns = t_ros_ns  # fallback

            dt = (t_ros_ns - t_prev_ns) * 1e-9
            if 1e-6 < dt < 1.0:
                dp = p_ned - p_prev_ned
                vx, vy, vz = (dp / dt).tolist()

        # Build PX4 VehicleOdometry (µs timestamps!)
        odom = VehicleOdometry()
        now_us = int(time_ns() // 1000)
        odom.timestamp = now_us                      # publish time (PX4 time recommended)
        odom.timestamp_sample = t_ros_us or now_us   # sample time (PX4 time recommended)

        # Frames (if available in your px4_msgs version)
        if hasattr(VehicleOdometry, 'POSE_FRAME_NED'):
            odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        if hasattr(VehicleOdometry, 'VELOCITY_FRAME_NED'):
            odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # Pose & orientation
        odom.position = [float(p_ned[0]), float(p_ned[1]), float(p_ned[2])]
        odom.q = [float(qw), float(qx), float(qy), float(qz)]

        # Linear & angular velocities
        odom.velocity = [float(vx), float(vy), float(vz)]
        odom.angular_velocity = [0.0, 0.0, 0.0]

        # Covariances (newer msg) or variances (older msg)
        if hasattr(odom, 'pose_covariance') and hasattr(odom, 'velocity_covariance'):
            odom.pose_covariance = self.pose_cov_21
            odom.velocity_covariance = self.vel_cov_21
        else:
            if hasattr(odom, 'position_variance'):
                odom.position_variance = [self.pos_var]*3
            if hasattr(odom, 'orientation_variance'):
                odom.orientation_variance = [self.ori_var]*3
            if hasattr(odom, 'velocity_variance'):
                odom.velocity_variance = [self.vel_var]*3

        if hasattr(odom, 'reset_counter'):
            odom.reset_counter = 0
        if hasattr(odom, 'quality'):
            odom.quality = 100

        # Publish at 30 Hz
        self.pub.publish(odom)

def main():
    rclpy.init()
    node = ViconToPX4EV()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
