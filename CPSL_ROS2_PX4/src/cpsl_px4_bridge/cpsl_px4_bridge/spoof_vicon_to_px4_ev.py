#!/usr/bin/env python3
import numpy as np
from time import time_ns

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped, Vector3Stamped
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleOdometry


# ---------------- Quaternion / rotation helpers ----------------
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
            w = (R[2,1]-R[1,2])/s
            x = 0.25*s
            y = (R[0,1]+R[1,0])/s
            z = (R[0,2]+R[2,0])/s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])*2.0
            w = (R[0,2]-R[2,0])/s
            x = (R[0,1]+R[1,0])/s
            y = 0.25*s
            z = (R[1,2]+R[2,1])/s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])*2.0
            w = (R[1,0]-R[0,1])/s
            x = (R[0,2]+R[2,0])/s
            y = (R[1,2]+R[2,1])/s
            z = 0.25*s
    return np.array([x, y, z, w], dtype=float)


# ---------------- ENU <-> NED helpers ----------------
C_ENU2NED = np.array([[0, 1, 0],
                      [1, 0, 0],
                      [0, 0,-1]], dtype=float)

def enu_pos_to_ned(p_enu):
    return (C_ENU2NED @ p_enu.reshape(3,1)).ravel()

def enu_quat_to_ned(q_enu_xyzw):
    x, y, z, w = q_enu_xyzw
    R_enu = quat_to_rotmat(x, y, z, w)
    R_ned = C_ENU2NED @ R_enu @ C_ENU2NED.T
    q_ned_xyzw = rotmat_to_quat(R_ned)
    x, y, z, w = q_ned_xyzw
    # PX4 expects [w, x, y, z]
    return [float(w), float(x), float(y), float(z)]


class SpoofViconToPX4EV(Node):
    """
    Subscribes:
      - Vicon pose (TransformStamped) in ENU frame
      - spoofing_position_cmd   (Vector3Stamped) in NED (VehicleOdometry frame)
      - spoofing_velocity_cmd   (Vector3Stamped) in NED (VehicleOdometry frame)
      - spoofing_enable         (Bool)

    Publishes:
      - VehicleOdometry to PX4 (/fmu/in/vehicle_visual_odometry) with injected offsets.
    """

    def __init__(self):
        super().__init__('spoof_vicon_to_px4_ev')

        # -------- Parameters --------
        self.declare_parameter('vicon_topic', '/vicon/x500_3/x500_3')
        self.declare_parameter('ev_topic_out', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('spoof_pos_topic', '/redirection/spoofing_position_cmd')
        self.declare_parameter('spoof_vel_topic', '/redirection/spoofing_velocity_cmd')
        self.declare_parameter('spoof_enable_topic', '/redirection/spoofing_enable')

        self.declare_parameter('use_fd_velocity', True)
        self.declare_parameter('pos_var', 0.0004)
        self.declare_parameter('vel_var', 0.0025)
        self.declare_parameter('ori_var', 1e-4)

        self.declare_parameter('start_after_s', -1.0)
        self.declare_parameter('stop_after_s', -1.0)
        self.declare_parameter('enable_on_start', False)

        vicon_topic   = self.get_parameter('vicon_topic').value
        ev_topic_out  = self.get_parameter('ev_topic_out').value
        pos_cmd_topic = self.get_parameter('spoof_pos_topic').value
        vel_cmd_topic = self.get_parameter('spoof_vel_topic').value
        en_topic      = self.get_parameter('spoof_enable_topic').value

        self.use_fd_velocity = bool(self.get_parameter('use_fd_velocity').value)
        self.pos_var = float(self.get_parameter('pos_var').value)
        self.vel_var = float(self.get_parameter('vel_var').value)
        self.ori_var = float(self.get_parameter('ori_var').value)

        self.start_after_s = float(self.get_parameter('start_after_s').value)
        self.stop_after_s  = float(self.get_parameter('stop_after_s').value)
        self.enable_on_start = bool(self.get_parameter('enable_on_start').value)

        # -------- QoS --------
        sensor_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # -------- I/O --------
        self.sub_vicon = self.create_subscription(TransformStamped, vicon_topic, self.cb_vicon, sensor_qos)
        self.sub_pos   = self.create_subscription(Vector3Stamped, pos_cmd_topic, self.cb_pos_cmd, 10)
        self.sub_vel   = self.create_subscription(Vector3Stamped, vel_cmd_topic, self.cb_vel_cmd, 10)
        self.sub_en    = self.create_subscription(Bool, en_topic, self.cb_enable, 10)

        self.pub_ev    = self.create_publisher(VehicleOdometry, ev_topic_out, 20)

        # -------- State --------
        self.prev_t_ns = None
        # self.prev_p_ned_raw = None

        self.pos_off_enu = np.zeros(3)
        self.vel_off_enu = np.zeros(3)

        self.node_start_ns = int(self.get_clock().now().nanoseconds)
        self.manual_enable = self.enable_on_start

        self.pose_cov_21 = self._compact_cov(self.pos_var, self.ori_var)
        self.vel_cov_21  = self._compact_cov(self.vel_var, 0.0)

        self._log_counter = 0
        self.get_logger().info(f'Listening Vicon ENU: {vicon_topic}')
        self.get_logger().info(f'Publishing PX4 EV: {ev_topic_out}')

    # ------- Helpers -------
    def _compact_cov(self, lin_var, ang_var):
        diag = [lin_var, lin_var, lin_var, ang_var, ang_var, ang_var]
        out = []
        for i in range(6):
            for j in range(i, 6):
                out.append(diag[i] if i == j else 0.0)
        return out

    def _spoof_active(self, now_ns: int) -> bool:
        active = self.manual_enable
        dt_s = (now_ns - self.node_start_ns) * 1e-9
        if self.start_after_s >= 0:
            active = active or (dt_s >= self.start_after_s)
        if self.stop_after_s >= 0 and dt_s >= self.stop_after_s:
            active = False
        return active

    # ------- Subscribers -------
    def cb_enable(self, msg: Bool):
        self.manual_enable = bool(msg.data)
        self.get_logger().info(f'Spoofing manual_enable -> {self.manual_enable}')

    def cb_pos_cmd(self, msg: Vector3Stamped):
        self.pos_off_enu = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)

    def cb_vel_cmd(self, msg: Vector3Stamped):
        self.vel_off_enu = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)

    def cb_vicon(self, msg: TransformStamped):
        # --- Time handling ---
        t_ros_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if t_ros_ns == 0:
            t_ros_ns = int(self.get_clock().now().nanoseconds)
        t_ros_us = int(t_ros_ns // 1000)
        now_us = int(time_ns() // 1000)

        # --- Pose ENU ---
        t = msg.transform.translation
        q = msg.transform.rotation
        p_enu_raw = np.array([t.x, t.y, t.z], dtype=float)
        q_enu = [q.x, q.y, q.z, q.w]

        # --- Spoof gating ---
        spoof_active = self._spoof_active(t_ros_ns)

        if spoof_active:
            p_enu = p_enu_raw + self.pos_off_enu
            # vx = vx_base + self.vel_off_enu[0]
            # vy = vy_base + self.vel_off_enu[1]
            # vz = vz_base + self.vel_off_enu[2]
        else:
            p_enu = p_enu_raw
            # vx, vy, vz = vx_base, vy_base, vz_base

        # --- Convert ENU -> NED ---
        p_ned = enu_pos_to_ned(p_enu)
        qw, qx, qy, qz = enu_quat_to_ned(q_enu)

        # --- Base velocity from RAW ---
        # vx_base = vy_base = vz_base = 0.0
        # if self.use_fd_velocity and self.prev_t_ns is not None and t_ros_ns > self.prev_t_ns:
        #     dt = (t_ros_ns - self.prev_t_ns) * 1e-9
        #     if 1e-6 < dt < 1.0 and self.prev_p_ned_raw is not None:
        #         dp = p_ned_raw - self.prev_p_ned_raw
        #         vx_base, vy_base, vz_base = (dp / dt).tolist()


        # --- Build VehicleOdometry ---
        odom = VehicleOdometry()
        odom.timestamp = now_us
        odom.timestamp_sample = t_ros_us

        if hasattr(VehicleOdometry, 'POSE_FRAME_NED'):
            odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        if hasattr(VehicleOdometry, 'VELOCITY_FRAME_NED'):
            odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        odom.position = [float(p_ned[0]), float(p_ned[1]), float(p_ned[2])]
        odom.q        = [float(qw), float(qx), float(qy), float(qz)]
        # odom.velocity = [float(vx), float(vy), float(vz)]
        odom.angular_velocity = [0.0, 0.0, 0.0]

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

        self.pub_ev.publish(odom)

        # self.prev_p_ned_raw = p_ned_raw
        self.prev_t_ns = t_ros_ns

        self._log_counter += 1
        if self._log_counter % 20 == 0:
            if spoof_active:
                self.get_logger().info(
                    f"[SPOOF ON] pos_off(ENU)={self.pos_off_enu.round(3)} vel_off(ENU)={self.vel_off_enu.round(3)}"
                )
            else:
                self.get_logger().info("[SPOOF OFF]")


def main():
    rclpy.init()
    node = SpoofViconToPX4EV()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
