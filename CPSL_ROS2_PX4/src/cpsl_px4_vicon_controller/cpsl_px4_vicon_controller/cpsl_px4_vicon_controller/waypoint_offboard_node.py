#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Robust PX4 Offboard waypoint follower using Vicon mocap (VehicleOdometry)
# - Frames: Commands are in PX4 local NED. Helpers provided for ENU↔NED.
# - Phases: WARMUP → TAKEOFF → MISSION → (auto LAND after hold) → LANDING
# - Fixes:
#   * Takeoff completion uses altitude-only + hysteresis (no fragile XY lock).
#   * Waypoint arrival uses relaxed radii, hysteresis, optional speed gate,
#     and a progress-along-segment fallback to avoid “hover forever”.
#   * Yaw is locked once at takeoff completion (unless overridden).
#
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)

# ===================== USER TRAJECTORY CONFIG =====================
TRAJ_MODE = "custom"   # "line" | "square" | "circle" | "custom"
FLIGHT_HEIGHT_M: float = 1.0

# 1) LINE: single target point after takeoff (ENU)
LINE_P1_ENU: Tuple[float, float, float] = (0.4, 0.7, FLIGHT_HEIGHT_M)

# 2) SQUARE (ENU corners)
SQUARE_POINTS_ENU: List[Tuple[float, float, float]] = [
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, 1.0, FLIGHT_HEIGHT_M),
    (1.0, 1.0, FLIGHT_HEIGHT_M),
    (1.0, 0.0, FLIGHT_HEIGHT_M),
]

# 3) CIRCLE (ENU)
CIRCLE_CENTER_ENU: Tuple[float, float, float] = (0.0, 0.0, FLIGHT_HEIGHT_M)
CIRCLE_RADIUS_M: float = 0.7
CIRCLE_NUM_POINTS: int = 16
CIRCLE_CLOCKWISE: bool = True
CIRCLE_START_ANGLE_DEG: float = 0.0

# 4) CUSTOM (ENU)
CUSTOM_WAYPOINTS_ENU: Dict[int, List[Tuple[float, float, float]]] = {
    0: [
        (0.00, 0.00, FLIGHT_HEIGHT_M),
        (0.00, 0.50, FLIGHT_HEIGHT_M),
        (0.00, 1.00, FLIGHT_HEIGHT_M),
        (0.00, 1.50, FLIGHT_HEIGHT_M),
        (0.00, 2.00, FLIGHT_HEIGHT_M),
    ],

    1: [
    (-0.4, -0.3, FLIGHT_HEIGHT_M),
    (0.4, 0.7, FLIGHT_HEIGHT_M),
    ],

    2: [
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    (0.0, 1.5, FLIGHT_HEIGHT_M),
    ],

    3: [
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    (0.0, -1.5, FLIGHT_HEIGHT_M),
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, 1.5, FLIGHT_HEIGHT_M),
    ],

    4: [
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    (0.0, -2.25, FLIGHT_HEIGHT_M),
    (0.0, -1.5, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, 0.75, FLIGHT_HEIGHT_M),
    (0.0, 1.5, FLIGHT_HEIGHT_M),
    ],    

    5: [
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    (0.0, -2.25, FLIGHT_HEIGHT_M),
    (0.0, -1.5, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, 0.75, FLIGHT_HEIGHT_M),
    (0.0, 1.5, FLIGHT_HEIGHT_M),
    ],  

    # Laps
     6: [
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    (0.0, -2.25, FLIGHT_HEIGHT_M),
    (0.0, -1.5, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, 0.75, FLIGHT_HEIGHT_M),
    (0.0, 1.5, FLIGHT_HEIGHT_M),
    (0.0, 0.75, FLIGHT_HEIGHT_M),
    (0.0, 0.0, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -0.75, FLIGHT_HEIGHT_M),
    (0.0, -1.5, FLIGHT_HEIGHT_M),
    (0.0, -2.25, FLIGHT_HEIGHT_M),
    (0.0, -3.0, FLIGHT_HEIGHT_M),
    ],

    # SEMICIRCLE
     7: [
    (0.75, -2.0, FLIGHT_HEIGHT_M),
    (0.43, -1.96, FLIGHT_HEIGHT_M),
    (0.12, -1.83, FLIGHT_HEIGHT_M),
    (-0.13, -1.63, FLIGHT_HEIGHT_M),
    (-0.33, -1.37, FLIGHT_HEIGHT_M),
    (-0.46, -1.07, FLIGHT_HEIGHT_M),
    (-0.5, -0.75, FLIGHT_HEIGHT_M),  # Middle point
    (-0.46, -0.43, FLIGHT_HEIGHT_M),
    (-0.33, -0.12, FLIGHT_HEIGHT_M),
    (-0.13, 0.13, FLIGHT_HEIGHT_M),
    (0.13, 0.33, FLIGHT_HEIGHT_M),
    (0.43, 0.46, FLIGHT_HEIGHT_M),
    (0.75, 0.5, FLIGHT_HEIGHT_M),
],

    #triangle
    8: [
    (0.3, -0.532, FLIGHT_HEIGHT_M),
    (0.55, -0.099, FLIGHT_HEIGHT_M),
    (0.8, 0.334, FLIGHT_HEIGHT_M),
    (1.05, 0.767, FLIGHT_HEIGHT_M),
    (1.3, 1.2, FLIGHT_HEIGHT_M),
    (1.3, 1.2, FLIGHT_HEIGHT_M),
    (0.8, 1.2, FLIGHT_HEIGHT_M),
    (0.3, 1.2, FLIGHT_HEIGHT_M),
    (-0.2, 1.2, FLIGHT_HEIGHT_M),
    (-0.7, 1.2, FLIGHT_HEIGHT_M),
    (-0.7, 1.2, FLIGHT_HEIGHT_M),
    (-0.45, 0.767, FLIGHT_HEIGHT_M),
    (-0.2, 0.334, FLIGHT_HEIGHT_M),
    (0.05, -0.099, FLIGHT_HEIGHT_M),
    (0.3, -0.532, FLIGHT_HEIGHT_M)
],
    #square
    9: [
    (1.1, -1.0, FLIGHT_HEIGHT_M),
    (0.6, -1.0, FLIGHT_HEIGHT_M),
    (0.1, -1.0, FLIGHT_HEIGHT_M),
    (-0.4, -1.0, FLIGHT_HEIGHT_M),
    (-0.4, -1.0, FLIGHT_HEIGHT_M),
    (-0.4, -0.5, FLIGHT_HEIGHT_M),
    (-0.4, 0.0, FLIGHT_HEIGHT_M),
    (-0.4, 0.5, FLIGHT_HEIGHT_M),
    (-0.4, 0.5, FLIGHT_HEIGHT_M),
    (0.1, 0.5, FLIGHT_HEIGHT_M),
    (0.6, 0.5, FLIGHT_HEIGHT_M),
    (1.1, 0.5, FLIGHT_HEIGHT_M),
    (1.1, 0.5, FLIGHT_HEIGHT_M),
    (1.1, 0.0, FLIGHT_HEIGHT_M),
    (1.1, -0.5, FLIGHT_HEIGHT_M),
    (1.1, -1.0, FLIGHT_HEIGHT_M),
],

    #zigzag
    10: [
    (1.1, -1.0, FLIGHT_HEIGHT_M),
    (0.6, -0.83, FLIGHT_HEIGHT_M),
    (0.1, -0.67, FLIGHT_HEIGHT_M),
    (-0.4, -0.5, FLIGHT_HEIGHT_M),
    (-0.4, -0.5, FLIGHT_HEIGHT_M),
    (0.1, -0.33, FLIGHT_HEIGHT_M),
    (0.6, -0.17, FLIGHT_HEIGHT_M),
    (1.1, 0.0, FLIGHT_HEIGHT_M),
    (1.1, 0.0, FLIGHT_HEIGHT_M),
    (0.6, 0.17, FLIGHT_HEIGHT_M),
    (0.1, 0.33, FLIGHT_HEIGHT_M),
    (-0.4, 0.5, FLIGHT_HEIGHT_M)
],

    #star
    11 : [
    # Bottom curved edge
    (1.1, -1.0, FLIGHT_HEIGHT_M),
    (0.8857, -0.8698, FLIGHT_HEIGHT_M),
    (0.6714, -0.7655, FLIGHT_HEIGHT_M),
    (0.4571, -0.7075, FLIGHT_HEIGHT_M),
    (0.2429, -0.7075, FLIGHT_HEIGHT_M),
    (0.0286, -0.7655, FLIGHT_HEIGHT_M),
    (-0.1857, -0.8698, FLIGHT_HEIGHT_M),
    (-0.4, -1.0, FLIGHT_HEIGHT_M),
    (-0.4, -1.0, FLIGHT_HEIGHT_M),

    # Left curved edge
    (-0.2698, -0.7857, FLIGHT_HEIGHT_M),
    (-0.1655, -0.5714, FLIGHT_HEIGHT_M),
    (-0.1075, -0.3571, FLIGHT_HEIGHT_M),
    (-0.1075, -0.1429, FLIGHT_HEIGHT_M),
    (-0.1655,  0.0714, FLIGHT_HEIGHT_M),
    (-0.2698,  0.2857, FLIGHT_HEIGHT_M),
    (-0.4,  0.5, FLIGHT_HEIGHT_M),
    (-0.4,  0.5, FLIGHT_HEIGHT_M),

    # Top curved edge
    (-0.1857, 0.3698, FLIGHT_HEIGHT_M),
    (0.0286,  0.2655, FLIGHT_HEIGHT_M),
    (0.2429,  0.2075, FLIGHT_HEIGHT_M),
    (0.4571,  0.2075, FLIGHT_HEIGHT_M),
    (0.6714,  0.2655, FLIGHT_HEIGHT_M),
    (0.8857,  0.3698, FLIGHT_HEIGHT_M),
    (1.1,  0.5, FLIGHT_HEIGHT_M),
    (1.1,  0.5, FLIGHT_HEIGHT_M),

    # Right curved edge
    (0.9698,  0.2857, FLIGHT_HEIGHT_M),
    (0.8655,  0.0714, FLIGHT_HEIGHT_M),
    (0.8075, -0.1429, FLIGHT_HEIGHT_M),
    (0.8075, -0.3571, FLIGHT_HEIGHT_M),
    (0.8655, -0.5714, FLIGHT_HEIGHT_M),
    (0.9698, -0.7857, FLIGHT_HEIGHT_M),
    (1.1, -1.0, FLIGHT_HEIGHT_M),
],

    # half period sinusoid/hill
    12 : [
    (-0.4000, -1.5000, FLIGHT_HEIGHT_M), # 1: Start point (X-Min)
    (-0.3633, -1.4000, FLIGHT_HEIGHT_M),
    (-0.2568, -1.3000, FLIGHT_HEIGHT_M),
    (-0.0908, -1.2000, FLIGHT_HEIGHT_M),
    (0.1182, -1.1000, FLIGHT_HEIGHT_M),
    (0.3500, -1.0000, FLIGHT_HEIGHT_M),
    (0.5818, -0.9000, FLIGHT_HEIGHT_M),
    (0.7908, -0.8000, FLIGHT_HEIGHT_M),
    (0.9568, -0.7000, FLIGHT_HEIGHT_M),
    (1.0633, -0.6000, FLIGHT_HEIGHT_M),
    (1.1000, -0.5000, FLIGHT_HEIGHT_M), # 11: Midpoint (X-Max)
    (1.0633, -0.4000, FLIGHT_HEIGHT_M),
    (0.9568, -0.3000, FLIGHT_HEIGHT_M),
    (0.7908, -0.2000, FLIGHT_HEIGHT_M),
    (0.5818, -0.1000, FLIGHT_HEIGHT_M),
    (0.3500, 0.0000, FLIGHT_HEIGHT_M), # 16: X-Centerline
    (0.1182, 0.1000, FLIGHT_HEIGHT_M),
    (-0.0908, 0.2000, FLIGHT_HEIGHT_M),
    (-0.2568, 0.3000, FLIGHT_HEIGHT_M),
    (-0.3633, 0.4000, FLIGHT_HEIGHT_M),
    (-0.4000, 0.5000, FLIGHT_HEIGHT_M)  # 21: End point (X-Min)
],

    # figure 8
    13: [
    (0.3500, -0.5000, FLIGHT_HEIGHT_M), # 1: Start/End/Center
    (0.5818, 0.0878, FLIGHT_HEIGHT_M),
    (0.7908, 0.4511, FLIGHT_HEIGHT_M),
    (0.9568, 0.4511, FLIGHT_HEIGHT_M),
    (1.0633, 0.0878, FLIGHT_HEIGHT_M),
    (1.1000, -0.5000, FLIGHT_HEIGHT_M), # 6: Right Peak (X=1.1)
    (1.0633, -1.0878, FLIGHT_HEIGHT_M),
    (0.9568, -1.4511, FLIGHT_HEIGHT_M),
    (0.7908, -1.4511, FLIGHT_HEIGHT_M),
    (0.5818, -1.0878, FLIGHT_HEIGHT_M),
    (0.3500, -0.5000, FLIGHT_HEIGHT_M), # 11: Center Crossing
    (0.1182, 0.0878, FLIGHT_HEIGHT_M),
    (-0.0908, 0.4511, FLIGHT_HEIGHT_M),
    (-0.2568, 0.4511, FLIGHT_HEIGHT_M),
    (-0.3633, 0.0878, FLIGHT_HEIGHT_M),
    (-0.4000, -0.5000, FLIGHT_HEIGHT_M), # 16: Left Peak (X=-0.4)
    (-0.3633, -1.0878, FLIGHT_HEIGHT_M),
    (-0.2568, -1.4511, FLIGHT_HEIGHT_M),
    (-0.0908, -1.4511, FLIGHT_HEIGHT_M),
    (0.1182, -1.0878, FLIGHT_HEIGHT_M),
    (0.3500, -0.5000, FLIGHT_HEIGHT_M)  # 21: End/Center
]



    
    
}

# ===================== FRAME HELPERS =====================
def now_us(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds / 1000)

# ENU→NED: NED=[yE, xE, -zU]
def enu_to_ned(p_enu):
    return np.array([p_enu[1], p_enu[0], -p_enu[2]], dtype=float)

def ned_to_enu(p_ned):
    return np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=float)

def yaw_enu_to_ned(yaw_enu):
    yaw = (math.pi/2.0) - yaw_enu
    return (yaw + math.pi) % (2*math.pi) - math.pi

def quat_yaw_ned(qw, qx, qy, qz) -> float:
    # PX4 VehicleOdometry yaw in NED from quaternion [w,x,y,z], body→world
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi

@dataclass
class Waypoint:
    x: float  # N (m)
    y: float  # E (m)
    z: float  # D (m, down +)
    yaw: float  # radians in NED (NaN means "don’t command")

    @staticmethod
    def from_enu(x_e, y_n, z_u, yaw_deg_enu=None):
        ned = enu_to_ned([x_e, y_n, z_u])
        if yaw_deg_enu is None:
            yaw = float('nan')
        else:
            yaw = yaw_enu_to_ned(math.radians(float(yaw_deg_enu)))
        return Waypoint(float(ned[0]), float(ned[1]), float(ned[2]), float(yaw))

# ===================== WAYPOINT GENERATORS =====================
def generate_line_waypoints() -> List[Waypoint]:
    x, y, z = LINE_P1_ENU
    return [Waypoint.from_enu(x, y, z)]

def generate_square_waypoints() -> List[Waypoint]:
    return [Waypoint.from_enu(*p) for p in SQUARE_POINTS_ENU]

def generate_circle_waypoints() -> List[Waypoint]:
    cx, cy, cz = CIRCLE_CENTER_ENU
    pts = []
    start = math.radians(CIRCLE_START_ANGLE_DEG)
    direction = -1.0 if CIRCLE_CLOCKWISE else 1.0
    for k in range(CIRCLE_NUM_POINTS):
        th = start + direction * (2.0 * math.pi) * (k / CIRCLE_NUM_POINTS)
        xe = cx + CIRCLE_RADIUS_M * math.cos(th)
        yn = cy + CIRCLE_RADIUS_M * math.sin(th)
        pts.append(Waypoint.from_enu(xe, yn, cz))
    return pts

def generate_custom_waypoints(custom_id: int) -> List[Waypoint]:
    return [Waypoint.from_enu(*p) for p in CUSTOM_WAYPOINTS_ENU[custom_id]]

def build_hardcoded_waypoints(custom_id: int = 6) -> List[Waypoint]:
    mode = TRAJ_MODE.lower()
    if mode == "line":
        wps = generate_line_waypoints()
    elif mode == "square":
        wps = generate_square_waypoints()
    elif mode == "circle":
        wps = generate_circle_waypoints()
    elif mode == "custom":
        if custom_id is None:
            raise ValueError("TRAJ_MODE='custom' requires a custom_id.")
        wps = generate_custom_waypoints(custom_id)
    else:
        raise ValueError(f"Unknown TRAJ_MODE '{TRAJ_MODE}'.")
    
    print(f"[DEBUG] build_hardcoded_waypoints: mode={mode}, custom_id={custom_id}, n_wps={len(wps)}")
    return wps

# ===================== NODE =====================
class WaypointOffboard(Node):
    def __init__(self):
        super().__init__('waypoint_offboard')

        # -------- Parameters --------
        # Acceptance radii (relaxed) and holds
        self.declare_parameter('xy_accept', 0.25)     # m (relaxed vs. 0.10)
        self.declare_parameter('z_accept',  0.15)     # m (relaxed vs. 0.08)
        self.declare_parameter('hold_time', 1.0)      # s (per-WP dwell)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)

        # Takeoff options (altitude-only completion)
        self.declare_parameter('enable_takeoff', True)
        self.declare_parameter('takeoff_height_m', 0.6)
        self.declare_parameter('takeoff_hold_s', 2.0)

        # Optional speed gate at waypoint
        self.declare_parameter('speed_gate_enable', True)
        self.declare_parameter('speed_gate_max', 0.05)  # m/s

        # Namespace (so you can change /cpsl_uav_7 easily)
        self.declare_parameter('ns', '/cpsl_uav_10')

        # -------- Mission --------
        waypoints_ned: List[Waypoint] = build_hardcoded_waypoints()
        self.get_logger().info(f"Mission type: {TRAJ_MODE.upper()}, total {len(waypoints_ned)} waypoint(s).")
        self.waypoints: List[Waypoint] = waypoints_ned

        self.xy_accept = float(self.get_parameter('xy_accept').value)
        self.z_accept  = float(self.get_parameter('z_accept').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.rate_hz   = float(self.get_parameter('publish_rate_hz').value)
        self.auto_arm  = bool(self.get_parameter('auto_arm').value)
        self.auto_offb = bool(self.get_parameter('auto_offboard').value)

        self.enable_takeoff   = bool(self.get_parameter('enable_takeoff').value)
        self.takeoff_height_m = float(self.get_parameter('takeoff_height_m').value)
        self.takeoff_hold_s   = float(self.get_parameter('takeoff_hold_s').value)

        self.speed_gate_enable = bool(self.get_parameter('speed_gate_enable').value)
        self.speed_gate_max    = float(self.get_parameter('speed_gate_max').value)

        ns = self.get_parameter('ns').get_parameter_value().string_value.rstrip('/')

        # -------- QoS --------
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # -------- Pub/Sub --------
        self.pub_offboard = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', 10)
        self.pub_traj     = self.create_publisher(TrajectorySetpoint,   f'{ns}/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd      = self.create_publisher(VehicleCommand,       f'{ns}/fmu/in/vehicle_command', 10)

        self.sub_odom = self.create_subscription(
            VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry', self.odom_cb, qos)

        # -------- State --------
        self.odom = None
        self.wp_idx = 0
        self.inside_since = None
        self.offboard_counter = 0
        self.mode_set = False
        self.armed = False
        self.final_hover_sent = False

        # Takeoff machine
        self.phase = 'WARMUP'
        self.to_target = None
        self.to_inside_since = None

        # Yaw to lock after takeoff
        self.after_takeoff_yaw = float('nan')

        # Mission-complete → land after delay
        self.mission_done_time = None
        self.landing_initiated = False
        self.LAND_DELAY_S = 20.0

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)

    # ---------- Helpers ----------
    def odom_cb(self, msg: VehicleOdometry):
        self.odom = msg

    def current_pos_ned(self):
        if self.odom is None:
            return None
        return np.array([self.odom.position[0], self.odom.position[1], self.odom.position[2]], dtype=float)

    def current_vel_ned(self):
        if self.odom is None:
            return None
        # VehicleOdometry.velocity is in local frame (NED)
        return np.array([self.odom.velocity[0], self.odom.velocity[1], self.odom.velocity[2]], dtype=float)

    def current_speed(self):
        v = self.current_vel_ned()
        if v is None:
            return 0.0
        return float(np.linalg.norm(v))

    def current_yaw_ned(self):
        if self.odom is None:
            return float('nan')
        qw, qx, qy, qz = self.odom.q
        return quat_yaw_ned(qw, qx, qy, qz)

    def send_vehicle_command(self, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        cmd = VehicleCommand()
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.param4 = float(param4)
        cmd.param5 = float(param5)
        cmd.param6 = float(param6)
        cmd.param7 = float(param7)
        cmd.command = int(command)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = now_us(self)
        self.pub_cmd.publish(cmd)

    def arm(self):
        self.get_logger().info('Arming…')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True

    def disarm(self):
        self.get_logger().info('Disarming…')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.armed = False

    def set_offboard_mode(self):
        self.get_logger().info('Switching to OFFBOARD…')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.mode_set = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        msg.timestamp = now_us(self)
        self.pub_offboard.publish(msg)

    def publish_traj_to(self, wp: Waypoint, force_yaw: float = None):
        ts = TrajectorySetpoint()
        ts.position = [float(wp.x), float(wp.y), float(wp.z)]
        ts.velocity = [float('nan')] * 3
        ts.acceleration = [float('nan')] * 3
        ts.jerk = [float('nan')] * 3
        ts.yawspeed = float('nan')

        yaw_cmd = None
        if force_yaw is not None and not math.isnan(force_yaw):
            yaw_cmd = force_yaw
        elif not math.isnan(self.after_takeoff_yaw):
            yaw_cmd = self.after_takeoff_yaw
        elif not math.isnan(wp.yaw):
            yaw_cmd = wp.yaw

        ts.yaw = float(yaw_cmd) if yaw_cmd is not None else float('nan')
        ts.timestamp = now_us(self)
        self.pub_traj.publish(ts)

    def publish_traj_position_only(self, pos_ned):
        ts = TrajectorySetpoint()
        ts.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        ts.velocity = [float('nan')] * 3
        ts.acceleration = [float('nan')] * 3
        ts.jerk = [float('nan')] * 3
        ts.yawspeed = float('nan')
        ts.yaw = float('nan')  # don’t force yaw during takeoff
        ts.timestamp = now_us(self)
        self.pub_traj.publish(ts)

    # ---------- Phase machine ----------
    def on_timer(self):
        self.publish_offboard_control_mode()
        p = self.current_pos_ned()

        # ---- WARMUP ----
        if self.phase == 'WARMUP':
            if self.enable_takeoff and p is not None:
                if self.to_target is None:
                    z_to = float(p[2] - self.takeoff_height_m)  # up is negative in NED
                    self.to_target = np.array([float(p[0]), float(p[1]), z_to], dtype=float)
                    enu = ned_to_enu(self.to_target)
                    self.get_logger().info(f"Warmup takeoff target ENU: x={enu[0]:.2f}, y={enu[1]:.2f}, z={enu[2]:.2f}")
                self.publish_traj_position_only(self.to_target)
            elif self.waypoints:
                self.publish_traj_to(self.waypoints[0])

            # Offboard warmup
            if self.offboard_counter < 10:
                self.offboard_counter += 1
                return
            else:
                if not self.mode_set and self.auto_offb:
                    self.set_offboard_mode()
                if not self.armed and self.auto_arm:
                    self.arm()
                self.phase = 'TAKEOFF'

        if p is None:
            return

        # ---- TAKEOFF ----
        if self.phase == 'TAKEOFF':
            if self.to_target is None:
                z_to = float(p[2] - self.takeoff_height_m)
                self.to_target = np.array([float(p[0]), float(p[1]), z_to], dtype=float)
                enu = ned_to_enu(self.to_target)
                self.get_logger().info(
                    f"Takeoff target ENU: x={enu[0]:.2f}, y={enu[1]:.2f}, z={enu[2]:.2f} (hold {self.takeoff_hold_s:.1f}s)"
                )

            self.publish_traj_position_only(self.to_target)

            # Altitude-only completion with hysteresis
            err = self.to_target - p
            z_err = abs(float(err[2]))
            Z_IN   = max(self.z_accept, 0.10)
            Z_KEEP = Z_IN + 0.05
            tnow = time.time()

            if z_err <= Z_IN:
                if self.to_inside_since is None:
                    self.to_inside_since = tnow
                elif z_err > Z_KEEP:
                    self.to_inside_since = None
            else:
                if z_err > Z_KEEP:
                    self.to_inside_since = None

            if self.to_inside_since and (tnow - self.to_inside_since) >= self.takeoff_hold_s:
                self.after_takeoff_yaw = self.current_yaw_ned()
                self.get_logger().info(f"Takeoff complete; lock yaw {self.after_takeoff_yaw:.3f} rad. → MISSION")
                self.phase = 'MISSION'
                self.to_target = None
                self.to_inside_since = None
            return

        # ---- MISSION ----
        if self.phase == 'MISSION':
            # Mission complete?
            if self.wp_idx >= len(self.waypoints):
                if self.mission_done_time is None:
                    self.mission_done_time = time.time()
                    if not self.final_hover_sent and self.waypoints:
                        self.get_logger().info('Mission complete — holding last setpoint.')
                        self.publish_traj_to(self.waypoints[-1], force_yaw=self.after_takeoff_yaw)
                        self.final_hover_sent = True

                if (not self.landing_initiated) and (time.time() - self.mission_done_time >= self.LAND_DELAY_S):
                    self.get_logger().info("Landing delay elapsed — initiating NAV_LAND.")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.landing_initiated = True
                    self.phase = 'LANDING'
                return

            # Track current waypoint
            tgt_wp = self.waypoints[self.wp_idx]
            self.publish_traj_to(tgt_wp, force_yaw=self.after_takeoff_yaw)

            tgt = np.array([tgt_wp.x, tgt_wp.y, tgt_wp.z], dtype=float)
            err = tgt - p
            xy_err = float(np.linalg.norm(err[:2]))
            z_err  = abs(float(err[2]))

            # Hysteresis bands
            XY_IN,  XY_KEEP = max(self.xy_accept, 0.20), max(self.xy_accept, 0.20) + 0.10
            Z_IN,   Z_KEEP  = max(self.z_accept,  0.12), max(self.z_accept,  0.12) + 0.06
            MIN_DWELL       = max(0.5, min(self.hold_time, 2.0))  # cap dwell to short value

            # Optional speed gate
            inside_pos = (xy_err <= XY_IN) and (z_err <= Z_IN)
            if self.speed_gate_enable:
                spd = self.current_speed()
                inside = inside_pos and (spd <= self.speed_gate_max)
            else:
                inside = inside_pos

            tnow = time.time()
            if inside:
                if self.inside_since is None:
                    self.inside_since = tnow
                else:
                    if (xy_err > XY_KEEP) or (z_err > Z_KEEP):
                        self.inside_since = None
            else:
                if (xy_err > XY_KEEP) or (z_err > Z_KEEP):
                    self.inside_since = None

            # Progress-along-segment fallback (avoid hanging just past WP)
            advanced = False
            if self.wp_idx > 0:
                p0 = np.array([self.waypoints[self.wp_idx-1].x,
                               self.waypoints[self.wp_idx-1].y,
                               self.waypoints[self.wp_idx-1].z], float)
                seg = tgt - p0
                L = float(np.linalg.norm(seg))
                if L > 1e-3:
                    progress = float(np.dot(p - p0, seg) / (L*L))
                    # If we've passed beyond the waypoint and are within loose band, advance
                    if progress >= 1.0 and xy_err <= XY_KEEP:
                        self.get_logger().info(f"Passed WP {self.wp_idx+1}/{len(self.waypoints)}; advancing.")
                        self.wp_idx += 1
                        self.inside_since = None
                        advanced = True

            if not advanced and self.inside_since and (tnow - self.inside_since) >= MIN_DWELL:
                self.get_logger().info(f"Reached WP {self.wp_idx+1}/{len(self.waypoints)}; advancing.")
                self.wp_idx += 1
                self.inside_since = None
            return

        # ---- LANDING ----
        if self.phase == 'LANDING':
            # NAV_LAND already sent; keep streaming OffboardControlMode harmlessly
            return

def main():
    rclpy.init()
    node = WaypointOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
