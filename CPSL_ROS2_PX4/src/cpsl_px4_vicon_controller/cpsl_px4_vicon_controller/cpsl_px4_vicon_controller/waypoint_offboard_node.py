#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

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

# ===================== USER TRAJECTORY CONFIG (EDIT THESE) =====================
# Choose one: "line", "square", "circle"
TRAJ_MODE = "line"   # "line" | "square" | "circle"

# All coordinates below are in ENU: (x_east, y_north, z_up) in meters
# 1) LINE: single target point after takeoff
LINE_P1_ENU: Tuple[float, float, float] = (0.0, 0.01, 0.6)

# 2) SQUARE: four corners 1→2→3→4 (takeoff point is **not** a corner)
# SQUARE_POINTS_ENU: List[Tuple[float, float, float]] = [
#     (-0.3, 0.0, 0.8),    # P1
#     (-0.3, -0.5, 0.8),   # P2
#     (1.3, -0.5, 0.8),    # P3
#     (1.3, 0.0, 0.8),     # P4
# ]

SQUARE_POINTS_ENU: List[Tuple[float, float, float]] = [
    (0.07, -0.50, 0.6)   # P1
    # (0.0, -0.5, 0.8),   # P2
    # (0.0, -0.5, 0.8),    # P3
    # (0.0, -0.5, 0.8),     # P4
]

# 3) CIRCLE: center + radius; creates NUM_POINTS waypoints around a circle at constant z
CIRCLE_CENTER_ENU: Tuple[float, float, float] = (0.0, 0.0, 0.6)
CIRCLE_RADIUS_M: float = 0.7
CIRCLE_NUM_POINTS: int = 16
CIRCLE_CLOCKWISE: bool = True
CIRCLE_START_ANGLE_DEG: float = 0.0  # 0° means start at (cx+R, cy)

# ==============================================================================

def now_us(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds / 1000)

# ENU -> NED mapping (for Vicon ENU -> PX4 local NED)
# NED = [y_enu, x_enu, -z_enu]
# yaw_ned = pi/2 - yaw_enu  (wrap to [-pi, pi])
def enu_to_ned(p_enu):
    return np.array([p_enu[1], p_enu[0], -p_enu[2]], dtype=float)

def yaw_enu_to_ned(yaw_enu):
    yaw = (math.pi/2.0) - yaw_enu
    return (yaw + math.pi) % (2*math.pi) - math.pi

def ned_to_enu(p_ned):
    # inverse of enu_to_ned: ENU = [y_ned, x_ned, -z_ned]
    return np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=float)

def quat_yaw_ned(qw, qx, qy, qz) -> float:
    """Compute yaw (heading) from PX4 VehicleOdometry quaternion (NED frame)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi

@dataclass
class Waypoint:
    x: float  # N (m)
    y: float  # E (m)
    z: float  # D (m, down positive in NED)
    yaw: float  # radians in NED

    @staticmethod
    def from_enu(x_e, y_n, z_u, yaw_deg_enu=None):
        ned = enu_to_ned([x_e, y_n, z_u])
        if yaw_deg_enu is None:
            yaw = float('nan')
        else:
            yaw = yaw_enu_to_ned(math.radians(float(yaw_deg_enu)))
        return Waypoint(float(ned[0]), float(ned[1]), float(ned[2]), float(yaw))

# ---------- Generators for hardcoded missions (produce NED waypoints) ----------
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

def build_hardcoded_waypoints() -> List[Waypoint]:
    mode = TRAJ_MODE.lower()
    if mode == "line":
        wps = generate_line_waypoints()
    elif mode == "square":
        wps = generate_square_waypoints()
    elif mode == "circle":
        wps = generate_circle_waypoints()
    else:
        raise ValueError(f"Unknown TRAJ_MODE '{TRAJ_MODE}'. Use 'line' | 'square' | 'circle'.")
    return wps

class WaypointOffboard(Node):
    def __init__(self):
        super().__init__('waypoint_offboard')

        # ---------------- Parameters (waypoints are HARDCODED) ----------------
        self.declare_parameter('xy_accept', 0.10)  # m
        self.declare_parameter('z_accept', 0.08)   # m
        self.declare_parameter('hold_time', 10.0)   # s
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)

        # --- Takeoff options ---
        self.declare_parameter('enable_takeoff', True)
        self.declare_parameter('takeoff_height_m', 0.6)
        self.declare_parameter('takeoff_hold_s', 10.0)

        # -------- Hardcoded mission (no YAML for waypoints) --------
        waypoints_ned: List[Waypoint] = build_hardcoded_waypoints()
        self.get_logger().info(f"Mission type: {TRAJ_MODE.upper()}, total {len(waypoints_ned)} waypoint(s).")
        self.waypoints: List[Waypoint] = waypoints_ned

        self.xy_accept = float(self.get_parameter('xy_accept').value)
        self.z_accept = float(self.get_parameter('z_accept').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)

        self.enable_takeoff = bool(self.get_parameter('enable_takeoff').value)
        self.takeoff_height_m = float(self.get_parameter('takeoff_height_m').value)
        self.takeoff_hold_s  = float(self.get_parameter('takeoff_hold_s').value)

        # ---------------- Publishers/Subscribers ----------------
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/cpsl_uav_7/fmu/in/offboard_control_mode', 10)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, '/cpsl_uav_7/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/cpsl_uav_7/fmu/in/vehicle_command', 10)

        self.sub_odom = self.create_subscription(
            VehicleOdometry, '/cpsl_uav_7/fmu/out/vehicle_odometry', self.odom_cb, qos)

        # ---------------- State ----------------
        self.odom = None
        self.wp_idx = 0
        self.inside_since = None
        self.offboard_counter = 0
        self.mode_set = False
        self.armed = False
        self.final_hover_sent = False

        # Takeoff state machine
        self.phase = 'WARMUP'          # WARMUP -> (TAKEOFF | MISSION | MISSION_DONE | LANDING)
        self.to_target = None          # NED [x, y, z] takeoff target
        self.to_inside_since = None

        # Yaw handling: record yaw after successful takeoff to reuse for all waypoints
        self.after_takeoff_yaw = float('nan')

        # Mission-complete → landing delay
        self.mission_done_time = None
        self.landing_initiated = False
        self.LAND_DELAY_S = 1000.0  # <-- land 10 s after mission complete

        self.timer = self.create_timer(1.0/self.rate_hz, self.on_timer)

    def odom_cb(self, msg: VehicleOdometry):
        self.odom = msg

    def current_pos_ned(self):
        if self.odom is None:
            return None
        return np.array([self.odom.position[0], self.odom.position[1], self.odom.position[2]], dtype=float)

    def current_yaw_ned(self):
        if self.odom is None:
            return float('nan')
        qw, qx, qy, qz = self.odom.q  # PX4 VehicleOdometry: [w, x, y, z]
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
        self.get_logger().info('Arming...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.armed = True

    def disarm(self):
        self.get_logger().info('Disarming...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.armed = False

    def set_offboard_mode(self):
        self.get_logger().info('Switching to OFFBOARD...')
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
        ts.yaw = float('nan')  # don't force yaw during takeoff
        ts.timestamp = now_us(self)
        self.pub_traj.publish(ts)

    def on_timer(self):
        # Always stream OffboardControlMode
        self.publish_offboard_control_mode()

        # We want to stream a position setpoint even during warmup
        p = self.current_pos_ned()

        # ---- PHASE: WARMUP ----
        if self.phase == 'WARMUP':
            if self.enable_takeoff and p is not None:
                if self.to_target is None:
                    z_to = float(p[2] - self.takeoff_height_m)  # up is negative in NED
                    self.to_target = np.array([float(p[0]), float(p[1]), z_to], dtype=float)
                    self.get_logger().info(
                        f"Warmup takeoff target (ENU): "
                        f"x={ned_to_enu(self.to_target)[0]:.3f}, "
                        f"y={ned_to_enu(self.to_target)[1]:.3f}, "
                        f"z={ned_to_enu(self.to_target)[2]:.3f}"
                    )
                self.publish_traj_position_only(self.to_target)
            elif self.waypoints:
                self.publish_traj_to(self.waypoints[0])

            # Offboard warmup cycles
            if self.offboard_counter < 10:
                self.offboard_counter += 1
                return
            else:
                if not self.mode_set and self.auto_offboard:
                    self.set_offboard_mode()
                if not self.armed and self.auto_arm:
                    self.arm()
                self.phase = 'TAKEOFF'

        # Need odometry beyond warmup
        if p is None:
            return

        # ---- PHASE: TAKEOFF ----
        if self.phase == 'TAKEOFF':
            if self.to_target is None:
                z_to = float(p[2] - self.takeoff_height_m)
                self.to_target = np.array([float(p[0]), float(p[1]), z_to], dtype=float)
                self.get_logger().info(
                    f"Takeoff target (ENU): x={ned_to_enu(self.to_target)[0]:.3f}, "
                    f"y={ned_to_enu(self.to_target)[1]:.3f}, z={ned_to_enu(self.to_target)[2]:.3f}  "
                    f"(hold {self.takeoff_hold_s:.1f}s)"
                )

            self.publish_traj_position_only(self.to_target)

            tgt_enu = ned_to_enu(self.to_target)
            pos_enu = ned_to_enu(p)
            self.get_logger().info(
                f"[TAKEOFF] Target ENU: x={tgt_enu[0]:.3f}, y={tgt_enu[1]:.3f}, z={tgt_enu[2]:.3f} | "
                f"Current ENU: x={pos_enu[0]:.3f}, y={pos_enu[1]:.3f}, z={pos_enu[2]:.3f}"
            )

            err = self.to_target - p
            xy_err = float(np.linalg.norm(err[:2]))
            z_err = abs(float(err[2]))
            inside = (xy_err <= self.xy_accept) and (z_err <= self.z_accept)

            tnow = time.time()
            if inside:
                if self.to_inside_since is None:
                    self.to_inside_since = tnow
                elif (tnow - self.to_inside_since) >= self.takeoff_hold_s:
                    self.after_takeoff_yaw = self.current_yaw_ned()
                    self.get_logger().info(
                        f"Takeoff complete; locking yaw for mission to {self.after_takeoff_yaw:.3f} rad."
                    )
                    self.phase = 'MISSION'
                    self.to_target = None
                    self.to_inside_since = None
            else:
                self.to_inside_since = None

            return

        # ---- PHASE: MISSION ----
        if self.phase == 'MISSION':
            # Mission complete?
            if self.wp_idx >= len(self.waypoints):
                # First time we notice completion: start delay timer
                if self.mission_done_time is None:
                    self.mission_done_time = time.time()
                    if not self.final_hover_sent:
                        self.get_logger().info('Mission complete — holding last setpoint.')
                        self.final_hover_sent = True

                # Keep holding last setpoint during the delay window
                if self.waypoints:
                    self.publish_traj_to(self.waypoints[-1], force_yaw=self.after_takeoff_yaw)

                # After LAND_DELAY_S seconds, initiate landing
                if (not self.landing_initiated) and (time.time() - self.mission_done_time >= self.LAND_DELAY_S):
                    self.get_logger().info("Landing delay elapsed — initiating landing (VEHICLE_CMD_NAV_LAND).")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.landing_initiated = True
                    self.phase = 'LANDING'
                return

            # Not complete yet → track current waypoint
            self.publish_traj_to(self.waypoints[self.wp_idx], force_yaw=self.after_takeoff_yaw)

            tgt = np.array([self.waypoints[self.wp_idx].x,
                            self.waypoints[self.wp_idx].y,
                            self.waypoints[self.wp_idx].z], dtype=float)
            tgt_enu = ned_to_enu(tgt)
            pos_enu = ned_to_enu(p)
            self.get_logger().info(
                f"[WP {self.wp_idx+1}/{len(self.waypoints)}] "
                f"Target ENU: x={tgt_enu[0]:.3f}, y={tgt_enu[1]:.3f}, z={tgt_enu[2]:.3f} | "
                f"Current ENU: x={pos_enu[0]:.3f}, y={pos_enu[1]:.3f}, z={pos_enu[2]:.3f}"
            )

            err = tgt - p
            xy_err = float(np.linalg.norm(err[:2]))
            z_err = abs(float(err[2]))
            inside = (xy_err <= self.xy_accept) and (z_err <= self.z_accept)

            tnow = time.time()
            if inside:
                if self.inside_since is None:
                    self.inside_since = tnow
                elif (tnow - self.inside_since) >= self.hold_time:
                    self.get_logger().info(f"Reached WP {self.wp_idx+1}/{len(self.waypoints)}; advancing.")
                    self.wp_idx += 1
                    self.inside_since = None
            else:
                self.inside_since = None

            return

        # ---- PHASE: LANDING ----
        if self.phase == 'LANDING':
            # We already sent NAV_LAND. PX4 will manage descent in AUTO.LAND.
            # It's okay to keep publishing OffboardControlMode; we don't send more setpoints.
            # (Optional) You could monitor altitude or landed state to disarm here if desired.
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
