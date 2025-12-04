#!/usr/bin/env python3
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

# ===================== USER TRAJECTORY CONFIG (EDIT THESE) =====================
# Choose one: "line", "square", "circle"
TRAJ_MODE = "line"   # "line" | "square" | "circle"
FLIGHT_HEIGHT_M: float = 1.0

# All coordinates below are in ENU: (x_east, y_north, z_up) in meters
# 1) LINE: single target point after takeoff
LINE_P1_ENU: Tuple[float, float, float] = (0.035, 0.037, 0.6)

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


#### NEW CUSTOM PATHS #####
# ZIGZAG: triangular wave path along y-axis with x oscillation
ZIGZAG_AMPLITUDE_M: float = 0.3        # Peak lateral deviation (half the total width)
ZIGZAG_SEGMENT_LENGTH_M: float = 0.5   # Distance between direction changes
ZIGZAG_LENGTH_M: float = 4.0           # Total path length along y-axis
ZIGZAG_NUM_POINTS: int = 40            # Number of waypoints

# SINUSOID: smooth sinusoidal path along y-axis with x oscillation
SINUSOID_AMPLITUDE_M: float = 0.3      # Peak lateral deviation (half the total width)
SINUSOID_WAVELENGTH_M: float = 1.0     # Distance for one complete wave (peak to peak to peak)
SINUSOID_LENGTH_M: float = 4.0         # Total path length along y-axis
SINUSOID_NUM_POINTS: int = 40          # Number of waypoints (more = smoother curve)

# FIGURE8: Lissajous curve figure-8 pattern
FIGURE8_WIDTH_M: float = 1.0           # Total width of figure-8 (x extent)
FIGURE8_LENGTH_M: float = 2.0          # Total length of figure-8 (y extent)
FIGURE8_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU
FIGURE8_NUM_LOOPS: int = 1             # Number of times to trace the figure-8
FIGURE8_NUM_POINTS: int = 60           # Number of waypoints per loop

# RACETRACK: Two parallel straights connected by semicircular turns
RACETRACK_STRAIGHT_LENGTH_M: float = 2.0   # Length of each straight segment
RACETRACK_TURN_RADIUS_M: float = 0.5       # Radius of semicircular turns
RACETRACK_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU
RACETRACK_NUM_POINTS: int = 60             # Total waypoints for full loop

# STAR: 5-pointed star pattern with sharp angular turns
STAR_RADIUS_M: float = 1.0             # Radius from center to star points
STAR_NUM_POINTS: int = 5               # Number of star points (typically 5)
STAR_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU

# STEP: Discrete lateral jumps at regular forward intervals (staircase pattern)
STEP_WIDTH_M: float = 0.3              # Lateral step size
STEP_LENGTH_M: float = 0.5             # Forward distance between steps
STEP_TOTAL_LENGTH_M: float = 4.0       # Total forward distance
STEP_NUM_STEPS: int = 8                # Number of lateral steps

# LAWNMOWER: Back-and-forth sweeping pattern for area coverage
LAWNMOWER_SWEEP_WIDTH_M: float = 2.0   # Width of each sweep (x extent)
LAWNMOWER_SWEEP_LENGTH_M: float = 0.5  # Forward distance between sweeps
LAWNMOWER_NUM_PASSES: int = 6          # Number of back-and-forth passes
LAWNMOWER_START_ENU: Tuple[float, float] = (0.0, 0.0)  # Starting corner (x, y)

# DIAMOND: Rhombus pattern with 45-degree diagonal movements
DIAMOND_WIDTH_M: float = 1.0           # Total width (x extent)
DIAMOND_LENGTH_M: float = 2.0          # Total length (y extent)
DIAMOND_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU

# ELLIPSE: Stretched circle with different major/minor axes
ELLIPSE_MAJOR_AXIS_M: float = 1.5      # Semi-major axis (half of longer dimension)
ELLIPSE_MINOR_AXIS_M: float = 0.75     # Semi-minor axis (half of shorter dimension)
ELLIPSE_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU
ELLIPSE_NUM_POINTS: int = 40           # Number of waypoints

# ROSE: Polar rose curve creating flower-like petal pattern
ROSE_AMPLITUDE_M: float = 1.0          # Maximum radius of rose petals
ROSE_NUM_PETALS: int = 4               # Number of petals (use odd for n petals, even for 2n petals)
ROSE_CENTER_ENU: Tuple[float, float] = (0.0, 1.0)  # Center point (x, y) in ENU
ROSE_NUM_POINTS: int = 80              # Number of waypoints (more = smoother curve)

# RANDOM: Pseudo-random waypoints within a bounded area
RANDOM_BOUND_X_M: float = 1.0          # Half-width of bounding box (x extent from center)
RANDOM_BOUND_Y_M: float = 2.0          # Half-length of bounding box (y extent from center)
RANDOM_CENTER_ENU: Tuple[float, float] = (0.0, 2.0)  # Center of bounding box (x, y) in ENU
RANDOM_NUM_POINTS: int = 15            # Number of random waypoints
RANDOM_SEED: int = 42                  # Random seed for reproducibility

# 5) CUSTOM: custom waypoints used in Peter & Ashir's experiments.


CUSTOM_WAYPOINTS_ENU: Dict[int, List[Tuple[float, float, float]]] = {
    # CUSTOM PATH 0: Straight line (4 meters)
    0: [
        (0.00, 0.00, FLIGHT_HEIGHT_M), # fly up to 1.0m
        (0.00, 0.50, FLIGHT_HEIGHT_M), # fly 0.5m north
        (0.00, 1.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.0m)
        (0.00, 1.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.5m)
        (0.00, 2.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 2.0m)
        (0.00, 2.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 2.5m)
        (0.00, 3.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.0m)
        (0.00, 3.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.5m)
        (0.00, 4.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 4.0m)
    ],
    1: [
        (0.00, 0.00, FLIGHT_HEIGHT_M), # fly up to 1.0m
        (0.00, 0.50, FLIGHT_HEIGHT_M), # fly 0.5m north
        (0.00, 1.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.0m)
        (0.00, 1.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.5m)
        (0.00, 2.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 2.0m)
        (0.00, 2.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 2.5m)
        (0.00, 3.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.0m)
        (0.00, 3.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.5m)
        (0.00, 4.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 4.0m)
        (0.00, 3.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.5m)
        (0.00, 3.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 3.0m)
        (0.00, 2.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 2.0m)
        (0.00, 1.50, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.5m)
        (0.00, 1.00, FLIGHT_HEIGHT_M), # fly 0.5m north (cumulative distance: 1.0m)
        (0.00, 0.50, FLIGHT_HEIGHT_M), # fly 0.5m north
        (0.00, 0.00, FLIGHT_HEIGHT_M), # fly back to 0
    ],
}

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

def generate_custom_zigzag_waypoints() -> List[Waypoint]:
    pts = []
    for i in range(ZIGZAG_NUM_POINTS):
        forward_distance = (i / (ZIGZAG_NUM_POINTS - 1)) * ZIGZAG_LENGTH_M
        # Triangular wave: position within current segment determines lateral offset
        segment_pos = (forward_distance % (2 * ZIGZAG_SEGMENT_LENGTH_M)) / ZIGZAG_SEGMENT_LENGTH_M
        if segment_pos <= 1.0:
            lateral_deviation = ZIGZAG_AMPLITUDE_M * segment_pos
        else:
            lateral_deviation = ZIGZAG_AMPLITUDE_M * (2.0 - segment_pos)
        pts.append(Waypoint.from_enu(lateral_deviation, forward_distance, FLIGHT_HEIGHT_M))
    return pts

def generate_custom_sinusoid_waypoints() -> List[Waypoint]:
    pts = []
    for i in range(SINUSOID_NUM_POINTS):
        forward_distance = (i / (SINUSOID_NUM_POINTS - 1)) * SINUSOID_LENGTH_M
        lateral_deviation = SINUSOID_AMPLITUDE_M * math.sin(2.0 * math.pi * forward_distance / SINUSOID_WAVELENGTH_M)
        pts.append(Waypoint.from_enu(lateral_deviation, forward_distance, FLIGHT_HEIGHT_M))
    return pts

def generate_custom_figure8_waypoints() -> List[Waypoint]:
    """Generate figure-8 (Lissajous curve) waypoints.
    
    Parametric form:
        x = (width/2) * sin(t)
        y = (length/2) * sin(2t)
    where t goes from 0 to 2*pi for one complete figure-8.
    """
    pts = []
    cx, cy = FIGURE8_CENTER_ENU
    total_points = FIGURE8_NUM_LOOPS * FIGURE8_NUM_POINTS
    for i in range(total_points):
        t = (i / total_points) * FIGURE8_NUM_LOOPS * 2.0 * math.pi
        lateral_deviation = (FIGURE8_WIDTH_M / 2.0) * math.sin(t)
        forward_distance = (FIGURE8_LENGTH_M / 2.0) * math.sin(2.0 * t)
        pts.append(Waypoint.from_enu(cx + lateral_deviation, cy + forward_distance, FLIGHT_HEIGHT_M))
    return pts

def generate_racetrack_waypoints() -> List[Waypoint]:
    """Generate racetrack/oval waypoints: two straights connected by semicircular turns."""
    pts = []
    cx, cy = RACETRACK_CENTER_ENU
    r = RACETRACK_TURN_RADIUS_M
    half_straight = RACETRACK_STRAIGHT_LENGTH_M / 2.0
    
    # Calculate points per section (2 straights + 2 semicircles)
    points_per_straight = RACETRACK_NUM_POINTS // 4
    points_per_turn = RACETRACK_NUM_POINTS // 4
    
    # Bottom straight (left to right)
    for i in range(points_per_straight):
        t = i / points_per_straight
        x = -half_straight + t * RACETRACK_STRAIGHT_LENGTH_M
        y = -r
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    # Right semicircle (bottom to top)
    for i in range(points_per_turn):
        t = i / points_per_turn
        angle = -math.pi / 2.0 + t * math.pi
        x = half_straight + r * math.cos(angle)
        y = r * math.sin(angle)
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    # Top straight (right to left)
    for i in range(points_per_straight):
        t = i / points_per_straight
        x = half_straight - t * RACETRACK_STRAIGHT_LENGTH_M
        y = r
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    # Left semicircle (top to bottom)
    for i in range(points_per_turn):
        t = i / points_per_turn
        angle = math.pi / 2.0 + t * math.pi
        x = -half_straight + r * math.cos(angle)
        y = r * math.sin(angle)
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_star_waypoints() -> List[Waypoint]:
    """Generate 5-pointed star waypoints with sharp angular turns."""
    pts = []
    cx, cy = STAR_CENTER_ENU
    n = STAR_NUM_POINTS
    
    # For a 5-pointed star, connect every 2nd point (skip 1)
    # Points are arranged at angles: 0, 72, 144, 216, 288 degrees
    # Star order: 0 -> 144 -> 288 -> 72 -> 216 -> 0
    for i in range(n + 1):  # +1 to close the star
        # Skip every other vertex to create star pattern
        vertex_idx = (i * 2) % n
        angle = math.radians(90 + vertex_idx * (360.0 / n))  # Start from top
        x = STAR_RADIUS_M * math.cos(angle)
        y = STAR_RADIUS_M * math.sin(angle)
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_step_waypoints() -> List[Waypoint]:
    """Generate staircase/step waypoints with discrete lateral jumps."""
    pts = []
    step_length = STEP_TOTAL_LENGTH_M / STEP_NUM_STEPS
    
    for i in range(STEP_NUM_STEPS + 1):
        forward_distance = i * step_length
        # Alternate lateral position: 0, +width, 0, +width, ...
        lateral_deviation = STEP_WIDTH_M if (i % 2 == 1) else 0.0
        
        if i > 0:
            # Add intermediate point at same y but new x (horizontal step)
            pts.append(Waypoint.from_enu(lateral_deviation, forward_distance - step_length, FLIGHT_HEIGHT_M))
        
        pts.append(Waypoint.from_enu(lateral_deviation, forward_distance, FLIGHT_HEIGHT_M))
    
    return pts

def generate_lawnmower_waypoints() -> List[Waypoint]:
    """Generate lawnmower/boustrophedon waypoints for area coverage."""
    pts = []
    sx, sy = LAWNMOWER_START_ENU
    
    for i in range(LAWNMOWER_NUM_PASSES):
        forward_distance = sy + i * LAWNMOWER_SWEEP_LENGTH_M
        
        if i % 2 == 0:
            # Sweep left to right
            pts.append(Waypoint.from_enu(sx, forward_distance, FLIGHT_HEIGHT_M))
            pts.append(Waypoint.from_enu(sx + LAWNMOWER_SWEEP_WIDTH_M, forward_distance, FLIGHT_HEIGHT_M))
        else:
            # Sweep right to left
            pts.append(Waypoint.from_enu(sx + LAWNMOWER_SWEEP_WIDTH_M, forward_distance, FLIGHT_HEIGHT_M))
            pts.append(Waypoint.from_enu(sx, forward_distance, FLIGHT_HEIGHT_M))
    
    return pts

def generate_diamond_waypoints() -> List[Waypoint]:
    """Generate diamond/rhombus waypoints with 45-degree diagonal movements."""
    pts = []
    cx, cy = DIAMOND_CENTER_ENU
    half_width = DIAMOND_WIDTH_M / 2.0
    half_length = DIAMOND_LENGTH_M / 2.0
    
    # Four corners: top, right, bottom, left, back to top
    corners = [
        (cx, cy + half_length),          # Top
        (cx + half_width, cy),           # Right
        (cx, cy - half_length),          # Bottom
        (cx - half_width, cy),           # Left
        (cx, cy + half_length),          # Back to top (close the shape)
    ]
    
    for x, y in corners:
        pts.append(Waypoint.from_enu(x, y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_ellipse_waypoints() -> List[Waypoint]:
    """Generate ellipse waypoints with configurable major/minor axes."""
    pts = []
    cx, cy = ELLIPSE_CENTER_ENU
    
    for i in range(ELLIPSE_NUM_POINTS):
        t = (i / ELLIPSE_NUM_POINTS) * 2.0 * math.pi
        x = ELLIPSE_MINOR_AXIS_M * math.cos(t)  # Minor axis along x
        y = ELLIPSE_MAJOR_AXIS_M * math.sin(t)  # Major axis along y
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_rose_waypoints() -> List[Waypoint]:
    """Generate polar rose curve waypoints creating flower-like petal pattern.
    
    Parametric form (polar):
        r = amplitude * cos(k * theta)
    where k = num_petals for odd petals, k = num_petals/2 for even petals.
    """
    pts = []
    cx, cy = ROSE_CENTER_ENU
    k = ROSE_NUM_PETALS
    
    for i in range(ROSE_NUM_POINTS):
        theta = (i / ROSE_NUM_POINTS) * 2.0 * math.pi
        r = ROSE_AMPLITUDE_M * math.cos(k * theta)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        pts.append(Waypoint.from_enu(cx + x, cy + y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_random_waypoints() -> List[Waypoint]:
    """Generate pseudo-random waypoints within a bounded area."""
    import random
    pts = []
    cx, cy = RANDOM_CENTER_ENU
    
    # Set seed for reproducibility
    random.seed(RANDOM_SEED)
    
    for _ in range(RANDOM_NUM_POINTS):
        x = cx + random.uniform(-RANDOM_BOUND_X_M, RANDOM_BOUND_X_M)
        y = cy + random.uniform(-RANDOM_BOUND_Y_M, RANDOM_BOUND_Y_M)
        pts.append(Waypoint.from_enu(x, y, FLIGHT_HEIGHT_M))
    
    return pts

def generate_custom_waypoints(custom_waypoint_id: int) -> List[Waypoint]:
    return [Waypoint.from_enu(*p) for p in CUSTOM_WAYPOINTS_ENU[custom_waypoint_id]]

def build_hardcoded_waypoints(custom_waypoint_id: int = None) -> List[Waypoint]:
    mode = TRAJ_MODE.lower()
    if mode == "line":
        wps = generate_line_waypoints()
    elif mode == "square":
        wps = generate_square_waypoints()
    elif mode == "circle":
        wps = generate_circle_waypoints()
    elif mode == "zigzag":
        wps = generate_custom_zigzag_waypoints()
    elif mode == "sinusoid":
        wps = generate_custom_sinusoid_waypoints()
    elif mode == "figure8":
        wps = generate_custom_figure8_waypoints()
    elif mode == "racetrack":
        wps = generate_racetrack_waypoints()
    elif mode == "star":
        wps = generate_star_waypoints()
    elif mode == "step":
        wps = generate_step_waypoints()
    elif mode == "lawnmower":
        wps = generate_lawnmower_waypoints()
    elif mode == "diamond":
        wps = generate_diamond_waypoints()
    elif mode == "ellipse":
        wps = generate_ellipse_waypoints()
    elif mode == "rose":
        wps = generate_rose_waypoints()
    elif mode == "random":
        wps = generate_random_waypoints()
    elif mode == "custom":
        wps = generate_custom_waypoints(custom_waypoint_id)
    else:
        raise ValueError(f"Unknown TRAJ_MODE '{TRAJ_MODE}'. Use 'line' | 'square' | 'circle' | 'zigzag' | 'sinusoid' | 'figure8' | 'racetrack' | 'star' | 'step' | 'lawnmower' | 'diamond' | 'ellipse' | 'rose' | 'random' | 'custom'.")
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
        self.LAND_DELAY_S = 10.0  # <-- land 10 s after mission complete

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
