#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped

def quat_to_rot(q):
    w,x,y,z = q
    xx=x*x; yy=y*y; zz=z*z
    wx=w*x; wy=w*y; wz=w*z
    xy=x*y; xz=x*z; yz=y*z
    return [
        [1-2*(yy+zz),   2*(xy-wz),     2*(xz+wy)],
        [  2*(xy+wz), 1-2*(xx+zz),     2*(yz-wx)],
        [  2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)],
    ]

def rot_to_quat(R):
    m00,m01,m02 = R[0]
    m10,m11,m12 = R[1]
    m20,m21,m22 = R[2]
    tr = m00+m11+m22
    if tr>0:
        S = math.sqrt(tr+1.0)*2.0
        w = 0.25*S
        x = (m21-m12)/S
        y = (m02-m20)/S
        z = (m10-m01)/S
    elif m00>m11 and m00>m22:
        S = math.sqrt(1.0+m00-m11-m22)*2.0
        w = (m21-m12)/S
        x = 0.25*S
        y = (m01+m10)/S
        z = (m02+m20)/S
    elif m11>m22:
        S = math.sqrt(1.0+m11-m00-m22)*2.0
        w = (m02-m20)/S
        x = (m01+m10)/S
        y = 0.25*S
        z = (m12+m21)/S
    else:
        S = math.sqrt(1.0+m22-m00-m11)*2.0
        w = (m10-m01)/S
        x = (m02+m20)/S
        y = (m12+m21)/S
        z = 0.25*S
    n = math.sqrt(w*w+x*x+y*y+z*z)
    return [w/n, x/n, y/n, z/n]

def mat_T(A): return [[A[j][i] for j in range(3)] for i in range(3)]
def mat_mul(A,B): return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def euler_zyx_from_R(R):
    # world->body, ENU/FLU; intrinsic ZYX
    roll  = math.atan2(R[2][1], R[2][2])    # about body X
    pitch = math.asin(-R[2][0])             # about body Y
    yaw   = math.atan2(R[1][0], R[0][0])    # about world Z
    return roll, pitch, yaw

class VOdometryToPoseENU(Node):
    def __init__(self):
        super().__init__('vehicle_odometry_to_pose_enu')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Change-of-basis: ENU->NED (right-multiply), FRD->FLU (left-multiply)
        self.C_W = [[0,1,0],[1,0,0],[0,0,-1]]       # world
        self.C_B = [[1,0,0],[0,-1,0],[0,0,-1]]      # body

        self.pub = self.create_publisher(PoseStamped, '/local_position/pose', qos)
        self.sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.cb, qos)

    def cb(self, msg: VehicleOdometry):
        # Position NED -> ENU
        x_n, y_e, z_d = msg.position
        x_e, y_n, z_u = y_e, x_n, -z_d

        # Orientation: PX4 gives body->world (FRD->NED). We need world->body (ENU->FLU).
        q_b2w_ned = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        R_B2W_NED = quat_to_rot(q_b2w_ned)
        R_W2B_NED = mat_T(R_B2W_NED)                    # invert
        R_W2B_ENU = mat_mul(self.C_B, mat_mul(R_W2B_NED, self.C_W))
        q_ros = rot_to_quat(R_W2B_ENU)

        # OPTIONAL: print ZYX Euler for debugging (degrees)
        # r,p,y = euler_zyx_from_R(R_W2B_ENU)
        # self.get_logger().info(f"roll={math.degrees(r):.1f}, pitch={math.degrees(p):.1f}, yaw={math.degrees(y):.1f}")

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x_e), float(y_n), float(z_u)
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = map(float, q_ros)
        self.pub.publish(ps)

def main():
    rclpy.init()
    rclpy.spin(VOdometryToPoseENU())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
