#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial


class ArduinoGateNode(Node):
    def __init__(self):
        super().__init__('arduino_gate_node')

        # ----------------- Parameters -----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # ----------------- Open Serial Once -----------------
        try:
            self.ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0.01,        # short timeouts for low latency
                write_timeout=0.01
            )
            self.get_logger().info(f'Opened serial {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Cannot open serial {port}: {e}')
            self.ser = None

        # ----------------- Subscriber -----------------
        # /gyro_offset_cutoff: Int32, 1 = ON, 0 = OFF
        self.sub = self.create_subscription(
            Int32,
            '/gyro_offset_cutoff',
            self.cmd_cb,
            10
        )

    def cmd_cb(self, msg: Int32):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error('Serial not open; cannot send command.')
            return

        val = int(msg.data)

        if val == 1:
            byte = b'1'   # turn Arduino gate ON
        elif val == 0:
            byte = b'0'   # turn Arduino gate OFF
        else:
            # Ignore unexpected values but log them
            self.get_logger().warn(
                f'Ignoring invalid /gyro_offset_cutoff value={val}, expected 0 or 1.'
            )
            return

        try:
            self.ser.write(byte)
            # no flush needed; OS will push quickly
            self.get_logger().debug(f'Sent byte {byte!r} to Arduino')
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoGateNode()
    try:
        rclpy.spin(node)
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
