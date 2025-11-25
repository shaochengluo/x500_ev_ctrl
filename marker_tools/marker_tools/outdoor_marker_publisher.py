#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

# ---------- POS/NEG marker times relative to bag start (seconds) ----------
marker_times = [
    21.5, 24.0,
    26.0, 28.0, 32.5,
    33.5, 35.5, 36.5,
    39.5, 41.0, 49.0, 51.0,
    53.5, 54.5
]

marker_signs = [
    'POS', 'NEG',
    'POS', 'POS', 'POS',
    'NEG', 'POS', 'POS',
    'POS', 'POS', 'NEG', 'POS',
    'POS', 'NEG'
]

SIGN_TO_VAL = {'POS': 10, 'NEG': -10}


class OutdoorMarkerPublisher(Node):
    def __init__(self):
        # Node name as shown in `ros2 node list`
        super().__init__('outdoor_marker_publisher')

        assert len(marker_times) == len(marker_signs), \
            "marker_times and marker_signs must have the same length."

        # Numeric topic for rqt_plot
        self.publisher_ = self.create_publisher(Int8, '/marker_pulse', 10)

        # Spike length in seconds (for visualization)
        self.pulse_duration = 0.1

        # Precompute pulses: (t_start, t_end, value)
        self.pulses = []
        for t, s in zip(marker_times, marker_signs):
            v = SIGN_TO_VAL[s]
            self.pulses.append((t, t + self.pulse_duration, v))

        # To print each marker only once when it is "triggered"
        self.triggered = [False] * len(marker_times)

        # Bag start time in ROS clock (None until we see /clock > 0)
        self.bag_start_time = None

        # Timer at 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info(
            f"OutdoorMarkerPublisher initialized with {len(self.pulses)} pulses."
        )

    def timer_callback(self):
        now = self.get_clock().now()

        # Wait until /clock is publishing non-zero time
        if self.bag_start_time is None:
            if now.nanoseconds == 0:
                # No /clock yet
                return

            # First non-zero sim time from /clock -> treat as bag start
            self.bag_start_time = now
            sim_sec = now.nanoseconds / 1e9
            self.get_logger().info(
                f"Bag start synchronized at sim_time={sim_sec:.3f}s. "
                f"Markers are offsets from this time."
            )
            return

        # Time since bag start (seconds)
        elapsed = (now - self.bag_start_time).nanoseconds / 1e9

        # Compute current value for the spike signal
        value = 0
        for t_start, t_end, v in self.pulses:
            if t_start <= elapsed < t_end:
                value = v
                break

        # Publish the numeric marker pulse
        msg = Int8()
        msg.data = int(value)
        self.publisher_.publish(msg)

        # Log each marker once when its start time is reached
        for i, (t_start, _, _) in enumerate(self.pulses):
            if not self.triggered[i] and elapsed >= t_start:
                self.triggered[i] = True
                sign = marker_signs[i]

                sim_sec = now.nanoseconds / 1e9
                self.get_logger().info(
                    f"Triggered marker {i}: "
                    f"sim_time={sim_sec:.3f}s, "
                    f"elapsed={elapsed:.3f}s, "
                    f"offset={t_start:.3f}s, "
                    f"sign={sign}"
                )
                # we keep looping so other markers that become due in this
                # callback also get logged

    @staticmethod
    def main(args=None):
        rclpy.init(args=args)
        node = OutdoorMarkerPublisher()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


def main(args=None):
    OutdoorMarkerPublisher.main(args)
