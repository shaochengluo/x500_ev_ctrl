#!/usr/bin/env python3
"""
ROS 2 Node: Gyro sine-offset monitor

Subscribes to either sensor_msgs/msg/Imu (/imu/data) or px4_msgs/msg/SensorCombined
(/sensor_combined). Maintains a rolling window and fits a sinusoid on a chosen
axis. Operates in phase mode: cuts off (disable) at the next sine peak (top) and
turns on (enable) at the next sine trough (bottom). Actions can be advanced by
separate delays for off and on (`time_delay_off`, `time_delay_on`) to account for
latency. The fit must be stable for
`debounce_count` evaluations, and the window must contain at least
`required_cycles` cycles before acting. After the first cutoff, the last sine
fit is locked and reused for all future peak/trough predictions (no refitting).
Optionally, after `max_periods` sine periods of toggling, the output is left
permanently enabled.

Outputs
- /attack_enable (std_msgs/Bool): True normally, False when cutoff condition met
- /gyro_offset_cutoff (std_msgs/Int32): 1 normally, 0 on cutoff (compat)
- /gyro_debug (std_msgs/String): optional debug info

Key params
- use_px4: bool = True
- imu_topic: str = "/imu/data"
- px4_topic: str = "/sensor_combined"
- window_secs: float = 10.0
- axis: {x,y,z} = "y"
- min_samples: int = 20
- hold_zero_sec: float = 0.0 (hold 0 for this many seconds, then return to 1)
- required_cycles: int = 2
- time_delay_off: float = 0.0 (lead time before peak)
- time_delay_on: float = 0.0 (lead time before trough)
- max_periods: int = 3 (number of sine periods to toggle before staying permanently ON; <0 for unlimited)
- log_on_trigger: bool = False (print console messages on cutoff/enable)
- min_r2: float = 0.2 (set <0 to disable)
- min_amp: float = 0.15 (set <0 to disable)
- min_freq: float = 0.1 (set <0 to disable)
- arm_after_sec: float = 1.0
- debounce_count: int = 1
- eval_stride: int = 5
- qos_reliable: bool = False
- qos_depth: int = 200
- publish_debug: bool = True
"""

from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String, Int32

try:
    from px4_msgs.msg import SensorCombined as Px4SensorCombined
    _HAS_PX4 = True
except Exception:
    _HAS_PX4 = False

try:
    from sensor_msgs.msg import Imu as RosImu
    _HAS_ROS_IMU = True
except Exception:
    _HAS_ROS_IMU = False

# SciPy imports for sine fitting
from scipy.optimize import curve_fit
from scipy.signal import get_window, detrend


def _sine_model(t, A, f, phi, C):
    return A * np.sin(2 * np.pi * f * t + phi) + C


def fit_sine_window(y: np.ndarray, t: np.ndarray, detrend_data: bool = True):
    y = np.asanyarray(y).ravel()
    t = np.asanyarray(t).ravel()
    N = len(y)
    if len(t) != N or N < 10:
        return None

    dt = np.diff(t)
    if np.any(dt <= 0):
        order = np.argsort(t)
        t = t[order]
        y = y[order]
        dt = np.diff(t)
    dt_mean = float(np.mean(dt)) if len(dt) > 0 else 0.0
    if dt_mean <= 0:
        return None

    t_uniform = np.linspace(t[0], t[-1], N)
    y_u = np.interp(t_uniform, t, y)
    y_proc = y_u - np.mean(y_u)
    if detrend_data:
        y_proc = detrend(y_proc)
    if N > 1:
        y_proc *= get_window('hann', N)
    Y = np.fft.rfft(y_proc)
    freqs = np.fft.rfftfreq(N, dt_mean)
    if len(Y) < 2:
        return None
    i_peak = int(np.argmax(np.abs(Y[1:])) + 1)
    f0 = max(1e-3, float(freqs[i_peak]))
    A0 = 2.0 * float(np.abs(Y[i_peak]) / N)
    phi0 = float(np.angle(Y[i_peak]))
    C0 = float(np.mean(y))

    best = None
    best_r2 = -np.inf
    phase_guesses = [phi0, phi0 + np.pi/2, phi0 + np.pi, phi0 + 3*np.pi/2]
    freq_guesses = [f0, f0 * 1.1, f0 * 0.9]
    for fg in freq_guesses:
        for pg in phase_guesses:
            p0 = [A0, fg, pg, C0]
            try:
                bounds = ([0, 1e-3, -2*np.pi, -np.inf], [np.inf, 50.0, 2*np.pi, np.inf])
                (A, f, phi, C), _ = curve_fit(_sine_model, t, y, p0=p0, bounds=bounds, maxfev=3000)
                y_fit = _sine_model(t, A, f, phi, C)
                ss_res = float(np.sum((y - y_fit) ** 2))
                ss_tot = float(np.sum((y - np.mean(y)) ** 2))
                R2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
                if R2 > best_r2:
                    best_r2 = R2
                    best = {"A": float(A), "f": float(f), "phi": float(phi), "C": float(C), "R2": float(R2)}
            except Exception:
                continue
    if best is None:
        y_fit = _sine_model(t, A0, f0, phi0, C0)
        ss_res = float(np.sum((y - y_fit) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        R2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
        best = {"A": float(A0), "f": float(f0), "phi": float(phi0), "C": float(C0), "R2": float(R2)}
    return best


class GyroSineOffsetNode(Node):
    def __init__(self) -> None:
        super().__init__('gyro_sine_offset_monitor')

        # Declare parameters
        self.declare_parameter('use_px4', False)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('px4_topic', '/sensor_combined')
        self.declare_parameter('window_secs', 5.0)
        self.declare_parameter('axis', 'x')
        self.declare_parameter('min_samples', 20)
        self.declare_parameter('hold_zero_sec', 0.0)
        # Phase triggering behavior
        self.declare_parameter('required_cycles', 2)
        self.declare_parameter('time_delay_off', 0.0)
        self.declare_parameter('time_delay_on', 0.0)
        self.declare_parameter('max_periods', 3)
        self.declare_parameter('log_on_trigger', True)
        # Robustness gates (disabled when negative)
        self.declare_parameter('min_r2', 0.5)
        self.declare_parameter('min_amp', 0.05)
        self.declare_parameter('min_freq', 0.1)
        self.declare_parameter('arm_after_sec', 1.0)
        self.declare_parameter('debounce_count', 3)
        # Performance knobs
        self.declare_parameter('eval_stride', 5)
        self.declare_parameter('progress_every', 100)
        self.declare_parameter('qos_reliable', False)
        self.declare_parameter('qos_depth', 200)
        self.declare_parameter('publish_debug', False)

        # Read parameters
        self.use_px4: bool = self.get_parameter('use_px4').get_parameter_value().bool_value
        self.imu_topic: str = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.px4_topic: str = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.window_secs: float = self.get_parameter('window_secs').get_parameter_value().double_value
        self.axis: str = self.get_parameter('axis').get_parameter_value().string_value
        self.min_samples: int = int(self.get_parameter('min_samples').get_parameter_value().integer_value)
        self.hold_zero_sec: float = self.get_parameter('hold_zero_sec').get_parameter_value().double_value
        self.required_cycles: int = int(self.get_parameter('required_cycles').get_parameter_value().integer_value)
        self.time_delay_off: float = float(self.get_parameter('time_delay_off').get_parameter_value().double_value)
        self.time_delay_on: float = float(self.get_parameter('time_delay_on').get_parameter_value().double_value)
        self.max_periods: int = int(self.get_parameter('max_periods').get_parameter_value().integer_value)
        self.log_on_trigger: bool = self.get_parameter('log_on_trigger').get_parameter_value().bool_value
        # Gates / arming / debounce
        _min_r2 = self.get_parameter('min_r2').get_parameter_value().double_value
        _min_amp = self.get_parameter('min_amp').get_parameter_value().double_value
        _min_freq = self.get_parameter('min_freq').get_parameter_value().double_value
        self.min_r2 = None if _min_r2 < 0 else float(_min_r2)
        self.min_amp = None if _min_amp < 0 else float(_min_amp)
        self.min_freq = None if _min_freq < 0 else float(_min_freq)
        self.arm_after_sec: float = self.get_parameter('arm_after_sec').get_parameter_value().double_value
        self.debounce_count: int = int(self.get_parameter('debounce_count').get_parameter_value().integer_value)
        # Performance
        self.eval_stride: int = max(1, int(self.get_parameter('eval_stride').get_parameter_value().integer_value))
        self.progress_every: int = int(self.get_parameter('progress_every').get_parameter_value().integer_value)
        self.publish_debug: bool = self.get_parameter('publish_debug').get_parameter_value().bool_value

        qos_reliable: bool = self.get_parameter('qos_reliable').get_parameter_value().bool_value
        qos_depth: int = int(self.get_parameter('qos_depth').get_parameter_value().integer_value)

        # Validate message availability
        if self.use_px4 and not _HAS_PX4:
            self.get_logger().warn('use_px4=True but px4_msgs not found. Falling back to sensor_msgs/Imu.')
            self.use_px4 = False
        if not self.use_px4 and not _HAS_ROS_IMU:
            raise RuntimeError('sensor_msgs/Imu not available. Install ros-<distro>-sensor-msgs or enable px4.')

        # QoS for sensor streams
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        if self.use_px4:
            self.sub = self.create_subscription(Px4SensorCombined, self.px4_topic, self._cb_px4, qos)
            self.get_logger().info(f'Subscribed to PX4 topic: {self.px4_topic}')
        else:
            self.sub = self.create_subscription(RosImu, self.imu_topic, self._cb_imu, qos)
            self.get_logger().info(f'Subscribed to IMU topic: {self.imu_topic}')

        # Publishers
        self.attack_pub = self.create_publisher(Bool, '/attack_enable', 10)
        self.cutoff_pub = self.create_publisher(Int32, '/gyro_offset_cutoff', 10)
        self.debug_pub = self.create_publisher(String, '/gyro_debug', 10) if self.publish_debug else None

        # Rolling buffer and history
        self.buf: Deque[Tuple[float, float, float, float]] = deque()  # (t, x, y, z)
        self.cutoff_latched: bool = False
        self.hold_until_time: Optional[float] = None  # when to release timed hold (if used)
        # Timing / stride / debounce state
        self._first_stamp: Optional[float] = None
        self._sample_counter: int = 0
        self._eval_counter: int = 0

        # Phase-mode state
        self._phase_fit_hits: int = 0  # consecutive good-fit evaluations
        self._prev_dt_peak: Optional[float] = None
        self._prev_dt_trough: Optional[float] = None
        self._phase_state_disabled: bool = False  # False=enabled, True=disabled
        self._warned_phase_axis: bool = False
        # Fit lock + cycle counting
        self._fit_locked: bool = False
        self._lock_f: Optional[float] = None
        self._lock_phi_abs: Optional[float] = None
        self._cycles_done: int = 0
        self._permanent_on: bool = False
        # Deterministic scheduler after fit lock
        self._T_lock: Optional[float] = None
        self._next_peak_time: Optional[float] = None
        self._next_trough_time: Optional[float] = None
        self._next_cutoff_time: Optional[float] = None
        self._next_enable_time: Optional[float] = None

        # Publish initial state (enabled)
        self.attack_pub.publish(Bool(data=True))
        self.cutoff_pub.publish(Int32(data=1))

        self.get_logger().info(
            'GyroSineOffsetNode started. '
            f'window={self.window_secs}s axis={self.axis} '
            f'hold_zero_sec={self.hold_zero_sec} min_samples={self.min_samples} '
            f'required_cycles={self.required_cycles} '
            f'time_delay_off={self.time_delay_off} time_delay_on={self.time_delay_on} '
            f'max_periods={self.max_periods} log_on_trigger={self.log_on_trigger}'
        )

    # ---------- Helpers ----------
    def _trim(self, t_now: float) -> None:
        cutoff = t_now - self.window_secs
        while self.buf and self.buf[0][0] < cutoff:
            self.buf.popleft()

    def _evaluate(self, t_now: float) -> None:
        n = len(self.buf)
        if n < self.min_samples:
            return
        # Build arrays
        t_arr = np.array([t for (t, _, _, _) in self.buf], dtype=float)
        t_rel = t_arr - t_arr[0]
        g = np.array([[x, y, z] for (_, x, y, z) in self.buf], dtype=float)
        dt_sample = float(np.mean(np.diff(t_arr))) if len(t_arr) > 1 else 0.0

        # If we are permanently ON, enforce ON and skip everything
        if self._permanent_on:
            if self._phase_state_disabled:
                self._phase_state_disabled = False
            self.attack_pub.publish(Bool(data=True))
            self.cutoff_pub.publish(Int32(data=1))
            return

        # Select axis for phase mode
        sel_axis = self.axis.lower()
        if sel_axis not in ('x', 'y', 'z'):
            if not self._warned_phase_axis:
                self.get_logger().warn(
                    f"axis must be one of x,y,z; got '{self.axis}'. Using 'y'."
                )
                self._warned_phase_axis = True
            sel_axis = 'y'
        axis_idx = {'x': 0, 'y': 1, 'z': 2}[sel_axis]

        two_pi = float(2.0 * np.pi)

        # If fit already locked, use locked parameters and deterministic schedule
        if self._fit_locked and (self._lock_f is not None) and (self._lock_phi_abs is not None):
            fhat = float(self._lock_f)
            phihat_abs = float(self._lock_phi_abs)
            if fhat <= 1e-9:
                return
            # Initialize schedule if needed, based purely on locked (f, phi)
            if self._T_lock is None:
                self._T_lock = 1.0 / fhat
            def pos_mod(x, m):
                return (x % m + m) % m
            theta_now = two_pi * fhat * t_now + phihat_abs
            # Compute next peak/trough absolute times if not set yet
            if (self._next_peak_time is None) or (self._next_trough_time is None):
                dtheta_peak = pos_mod((0.5 * np.pi) - theta_now, two_pi)
                dtheta_trough = pos_mod((1.5 * np.pi) - theta_now, two_pi)
                dt_peak = float(dtheta_peak / (two_pi * fhat))
                dt_trough = float(dtheta_trough / (two_pi * fhat))
                self._next_peak_time = t_now + dt_peak
                self._next_trough_time = t_now + dt_trough
                self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)
                self._next_enable_time = self._next_trough_time - float(self.time_delay_on)
            # For debug values, compute current dt to scheduled actions (optional)
            dt_peak = float(max(0.0, (self._next_cutoff_time or (t_now)) - t_now))
            dt_trough = float(max(0.0, (self._next_enable_time or (t_now)) - t_now))
        else:
            # Fit only until first cutoff (lock time)
            res = fit_sine_window(g[:, axis_idx], t_rel)
            if res is None:
                return
            Ahat = float(res['A'])
            fhat = float(res['f'])
            phihat = float(res['phi'])
            R2hat = float(res['R2'])

            # Gating on quality and cycles
            ok_r2 = (self.min_r2 is None) or (R2hat >= self.min_r2)
            ok_amp = (self.min_amp is None) or (Ahat >= self.min_amp)
            ok_f = (self.min_freq is None) or (fhat >= self.min_freq)
            cycles_in_window = fhat * float(self.window_secs)
            fit_good = ok_r2 and ok_amp and ok_f and (cycles_in_window >= float(self.required_cycles))

            # Debounce on fit quality
            if fit_good:
                self._phase_fit_hits += 1
            else:
                self._phase_fit_hits = 0
                self._prev_dt_peak = None
                self._prev_dt_trough = None
                return

            # Arm time
            if (self._first_stamp is not None) and ((t_now - self._first_stamp) < float(self.arm_after_sec)):
                return
            if self._phase_fit_hits < max(1, int(self.debounce_count)):
                return

            # Phase calculations using relative time base
            def pos_mod(x, m):
                return (x % m + m) % m
            theta_now = two_pi * fhat * t_rel[-1] + phihat
            dtheta_peak = pos_mod((0.5 * np.pi) - theta_now, two_pi)
            dtheta_trough = pos_mod((1.5 * np.pi) - theta_now, two_pi)
            dt_peak = float(dtheta_peak / (two_pi * fhat))
            dt_trough = float(dtheta_trough / (two_pi * fhat))

        if self._fit_locked:
            # Deterministic schedule: trigger exactly at scheduled times
            entered_peak_window = (self._next_cutoff_time is not None) and (t_now >= self._next_cutoff_time)
            entered_trough_window = (self._next_enable_time is not None) and (t_now >= self._next_enable_time)
        else:
            # Pre-lock detection based on phase window crossing
            thr_off = self.time_delay_off if self.time_delay_off > 0.0 else max(1.5 * dt_sample, 1e-3)
            thr_on = self.time_delay_on if self.time_delay_on > 0.0 else max(1.5 * dt_sample, 1e-3)
            entered_peak_window = (
                (self._prev_dt_peak is not None) and (self._prev_dt_peak > thr_off) and (dt_peak <= thr_off)
            )
            entered_trough_window = (
                (self._prev_dt_trough is not None) and (self._prev_dt_trough > thr_on) and (dt_trough <= thr_on)
            )
            # Update prev dt trackers for next evaluation
            self._prev_dt_peak = dt_peak
            self._prev_dt_trough = dt_trough

        # Timed hold
        if (self.hold_zero_sec is not None) and (self.hold_zero_sec > 0.0):
            if self.hold_until_time is not None and t_now < self.hold_until_time:
                if not self.cutoff_latched:
                    self.cutoff_latched = True
                self._phase_state_disabled = True
                self.attack_pub.publish(Bool(data=False))
                self.cutoff_pub.publish(Int32(data=0))
                if self.publish_debug:
                    cycles = float(self.window_secs) * float(fhat)
                    dbg = (
                        f"phase hold t={t_now:.3f} dt_peak={dt_peak:.3f} dt_trough={dt_trough:.3f} "
                        f"fit_hits={self._phase_fit_hits} cycles={cycles:.2f}"
                    )
                    self.debug_pub and self.debug_pub.publish(String(data=dbg))
                return
            if self.hold_until_time is not None and t_now >= self.hold_until_time:
                self.hold_until_time = None
                self.cutoff_latched = False

        action_taken = False
        # Cut off at peak (early by time_delay_off or sampling epsilon)
        if entered_peak_window and not self._phase_state_disabled and not self._permanent_on:
            self._phase_state_disabled = True
            self.attack_pub.publish(Bool(data=False))
            self.cutoff_pub.publish(Int32(data=0))
            self.cutoff_latched = True
            action_taken = True
            if self.log_on_trigger:
                self.get_logger().warn(
                    f"Cutoff @peak t_data={t_now:.3f}s (axis={sel_axis}) "
                    f"dt_peak={dt_peak:.3f}s delay_off={self.time_delay_off:.3f}s"
                )
            # If this is the first cutoff, lock the fit for future predictions (absolute phase)
            if not self._fit_locked:
                # Compute absolute phase offset from the last fit
                # phihat was estimated with t = t_rel = t_abs - t_arr[0] -> phi_abs = phi - 2π f * t_arr[0]
                phi_abs = (phihat - two_pi * fhat * t_arr[0]) if 'phihat' in locals() else phihat_abs
                self._fit_locked = True
                self._lock_f = fhat
                self._lock_phi_abs = float(phi_abs)
                if self.publish_debug:
                    self.get_logger().info(
                        f'Locked sine fit @ t_data={t_now:.3f}s: '
                        f'f={self._lock_f:.4f} phi_abs={self._lock_phi_abs:.3f}'
                    )
            if (self.hold_zero_sec is not None) and (self.hold_zero_sec > 0.0):
                self.hold_until_time = t_now + float(self.hold_zero_sec)
            if self.publish_debug:
                self._publish_phase_debug(t_now, sel_axis, fhat, (phihat if 'phihat' in locals() else self._lock_phi_abs), dt_peak, dt_trough, 'cutoff@peak')
            # Advance cutoff schedule to the next peak time (keep strictly after now)
            if self._fit_locked and (self._T_lock is not None):
                if self._next_peak_time is None:
                    self._next_peak_time = t_now + self._T_lock
                else:
                    while self._next_peak_time - float(self.time_delay_off) <= t_now:
                        self._next_peak_time += self._T_lock
                self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)

        # Turn on at trough (early by time_delay_on or sampling epsilon) if not in timed hold
        if (self.hold_until_time is None) and entered_trough_window and self._phase_state_disabled:
            self._phase_state_disabled = False
            self.attack_pub.publish(Bool(data=True))
            self.cutoff_pub.publish(Int32(data=1))
            self.cutoff_latched = False
            action_taken = True
            if self.log_on_trigger:
                self.get_logger().info(
                    f"Enable @trough t_data={t_now:.3f}s (axis={sel_axis}) "
                    f"dt_trough={dt_trough:.3f}s delay_on={self.time_delay_on:.3f}s"
                )
            # Count completed periods on trough enables
            if self._fit_locked and (self.max_periods is not None) and (self.max_periods > 0):
                self._cycles_done += 1
                if self._cycles_done >= int(self.max_periods):
                    # Permanently ON from now on
                    self._permanent_on = True
                    self.hold_until_time = None
                    if self.publish_debug:
                        self.get_logger().info(
                            f'Reached max_periods={self.max_periods} @ t_data={t_now:.3f}s. '
                            f'Permanently enabling output.'
                        )
            if self.publish_debug:
                self._publish_phase_debug(t_now, sel_axis, fhat, (phihat if 'phihat' in locals() else self._lock_phi_abs), dt_peak, dt_trough, 'enable@trough')
            # Advance enable schedule to the next trough time (keep strictly after now)
            if self._fit_locked and (self._T_lock is not None):
                if self._next_trough_time is None:
                    self._next_trough_time = t_now + 0.5 * self._T_lock
                else:
                    while self._next_trough_time - float(self.time_delay_on) <= t_now:
                        self._next_trough_time += self._T_lock
                self._next_enable_time = self._next_trough_time - float(self.time_delay_on)

        # Progress logging
        self._eval_counter += 1
        if self.progress_every and (self._eval_counter % int(self.progress_every) == 0):
            t_rel_global = (t_now - (self._first_stamp if self._first_stamp is not None else t_arr[0]))
            self.get_logger().info(f"phase eval {self._eval_counter} @ t={t_rel_global:.2f}s")

        # If no action, still optionally publish debug
        if (not action_taken) and self.publish_debug:
            self._publish_phase_debug(t_now, sel_axis, fhat, phihat, dt_peak, dt_trough, 'idle')
        return

        

        # Debounce accumulation
        

    def _publish_phase_debug(self, t_now: float, axis: str, fhat: float, phihat: float,
                              dt_peak: float, dt_trough: float, state: str) -> None:
        if self.debug_pub is None:
            return
        line = (
            f"t={t_now:.3f} mode=phase axis={axis} f={fhat:.4f} phi={phihat:.3f} "
            f"dt_peak={dt_peak:.3f}s dt_trough={dt_trough:.3f}s "
            f"delay_off={self.time_delay_off:.3f}s delay_on={self.time_delay_on:.3f}s "
            f"fit_hits={self._phase_fit_hits} state={state} disabled={self._phase_state_disabled}"
        )
        self.debug_pub.publish(String(data=line))

    # ---------- Callbacks ----------
    def _push(self, t_sec: float, wx: float, wy: float, wz: float) -> None:
        if self._first_stamp is None:
            self._first_stamp = t_sec
        self.buf.append((t_sec, wx, wy, wz))
        self._trim(t_sec)
        self._sample_counter += 1
        if (self._sample_counter % int(self.eval_stride)) != 0:
            return
        self._evaluate(t_sec)

    def _cb_px4(self, msg: 'Px4SensorCombined') -> None:
        # PX4 timestamp commonly in microseconds
        t_sec = float(msg.timestamp) * 1e-6
        wx, wy, wz = float(msg.gyro_rad[0]), float(msg.gyro_rad[1]), float(msg.gyro_rad[2])
        self._push(t_sec, wx, wy, wz)

    def _cb_imu(self, msg: 'RosImu') -> None:
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wx, wy, wz = float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z)
        self._push(t_sec, wx, wy, wz)


def main() -> None:
    rclpy.init()
    node = GyroSineOffsetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
