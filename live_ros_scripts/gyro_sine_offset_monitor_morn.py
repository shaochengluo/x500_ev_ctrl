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

Key params
- use_px4: bool = True
- imu_topic: str = "/imu/data"
- px4_topic: str = "/sensor_combined"
- window_secs: float = 10.0
- axis: {x,y,z} = "y"
- min_samples: int = 20
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
- qos_reliable: bool = False
- qos_depth: int = 200
"""

from collections import deque
from typing import Deque, Dict, Optional, Tuple

import math

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
        # Phase triggering behavior
        self.declare_parameter('required_cycles', 2)
        self.declare_parameter('time_delay_off', 0.0)
        self.declare_parameter('time_delay_on', 0.0)
        self.declare_parameter('max_periods', 6)
        self.declare_parameter('log_on_trigger', True)
        # Linear drift configuration (post-lock, no more sine fits)
        self.declare_parameter('enable_linear_drift', False)
        self.declare_parameter('linear_drift_window_sec', 5.0)
        self.declare_parameter('max_drift_hz_per_s', 0.1)
        # Robustness gates (disabled when negative)
        self.declare_parameter('min_r2', 0.5)
        self.declare_parameter('min_amp', 0.05)
        self.declare_parameter('min_freq', 0.1)
        self.declare_parameter('arm_after_sec', 0.0)
        self.declare_parameter('debounce_count', 3)
        # Lock-on-confirm (no firing) parameters
        self.declare_parameter('lock_on_confirm', True)
        self.declare_parameter('lock_min_margin_ms', 80.0)
        # One-shot post-lock prefire for the first scheduled cutoff (milliseconds)
        self.declare_parameter('postlock_first_prefire_ms', 0)  # set 0 to disable
        # (pre-lock prefire removed)
        # Performance knobs
        self.declare_parameter('eval_stride_prelock', 50)
        self.declare_parameter('eval_stride_postlock', 1)
        self.declare_parameter('progress_every', 100)
        self.declare_parameter('qos_reliable', False)
        self.declare_parameter('qos_depth', 200)
        self.declare_parameter('publish_schedule_debug', True)

        # Read parameters
        self.use_px4: bool = self.get_parameter('use_px4').get_parameter_value().bool_value
        self.imu_topic: str = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.px4_topic: str = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.window_secs: float = self.get_parameter('window_secs').get_parameter_value().double_value
        self.axis: str = self.get_parameter('axis').get_parameter_value().string_value
        self.min_samples: int = int(self.get_parameter('min_samples').get_parameter_value().integer_value)
        self.required_cycles: int = int(self.get_parameter('required_cycles').get_parameter_value().integer_value)
        self.time_delay_off: float = float(self.get_parameter('time_delay_off').get_parameter_value().double_value)
        self.time_delay_on: float = float(self.get_parameter('time_delay_on').get_parameter_value().double_value)
        self.max_periods: int = int(self.get_parameter('max_periods').get_parameter_value().integer_value)
        self.log_on_trigger: bool = self.get_parameter('log_on_trigger').get_parameter_value().bool_value
        self.enable_linear_drift: bool = self.get_parameter('enable_linear_drift').get_parameter_value().bool_value
        self.linear_drift_window_sec: float = self.get_parameter('linear_drift_window_sec').get_parameter_value().double_value
        self.max_drift_hz_per_s: float = self.get_parameter('max_drift_hz_per_s').get_parameter_value().double_value
        # Gates / arming / debounce
        _min_r2 = self.get_parameter('min_r2').get_parameter_value().double_value
        _min_amp = self.get_parameter('min_amp').get_parameter_value().double_value
        _min_freq = self.get_parameter('min_freq').get_parameter_value().double_value
        self.min_r2 = None if _min_r2 < 0 else float(_min_r2)
        self.min_amp = None if _min_amp < 0 else float(_min_amp)
        self.min_freq = None if _min_freq < 0 else float(_min_freq)
        self.arm_after_sec: float = self.get_parameter('arm_after_sec').get_parameter_value().double_value
        self.debounce_count: int = int(self.get_parameter('debounce_count').get_parameter_value().integer_value)
        # (pre-lock prefire params removed)
        # Performance
        prelock = int(self.get_parameter('eval_stride_prelock').get_parameter_value().integer_value)
        postlock = int(self.get_parameter('eval_stride_postlock').get_parameter_value().integer_value)
        self.eval_stride_prelock: int = max(1, prelock)
        self.eval_stride_postlock: int = max(1, postlock)
        self.progress_every: int = int(self.get_parameter('progress_every').get_parameter_value().integer_value)
        self.publish_schedule_debug: bool = self.get_parameter('publish_schedule_debug').get_parameter_value().bool_value
        # Lock-on-confirm knobs
        self.lock_on_confirm: bool = self.get_parameter('lock_on_confirm').get_parameter_value().bool_value
        self.lock_min_margin_s: float = float(self.get_parameter('lock_min_margin_ms').get_parameter_value().double_value) * 1e-3
        # Post-lock first-cutoff prefire
        self._postlock_first_prefire_s: float = float(
            self.get_parameter('postlock_first_prefire_ms').get_parameter_value().double_value
        ) * 1e-3
        self._postlock_first_prefire_used: bool = False

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
        self.sched_pub = self.create_publisher(String, '/gyro_schedule_debug', 10) if self.publish_schedule_debug else None

        # Rolling buffer and history
        self.buf: Deque[Tuple[float, float, float, float]] = deque()  # (t, x, y, z)
        self.cutoff_latched: bool = False
        
        # Timing / stride / debounce state
        self._first_stamp: Optional[float] = None
        self._first_ros_time: Optional[float] = None
        self._last_action_ros_time: Optional[float] = None
        self._last_action_name: Optional[str] = None
        self._sample_counter: int = 0
        self._eval_counter: int = 0
        self._obs: Dict[str, int] = {"coalesced": 0, "overdue_c": 0, "overdue_e": 0}
        # (pre-lock prefire bookkeeping removed)

        # Phase-mode state
        self._phase_fit_hits: int = 0  # consecutive good-fit evaluations
        self._prev_dt_peak: Optional[float] = None
        self._prev_dt_trough: Optional[float] = None
        self._phase_state_disabled: bool = False  # False=enabled, True=disabled
        self._warned_phase_axis: bool = False
        # Fit lock + cycle counting
        self._fit_locked: bool = False
        self._lock_established: bool = False  # true once we've locked without firing
        self._lock_f: Optional[float] = None
        self._lock_phi_abs: Optional[float] = None
        self._t_lock: Optional[float] = None
        # Pre-lock frequency history to estimate drift
        self._freq_hist: Deque[Tuple[float, float]] = deque()
        self._f_drift: float = 0.0  # Hz/s (linear drift)
        self._cycles_done: int = 0
        self._permanent_on: bool = False
        # Deterministic scheduler after fit lock
        self._T_lock: Optional[float] = None
        self._next_peak_time: Optional[float] = None
        self._next_trough_time: Optional[float] = None
        self._next_cutoff_time: Optional[float] = None
        self._next_enable_time: Optional[float] = None

        # --- Arrival spacing diagnostics (observe-only) ---
        self._last_ros_arrival = None
        self._last_t_data = None
        self._burst_counter = 0

        # Only log arrivals between a cutoff and the next enable
        self._log_arrivals = False        # False until first cutoff fires
        self._arr_samples = 0
        self._sum_d_ros = 0.0
        self._sum_d_data = 0.0
        self._burst_max_ratio = 0.0
        # Arrival-window mode: "off" | "lock_to_cutoff" | "cutoff_to_enable"
        self._arr_window_mode = "off"

        # (pre-lock bias bookkeeping removed)

        # Publish initial state (enabled)
        self.attack_pub.publish(Bool(data=True))
        self.cutoff_pub.publish(Int32(data=1))
        # Optional log for prefire setting
        try:
            self.get_logger().info(
                f"Post-lock first-cutoff prefire={self._postlock_first_prefire_s*1e3:.0f} ms"
            )
        except Exception:
            pass

        self.get_logger().info(
            'GyroSineOffsetNode started. '
            f'window={self.window_secs}s axis={self.axis} '
            f'min_samples={self.min_samples} '
            f'required_cycles={self.required_cycles} '
            f'time_delay_off={self.time_delay_off} time_delay_on={self.time_delay_on} '
            f'max_periods={self.max_periods} log_on_trigger={self.log_on_trigger} '
            f'linear_drift=({self.enable_linear_drift}, window={self.linear_drift_window_sec}s, max_slope={self.max_drift_hz_per_s}Hz/s)'
        )
        try:
            self.get_logger().info(
                f"Stride(pre-lock={self.eval_stride_prelock}, post-lock={self.eval_stride_postlock})"
            )
        except Exception:
            pass
        # (pre-lock prefire startup log removed)
        # (pre-lock bias startup log removed)

    # ---------- Helpers ----------
    def _trim(self, t_now: float) -> None:
        cutoff = t_now - self.window_secs
        while self.buf and self.buf[0][0] < cutoff:
            self.buf.popleft()

    def _establish_lock_and_schedule(self, t_now: float, fhat: float, phihat: float, t0_of_fit: float) -> None:
        """Freeze f and absolute phase, then pre-compute the next trough/peak without firing."""
        two_pi = 2.0 * np.pi
        # Absolute phase: the fit used t_rel = t_abs - t0_of_fit
        phi_abs = phihat - two_pi * fhat * t0_of_fit

        self._fit_locked = True
        self._lock_f = max(fhat, 1e-9)
        self._lock_phi_abs = float(phi_abs)
        self._t_lock = float(t_now)
        self._T_lock = 1.0 / self._lock_f
        # Optionally keep your drift estimate here if you want

        # Phase now
        theta_now = two_pi * self._lock_f * t_now + self._lock_phi_abs

        # cycles to upcoming anchors
        def pos_mod(x, m):
            return (x % m + m) % m
        cycles_to_peak = pos_mod((0.5 * np.pi) - theta_now, two_pi) / two_pi
        cycles_to_trough = pos_mod((1.5 * np.pi) - theta_now, two_pi) / two_pi

        # Time deltas (no drift case; keep your drift solver if enabled)
        dt_peak = cycles_to_peak / self._lock_f
        dt_trough = cycles_to_trough / self._lock_f

        # Absolute times
        self._next_peak_time = t_now + dt_peak
        self._next_trough_time = t_now + dt_trough

        # Action times with delays
        self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)
        self._next_enable_time = self._next_trough_time - float(self.time_delay_on)

        # Ensure both are strictly in the future
        if self._next_cutoff_time <= t_now:
            self._next_peak_time += self._T_lock
            self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)
        if self._next_enable_time <= t_now:
            self._next_trough_time += self._T_lock
            self._next_enable_time = self._next_trough_time - float(self.time_delay_on)

        self._lock_established = True
        try:
            self.get_logger().info(
                "LOCK established without firing: f=%.6fHz T=%.3fms first_cutoff@%.6f first_enable@%.6f (t_now=%.6f)" %
                (self._lock_f, self._T_lock * 1e3, self._next_cutoff_time, self._next_enable_time, t_now)
            )
        except Exception:
            pass

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
            if self._T_lock is None:
                self._T_lock = 1.0 / fhat
            def pos_mod(x, m):
                return (x % m + m) % m
            # Phase now with optional linear drift
            if self.enable_linear_drift and (self._t_lock is not None):
                dt_from_lock = float(t_now - self._t_lock)
                theta_now = phihat_abs + two_pi * (fhat * dt_from_lock + 0.5 * self._f_drift * dt_from_lock * dt_from_lock)
            else:
                theta_now = two_pi * fhat * t_now + phihat_abs
            # Compute next anchors if not set yet
            if (self._next_peak_time is None) or (self._next_trough_time is None):
                # cycles to next target from current phase
                cycles_to_peak = float(pos_mod((0.5 * np.pi) - theta_now, two_pi) / two_pi)
                cycles_to_trough = float(pos_mod((1.5 * np.pi) - theta_now, two_pi) / two_pi)
                # Solve for time to accumulate given cycles with linear drift: a*t^2 + b*t - c = 0
                def solve_dt_from_cycles(t_start: float, cycles: float) -> float:
                    if self.enable_linear_drift and (self._t_lock is not None):
                        f_cur = fhat + self._f_drift * (t_start - self._t_lock)
                        a = 0.5 * self._f_drift
                        b = f_cur
                        c = cycles
                        if abs(a) < 1e-9:
                            return c / max(b, 1e-9)
                        disc = b * b + 4.0 * a * c
                        if disc < 0:
                            return c / max(b, 1e-9)
                        return (-b + np.sqrt(disc)) / (2.0 * a)
                    else:
                        return cycles / max(fhat, 1e-9)
                dt_peak = float(solve_dt_from_cycles(t_now, cycles_to_peak))
                dt_trough = float(solve_dt_from_cycles(t_now, cycles_to_trough))
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
                # Collect frequency history for drift estimation (pre-lock only)
                try:
                    self._freq_hist.append((t_now, float(fhat)))
                    # Trim by time window
                    tw = float(self.linear_drift_window_sec)
                    if tw > 0:
                        t_cut = t_now - tw
                        while self._freq_hist and self._freq_hist[0][0] < t_cut:
                            self._freq_hist.popleft()
                except Exception:
                    pass
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

            # We reach this point only when fit_good, debounce satisfied, and armed.
            if self.lock_on_confirm and not self._fit_locked:
                # Compute dt to the next peak/trough right now to avoid locking on top of an anchor
                two_pi = 2.0 * np.pi
                t_arr = np.array([t for (t, _, _, _) in self.buf], dtype=float)
                t_rel = t_arr - t_arr[0]
                theta_now = two_pi * fhat * t_rel[-1] + phihat
                dtheta_peak = ((0.5 * np.pi) - theta_now) % (two_pi)
                dtheta_trough = ((1.5 * np.pi) - theta_now) % (two_pi)
                dt_peak = float(dtheta_peak / (two_pi * fhat))
                dt_trough = float(dtheta_trough / (two_pi * fhat))

                # Only establish the lock if we have some margin from the immediate anchors
                if (dt_peak > self.lock_min_margin_s) and (dt_trough > self.lock_min_margin_s):
                    # Freeze f/phi and pre-compute the schedule WITHOUT firing
                    self._establish_lock_and_schedule(t_now, fhat, phihat, t_arr[0])
                    # Right after lock established without firing, measure arrivals until first cutoff
                    try:
                        self._arr_open("lock_to_cutoff", "lock→first cutoff")
                    except Exception:
                        pass
                    # Skip the rest of the pre-lock firing logic for this eval
                    return
                # Else: too close to an anchor; let next eval lock (or let normal crossing fire)

            # Phase calculations using relative time base (pre-lock)
            def pos_mod(x, m):
                return (x % m + m) % m
            theta_now = two_pi * fhat * t_rel[-1] + phihat
            dtheta_peak = pos_mod((0.5 * np.pi) - theta_now, two_pi)
            dtheta_trough = pos_mod((1.5 * np.pi) - theta_now, two_pi)
            dt_peak = float(dtheta_peak / (two_pi * fhat))
            dt_trough = float(dtheta_trough / (two_pi * fhat))

        if self._fit_locked:
            # Derive debug dts from deterministic schedule
            dt_peak = float(max(0.0, (self._next_cutoff_time or t_now) - t_now))
            dt_trough = float(max(0.0, (self._next_enable_time or t_now) - t_now))
            # Deterministic schedule: trigger at scheduled times with optional one-shot prefire (cutoff only)
            entered_peak_window = False
            if self._next_cutoff_time is not None:
                if (not self._postlock_first_prefire_used) and (self._postlock_first_prefire_s > 0.0):
                    t_fire_off = self._next_cutoff_time - self._postlock_first_prefire_s
                else:
                    t_fire_off = self._next_cutoff_time
                entered_peak_window = (t_now >= t_fire_off)
                # Optional debug
                try:
                    self.get_logger().debug(
                        f"cutoff sched={self._next_cutoff_time:.6f} prefire@{t_fire_off:.6f} "
                        f"used={self._postlock_first_prefire_used} now={t_now:.6f}"
                    )
                except Exception:
                    pass
            entered_trough_window = (self._next_enable_time is not None) and (t_now >= self._next_enable_time)
        else:
            # --- Pre-lock detection based on phase window crossing ---
            thr_off = self.time_delay_off if self.time_delay_off > 0.0 else max(1.5 * dt_sample, 1e-3)
            thr_on  = self.time_delay_on  if self.time_delay_on  > 0.0 else max(1.5 * dt_sample, 1e-3)

            entered_peak_window = (
                (self._prev_dt_peak is not None) and (self._prev_dt_peak > thr_off) and (dt_peak <= thr_off)
            )
            entered_trough_window = (
                (self._prev_dt_trough is not None) and (self._prev_dt_trough > thr_on) and (dt_trough <= thr_on)
            )
            # Track for next eval
            self._prev_dt_peak = dt_peak
            self._prev_dt_trough = dt_trough

        # ---- Coalesced-window detector (diagnostic only) ----
        try:
            self._eval_seq += 1
        except AttributeError:
            self._eval_seq = 1
        state0 = "DISABLED" if self._phase_state_disabled else "ENABLED"
        try:
            self.get_logger().debug(
                "PRE flags: eval=%d state=%s entered_peak=%s entered_trough=%s "
                "dt_peak=%.4f dt_trough=%.4f t_now=%.6f" %
                (self._eval_seq, state0, entered_peak_window, entered_trough_window,
                 float(dt_peak), float(dt_trough), float(t_now))
            )
        except Exception:
            pass
        if entered_peak_window and entered_trough_window:
            self._obs["coalesced"] = int(self._obs.get("coalesced", 0)) + 1
            try:
                self.get_logger().warn(
                    "COALESCED windows: both peak and trough eligible in same eval "
                    "(eval=%d, state=%s, dt_peak=%.4f, dt_trough=%.4f, t_now=%.6f)" %
                    (self._eval_seq, state0, float(dt_peak), float(dt_trough), float(t_now))
                )
            except Exception:
                pass

        # ---- Overdue-anchor counter (locked schedule behind) ----
        try:
            T_lock_for_dbg = self._T_lock if self._T_lock else (1.0 / max((self._lock_f or 1e-6), 1e-6))
        except Exception:
            T_lock_for_dbg = float('nan')
        overdue_cut = 0
        overdue_en = 0
        guard = 1e-4
        if self._fit_locked:
            try:
                if (self._next_cutoff_time is not None) and (t_now > (self._next_cutoff_time + guard)):
                    overdue_cut = 1 + int(math.floor((t_now - self._next_cutoff_time) / max(T_lock_for_dbg, 1e-6)))
                if (self._next_enable_time is not None) and (t_now > (self._next_enable_time + guard)):
                    overdue_en = 1 + int(math.floor((t_now - self._next_enable_time) / max(T_lock_for_dbg, 1e-6)))
                if overdue_cut or overdue_en:
                    self._obs["overdue_c"] = int(self._obs.get("overdue_c", 0)) + int(overdue_cut)
                    self._obs["overdue_e"] = int(self._obs.get("overdue_e", 0)) + int(overdue_en)
                    self.get_logger().warn(
                        "OVERDUE anchors: cutoffs=%d enables=%d (t_now=%.6f, next_cut=%.6f, next_en=%.6f, T=%.6f)" %
                        (overdue_cut, overdue_en, float(t_now),
                         (self._next_cutoff_time if self._next_cutoff_time is not None else float('nan')),
                         (self._next_enable_time if self._next_enable_time is not None else float('nan')),
                         float(T_lock_for_dbg))
                    )
            except Exception:
                pass

        # Periodic observation summary
        try:
            if (self._eval_seq % 200) == 0:
                self.get_logger().info(
                    "OBS summary: coalesced=%d overdue_cut=%d overdue_en=%d" %
                    (int(self._obs.get("coalesced", 0)), int(self._obs.get("overdue_c", 0)), int(self._obs.get("overdue_e", 0)))
                )
        except Exception:
            pass

        

        action_taken = False
        # Cut off at peak (early by time_delay_off or sampling epsilon)
        if entered_peak_window and not self._phase_state_disabled and not self._permanent_on:
            self._phase_state_disabled = True
            # Action publish telemetry (cutoff)
            try:
                ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
            except Exception:
                ros_now = float('nan')
            err_sched = float('nan')
            if self._fit_locked:
                t_sched_dbg = self._next_cutoff_time
                if t_sched_dbg is not None:
                    err_sched = t_now - float(t_sched_dbg)
            try:
                self.get_logger().info(
                    "PUB cutoff: t_data=%.6f ros_now=%.6f err_sched=%.6f next_cut=%.6f next_en=%.6f" %
                    (float(t_now), float(ros_now), float(err_sched),
                     (self._next_cutoff_time if self._next_cutoff_time is not None else float('nan')),
                     (self._next_enable_time if self._next_enable_time is not None else float('nan')))
                )
            except Exception:
                pass
            self._last_action_ros_time = ros_now
            self._last_action_name = "cutoff"
            # Publish
            self.attack_pub.publish(Bool(data=False))
            self.cutoff_pub.publish(Int32(data=0))
            self.cutoff_latched = True
            action_taken = True
            # Consume the one-shot prefire after the FIRST post-lock cutoff
            if self._fit_locked and (not self._postlock_first_prefire_used):
                self._postlock_first_prefire_used = True
            # Close lock→first cutoff window if open, then open cutoff→enable window
            if self._arr_window_mode == "lock_to_cutoff":
                try:
                    self._arr_close("lock→first cutoff")
                except Exception:
                    pass
            try:
                self._arr_open("cutoff_to_enable", "cutoff→next enable")
            except Exception:
                pass
            # (pre-lock bias one-shot marker removed)
            t = self.get_clock().now()  # ROS time
            # Estimate ROS time aligned to sensor time origin (first sample)
            ros_est = None
            try:
                if (self._first_ros_time is not None) and (self._first_stamp is not None):
                    ros_est = self._first_ros_time + (t_now - self._first_stamp)
            except Exception:
                ros_est = None
            
            if self.log_on_trigger:
                self.get_logger().warn(
                    f"Cutoff @peak t_data={t_now:.3f}s (axis={sel_axis}) "
                    f"dt_peak={dt_peak:.3f}s delay_off={self.time_delay_off:.3f}s"
                    f"ROS time: {t.nanoseconds/1e9:.9f} sec ({t.nanoseconds} ns)"
                    + (f" ros_est: {ros_est:.9f}" if ros_est is not None else "")
                )
            # Schedule debug: report scheduled vs actual (locked mode only)
            if self.publish_schedule_debug and self._fit_locked:
                self._publish_sched_debug(
                    event='cutoff', axis=sel_axis,
                    t_sched=self._next_cutoff_time,
                    t_data=t_now,
                    f_use=self._lock_f if self._lock_f is not None else fhat,
                    drift=self._f_drift,
                    dt_sample=dt_sample,
                )
            # If this is the first cutoff, lock the fit for future predictions (absolute phase)
            if not self._fit_locked:
                # Compute absolute phase offset from the last fit
                # phihat was estimated with t = t_rel = t_abs - t_arr[0] -> phi_abs = phi - 2π f * t_arr[0]
                phi_abs = (phihat - two_pi * fhat * t_arr[0]) if 'phihat' in locals() else phihat_abs
                self._fit_locked = True
                # Optionally estimate linear frequency drift from recent history
                if self.enable_linear_drift and len(self._freq_hist) >= 2:
                    try:
                        import numpy as _np
                        tt = _np.array([p[0] for p in self._freq_hist], dtype=float)
                        ff = _np.array([p[1] for p in self._freq_hist], dtype=float)
                        t_ref = float(t_now)
                        x = tt - t_ref  # center for numerical stability
                        m, b = _np.polyfit(x, ff, 1)
                        m = float(_np.clip(m, -abs(self.max_drift_hz_per_s), abs(self.max_drift_hz_per_s)))
                        f_lock_est = float(max(b, 1e-6))
                        self._f_drift = m
                        self._lock_f = f_lock_est
                    except Exception:
                        self._lock_f = fhat
                        self._f_drift = 0.0
                else:
                    self._lock_f = fhat
                    self._f_drift = 0.0
                self._lock_phi_abs = float(phi_abs)
                self._t_lock = float(t_now)
            # Immediately schedule BOTH next trough and next peak using the peak anchor
            if self._fit_locked:
                def solve_dt_from_cycles(t_start: float, cycles: float) -> float:
                    if self.enable_linear_drift and (self._t_lock is not None):
                        f_cur = self._lock_f + self._f_drift * (t_start - self._t_lock)
                        a = 0.5 * self._f_drift
                        b = f_cur
                        c = cycles
                        if abs(a) < 1e-9:
                            return c / max(b, 1e-9)
                        disc = b*b + 4.0*a*c
                        if disc < 0:
                            return c / max(b, 1e-9)
                        return (-b + np.sqrt(disc)) / (2.0*a)
                    else:
                        return cycles / max(self._lock_f, 1e-9)
                # The peak anchor that just caused cutoff corresponds to the scheduled peak
                peak_anchor = self._next_peak_time if (self._next_peak_time is not None) else (t_now + float(self.time_delay_off))
                # Next trough is +0.5 cycles from this peak
                half = solve_dt_from_cycles(peak_anchor, 0.5)
                self._next_trough_time = peak_anchor + half
                self._next_enable_time = self._next_trough_time - float(self.time_delay_on)
                # Ensure enable is strictly in the future
                while self._next_enable_time <= t_now:
                    # advance trough/enable by +1 cycle
                    step1 = solve_dt_from_cycles(self._next_trough_time, 1.0)
                    self._next_trough_time += step1
                    self._next_enable_time = self._next_trough_time - float(self.time_delay_on)
                # Next peak is +1.0 cycles from this peak
                full = solve_dt_from_cycles(peak_anchor, 1.0)
                self._next_peak_time = peak_anchor + full
                self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)
                while self._next_cutoff_time <= t_now:
                    step2 = solve_dt_from_cycles(self._next_peak_time, 1.0)
                    self._next_peak_time += step2
                    self._next_cutoff_time = self._next_peak_time - float(self.time_delay_off)

        # Turn on at trough (early by time_delay_on or sampling epsilon)
        if entered_trough_window and self._phase_state_disabled:
            self._phase_state_disabled = False
            # Action publish telemetry (enable)
            try:
                ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
            except Exception:
                ros_now = float('nan')
            err_sched = float('nan')
            if self._fit_locked:
                t_sched_dbg = self._next_enable_time
                if t_sched_dbg is not None:
                    err_sched = t_now - float(t_sched_dbg)
            delta_ros_from_prev = (ros_now - self._last_action_ros_time) if (self._last_action_ros_time is not None) else float('nan')
            last_name = self._last_action_name or "none"
            try:
                self.get_logger().info(
                    "PUB enable: t_data=%.6f ros_now=%.6f err_sched=%.6f ΔROS_from_prev=%.6f last=%s next_cut=%.6f next_en=%.6f" %
                    (float(t_now), float(ros_now), float(err_sched), float(delta_ros_from_prev),
                     last_name,
                     (self._next_cutoff_time if self._next_cutoff_time is not None else float('nan')),
                     (self._next_enable_time if self._next_enable_time is not None else float('nan')))
                )
            except Exception:
                pass
            self._last_action_ros_time = ros_now
            self._last_action_name = "enable"
            # Publish
            self.attack_pub.publish(Bool(data=True))
            self.cutoff_pub.publish(Int32(data=1))
            self.cutoff_latched = False
            action_taken = True
            # Close cutoff→enable window if open
            if self._arr_window_mode == "cutoff_to_enable":
                try:
                    self._arr_close("cutoff→next enable")
                except Exception:
                    pass
            t = self.get_clock().now()  # ROS time
            # Estimate ROS time aligned to sensor time origin (first sample)
            ros_est = None
            try:
                if (self._first_ros_time is not None) and (self._first_stamp is not None):
                    ros_est = self._first_ros_time + (t_now - self._first_stamp)
            except Exception:
                ros_est = None
            if self.log_on_trigger:
                self.get_logger().info(
                    f"Enable @trough t_data={t_now:.3f}s (axis={sel_axis}) "
                    f"dt_trough={dt_trough:.3f}s delay_on={self.time_delay_on:.3f}s"
                    f"ROS time: {t.nanoseconds/1e9:.9f} sec ({t.nanoseconds} ns)"
                    + (f" ros_est: {ros_est:.9f}" if ros_est is not None else "")
                )
            if self.publish_schedule_debug and self._fit_locked:
                self._publish_sched_debug(
                    event='enable', axis=sel_axis,
                    t_sched=self._next_enable_time,
                    t_data=t_now,
                    f_use=self._lock_f if self._lock_f is not None else fhat,
                    drift=self._f_drift,
                    dt_sample=dt_sample,
                )
            # Count completed periods on trough enables
            if self._fit_locked and (self.max_periods is not None) and (self.max_periods > 0):
                self._cycles_done += 1
                if self._cycles_done >= int(self.max_periods):
                    # Permanently ON from now on
                    self._permanent_on = True
                    
            # Advance enable schedule to the next trough time (keep strictly after now)
            if self._fit_locked:
                def solve_dt_from_cycles(t_start: float, cycles: float) -> float:
                    if self.enable_linear_drift and (self._t_lock is not None):
                        f_cur = self._lock_f + self._f_drift * (t_start - self._t_lock)
                        a = 0.5 * self._f_drift
                        b = f_cur
                        c = cycles
                        if abs(a) < 1e-9:
                            return c / max(b, 1e-9)
                        disc = b*b + 4.0*a*c
                        if disc < 0:
                            return c / max(b, 1e-9)
                        return (-b + np.sqrt(disc)) / (2.0*a)
                    else:
                        return cycles / max(self._lock_f, 1e-9)
                anchor = self._next_trough_time if (self._next_trough_time is not None) else t_now
                step = solve_dt_from_cycles(anchor, 1.0)
                self._next_trough_time = anchor + step
                while (self._next_trough_time - float(self.time_delay_on)) <= t_now:
                    self._next_trough_time += solve_dt_from_cycles(self._next_trough_time, 1.0)
                self._next_enable_time = self._next_trough_time - float(self.time_delay_on)

        # Progress logging
        self._eval_counter += 1
        if self.progress_every and (self._eval_counter % int(self.progress_every) == 0):
            t_rel_global = (t_now - (self._first_stamp if self._first_stamp is not None else t_arr[0]))
            self.get_logger().info(f"phase eval {self._eval_counter} @ t={t_rel_global:.2f}s")

        return

        

        # Debounce accumulation
        

    

    def _publish_sched_debug(self, event: str, axis: str, t_sched: float, t_data: float,
                              f_use: float, drift: float, dt_sample: float) -> None:
        if getattr(self, 'sched_pub', None) is None:
            return
        try:
            err = (t_data - t_sched) if (t_sched is not None) else float('nan')
            t_s = f"{t_sched:.6f}" if (t_sched is not None) else "nan"
            line = (
                f"event={event} axis={axis} t_sched={t_s} t_data={t_data:.6f} "
                f"err={err:.6f} f={f_use:.6f} drift={drift:.6f} dt_sample={dt_sample:.6f} "
                f"strides={self.eval_stride_prelock}/{self.eval_stride_postlock}"
            )
            self.sched_pub.publish(String(data=line))
        except Exception:
            pass

    def _observe_arrival_spacing(self, t_sec: float) -> None:
        """Observe ΔROS and Δt_data for each IMU callback, but only when _log_arrivals=True."""
        try:
            ros_now = float(self.get_clock().now().nanoseconds) * 1e-9
        except Exception:
            return

        # Always update last seen stamps, but only compute/log when window is open
        if (self._last_ros_arrival is not None) and (self._last_t_data is not None):
            d_ros = ros_now - self._last_ros_arrival   # seconds
            d_data = t_sec  - self._last_t_data        # seconds

            if self._log_arrivals:
                self._arr_samples += 1
                self._sum_d_ros  += d_ros
                self._sum_d_data += d_data

                # Periodic lightweight line (every 50 observed samples in-window)
                if (self._arr_samples % 50) == 0:
                    self.get_logger().info(
                        f"ARR dx: ΔROS={d_ros*1e3:.3f} ms, Δt_data={d_data*1e3:.3f} ms"
                    )

                # Backlog signature: ROS spacing << sensor spacing
                if (d_ros < 0.003) and (d_data > 0.008):
                    self._burst_counter += 1
                    ratio = d_data / max(d_ros, 1e-9)
                    if ratio > self._burst_max_ratio:
                        self._burst_max_ratio = ratio
                    # Emit at a few milestones to avoid spam
                    if self._burst_counter in (1, 10, 50, 100):
                        self.get_logger().warn(
                            f"Backlog chewing: ΔROS={d_ros*1e3:.2f} ms vs Δt_data={d_data*1e3:.2f} ms "
                            f"(x{ratio:.1f} faster)"
                        )
                else:
                    # If we were in a burst and it stops while window is still open, mark the drain
                    if self._burst_counter > 0:
                        self.get_logger().info(f"Backlog drained after {self._burst_counter} msgs (window still open)")
                        self._burst_counter = 0

        # Update "last" pointers every callback (even when window closed)
        self._last_ros_arrival = ros_now
        self._last_t_data = t_sec

    def _arr_open(self, mode: str, label: str) -> None:
        self._arr_window_mode = mode
        self._log_arrivals = True
        self._arr_samples = 0
        self._sum_d_ros = 0.0
        self._sum_d_data = 0.0
        self._burst_counter = 0
        self._burst_max_ratio = 0.0
        try:
            self.get_logger().info(f"ARR window opened ({label})")
        except Exception:
            pass

    def _arr_close(self, label: str) -> None:
        if self._log_arrivals:
            mean_d_ros = (self._sum_d_ros / self._arr_samples) if self._arr_samples else float('nan')
            mean_d_data = (self._sum_d_data / self._arr_samples) if self._arr_samples else float('nan')
            try:
                self.get_logger().info(
                    "ARR window summary: samples=%d  ΣΔROS=%.3f ms  ΣΔt_data=%.3f ms  "
                    "meanΔROS=%.3f ms  meanΔt_data=%.3f ms  burst_msgs=%d  max_ratio=%.1f" %
                    (self._arr_samples, self._sum_d_ros*1e3, self._sum_d_data*1e3,
                     mean_d_ros*1e3, mean_d_data*1e3, self._burst_counter, self._burst_max_ratio)
                )
            except Exception:
                pass
        self._log_arrivals = False
        self._arr_window_mode = "off"
        try:
            self.get_logger().info(f"ARR window closed ({label})")
        except Exception:
            pass

    def _near_anchor(self, t_now: float, margin_s: float = 0.025) -> bool:
        if not self._fit_locked:
            return False
        targets = []
        if (self._next_cutoff_time is not None) and (not self._phase_state_disabled):
            targets.append(self._next_cutoff_time)
        if (self._next_enable_time is not None) and (self._phase_state_disabled):
            targets.append(self._next_enable_time)
        try:
            return any((0.0 <= (t - t_now) <= float(margin_s)) for t in targets)
        except Exception:
            return False

    # (pre-lock bias helper removed)

    # ---------- Callbacks ----------
    def _push(self, t_sec: float, wx: float, wy: float, wz: float) -> None:
        if self._first_stamp is None:
            self._first_stamp = t_sec
            try:
                t_ros = self.get_clock().now()
                t_ros_sec = float(t_ros.nanoseconds) * 1e-9
                self._first_ros_time = t_ros_sec
                self.get_logger().info(f"First sample: t_data={t_sec:.6f}s, ros_time={t_ros_sec:.6f}s")
            except Exception:
                self.get_logger().info(f"First sample: t_data={t_sec:.6f}s (ROS time unavailable)")
        # Observe arrival spacing for diagnostics
        self._observe_arrival_spacing(t_sec)
        self.buf.append((t_sec, wx, wy, wz))
        self._trim(t_sec)
        self._sample_counter += 1
        # (previous drift-based pre-lock bias removed; using recent-compression method only)
        # Log at specific sample counts (1000th and 2000th)
        if self._sample_counter in (1000, 2000):
            try:
                t_ros = self.get_clock().now()
                t_ros_sec = float(t_ros.nanoseconds) * 1e-9
                self.get_logger().info(
                    f"Sample {self._sample_counter}: t_data={t_sec:.6f}s, ros_time={t_ros_sec:.6f}s"
                )
            except Exception:
                self.get_logger().info(
                    f"Sample {self._sample_counter}: t_data={t_sec:.6f}s (ROS time unavailable)"
                )
        # Choose stride based on lock state; optionally force eval near anchors
        current_stride = self.eval_stride_postlock if self._fit_locked else self.eval_stride_prelock
        force_eval = False
        if self._fit_locked:
            # dt-based decision uses sensor time
            try:
                force_eval = self._near_anchor(t_sec, margin_s=0.025)
            except Exception:
                force_eval = False
        if (self._sample_counter % int(current_stride)) != 0 and not force_eval:
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
