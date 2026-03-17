#!/usr/bin/env python3
"""Compare IMU vs ground-truth odometry in a live plot.

Subscribes to:
- sensor_msgs/Imu (orientation + linear_acceleration)
- nav_msgs/Odometry (pose + twist)

It estimates vertical velocity/position from IMU by:
1) rotating body acceleration into world frame using IMU orientation
2) removing gravity
3) integrating to vz and z

This is intentionally a debugging tool; the IMU-integrated z will drift.

Usage examples:
  ros2 run am_description gt_imu_compare_plot.py
  ros2 run am_description gt_imu_compare_plot.py --imu-topic /imu --gt-topic /ground_truth/odom

If matplotlib is missing:
  pip install matplotlib numpy
"""

from __future__ import annotations

import argparse
import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp) -> float:
    # stamp is builtin_interfaces/Time
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def quat_to_rot_matrix_xyzw(qx: float, qy: float, qz: float, qw: float):
    """Quaternion (x,y,z,w) -> 3x3 rotation matrix.

    This matches ROS geometry_msgs conventions.
    """

    # Normalize to avoid scaling issues
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    qx /= n
    qy /= n
    qz /= n
    qw /= n

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    return (
        (r00, r01, r02),
        (r10, r11, r12),
        (r20, r21, r22),
    )


def mat3_mul_vec3(m, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


class ImuGtComparePlot(Node):
    def __init__(
        self,
        imu_topic: str,
        gt_topic: str,
        window_s: float,
        g: float,
        max_points: int,
        plot_hz: float,
    ):
        super().__init__('imu_gt_compare_plot')

        self.imu_topic = imu_topic
        self.gt_topic = gt_topic
        self.window_s = float(window_s)
        self.g = float(g)
        self.max_points = int(max_points)
        self.plot_period = 1.0 / float(plot_hz) if plot_hz > 0.0 else 0.1

        self._imu_t: Deque[float] = deque(maxlen=self.max_points)
        self._imu_az_w: Deque[float] = deque(maxlen=self.max_points)
        self._imu_vz: Deque[float] = deque(maxlen=self.max_points)
        self._imu_z: Deque[float] = deque(maxlen=self.max_points)
        self._imu_last_t: Optional[float] = None

        self._gt_t: Deque[float] = deque(maxlen=self.max_points)
        self._gt_z: Deque[float] = deque(maxlen=self.max_points)
        self._gt_vz: Deque[float] = deque(maxlen=self.max_points)

        self._last_imu_wall: float = time.time()
        self._last_gt_wall: float = time.time()

        self.create_subscription(Imu, imu_topic, self._on_imu, 50)
        self.create_subscription(Odometry, gt_topic, self._on_gt, 10)

        self.get_logger().info(f"Subscribing IMU: {imu_topic}")
        self.get_logger().info(f"Subscribing GT : {gt_topic}")

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu_wall = time.time()
        t = stamp_to_sec(msg.header.stamp)
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        q = msg.orientation
        r_w_b = quat_to_rot_matrix_xyzw(q.x, q.y, q.z, q.w)
        a_b = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        a_w = mat3_mul_vec3(r_w_b, a_b)

        # Remove gravity assuming +Z is up in world.
        az_w = a_w[2] - self.g

        if self._imu_last_t is None:
            # Initialize integrated states at the first message
            self._imu_last_t = t
            self._imu_t.append(t)
            self._imu_az_w.append(az_w)
            self._imu_vz.append(0.0)
            self._imu_z.append(0.0)
            return

        dt = t - self._imu_last_t
        self._imu_last_t = t
        if not (0.0 < dt < 0.2):
            # Skip large/negative dt to avoid spikes (e.g. sim reset).
            return

        vz_prev = self._imu_vz[-1] if self._imu_vz else 0.0
        z_prev = self._imu_z[-1] if self._imu_z else 0.0

        vz = vz_prev + az_w * dt
        z = z_prev + vz * dt

        self._imu_t.append(t)
        self._imu_az_w.append(az_w)
        self._imu_vz.append(vz)
        self._imu_z.append(z)

    def _on_gt(self, msg: Odometry) -> None:
        self._last_gt_wall = time.time()
        t = stamp_to_sec(msg.header.stamp)
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        z = msg.pose.pose.position.z
        vz = msg.twist.twist.linear.z

        self._gt_t.append(t)
        self._gt_z.append(z)
        self._gt_vz.append(vz)

    def have_imu(self) -> bool:
        return len(self._imu_t) > 0

    def have_gt(self) -> bool:
        return len(self._gt_t) > 0

    def time_since_imu(self) -> float:
        return time.time() - self._last_imu_wall

    def time_since_gt(self) -> float:
        return time.time() - self._last_gt_wall

    def get_plot_period(self) -> float:
        return self.plot_period

    def get_windowed_series(self):
        """Return windowed series, aligned only by time range (no resampling)."""
        t_now = 0.0
        if self._gt_t:
            t_now = self._gt_t[-1]
        elif self._imu_t:
            t_now = self._imu_t[-1]

        t_min = t_now - self.window_s if t_now > 0.0 else -1e9

        def window(deq_t: Deque[float], deq_y: Deque[float]):
            ts = list(deq_t)
            ys = list(deq_y)
            if not ts:
                return [], []
            # Keep only points within [t_min, t_now]
            i0 = 0
            while i0 < len(ts) and ts[i0] < t_min:
                i0 += 1
            ts = ts[i0:]
            ys = ys[i0:]
            # Make time relative for plotting
            if ts:
                t0 = ts[0]
                ts = [t - t0 for t in ts]
            return ts, ys

        imu_t, imu_z = window(self._imu_t, self._imu_z)
        _, imu_vz = window(self._imu_t, self._imu_vz)
        _, imu_az = window(self._imu_t, self._imu_az_w)

        gt_t, gt_z = window(self._gt_t, self._gt_z)
        _, gt_vz = window(self._gt_t, self._gt_vz)

        return (imu_t, imu_z, imu_vz, imu_az, gt_t, gt_z, gt_vz)


def main():
    parser = argparse.ArgumentParser(description="Plot IMU-integrated z vs GT odom z.")
    parser.add_argument('--imu-topic', default='/aerial_manipulator/imu', help='IMU topic (sensor_msgs/Imu)')
    parser.add_argument('--gt-topic', default='/ground_truth/odom', help='GT topic (nav_msgs/Odometry)')
    parser.add_argument('--window', type=float, default=20.0, help='Plot window (seconds)')
    parser.add_argument('--g', type=float, default=9.81, help='Gravity magnitude (m/s^2)')
    parser.add_argument('--max-points', type=int, default=5000, help='Max buffered points per series')
    parser.add_argument('--plot-hz', type=float, default=10.0, help='Plot refresh rate (Hz)')
    parser.add_argument('--no-plot', action='store_true', help='Do not open a plot; only print stats')
    args = parser.parse_args()

    rclpy.init()
    node = ImuGtComparePlot(
        imu_topic=args.imu_topic,
        gt_topic=args.gt_topic,
        window_s=args.window,
        g=args.g,
        max_points=args.max_points,
        plot_hz=args.plot_hz,
    )

    if args.no_plot:
        try:
            last_print = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                now = time.time()
                if now - last_print > 1.0:
                    last_print = now
                    node.get_logger().info(
                        f"imu_msgs={len(node._imu_t)} gt_msgs={len(node._gt_t)} "
                        f"since_imu={node.time_since_imu():.2f}s since_gt={node.time_since_gt():.2f}s"
                    )
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    try:
        import matplotlib.pyplot as plt
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(
            "matplotlib not available. Install with: pip install matplotlib numpy\n"
            f"Import error: {e}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    plt.ion()
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
    fig.canvas.manager.set_window_title('IMU vs GT compare')

    (line_gt_z,) = ax1.plot([], [], label='z_gt (m)')
    (line_imu_z,) = ax1.plot([], [], label='z_imu_integrated (m)')
    ax1.set_ylabel('z (m)')
    ax1.grid(True)
    ax1.legend(loc='upper left')

    (line_gt_vz,) = ax2.plot([], [], label='vz_gt (m/s)')
    (line_imu_vz,) = ax2.plot([], [], label='vz_imu_integrated (m/s)')
    ax2.set_ylabel('vz (m/s)')
    ax2.grid(True)
    ax2.legend(loc='upper left')

    (line_imu_az,) = ax3.plot([], [], label='az_world_minus_g (m/s^2)')
    ax3.set_ylabel('az (m/s^2)')
    ax3.set_xlabel('t (s, window-relative)')
    ax3.grid(True)
    ax3.legend(loc='upper left')

    last_plot = 0.0

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.02)

            now = time.time()
            if now - last_plot < node.get_plot_period():
                continue
            last_plot = now

            (imu_t, imu_z, imu_vz, imu_az, gt_t, gt_z, gt_vz) = node.get_windowed_series()

            line_gt_z.set_data(gt_t, gt_z)
            line_imu_z.set_data(imu_t, imu_z)

            line_gt_vz.set_data(gt_t, gt_vz)
            line_imu_vz.set_data(imu_t, imu_vz)

            line_imu_az.set_data(imu_t, imu_az)

            # Autoscale each axis based on available data
            for ax in (ax1, ax2, ax3):
                ax.relim()
                ax.autoscale_view(True, True, True)

            # Visual heartbeat on message liveness
            title = (
                f"IMU {args.imu_topic} ({'OK' if node.time_since_imu() < 1.0 else 'STALE'}) | "
                f"GT {args.gt_topic} ({'OK' if node.time_since_gt() < 1.0 else 'STALE'})"
            )
            fig.suptitle(title)

            plt.pause(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
