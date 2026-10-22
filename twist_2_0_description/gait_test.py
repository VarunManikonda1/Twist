"""Closed-loop check of the walking pipeline in Gazebo.

Drives /cmd_vel through a sequence of motions and measures what the robot actually did using
ground-truth /odom. Prints a table and exits non-zero if any motion fails.

  ros2 run twist_2_0_description gait_test --ros-args -p use_sim_time:=true
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# (name, vx, vy, wz, seconds)
SEQUENCE = [
    ("stand", 0.0, 0.0, 0.0, 3.0),
    ("forward", 0.10, 0.0, 0.0, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("backward", -0.10, 0.0, 0.0, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("strafe_left", 0.0, 0.08, 0.0, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("strafe_right", 0.0, -0.08, 0.0, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("turn_ccw", 0.0, 0.0, 0.6, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("turn_cw", 0.0, 0.0, -0.6, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("arc_left", 0.10, 0.0, 0.4, 6.0),
    ("stop", 0.0, 0.0, 0.0, 2.0),
    ("fast_forward", 0.15, 0.0, 0.0, 6.0),
    ("diagonal", 0.08, -0.06, 0.0, 6.0),
    ("stop", 0.0, 0.0, 0.0, 3.0),
]

SETTLE = 1.0  # s ignored at the start of each segment (acceleration ramp)


def _yaw_roll_pitch(q) -> tuple[float, float, float]:
    x, y, z, w = q.x, q.y, q.z, q.w
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return yaw, roll, pitch


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GaitTest(Node):
    def __init__(self) -> None:
        super().__init__("gait_test")
        self.declare_parameter("wait_for_stand", 6.0)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._odom = None
        self._t_first = None
        self._seg = -1
        self._seg_t0 = None
        self._samples = []
        self.results = []
        self.done = False
        self.create_timer(0.02, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sample(self) -> tuple[float, float, float, float, float, float, float]:
        p = self._odom.pose.pose.position
        yaw, roll, pitch = _yaw_roll_pitch(self._odom.pose.pose.orientation)
        return self._now(), p.x, p.y, p.z, yaw, roll, pitch

    def _tick(self) -> None:
        if self.done or self._odom is None or self._now() == 0.0:
            return
        now = self._now()
        if self._t_first is None:
            self._t_first = now
            self.get_logger().info("Got /odom; waiting for the robot to stand up...")
        if self._seg < 0:
            if now - self._t_first < self.get_parameter("wait_for_stand").value:
                return
            self._start_segment(0, now)

        name, vx, vy, wz, dur = SEQUENCE[self._seg]
        elapsed = now - self._seg_t0
        if elapsed >= SETTLE:
            self._samples.append(self._sample())
        if elapsed >= dur:
            self._evaluate()
            if self._seg + 1 >= len(SEQUENCE):
                self._pub.publish(Twist())
                self.done = True
                return
            self._start_segment(self._seg + 1, now)
            name, vx, vy, wz, dur = SEQUENCE[self._seg]

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self._pub.publish(msg)

    def _start_segment(self, idx: int, now: float) -> None:
        self._seg = idx
        self._seg_t0 = now
        self._samples = []
        self.get_logger().info(f"--> {SEQUENCE[idx][0]}")

    def _evaluate(self) -> None:
        name, vx, vy, wz, _dur = SEQUENCE[self._seg]
        s = self._samples
        dt = s[-1][0] - s[0][0]
        # Integrate displacement in the (moving) body frame so arcs are measured correctly.
        bx = by = dyaw = 0.0
        for a, b in zip(s, s[1:]):
            dx, dy = b[1] - a[1], b[2] - a[2]
            bx += math.cos(a[4]) * dx + math.sin(a[4]) * dy
            by += -math.sin(a[4]) * dx + math.cos(a[4]) * dy
            dyaw += _wrap(b[4] - a[4])
        mvx, mvy, mwz = bx / dt, by / dt, dyaw / dt
        zmin = min(r[3] for r in s)
        tilt = max(max(abs(r[5]), abs(r[6])) for r in s)

        ok = zmin > 0.09 and tilt < 0.35
        notes = []
        if not ok:
            notes.append("fell/low body")
        moving = any(abs(c) > 1e-6 for c in (vx, vy, wz))
        for label, cmd, meas in (("vx", vx, mvx), ("vy", vy, mvy), ("wz", wz, mwz)):
            if abs(cmd) > 1e-6:
                # Right direction, at 50..150% of the commanded speed.
                if not 0.5 * abs(cmd) <= meas * math.copysign(1.0, cmd) <= 1.5 * abs(cmd):
                    ok = False
                    notes.append(f"{label} {meas:+.3f} vs {cmd:+.3f}")
            else:
                # Uncommanded axes: limited cross-drift while walking, ~still when stopped.
                if moving:
                    lim = 0.15 if label == "wz" else 0.03
                else:
                    lim = 0.05 if label == "wz" else 0.01
                if abs(meas) > lim:
                    ok = False
                    notes.append(f"{label} drift {meas:+.3f}")
        self.results.append((name, vx, vy, wz, mvx, mvy, mwz, zmin, tilt, ok, ", ".join(notes)))
        self.get_logger().info(
            f"{name:12s} cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f}) meas=({mvx:+.3f},{mvy:+.3f},"
            f"{mwz:+.3f}) zmin={zmin:.3f} tilt={math.degrees(tilt):.1f}deg "
            f"{'PASS' if ok else 'FAIL'} {', '.join(notes)}"
        )


def main() -> None:
    rclpy.init()
    node = GaitTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    print("\nsegment       cmd(vx,vy,wz)          measured(vx,vy,wz)        zmin   tilt  result")
    for name, vx, vy, wz, mvx, mvy, mwz, zmin, tilt, ok, notes in node.results:
        print(
            f"{name:12s} ({vx:+.2f},{vy:+.2f},{wz:+.2f})   ({mvx:+.3f},{mvy:+.3f},{mwz:+.3f})"
            f"   {zmin:.3f}  {math.degrees(tilt):4.1f}  {'PASS' if ok else 'FAIL'} {notes}"
        )
    passed = node.results and all(r[9] for r in node.results)
    print(f"\nOVERALL: {'PASS' if passed else 'FAIL'}")
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
