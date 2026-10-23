"""Drive the robot forward and backward in Gazebo and record every servo's angle.

    ros2 launch twist_walker sim.launch.py      # terminal 1
    ros2 run twist_walker record_walk --ros-args \\
        --params-file install/twist_walker/share/twist_walker/config/motor_ids.yaml   # terminal 2

The node sends /cmd_vel itself (stand -> forward -> stop -> backward -> stop), records
each joint command the gait node sends to the motors (50 Hz) and writes to out_dir:
    walk_<time>_ticks.csv  t, phase, ID1..ID12 as Feetech STS3215 positions (0..4095)
    walk_<time>_deg.csv    t, phase, ID1..ID12 as servo angles 0..360 deg (= ticks * 360/4096)
    walk_<time>_ids.txt    which ID is which joint, with direction/offset used
    walk_<time>_forward_deg.csv / _backward_deg.csv
                           only the forward / backward part, servo degrees, time from 0
Times start at 0 and are in seconds, so each row can be sent to the servos at its time.
"""

import csv
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

DEFAULT_JOINTS = [f"{l}_{j}" for l in ("FL", "FR", "RL", "RR") for j in ("hip_yaw", "hip_pitch", "knee")]


class RecordWalk(Node):
    def __init__(self):
        super().__init__("record_walk")
        p = self.declare_parameter
        self.joints = list(p("joints", DEFAULT_JOINTS).value)
        n = len(self.joints)
        self.ids = list(p("ids", list(range(1, n + 1))).value)
        self.direction = list(p("direction", [1] * n).value)
        self.offset = list(p("offset", [0] * n).value)
        self.ticks_per_rev = p("ticks_per_rev", 4096).value
        self.center = p("center", 2048).value
        speed = p("speed", 0.08).value
        stand, pause = p("stand_time", 3.0).value, p("pause_time", 1.5).value
        out_dir = os.path.expanduser(p("out_dir", "~/motor_recordings").value)

        # (phase name, duration, forward speed)
        self.script = [("stand", stand, 0.0),
                       ("forward", p("forward_time", 5.0).value, speed),
                       ("stop", pause, 0.0),
                       ("backward", p("backward_time", 5.0).value, -speed),
                       ("stop", pause, 0.0)]

        os.makedirs(out_dir, exist_ok=True)
        base = os.path.join(out_dir, time.strftime("walk_%Y%m%d_%H%M%S"))
        self.paths = (base + "_ticks.csv", base + "_deg.csv", base + "_ids.txt")
        self.f_ticks = open(self.paths[0], "w", newline="")
        self.f_deg = open(self.paths[1], "w", newline="")
        header = ["t", "phase"] + [f"ID{i}" for i in self.ids]
        self.w_ticks = csv.writer(self.f_ticks)
        self.w_deg = csv.writer(self.f_deg)
        self.w_ticks.writerow(header)
        self.w_deg.writerow(header)
        with open(self.paths[2], "w") as f:
            f.write("ID  joint          direction  offset\n")
            for i, j, d, o in zip(self.ids, self.joints, self.direction, self.offset):
                f.write(f"{i:<4}{j:<15}{d:>9}{o:>8}\n")
            f.write(f"\nticks = {self.center} + direction * rad * {self.ticks_per_rev}/(2*pi) + offset\n")
            f.write(f"servo deg = ticks * 360/{self.ticks_per_rev}   ({self.center} ticks = "
                    f"{self.center * 360 / self.ticks_per_rev:.0f} deg = joint at 0 in the sim)\n")

        self.settle = p("settle_time", 4.0).value
        self.first_cmd = None
        self.walking = False             # true once the gait node has finished standing up
        self.t0 = None
        self.rows = 0
        self.deg_rows = []               # (t, phase, [deg per ID]) for the forward/backward files
        self.base = base
        self.phase = "waiting"
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Float64MultiArray, "/leg_controller/commands", self.on_cmd, 10)
        self.create_timer(0.05, self.drive)
        self.get_logger().info("waiting for the gait node to finish standing up ...")

    def elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def drive(self):
        if not self.walking:
            # gait_node blends into its stand over startup_time (2 s); give it time to settle
            now = self.get_clock().now()
            if self.first_cmd is None or (now - self.first_cmd).nanoseconds * 1e-9 < self.settle:
                return
            self.walking = True
            self.t0 = now
            self.get_logger().info("robot is standing, starting recording")
        t = self.elapsed()
        for name, dur, vx in self.script:
            if t < dur:
                if name != self.phase:
                    self.get_logger().info(f"{name} ({vx:+.2f} m/s for {dur:.1f} s)")
                self.phase = name
                msg = Twist()
                msg.linear.x = vx
                self.cmd_pub.publish(msg)
                return
            t -= dur
        self.cmd_pub.publish(Twist())
        self.finish()

    def to_ticks(self, k, rad):
        tick = self.center + self.direction[k] * rad * self.ticks_per_rev / (2 * math.pi) + self.offset[k]
        return min(max(tick, 0.0), self.ticks_per_rev - 1.0)

    def on_cmd(self, msg):
        if self.first_cmd is None:
            self.first_cmd = self.get_clock().now()
        if not self.walking or self.phase == "waiting":
            return
        t = f"{self.elapsed():.3f}"
        ticks = [self.to_ticks(k, q) for k, q in enumerate(msg.data)]
        self.w_ticks.writerow([t, self.phase] + [int(round(x)) for x in ticks])
        deg = [f"{x * 360.0 / self.ticks_per_rev:.2f}" for x in ticks]
        self.w_deg.writerow([t, self.phase] + deg)
        self.deg_rows.append((self.elapsed(), self.phase, deg))
        self.rows += 1

    def finish(self):
        self.f_ticks.close()
        self.f_deg.close()
        paths = list(self.paths)
        for phase in ("forward", "backward"):
            rows = [r for r in self.deg_rows if r[1] == phase]
            if not rows:
                continue
            path = f"{self.base}_{phase}_deg.csv"
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["time_s"] + [f"ID{i}" for i in self.ids])
                for t, _, deg in rows:
                    w.writerow([f"{t - rows[0][0]:.3f}"] + deg)
            paths.append(path)
        self.get_logger().info(f"done, {self.rows} rows written:\n  " + "\n  ".join(paths))
        raise SystemExit


def main():
    rclpy.init()
    node = RecordWalk()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if not node.f_ticks.closed:
            node.f_ticks.close()
            node.f_deg.close()
        node.destroy_node()
        rclpy.try_shutdown()
