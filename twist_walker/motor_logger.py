"""Record every motor's commanded and measured values while the robot walks.

    ros2 run twist_walker motor_logger                       # writes ~/motor_log_<time>.csv
    ros2 run twist_walker motor_logger --ros-args -p file:=/tmp/walk.csv

One CSV row per /joint_states message (sim time), with for each of the 12 joints:
    <joint>_cmd  last position command sent by the gait node (rad)
    <joint>_pos  measured position (rad)
    <joint>_vel  measured velocity (rad/s)
    <joint>_eff  measured effort / torque (Nm)
A compact table of the latest values is also printed once per second.
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Same order as leg_controller in config/controllers.yaml
JOINTS = [f"{leg}_{j}" for leg in ("FL", "FR", "RL", "RR") for j in ("hip_yaw", "hip_pitch", "knee")]


class MotorLogger(Node):
    def __init__(self):
        super().__init__("motor_logger")
        default = os.path.expanduser(time.strftime("~/motor_log_%Y%m%d_%H%M%S.csv"))
        path = self.declare_parameter("file", default).value

        self.cmd = {j: float("nan") for j in JOINTS}
        self.latest = None
        self.rows = 0

        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t"] + [f"{j}_{k}" for j in JOINTS for k in ("cmd", "pos", "vel", "eff")])

        self.create_subscription(Float64MultiArray, "/leg_controller/commands", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 50)
        self.create_timer(1.0, self.print_table)
        self.get_logger().info(f"logging motors to {path}")

    def on_cmd(self, msg):
        for j, q in zip(JOINTS, msg.data):
            self.cmd[j] = q

    def on_state(self, msg):
        def field(arr, i):
            return arr[i] if i < len(arr) else float("nan")

        idx = {n: i for i, n in enumerate(msg.name)}
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vals = {}
        for j in JOINTS:
            i = idx.get(j)
            if i is None:
                vals[j] = (self.cmd[j], float("nan"), float("nan"), float("nan"))
            else:
                vals[j] = (self.cmd[j], field(msg.position, i), field(msg.velocity, i), field(msg.effort, i))
        self.w.writerow([f"{t:.4f}"] + [f"{v:.5f}" for j in JOINTS for v in vals[j]])
        self.latest = (t, vals)
        self.rows += 1

    def print_table(self):
        if self.latest is None:
            self.get_logger().info("waiting for /joint_states ...")
            return
        self.f.flush()
        t, vals = self.latest
        lines = [f"t={t:8.2f}s  rows={self.rows}",
                 f"{'joint':<14}{'cmd':>9}{'pos':>9}{'vel':>9}{'effort':>9}"]
        for j in JOINTS:
            c, p, v, e = vals[j]
            lines.append(f"{j:<14}{c:9.3f}{p:9.3f}{v:9.3f}{e:9.3f}")
        print("\n".join(lines) + "\n", flush=True)

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
