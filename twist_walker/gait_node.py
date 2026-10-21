"""ROS 2 node: /cmd_vel -> trot gait -> IK -> /leg_controller/commands.

Pipeline every tick (default 50 Hz, on simulation time):
  1. smooth the latest /cmd_vel
  2. gait planner gives a target point for each foot
  3. IK (built from /robot_description) turns each foot point into 3 joint angles
  4. publish all 12 angles to the position controller
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from twist_walker.gait import TrotGait
from twist_walker.kinematics import load_legs

LEGS = ("FL", "FR", "RL", "RR")


class GaitNode(Node):
    def __init__(self):
        super().__init__("gait_node")
        p = self.declare_parameter
        self.joint_order = list(p("joints", [f"{l}_{j}" for l in LEGS for j in ("hip_yaw", "hip_pitch", "knee")]).value)
        self.stand_height = p("stand_height", 0.12).value
        self.stance_out = p("stance_out", 0.095).value
        self.period = p("period", 0.8).value
        self.step_height = p("step_height", 0.02).value
        self.max_step = p("max_step", 0.05).value
        self.max_v = (p("max_vx", 0.12).value, p("max_vy", 0.08).value, p("max_wz", 0.6).value)
        self.tau = p("cmd_smoothing", 0.3).value
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        rate = p("rate", 50.0).value
        self.startup_time = p("startup_time", 2.0).value
        self.seed = (0.0, p("seed_hip_pitch", -0.6).value, p("seed_knee", 0.2).value)

        self.legs = None
        self.gait = None
        self.q = {}                      # last IK solution per leg (also the next IK seed)
        self.q_stand = None
        self.q_start = None              # joint angles at startup (from /joint_states)
        self.t_start = None
        self.cmd_raw = np.zeros(3)
        self.cmd = np.zeros(3)
        self.last_cmd_time = None
        self.last_tick = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/leg_controller/commands", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("waiting for /robot_description ...")

    # ---------------- setup ----------------
    def on_description(self, msg):
        if self.legs is not None:
            return
        try:
            self.legs = load_legs(msg.data, LEGS)
        except ValueError as e:
            self.get_logger().error(f"could not build leg kinematics from URDF: {e}")
            return
        nominal = {}
        for name, leg in self.legs.items():
            hip = leg.hip_position
            out = np.array([hip[0], hip[1], 0.0])
            out /= max(np.linalg.norm(out), 1e-9)
            nominal[name] = np.array([hip[0], hip[1], 0.0]) + self.stance_out * out
            nominal[name][2] = -self.stand_height
        self.gait = TrotGait(nominal, self.period, self.step_height, self.max_step)

        self.q_stand = {}
        for name, leg in self.legs.items():
            q, err = leg.ik(nominal[name], self.seed, iters=200)
            self.q_stand[name] = q
            self.q[name] = q.copy()
            self.get_logger().info(
                f"{name}: foot {np.round(leg.foot_point, 3)} in {leg.chain[-1].child}, "
                f"stand q = {np.round(q, 3)} (IK error {err * 1000:.1f} mm)")
            if err > 0.003:
                self.get_logger().warn(
                    f"{name}: stand pose not reachable; lower stand_height or stance_out")

    def on_joint_states(self, msg):
        if self.q_start is None and self.legs is not None:
            pos = dict(zip(msg.name, msg.position))
            if all(n in pos for n in self.joint_order):
                self.q_start = {l: np.array([pos[n] for n in self.legs[l].joint_names]) for l in LEGS}

    def on_cmd(self, msg):
        self.cmd_raw = np.array([
            float(np.clip(msg.linear.x, -self.max_v[0], self.max_v[0])),
            float(np.clip(msg.linear.y, -self.max_v[1], self.max_v[1])),
            float(np.clip(msg.angular.z, -self.max_v[2], self.max_v[2])),
        ])
        self.last_cmd_time = self.get_clock().now()

    # ---------------- control loop ----------------
    def tick(self):
        if self.legs is None or self.q_start is None:
            return
        now = self.get_clock().now()
        if self.t_start is None:
            self.t_start = now
            self.last_tick = now
            self.get_logger().info("standing up ...")
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0:
            return                     # sim clock paused
        elapsed = (now - self.t_start).nanoseconds * 1e-9

        # Phase 1: blend smoothly from the spawn pose into the stand pose
        if elapsed < self.startup_time:
            s = elapsed / self.startup_time
            s = 0.5 - 0.5 * math.cos(math.pi * s)
            out = {l: (1 - s) * self.q_start[l] + s * self.q_stand[l] for l in LEGS}
            self.publish(out)
            return

        # Phase 2: walk according to /cmd_vel
        if self.last_cmd_time is None or (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.cmd_raw = np.zeros(3)
        alpha = min(1.0, dt / max(self.tau, 1e-3))
        self.cmd += alpha * (self.cmd_raw - self.cmd)
        if np.all(np.abs(self.cmd) < 1e-3) and np.all(self.cmd_raw == 0.0):
            self.cmd[:] = 0.0

        targets = self.gait.update(dt, *self.cmd)
        for name, leg in self.legs.items():
            q, _ = leg.ik(targets[name], self.q[name], iters=15)
            self.q[name] = q
        self.publish(self.q)

    def publish(self, q_by_leg):
        values = {}
        for name, leg in self.legs.items():
            for jn, v in zip(leg.joint_names, q_by_leg[name]):
                values[jn] = float(v)
        self.pub.publish(Float64MultiArray(data=[values[n] for n in self.joint_order]))


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
