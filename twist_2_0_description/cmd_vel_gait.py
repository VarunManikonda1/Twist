"""Subscribe to /cmd_vel and walk the robot with a trot gait (omni or ackermann).

Startup: waits for /joint_states, smoothly moves from the current joint angles to the stand
pose, then follows /cmd_vel. Runs on the ROS clock, so set use_sim_time:=true in Gazebo.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from twist_2_0_description.gait import GaitController, GaitParams
from twist_2_0_description.joints import JOINT_NAMES

_GAIT_FIELDS = (
    "body_height",
    "foot_x",
    "foot_y",
    "step_height",
    "cycle_period",
    "swing_fraction",
    "gait_style",
    "max_vx",
    "max_vy",
    "max_wz",
    "max_lin_accel",
    "max_ang_accel",
    "max_foot_offset",
    "max_swing_speed",
)


class CmdVelGaitNode(Node):
    def __init__(self, name: str = "cmd_vel_gait") -> None:
        super().__init__(name)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("stand_duration", 2.0)
        self.declare_parameter("cmd_timeout", 0.0)
        self.declare_parameter("drive_mode", "omni")
        defaults = GaitParams()
        for field in _GAIT_FIELDS:
            self.declare_parameter(field, getattr(defaults, field))

        self._gait = GaitController(self._gait_params())
        self._stand_q = self._gait.stand_positions()
        self._cmd = Twist()
        self._cmd_stamp: Time | None = None
        self._drive_mode = self.get_parameter("drive_mode").value
        self._start_q: list[float] | None = None
        self._stand_t0: Time | None = None
        self._last_tick: Time | None = None
        self._ready = False
        self._last_logged = None
        self._params_dirty = False

        self._pub = self.create_publisher(JointTrajectory, "/leg_controller/joint_trajectory", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "/drive_mode", self._on_drive_mode, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.add_on_set_parameters_callback(self._on_params)

        self.create_timer(1.0 / self.get_parameter("publish_rate").value, self._on_timer)
        self.get_logger().info("Waiting for /joint_states before standing up...")

    # --- inputs ---------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd = msg
        self._cmd_stamp = self.get_clock().now()

    def _on_drive_mode(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode in ("omni", "ackermann", "car"):
            self._drive_mode = "ackermann" if mode in ("ackermann", "car") else "omni"
            self.get_logger().info(f"Drive mode -> {self._drive_mode}")

    def _on_joint_states(self, msg: JointState) -> None:
        if self._start_q is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            self._start_q = [pos[n] for n in JOINT_NAMES]

    def _on_params(self, _params) -> SetParametersResult:
        # Values are stored after this callback returns; reload them on the next tick.
        self._params_dirty = True
        return SetParametersResult(successful=True)

    def _gait_params(self) -> GaitParams:
        return GaitParams(**{f: self.get_parameter(f).value for f in _GAIT_FIELDS})

    def target_twist(self) -> tuple[float, float, float]:
        now = self.get_clock().now()
        timeout = self.get_parameter("cmd_timeout").value
        if self._cmd_stamp is None:
            return 0.0, 0.0, 0.0
        # timeout <= 0 latches the last command (teleop_twist_keyboard publishes once per key).
        if timeout > 0.0 and (now - self._cmd_stamp).nanoseconds * 1e-9 > timeout:
            return 0.0, 0.0, 0.0
        vx, vy, wz = self._cmd.linear.x, self._cmd.linear.y, self._cmd.angular.z
        if self._drive_mode == "ackermann":
            vy = 0.0
        return vx, vy, wz

    # --- output ---------------------------------------------------------------------
    def _publish(self, positions: list[float]) -> None:
        horizon = 2.0 / self.get_parameter("publish_rate").value
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.nanosec = int(horizon * 1e9)
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        msg.points = [point]
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._start_q is None or now.nanoseconds == 0:
            return

        if not self._ready:
            if self._stand_t0 is None:
                self._stand_t0 = now
                self.get_logger().info("Standing up...")
            dur = self.get_parameter("stand_duration").value
            a = min(1.0, (now - self._stand_t0).nanoseconds * 1e-9 / max(dur, 1e-3))
            a = a * a * (3.0 - 2.0 * a)
            self._publish([s + a * (g - s) for s, g in zip(self._start_q, self._stand_q)])
            if a >= 1.0:
                self._ready = True
                self._last_tick = now
                self.get_logger().info("Standing. Send geometry_msgs/Twist on /cmd_vel to walk.")
            return

        if self._params_dirty:
            self._params_dirty = False
            self._gait.set_params(self._gait_params())

        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        vx, vy, wz = self.target_twist()

        logged = (round(vx, 2), round(vy, 2), round(wz, 2))
        if logged != self._last_logged:
            self._last_logged = logged
            self.get_logger().info(f"cmd vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")

        positions = self._gait.update(dt, vx, vy, wz)
        if all(math.isfinite(v) for v in positions):
            self._publish(positions)


def main() -> None:
    rclpy.init()
    node = CmdVelGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
