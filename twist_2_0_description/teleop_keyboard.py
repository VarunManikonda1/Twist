"""Keyboard teleop: publishes /cmd_vel and /drive_mode (omni vs ackermann)."""

import sys
import termios
import threading
import tty
from select import select

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

HELP_OMNI = """
Twist 2.0 teleop — OMNIDIRECTIONAL (mecanum-style)
  u / o     rotate CCW / CW
  i / ,     forward / back
  j / l     strafe left / right
  space     stop
  m         switch to ackermann (car) mode
  q         quit
"""

HELP_ACK = """
Twist 2.0 teleop — ACKERMANN / diff-drive style
  i / ,     forward / back
  j / l     forward + turn left / forward + turn right
  u / o     rotate in place CCW / CW
  space     stop
  m         switch to omni mode
  q         quit
"""


class TeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("teleop_keyboard")
        self.declare_parameter("drive_mode", "omni")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.60)
        self.declare_parameter("ackermann_turn_gain", 0.85)
        self.declare_parameter("publish_rate", 20.0)

        self._mode = self.get_parameter("drive_mode").value
        if self._mode not in ("omni", "ackermann"):
            self._mode = "omni"
        self._speed = self.get_parameter("linear_speed").value
        self._ang = self.get_parameter("angular_speed").value
        self._ack_turn = self.get_parameter("ackermann_turn_gain").value
        self._twist = Twist()
        self._lock = threading.Lock()

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_mode = self.create_publisher(String, "/drive_mode", 10)
        self._publish_mode()

        rate = self.get_parameter("publish_rate").value
        self.create_timer(1.0 / rate, self._publish_cmd)
        self._print_help()

    def _print_help(self) -> None:
        print(HELP_OMNI if self._mode == "omni" else HELP_ACK)

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode
        self._pub_mode.publish(msg)
        self.get_logger().info(f"Teleop drive mode: {self._mode}")

    def _publish_cmd(self) -> None:
        with self._lock:
            self._pub_cmd.publish(self._twist)

    def _stop(self) -> None:
        self._twist = Twist()

    def _apply_key(self, key: str) -> None:
        with self._lock:
            if self._mode == "omni":
                self._apply_key_omni(key)
            else:
                self._apply_key_ackermann(key)

    def _apply_key_omni(self, key: str) -> None:
        if key == "i":
            self._twist.linear.x = self._speed
        elif key == ",":
            self._twist.linear.x = -self._speed
        elif key == "j":
            self._twist.linear.y = self._speed
        elif key == "l":
            self._twist.linear.y = -self._speed
        elif key == "u":
            self._twist.angular.z = self._ang
        elif key == "o":
            self._twist.angular.z = -self._ang

    def _apply_key_ackermann(self, key: str) -> None:
        if key == "i":
            self._twist = Twist()
            self._twist.linear.x = self._speed
        elif key == ",":
            self._twist = Twist()
            self._twist.linear.x = -self._speed
        elif key == "j":
            self._twist = Twist()
            self._twist.linear.x = self._speed * 0.65
            self._twist.angular.z = self._ang * self._ack_turn
        elif key == "l":
            self._twist = Twist()
            self._twist.linear.x = self._speed * 0.65
            self._twist.angular.z = -self._ang * self._ack_turn
        elif key == "u":
            self._twist = Twist()
            self._twist.angular.z = self._ang
        elif key == "o":
            self._twist = Twist()
            self._twist.angular.z = -self._ang

    def run_keyboard_loop(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().error("teleop_keyboard requires an interactive terminal (TTY).")
            return

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                readable, _, _ = select([sys.stdin], [], [], 0.05)
                if not readable:
                    continue
                ch = sys.stdin.read(1)
                if ch in ("\x03", "q"):
                    break
                if ch == " ":
                    self._stop()
                    continue
                if ch == "m":
                    self._mode = "ackermann" if self._mode == "omni" else "omni"
                    self._stop()
                    self._publish_mode()
                    self._print_help()
                    continue
                if ch in "uiol,j":
                    self._apply_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self._stop()
            self._publish_cmd()


def main() -> None:
    rclpy.init()
    node = TeleopKeyboard()
    thread = threading.Thread(target=node.run_keyboard_loop, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
