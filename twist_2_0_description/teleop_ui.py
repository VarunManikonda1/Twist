"""Tkinter teleop UI: publishes /cmd_vel and /drive_mode."""

import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

BG = "#1b1f24"
PANEL = "#252b33"
FG = "#e8eaed"
MUTED = "#9aa0a6"
ACCENT = "#3d8bfd"
STOP = "#c62828"
OMNI = "#2e7d32"
ACK = "#ef6c00"
BTN = "#343c46"
BTN_ACTIVE = "#4a5563"


class TeleopUiNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_ui")
        self.declare_parameter("drive_mode", "omni")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.60)
        self.declare_parameter("ackermann_turn_gain", 0.85)
        self.declare_parameter("publish_rate", 20.0)

        self.mode = self.get_parameter("drive_mode").value
        if self.mode not in ("omni", "ackermann"):
            self.mode = "omni"
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.ackermann_turn_gain = float(self.get_parameter("ackermann_turn_gain").value)

        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._lock = threading.Lock()

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_mode = self.create_publisher(String, "/drive_mode", 10)
        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._publish)
        self.publish_mode()

    def publish_mode(self) -> None:
        msg = String()
        msg.data = self.mode
        self._pub_mode.publish(msg)
        self.get_logger().info(f"Drive mode -> {self.mode}")

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        with self._lock:
            self._vx, self._vy, self._wz = vx, vy, wz

    def stop(self) -> None:
        self.set_command(0.0, 0.0, 0.0)

    def _publish(self) -> None:
        with self._lock:
            vx, vy, wz = self._vx, self._vy, self._wz
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._pub_cmd.publish(msg)


class TeleopWindow:
    def __init__(self, node: TeleopUiNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Twist 2.0 Teleop")
        self.root.configure(bg=BG)
        self.root.minsize(420, 540)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.status = tk.StringVar()
        self.mode_label = tk.StringVar()
        self.hint = tk.StringVar()
        self._refresh_labels()
        self._build()
        self.root.bind("<KeyPress>", self._on_key)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

    def _build(self) -> None:
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=14, pady=10)
        tk.Label(header, text="Twist 2.0", font=("Sans", 18, "bold"), fg=FG, bg=BG).pack(anchor="w")
        tk.Label(header, textvariable=self.mode_label, font=("Sans", 11), fg=MUTED, bg=BG).pack(anchor="w")

        mode_row = tk.Frame(self.root, bg=BG)
        mode_row.pack(fill="x", padx=14, pady=(0, 8))
        self._colored_btn(mode_row, "Omni (mecanum)", OMNI, lambda: self._set_mode("omni")).pack(
            side="left", expand=True, fill="x", padx=(0, 6)
        )
        self._colored_btn(mode_row, "Ackermann (car)", ACK, lambda: self._set_mode("ackermann")).pack(
            side="left", expand=True, fill="x", padx=(6, 0)
        )

        speeds = tk.Frame(self.root, bg=PANEL)
        speeds.pack(fill="x", padx=14, pady=8)
        self._add_scale(speeds, "Linear speed", "linear_speed", 0.05, 0.8, self.node.linear_speed)
        self._add_scale(speeds, "Angular speed", "angular_speed", 0.1, 1.5, self.node.angular_speed)

        pad = tk.Frame(self.root, bg=BG)
        pad.pack(pady=12)
        self._hold_btn(pad, "Rotate CCW\n(Z+)", "rotate_ccw").grid(row=0, column=0, padx=6, pady=6)
        self._hold_btn(pad, "Forward", "forward").grid(row=0, column=1, padx=6, pady=6)
        self._hold_btn(pad, "Rotate CW\n(Z−)", "rotate_cw").grid(row=0, column=2, padx=6, pady=6)
        self._hold_btn(pad, "Left", "left").grid(row=1, column=0, padx=6, pady=6)
        tk.Button(
            pad,
            text="STOP",
            width=10,
            height=3,
            bg=STOP,
            fg="white",
            activebackground="#e53935",
            activeforeground="white",
            relief="flat",
            font=("Sans", 12, "bold"),
            command=self._stop,
        ).grid(row=1, column=1, padx=6, pady=6)
        self._hold_btn(pad, "Right", "right").grid(row=1, column=2, padx=6, pady=6)
        self._hold_btn(pad, "Back", "back").grid(row=2, column=1, padx=6, pady=6)

        tk.Label(
            self.root,
            textvariable=self.hint,
            font=("Sans", 10),
            fg=MUTED,
            bg=BG,
            justify="left",
            wraplength=390,
        ).pack(fill="x", padx=14, pady=(4, 2))
        tk.Label(
            self.root,
            textvariable=self.status,
            font=("Sans", 10, "bold"),
            fg=ACCENT,
            bg=BG,
        ).pack(fill="x", padx=14, pady=(0, 12))

    def _colored_btn(self, parent, text: str, color: str, command) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            bg=color,
            fg="white",
            activebackground=color,
            activeforeground="white",
            relief="flat",
            font=("Sans", 10, "bold"),
            height=2,
            command=command,
        )

    def _hold_btn(self, parent, text: str, action: str) -> tk.Button:
        btn = tk.Button(
            parent,
            text=text,
            width=10,
            height=3,
            bg=BTN,
            fg=FG,
            activebackground=BTN_ACTIVE,
            activeforeground=FG,
            relief="flat",
            font=("Sans", 10, "bold"),
        )
        btn.bind("<ButtonPress-1>", lambda _e, a=action: self._omni_or_ack(a))
        btn.bind("<ButtonRelease-1>", lambda _e: self._stop())
        return btn

    def _add_scale(self, parent, label: str, attr: str, lo: float, hi: float, value: float) -> None:
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", padx=10, pady=8)
        tk.Label(row, text=label, fg=FG, bg=PANEL, font=("Sans", 10)).pack(anchor="w")
        scale = tk.Scale(
            row,
            from_=lo,
            to=hi,
            resolution=0.05,
            orient="horizontal",
            bg=PANEL,
            fg=FG,
            troughcolor=BTN,
            highlightthickness=0,
            length=360,
            command=lambda v, a=attr: setattr(self.node, a, float(v)),
        )
        scale.set(value)
        scale.pack(fill="x")

    def _set_mode(self, mode: str) -> None:
        self.node.mode = mode
        self.node.stop()
        self.node.publish_mode()
        self._refresh_labels()
        self._stop()

    def _refresh_labels(self) -> None:
        if self.node.mode == "omni":
            self.mode_label.set("Mode: Omni — strafe like mecanum wheels")
            self.hint.set("Hold a button to move. Left/Right strafes. Rotate turns in place.")
        else:
            self.mode_label.set("Mode: Ackermann — car / differential turn")
            self.hint.set(
                "Hold Forward to drive. Left/Right = drive forward while turning. Rotate spins in place."
            )
        self.status.set("Idle")

    def _omni_or_ack(self, action: str) -> None:
        v = self.node.linear_speed
        w = self.node.angular_speed
        if self.node.mode == "omni":
            cmds = {
                "forward": (v, 0.0, 0.0),
                "back": (-v, 0.0, 0.0),
                "left": (0.0, v, 0.0),
                "right": (0.0, -v, 0.0),
                "rotate_ccw": (0.0, 0.0, w),
                "rotate_cw": (0.0, 0.0, -w),
            }
        else:
            turn = w * self.node.ackermann_turn_gain
            cmds = {
                "forward": (v, 0.0, 0.0),
                "back": (-v, 0.0, 0.0),
                "left": (v * 0.65, 0.0, turn),
                "right": (v * 0.65, 0.0, -turn),
                "rotate_ccw": (0.0, 0.0, w),
                "rotate_cw": (0.0, 0.0, -w),
            }
        vx, vy, wz = cmds[action]
        self.node.set_command(vx, vy, wz)
        self.node.get_logger().info(f"UI {action} mode={self.node.mode} vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}")
        labels = {
            "forward": "Forward",
            "back": "Back",
            "left": "Strafe left" if self.node.mode == "omni" else "Forward + turn left",
            "right": "Strafe right" if self.node.mode == "omni" else "Forward + turn right",
            "rotate_ccw": "Rotate CCW",
            "rotate_cw": "Rotate CW",
        }
        self.status.set(labels[action])

    def _stop(self) -> None:
        self.node.stop()
        self.status.set("Idle")

    def _on_key(self, event: tk.Event) -> None:
        key = event.keysym.lower()
        mapping = {
            "i": "forward",
            "comma": "back",
            "j": "left",
            "l": "right",
            "u": "rotate_ccw",
            "o": "rotate_cw",
        }
        if key == "space":
            self._stop()
        elif key == "m":
            self._set_mode("ackermann" if self.node.mode == "omni" else "omni")
        elif key in mapping:
            self._omni_or_ack(mapping[key])

    def _on_key_release(self, event: tk.Event) -> None:
        if event.keysym.lower() in ("i", "comma", "j", "l", "u", "o"):
            self._stop()

    def _on_close(self) -> None:
        self.node.stop()
        self.node._publish()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = TeleopUiNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        TeleopWindow(node).run()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
