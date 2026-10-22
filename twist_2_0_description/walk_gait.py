"""Stand up, then walk with a fixed body twist (no /cmd_vel needed).

Same gait engine as cmd_vel_gait; the walk command comes from parameters walk_vx, walk_vy,
walk_wz (change them live with `ros2 param set /walk_gait walk_vx 0.1`).
"""

import rclpy

from twist_2_0_description.cmd_vel_gait import CmdVelGaitNode


class WalkGaitNode(CmdVelGaitNode):
    def __init__(self) -> None:
        super().__init__("walk_gait")
        self.declare_parameter("walk_vx", 0.1)
        self.declare_parameter("walk_vy", 0.0)
        self.declare_parameter("walk_wz", 0.0)

    def target_twist(self) -> tuple[float, float, float]:
        g = self.get_parameter
        return g("walk_vx").value, g("walk_vy").value, g("walk_wz").value


def main() -> None:
    rclpy.init()
    node = WalkGaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
