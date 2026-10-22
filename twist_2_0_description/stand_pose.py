"""Publish a single stand joint trajectory to leg_controller (use after Gazebo is up)."""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from twist_2_0_description.gait import GaitController
from twist_2_0_description.joints import JOINT_NAMES


class StandPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("stand_pose")
        self._pub = self.create_publisher(
            JointTrajectory, "/leg_controller/joint_trajectory", 10
        )
        self._timer = self.create_timer(1.0, self._publish_once)
        self._sent = False

    def _publish_once(self) -> None:
        if self._sent:
            return
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = GaitController().stand_positions()
        point.time_from_start.sec = 3
        msg.points = [point]
        self._pub.publish(msg)
        self._sent = True
        self.get_logger().info("Published stand pose to /leg_controller/joint_trajectory")


def main() -> None:
    rclpy.init()
    node = StandPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
