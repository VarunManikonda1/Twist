"""RViz + joint sliders, to inspect the URDF.  ros2 launch twist_walker display.launch.py
In RViz: set Fixed Frame = base_link, then Add -> RobotModel (Description Topic: /robot_description)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("twist_walker")
    with open(os.path.join(share, "urdf", "twist_2_0.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
