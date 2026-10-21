"""Gazebo Harmonic + Twist 2.0 + controllers + gait.

    ros2 launch twist_walker sim.launch.py               # everything, robot stands up
    ros2 launch twist_walker sim.launch.py gait:=false   # robot only, no gait node

Start-up order (each step waits for the previous one to finish):
    gz sim -> spawn robot -> joint_state_broadcaster -> leg_controller -> gait_node
"""

import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "twist_walker"
FOOT_FRICTION = "1.5"


def build_robot_description(share):
    """Expand the xacro, then give every lower-leg link high friction so feet grip."""
    xml = xacro.process_file(os.path.join(share, "urdf", "twist_walker.urdf.xacro")).toxml()
    root = ET.fromstring(xml)
    for j in root.findall("joint"):
        if j.get("name", "").endswith("_knee"):
            g = ET.SubElement(root, "gazebo", reference=j.find("child").get("link"))
            ET.SubElement(g, "mu1").text = FOOT_FRICTION
            ET.SubElement(g, "mu2").text = FOOT_FRICTION
    return ET.tostring(root, encoding="unicode")


def generate_launch_description():
    share = get_package_share_directory(PKG)
    robot_description = build_robot_description(share)
    use_gait = LaunchConfiguration("gait")

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 2 empty.sdf", "on_exit_shutdown": "true"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "twist_2_0",
                   "-z", LaunchConfiguration("spawn_z")],
    )

    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    legs = Node(package="controller_manager", executable="spawner", output="screen",
                arguments=["leg_controller", "--controller-manager", "/controller_manager"])

    gait = Node(
        package=PKG, executable="gait_node", output="screen", condition=IfCondition(use_gait),
        parameters=[os.path.join(share, "config", "gait.yaml")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gait", default_value="true", description="start the gait node"),
        DeclareLaunchArgument("spawn_z", default_value="0.12", description="spawn height (m)"),
        # lets Gazebo resolve package://twist_walker/meshes/... in the URDF
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(share)),
        gz, rsp, clock_bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[legs])),
        RegisterEventHandler(OnProcessExit(target_action=legs, on_exit=[gait])),
    ])
