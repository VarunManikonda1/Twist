# Gazebo (gz-sim) + ros2_control:
#   ros2 launch twist_2_0_description gazebo.launch.py
# Headless (no GUI):
#   ros2 launch twist_2_0_description gazebo.launch.py gz_args:="-r -s empty.sdf"
# Optional stand pose:
#   ros2 run twist_2_0_description stand_pose
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from twist_2_0_description.urdf_paths import package_uris_to_files

PACKAGE_NAME = "twist_2_0_description"


def generate_launch_description():
    share = get_package_share_directory(PACKAGE_NAME)
    # Gazebo resolves package:// as model://<pkg>/...; add share/ so model name matches the package.
    gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.dirname(share),
    )

    desc = xacro.process_file(
        os.path.join(share, "urdf", "twist_2_0_gazebo.urdf.xacro")
    ).toxml()
    desc = package_uris_to_files(desc, PACKAGE_NAME, share)

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": desc, "use_sim_time": True}],
    )
    # Spawn from the string, not the topic: another robot_state_publisher (e.g. display.launch.py)
    # may publish a URDF without the ros2_control block, leaving no controller_manager.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", desc, "-name", "twist_2_0", "-z", "0.15"],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Ground-truth base odometry from the OdometryPublisher system in the xacro.
            "/model/twist_2_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        remappings=[("/model/twist_2_0/odometry", "/odom")],
        parameters=[{"use_sim_time": True}],
    )
    # One spawner for both controllers avoids the spawner lock contention.
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "leg_controller"],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gz_args", default_value="-r empty.sdf"),
        gz_resource_path,
        gz,
        rsp,
        spawn,
        bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
    ])
