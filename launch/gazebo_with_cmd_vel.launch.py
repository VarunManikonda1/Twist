# Gazebo + controllers + cmd_vel_gait (walks from /cmd_vel):
#   ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py
#   ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py gz_args:="-r -s empty.sdf"  # headless
#
# In a separate terminal, launch teleop:
#   ros2 launch twist_2_0_description teleop_keyboard.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("twist_2_0_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "gazebo.launch.py")),
        launch_arguments={"gz_args": LaunchConfiguration("gz_args")}.items(),
    )
    # Waits for /joint_states (i.e. controllers up), stands, then follows /cmd_vel.
    cmd_vel_gait = Node(
        package="twist_2_0_description",
        executable="cmd_vel_gait",
        name="cmd_vel_gait",
        output="screen",
        parameters=[
            os.path.join(pkg, "config", "walk_params.yaml"),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gz_args", default_value="-r empty.sdf"),
        gazebo,
        cmd_vel_gait,
    ])
