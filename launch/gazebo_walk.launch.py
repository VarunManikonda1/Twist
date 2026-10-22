# Gazebo + controllers + automatic forward trot (no teleop needed):
#   ros2 launch twist_2_0_description gazebo_walk.launch.py
# Change the walk live, e.g.:
#   ros2 param set /walk_gait walk_vx 0.0 ; ros2 param set /walk_gait walk_wz 0.5
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
    walk = Node(
        package="twist_2_0_description",
        executable="walk_gait",
        name="walk_gait",
        output="screen",
        parameters=[
            os.path.join(pkg, "config", "walk_params.yaml"),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gz_args", default_value="-r empty.sdf"),
        gazebo,
        walk,
    ])
