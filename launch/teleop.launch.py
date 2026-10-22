# Teleoperation interface (run in separate terminal after Gazebo):
#   Terminal 1: ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py
#   Terminal 2: ros2 launch twist_2_0_description teleop.launch.py
#
# This defaults to keyboard teleop. Press 'q' to quit.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Keyboard teleop (terminal-based, press keys to move)
    teleop_keyboard = Node(
        package="twist_2_0_description",
        executable="teleop_keyboard",
        name="teleop_keyboard",
        output="screen",
        # teleop_keyboard reads raw keys, so it needs its own TTY; ros2 launch has none.
        prefix="gnome-terminal --wait --",
        parameters=[
            {
                "drive_mode": "omni",
                "linear_speed": 0.10,
                "angular_speed": 0.60,
                "ackermann_turn_gain": 0.85,
                "publish_rate": 20.0,
            }
        ],
    )

    # UI teleop (graphical interface) - uncomment to use instead
    # teleop_ui = Node(
    #     package="twist_2_0_description",
    #     executable="teleop_ui",
    #     name="teleop_ui",
    #     output="screen",
    # )

    return LaunchDescription([
        teleop_keyboard,
        # teleop_ui,  # Uncomment for GUI
    ])
