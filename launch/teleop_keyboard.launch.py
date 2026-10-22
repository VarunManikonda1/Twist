# Keyboard teleoperation (run in separate terminal after Gazebo):
#   Terminal 1: ros2 launch twist_2_0_description gazebo_with_cmd_vel.launch.py
#   Terminal 2: ros2 launch twist_2_0_description teleop_keyboard.launch.py
#
# Controls (OMNI mode):
#   i / ,     forward / back
#   j / l     strafe left / right
#   u / o     rotate CCW / CW
#   space     stop
#   m         switch to ackermann mode
#   q         quit
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop = Node(
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

    return LaunchDescription([teleop])
