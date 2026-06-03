"""Keyboard teleop for driving the robot from a dedicated terminal."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--",
                "bash",
                "-lc",
                "source /opt/ros/humble/setup.bash; "
                "source /home/prajwal/ros2_ws/install/setup.bash; "
                "exec ros2 run teleop_twist_keyboard teleop_twist_keyboard",
            ],
            output="screen",
        ),
    ])
