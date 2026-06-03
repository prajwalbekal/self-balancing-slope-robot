"""View the robot in RViz (no physics). Uses joint_state_publisher_gui sliders."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("two_wheel_robot")
    xacro_path = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    rviz_cfg   = os.path.join(pkg_share, "rviz", "view.rviz")

    robot_description = Command(["xacro ", xacro_path, " use_sim:=false"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),

        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        ),

        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
