"""View the robot URDF in Gazebo Classic without running physics.

This launch is for inspecting the imported model. It starts Gazebo paused,
spawns the URDF visuals/collisions, and intentionally skips ros2_control and
the balance controller so the two-wheel robot does not immediately fall over.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("two_wheel_robot")
    xacro_path = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "empty.world")
    gazebo_share = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_z = LaunchConfiguration("spawn_z")

    robot_description_str = subprocess.check_output(
        [
            "xacro",
            xacro_path,
            "use_sim:=true",
            "use_ros2_control:=false",
            "use_mesh_visuals:=true",
        ]
    ).decode()
    robot_description = {"robot_description": robot_description_str}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "false",
            "pause": "true",
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "two_wheel_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", spawn_z,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("spawn_z", default_value="0.001"),
        gazebo,
        rsp,
        spawn,
    ])
