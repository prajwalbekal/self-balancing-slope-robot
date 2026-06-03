"""Gazebo teleop demo: paused physics + keyboard-driven model motion."""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("two_wheel_robot")
    xacro_path = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "empty.world")
    gazebo_share = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time")

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
            "-z", "0.001",
        ],
        output="screen",
    )

    driver = Node(
        package="two_wheel_robot",
        executable="kinematic_teleop_driver.py",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    teleop = ExecuteProcess(
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
    )

    focus_camera = ExecuteProcess(
        cmd=[
            "gz",
            "topic",
            "-p",
            "/gazebo/default/gzclient_camera/cmd",
            "-m",
            'follow_model: "two_wheel_robot"',
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo,
        rsp,
        spawn,
        TimerAction(period=2.0, actions=[driver]),
        TimerAction(period=3.0, actions=[focus_camera]),
        TimerAction(period=3.5, actions=[teleop]),
    ])
