"""Straight-road launch — paused-start approach.

Key insight: if Gazebo starts PAUSED the robot never falls before controllers
load.  All controllers activate against a stationary upright robot.  Then we
unpause physics and the balance controller (already warm) catches the robot
in the first IMU cycle.  No snap_upright needed at all.

Sequence:
  1.  Gazebo started with paused=true
  2.  robot_state_publisher
  3.  spawn_entity  (works fine while paused in Gazebo Classic)
  4.  joint_state_broadcaster spawner
  5.  effort_controllers spawner
  6.  balance_controller (use_sim_time=False — wall-clock timer, ready before unpause)
  7.  2 s later: unpause_physics (one-shot service call)
  8.  5 s later: odom_publisher + road_driver start
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share    = get_package_share_directory("two_wheel_robot")
    gazebo_share = get_package_share_directory("gazebo_ros")

    xacro_path = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "road.world")

    use_sim_time = LaunchConfiguration("use_sim_time")

    ros2_control_plugin = os.path.join(
        get_package_prefix("gazebo_ros2_control"), "lib",
        "libgazebo_ros2_control.so"
    )

    robot_description_str = subprocess.check_output(
        ["xacro", xacro_path,
         "use_sim:=true",
         "use_mesh_visuals:=true",
         f"ros2_control_plugin:={ros2_control_plugin}"]
    ).decode()
    robot_description = {"robot_description": robot_description_str}

    # ── 1. Gazebo PAUSED ──────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world":   world_path,
            "paused":  "true",      # robot never falls before controllers load
            "verbose": "false",
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    # ── 3. Spawn robot just past the start line, facing +X ─────────────
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "two_wheel_robot",
            "-x", "0.8",
            "-y", "0.0",
            "-z", "0.001",
            "-Y", "0.0",
        ],
        output="screen",
    )

    # ── 4 & 5. Controllers ───────────────────────────────────────────────
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    spawn_effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controllers",
                   "--controller-manager", "/controller_manager"],
    )

    # ── 6. balance_controller (wall-clock timer so it's ready before unpause)
    balance = Node(
        package="two_wheel_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # ── 7. Unpause physics (2 s after balance starts so it has initialised) ─
    unpause = ExecuteProcess(
        cmd=["ros2", "service", "call",
             "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    # ── 8. odom + road driver (5 s after balance — robot balanced by then) ─
    odom_publisher = Node(
        package="two_wheel_robot",
        executable="odom_publisher.py",
        name="odom_publisher",
        parameters=[{"use_sim_time": False}],   # wall clock — immune to sim resets
        output="screen",
    )

    road_driver = Node(
        package="two_wheel_robot",
        executable="road_drive.py",
        name="road_driver",
        parameters=[{
            "use_sim_time": False,
            "road_length":  15.5,
            "linear_vel":    0.12,
            "decel_zone":    2.0,
            "heading_kp":    0.4,
            "start_delay":   3.0,   # 3 s after node start → 8 s total after balance
        }],
        output="screen",
    )

    # ── Sequencing ────────────────────────────────────────────────────────
    after_spawn  = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[spawn_jsb])
    )
    after_jsb    = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_effort])
    )
    after_effort = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_effort,
            on_exit=[
                balance,
                TimerAction(period=2.0, actions=[unpause]),
                TimerAction(period=5.0, actions=[odom_publisher, road_driver]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo,
        rsp,
        spawn,
        after_spawn,
        after_jsb,
        after_effort,
    ])
