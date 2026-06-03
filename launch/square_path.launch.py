"""Square-path autonomous navigation launch.

Sequence:
  1. Gazebo with square_path.world
  2. robot_state_publisher
  3. spawn_entity (robot at origin, facing +X toward corner B)
  4. joint_state_broadcaster spawner
  5. effort_controllers spawner
  6. balance_controller   (use_sim_time=False, respawn)
  7. snap_upright          (3.5 s after balance starts)
  8. odom_publisher        (7 s after balance starts — odom must be ready first)
  9. square_path follower  (7 s after balance starts — starts its own 5 s
                            stabilisation delay internally, so first motion
                            at ~12 s after effort_controllers exits)
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    xacro_path  = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    world_path  = os.path.join(pkg_share, "worlds", "square_path.world")

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

    # 1. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "false"}.items(),
    )

    # 2. robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    # 3. Spawn robot at corner A (origin), facing +X toward corner B
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "two_wheel_robot",
            "-x", "0.0", "-y", "0.0", "-z", "0.001",
            "-Y", "0.0",   # yaw=0 → facing +X (East, toward corner B)
        ],
        output="screen",
    )

    # 4. joint_state_broadcaster
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # 5. effort_controllers
    spawn_effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controllers",
                   "--controller-manager", "/controller_manager"],
    )

    # 6. balance_controller (wall-clock timer so reset_simulation doesn't stall it)
    balance = Node(
        package="two_wheel_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[{"use_sim_time": False}],
        output="screen",
        respawn=True,
        respawn_delay=0.1,
    )

    # 7. snap_upright — resets robot to upright 3.5 s after controllers load
    snap_upright = Node(
        package="two_wheel_robot",
        executable="snap_upright.py",
        name="snap_upright",
        output="screen",
    )

    # 8. odom_publisher — wheel-encoder odometry for the path follower
    odom_publisher = Node(
        package="two_wheel_robot",
        executable="odom_publisher.py",
        name="odom_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 9. square_path follower — sends cmd_vel commands around the 3 m square
    square_path = Node(
        package="two_wheel_robot",
        executable="square_path.py",
        name="square_path_follower",
        parameters=[{
            "use_sim_time": True,
            "side_length":  5.0,   # 5 m × 5 m square
            "linear_vel":   0.20,
            "angular_vel":  0.35,
            "start_delay":  5.0,   # internal wait after node start
        }],
        output="screen",
    )

    # Sequencing
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
                TimerAction(period=3.5, actions=[snap_upright]),
                # odom + path follower start 7 s after effort controllers load;
                # path follower then waits its own 5 s → first move at ~12 s
                TimerAction(period=7.0, actions=[odom_publisher, square_path]),
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
