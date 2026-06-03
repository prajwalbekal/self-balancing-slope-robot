"""Full indoor navigation launch file.

Sequence:
  1.  Gazebo Classic with house.world (running, not paused)
  2.  robot_state_publisher
  3.  spawn_entity.py  (robot at x=0, y=0, z=0.001)
  4.  joint_state_broadcaster spawner  (after spawn exits)
  5.  effort_controllers spawner       (after jsb exits)
  6.  balance_controller node          (after effort exits, use_sim_time=False)
  7.  snap_upright                     (3.5 s after effort exits)
  8.  odom_publisher node              (use_sim_time=True)
  9.  slam_toolbox async node          (use_sim_time=True)
  10. nav2 nodes (controller_server, planner_server, recoveries_server,
                  bt_navigator, waypoint_follower, velocity_smoother)
  11. nav2_lifecycle_manager           (autostart=True)
  12. RViz2 with nav2_default_view.rviz
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
    nav2_share   = get_package_share_directory("nav2_bringup")

    xacro_path      = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    nav2_params     = os.path.join(pkg_share, "config", "nav2_params.yaml")
    slam_params     = os.path.join(pkg_share, "config", "slam_params.yaml")
    rviz_config     = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_arg    = LaunchConfiguration("world")

    ros2_control_plugin = os.path.join(
        get_package_prefix("gazebo_ros2_control"), "lib",
        "libgazebo_ros2_control.so"
    )

    # Process xacro at launch-file evaluation time (same pattern as gazebo.launch.py)
    robot_description_str = subprocess.check_output(
        ["xacro", xacro_path,
         "use_sim:=true",
         "use_mesh_visuals:=true",
         f"ros2_control_plugin:={ros2_control_plugin}"]
    ).decode()
    robot_description = {"robot_description": robot_description_str}

    # ------------------------------------------------------------------ #
    # 1. Gazebo Classic (NOT paused so controllers can activate)
    # ------------------------------------------------------------------ #
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_arg,
            "verbose": "false",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. robot_state_publisher
    # ------------------------------------------------------------------ #
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # 3. Spawn robot entity
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 4. joint_state_broadcaster  (after spawn exits)
    # ------------------------------------------------------------------ #
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # ------------------------------------------------------------------ #
    # 5. effort_controllers  (after jsb exits)
    # ------------------------------------------------------------------ #
    spawn_effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controllers",
                   "--controller-manager", "/controller_manager"],
    )

    # ------------------------------------------------------------------ #
    # 6. balance_controller  (after effort exits, use_sim_time=False)
    # ------------------------------------------------------------------ #
    balance = Node(
        package="two_wheel_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[{"use_sim_time": False}],
        output="screen",
        respawn=True,
        respawn_delay=0.1,
    )

    # ------------------------------------------------------------------ #
    # 7. snap_upright  (3.5 s after effort exits)
    # ------------------------------------------------------------------ #
    snap_upright = Node(
        package="two_wheel_robot",
        executable="snap_upright.py",
        name="snap_upright",
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Event-handler sequencing: spawn -> jsb -> effort -> balance + snap
    # ------------------------------------------------------------------ #
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
            ],
        )
    )

    # ------------------------------------------------------------------ #
    # 8. odom_publisher  (use_sim_time=True)
    # ------------------------------------------------------------------ #
    odom_publisher = Node(
        package="two_wheel_robot",
        executable="odom_publisher.py",
        name="odom_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 9. slam_toolbox  (use_sim_time=True)
    # ------------------------------------------------------------------ #
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": True}],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 10. Nav2 nodes
    # ------------------------------------------------------------------ #
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[nav2_params],
        output="screen",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[nav2_params],
        output="screen",
    )

    recoveries_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_params],
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[nav2_params],
        output="screen",
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        parameters=[nav2_params],
        output="screen",
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[nav2_params],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 11. Nav2 lifecycle manager
    # ------------------------------------------------------------------ #
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        }],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # 12. RViz2 with nav2_default_view.rviz
    # ------------------------------------------------------------------ #
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Assemble LaunchDescription
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_share, "worlds", "house.world"),
            description="Path to Gazebo world file",
        ),
        # Simulation
        gazebo,
        rsp,
        spawn,
        after_spawn,
        after_jsb,
        after_effort,
        # Navigation stack
        odom_publisher,
        slam_toolbox,
        controller_server,
        planner_server,
        recoveries_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        # Visualization
        rviz,
    ])
