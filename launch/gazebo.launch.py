"""Full simulation: Gazebo Classic + robot_state_publisher + ros2_control
spawners + balance controller.

Sequence:
  1. Gazebo Classic  -- running (controllers cannot activate while paused)
  2. robot_state_publisher
  3. spawn_entity.py  -- robot placed at z=0.001; may tip before controllers load
  4. joint_state_broadcaster spawner
  5. effort_controllers spawner
  6. balance_controller node
  7. set_model_state (1 s after balance_controller launches) -- snaps robot upright
     so the already-running balance controller catches it immediately
"""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, ExecuteProcess, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share      = get_package_share_directory("two_wheel_robot")
    xacro_path     = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    world_path     = os.path.join(pkg_share, "worlds", "empty.world")
    rviz_cfg       = os.path.join(pkg_share, "rviz", "view.rviz")
    gazebo_share   = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_z      = LaunchConfiguration("spawn_z")
    open_rviz    = LaunchConfiguration("open_rviz")

    ros2_control_plugin = os.path.join(
        get_package_prefix("gazebo_ros2_control"), "lib",
        "libgazebo_ros2_control.so"
    )

    robot_description_str = subprocess.check_output(
        ["xacro", xacro_path, "use_sim:=true",
         f"ros2_control_plugin:={ros2_control_plugin}"]
    ).decode()
    robot_description = {"robot_description": robot_description_str}

    # ---------- 1. Gazebo Classic (running -- controllers activate fine) ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_path, "verbose": "false"}.items(),
    )

    # ---------- 2. robot_state_publisher ----------
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    # ---------- 3. Spawn robot ----------
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "two_wheel_robot",
            "-x", "0.0", "-y", "0.0", "-z", spawn_z,
        ],
        output="screen",
    )

    # ---------- 4. joint_state_broadcaster ----------
    spawn_jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    # ---------- 5. effort_controllers ----------
    spawn_effort = Node(
        package="controller_manager", executable="spawner",
        arguments=["effort_controllers",
                   "--controller-manager", "/controller_manager"],
    )

    # ---------- 6. Balance controller ----------
    # respawn=True: if reset_simulation sends the clock backward the node
    # crashes; auto-respawn gives a fresh instance that catches the robot.
    balance = Node(
        package="two_wheel_robot", executable="balance_controller.py",
        name="balance_controller",
        # use_sim_time=False so reset_simulation doesn't stall the 200 Hz timer:
        # when Gazebo resets the clock to 0 the timer would otherwise not fire
        # again until sim time caught back up (~2 s blackout with no torque output).
        # Wall-clock timing is fine here — the IMU watchdog already uses monotonic().
        parameters=[{"use_sim_time": False}],
        output="screen",
        respawn=True,
        respawn_delay=0.1,
    )

    # ---------- 7. Snap robot upright THEN start balance controller ----------
    # snap_upright.py resets body + both wheel links to zero velocity so the
    # spinning wheels cannot re-tip the body as soon as it is placed upright.
    # balance_controller starts only after this script exits successfully.
    snap_upright = Node(
        package="two_wheel_robot", executable="snap_upright.py",
        name="snap_upright", output="screen",
    )

    # Ordering: spawn -> jsb -> effort -> balance (immediately) + snap 3.5 s later
    # balance_controller starts FIRST so it is already running when snap_upright
    # fires.  snap_upright resets the physics with the controller live, so the
    # controller catches the upright robot in the same control cycle as unpause.
    after_spawn   = RegisterEventHandler(
        OnProcessExit(target_action=spawn,        on_exit=[spawn_jsb]))
    after_jsb     = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb,    on_exit=[spawn_effort]))
    after_effect  = RegisterEventHandler(
        OnProcessExit(target_action=spawn_effort,
                      on_exit=[balance,
                                TimerAction(period=3.5, actions=[snap_upright])]))
    after_snap    = None   # kept for LaunchDescription compatibility

    # ---------- Optional RViz ----------
    from launch.conditions import IfCondition
    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("spawn_z",      default_value="0.001",
            description="Initial spawn height (m). Robot may tip before controllers load; "
                        "set_model_state resets it upright once the balance controller is live."),
        DeclareLaunchArgument("open_rviz",    default_value="false"),
        gazebo,
        rsp,
        spawn,
        after_spawn,
        after_jsb,
        after_effect,
        rviz,
    ])
