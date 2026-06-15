"""Sloped-road launch — same paused-start approach as road.launch.py.

World: worlds/road_slope.world
  x 0-6   flat
  x 6-12  ramp, 8 deg incline (rise 0.843 m)
  x 12-18 elevated plateau

The robot spawns on the flat section, balances, then drives up the ramp
and across the plateau.  The slope-capable balance controller learns the
gravity feed-forward torque on the way up (watch it on /slope_estimate).

Sequence (identical to road.launch.py):
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
    world_path = os.path.join(pkg_share, "worlds", "road_slope.world")

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

    # ── 6. balance_controller (slope-capable) ────────────────────────────
    balance = Node(
        package="two_wheel_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[{
            "use_sim_time": False,
            # Slope compensation (defaults shown explicitly for easy tuning)
            # ki_tau 12 builds the 1.36 N·m holding torque in ~1.4 s.  (Lower
            # values let the robot oscillate more through the transition.)
            "ki_tau":      12.0,
            # max_tau_ff 2.5→4.0: with vel_integral now bounded (no windup
            # instability) the feed-forward can safely supply the climbing
            # torque.  At 2.5 the net wheel torque (~1.06 N·m, after the small
            # fixed pitch_err cancellation) sat just BELOW the 1.36 N·m holding
            # torque, so the robot held station on the ramp but couldn't climb.
            # 4.0 gives net ~2.5 N·m → climbs; ff-decay still sheds it at the
            # crest so it doesn't over-thrust onto the plateau.
            "max_tau_ff":   4.0,
            "max_lean":     0.25,   # rad — headroom for the ramp-entry transient
            # kp_vel halved 0.25→0.10: the previous run engaged the ramp, held
            # without runaway, and briefly climbed — then tipped because when
            # the wheels broke free the robot overspeed and the outer velocity
            # loop slammed target_pitch negative, oscillating into a tip.  A
            # gentler outer loop lets the gravity feed-forward (not the lean
            # demand) carry the climb, damping that post-break-free overshoot.
            "kp_vel":       0.10,
            # ki_vel kept at 0.06 (winds target_pitch to lean into the climb —
            # required to climb), but the integral is hard-clamped (below) so
            # it supplies the steady climbing lean WITHOUT the unbounded windup
            # that pushed target_pitch past the trackable pitch and stalled the
            # climb.
            "ki_vel":       0.06,
            # max_vel_integral 0.5: caps the climbing lean at ~ki_vel*0.5 =
            # 0.03 rad (+ the kp_vel term) — enough to climb, bounded so it
            # can't over-wind and cancel the feed-forward.
            "max_vel_integral": 0.5,
            # max_torque kept at 8.0: a 3.5 cap (near the static traction
            # limit) starved the balance loop of authority at the crest and
            # caused a tff-thrashing limit cycle, and the wheels slip anyway
            # once they unload during a tip — so the cap traded the stop-slip
            # for worse crest behaviour.  The crest overspeed is instead
            # reduced at its source by rounding the x=12 convex corner.
            "max_torque":   8.0,
            "safety_pitch": 0.70,   # ~40 deg; enough headroom after the physics fix
        }],
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

    # Wheel-path distance start→stop: 5.2 m flat + 6.06 m ramp + ~4.2 m
    # plateau ≈ 15.5 m (odom integrates wheel arc length, i.e. distance
    # along the road surface, so the same road_length works on the slope).
    road_driver = Node(
        package="two_wheel_robot",
        executable="road_drive.py",
        name="road_driver",
        parameters=[{
            "use_sim_time": False,
            "road_length":  15.5,
            "linear_vel":    0.08,   # proven stable on the flat; 0.15 was too aggressive and tipped at drive onset
            # decel_zone 2.0: a longer 4.0 zone reached into the upper ramp and
            # made the robot decelerate while still climbing → oscillation/tip.
            # The crest→plateau overspeed-at-stop is a separate issue needing
            # slope-aware feed-forward decay (see memory).
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
