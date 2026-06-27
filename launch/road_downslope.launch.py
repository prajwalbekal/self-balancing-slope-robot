"""Down-sloped-road launch — DESCENDS an 8 deg decline.

World: worlds/road_downslope.world
  x 0-6    flat (top, z = 0)
  x 6-12   decline, 8 deg downhill (drop 0.843 m)
  x 12-18  lower flat (z = -0.843)

The robot spawns on the upper flat, balances, drives over the brow at x=6
and descends the decline.  Unlike the uphill course, gravity now ASSISTS
forward motion, so the controller must HOLD THE ROBOT BACK while staying
gravity-vertical (the gyro/IMU keeps "upright" defined by gravity, so it
self-balances on the incline).

Downhill controller note — the ONE difference from road_slope.launch.py:
  The slope-capable controller has a "feed-forward decay" that bleeds the
  gravity feed-forward torque toward zero whenever the robot OVERSPEEDS
  (v_err < -ff_decay_v).  That was added for the uphill->plateau crest, where
  overspeed meant "you've crested onto flat ground, drop the climbing torque."
  On a DESCENT, overspeed is the *normal* condition (gravity pulls you down),
  and we WANT the feed-forward integrator to build a steady NEGATIVE (braking)
  torque.  The integrator itself is sign-free (tau_ff += ki_tau*v_err*dt, so
  v_err<0 winds it negative); we only need to stop the decay branch from
  erasing it.  Setting ff_decay_v very high (10 m/s, never reached) disables
  the decay so the feed-forward holds the robot back against gravity.  On the
  lower flat the same integrator self-corrects tau_ff back toward zero.

Sequence (identical to road_slope.launch.py):
  1.  Gazebo started paused
  2.  robot_state_publisher
  3.  spawn_entity (paused)
  4.  joint_state_broadcaster spawner
  5.  effort_controllers spawner
  6.  balance_controller (use_sim_time=False)
  7.  2 s later: unpause_physics
  8.  5 s later: odom_publisher + road_driver

STATUS / KNOWN LIMITATION:
  SLOW DESCENT — DONE.  With the decoupled brake feed-forward (max_v_est_ff)
  and the gentle 2 m foot fillet, the robot descends the 8 deg decline at a
  steady, controllable ~0.10 m/s, staying upright (pitch ~0).  This is the
  achieved goal.

  STOP-AND-HOLD AT THE BOTTOM — FUTURE WORK.  The robot does not yet come to
  a clean, held stop at the foot.  This controller is VELOCITY-only: it has
  no position setpoint, and on a downslope the stiff balance loop cancels any
  net braking torque (it can hold a speed but cannot drive the speed below the
  gravity-set equilibrium).  So it reaches the foot still carrying momentum,
  the wheels break traction at the transition, and it pitches forward.
  Tuning (max_torque, linear_vel, the foot fillet, the stop clamp) does NOT
  fix this — it needs a POSITION / station-keeping outer loop that decelerates
  to a target stop-point and holds it (arriving at true v=0, no momentum to
  slip on).  That is a controller change, not a parameter, and is left as the
  next task.
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
    world_path = os.path.join(pkg_share, "worlds", "road_downslope.world")

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
            "paused":  "true",
            "verbose": "false",
            "gui":     LaunchConfiguration("gui"),   # gui:=false for headless runs
        }.items(),
    )

    # ── 2. robot_state_publisher ──────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{**robot_description, "use_sim_time": use_sim_time}],
    )

    # ── 3. Spawn robot just past the start line, on the upper flat, +X ────
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

    # ── 6. balance_controller (slope-capable, tuned for DESCENT) ──────────
    balance = Node(
        package="two_wheel_robot",
        executable="balance_controller.py",
        name="balance_controller",
        parameters=[{
            "use_sim_time": False,
            # ---- Traction-limited descent (kills the wheel-slip runaway) ----
            # At the brow a brief overspeed drove target_pitch negative; the
            # stiff inner loop saturated torque to +8 N·m, the wheels SLIPPED
            # (spun to ~400 rad/s = a false 30 m/s), which pinned verr and kept
            # torque saturated → forward runaway → backward tip.  Three changes
            # keep the wheels gripping:
            #   max_torque 2.5  — the measured wheel traction budget is only
            #     mu*(m*g/2)*r = 0.8*(12*9.81/2)*0.075 ≈ 3.5 N·m per wheel, so a
            #     3.5 cap sat right AT the limit and the wheels slipped whenever
            #     weight transfer unloaded a wheel (seen as a pinned slip-spin at
            #     the stop -> backward tip).  2.5 (~70% of traction) keeps a
            #     grip margin so the wheels never break loose; still ample
            #     authority (holding 8° needs ~0.7 N·m/wheel, and a 0.1 rad
            #     balance error needs ~1.7 N·m).
            "max_torque":   2.5,
            #   kp_pitch/kd_pitch softened (120/15 → 70/12) — a 0.1 rad pitch
            #     error no longer instantly saturates torque, so the loop rides
            #     the descent instead of slamming the wheels.
            "kp_pitch":    70.0,
            "kd_pitch":    12.0,
            #   vel_accel_limit 2.0 (from 5.0) — the slip-robust speed estimate
            #     slews slower, so wheel slip can't fool it into a huge verr
            #     (real descent acceleration is small, so true motion is kept).
            "vel_accel_limit": 2.0,
            #   max_v_est 0.6 — ceiling on the speed estimate.  Too tight (0.15)
            #     BLINDED the controller: the robot really accelerates to ~1.8
            #     m/s at the foot transition, but a 0.15 clamp kept v_est reading
            #     0.15 so the loop never braked the residual speed and slowly
            #     diverged to a tip on the flat.  0.6 lets the loop SEE and brake
            #     the foot/flat speed (negative v_err -> lean back -> decelerate)
            #     while still rejecting the absurd >2 m/s readings of any slip.
            #     Slip itself is prevented by the sub-traction max_torque cap.
            #     0.15 while driving keeps the descent stable; opened at the stop
            #     (max_v_est_stop) so the halt can brake the real residual speed.
            #
            #     *** "KEEP IT SLOW FROM THE BROW" ***  The LEAN loop stays blind
            #     at 0.15 on purpose: a larger view makes it command a backward
            #     lean that the inner loop achieves by first driving the wheels
            #     FORWARD (non-minimum-phase) — on a decline that accelerates the
            #     robot, and once it is already fast the back-lean correction goes
            #     violently unstable (±25° pitch swings observed at 0.25-0.45).
            #     So we do NOT use the lean loop to brake the descent.  Instead the
            #     BRAKE feed-forward gets its OWN loose view (max_v_est_ff) and
            #     holds the descent at its low entry speed from the brow onward —
            #     it cancels gravity before the robot can build speed, so the lean
            #     loop never has to deal with a fast robot.
            "max_v_est":      0.15,
            # Loose ceiling for the brake feed-forward: it sees the true descent
            # overspeed (up to 1.0 m/s) and winds the gravity-cancelling torque in
            # under a second at the brow — fast enough to stop the robot building
            # speed — while still rejecting the >1 m/s readings a real wheel slip
            # would produce.  This is the key change that holds the descent slow.
            "max_v_est_ff":   1.0,
            "max_v_est_stop": 2.0,
            # Gravity feed-forward integrator: on the descent v_err goes
            # persistently negative (gravity speeds the robot up), so this winds
            # tau_ff NEGATIVE to a steady braking torque (~-1.36 N·m holds 8°).
            # ki_tau GENTLE (5, not the uphill 12): a fast integrator slammed
            # tau_ff to its -4 N·m clamp the instant the robot overspeed at the
            # brow, and that max braking + the velocity-loop lean over-corrected
            # into a BACKWARD tip.  A slow build lets braking come on smoothly.
            # ki_tau 5 -> 6: now that the brake sees the REAL overspeed
            # (max_v_est_ff) its v_err_ff is large at the brow, so even a modest
            # gain winds the holding/braking torque in <1 s — fast enough to catch
            # the brow before gravity runs the robot away.
            "ki_tau":       6.0,
            # max_tau_ff 2.0 -> 2.5: holding 8° needs ~1.36 N·m, but the brake must
            # briefly exceed that to DECELERATE a transient overspeed back to the
            # setpoint; 2.5 gives margin while the ff_decay gate still bleeds it to
            # zero at the stop so it can't tip the halted robot.
            "max_tau_ff":   2.5,
            # *** Downhill feed-forward management ***
            # The braking feed-forward must BUILD while descending but RELEASE
            # when the robot is told to stop (else the wound-up -2 N·m brake
            # pitches the halted robot over backward at the bottom).  Gate the
            # decay on commanded speed: while driving down (cmd=0.05) the decay
            # is inhibited so tau_ff builds the braking torque; as the driver
            # ramps cmd to 0 to stop, |cmd| drops below the gate and the
            # overspeed decay bleeds tau_ff back toward zero for a clean halt.
            "ff_decay_cmd_gate": 0.03,
            "ff_decay":          3.0,    # 1/s bleed rate once stopping
            "ff_decay_v":        0.05,
            # Release the velocity integral when commanded to stop: during the
            # descent v_err stays negative and winds vel_integral to its clamp;
            # left wound after the halt it holds a forward-lean demand that drives
            # the wheels (slip) and tips the stopped robot.  Bleed it to zero.
            "stop_cmd_gate":     0.03,
            "vint_stop_decay":   4.0,
            # max_lean 0.20: enough lean-back authority to brake the foot/flat
            # residual speed, but still well short of a tip-over demand.
            "max_lean":     0.20,
            # Outer loop sized for the brow entry: too weak (0.03) and the robot
            # pitches forward off the brow; these values manage the brow AND,
            # with the tight max_v_est clamp limiting v_err on the decline, don't
            # over-demand a lean that cancels the tff brake.
            "kp_vel":       0.06,
            "ki_vel":       0.06,
            "max_vel_integral": 0.5,
            "safety_pitch": 0.70,   # ~40 deg tip-over threshold
            # ---- Inertial sensing ----
            # The GYROSCOPE is always in use: imu.angular_velocity.y feeds the
            # inner-loop derivative (kd_pitch) damping every cycle — it's what
            # keeps the balance critically damped.
            # The ACCELEROMETER fusion (complementary filter) is left OFF for now:
            # enabling it destabilised this controller (the stiff inner loop turns
            # the accel's specific-force bias into a limit cycle), so pitch stays
            # on the reliable Gazebo orientation source.  Re-enabling accel fusion
            # needs a softer inner loop + higher cf_alpha and is a separate effort.
            "use_complementary_filter": False,
            "cf_alpha":     0.98,
        }],
        output="screen",
    )

    # ── 7. Unpause physics (2 s after balance starts) ─────────────────────
    unpause = ExecuteProcess(
        cmd=["ros2", "service", "call",
             "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    # ── 8. odom + road driver (5 s after balance) ─────────────────────────
    odom_publisher = Node(
        package="two_wheel_robot",
        executable="odom_publisher.py",
        name="odom_publisher",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    # Wheel-path distance start→stop: 5.2 m upper flat + 6.06 m decline +
    # ~4.2 m lower flat ≈ 15.5 m (odom integrates wheel arc length, i.e.
    # distance along the road surface, so the same road_length works).
    road_driver = Node(
        package="two_wheel_robot",
        executable="road_drive.py",
        name="road_driver",
        parameters=[{
            "use_sim_time": False,
            "road_length":  15.5,
            # 0.05 m/s — a low descent command.  The decoupled brake feed-forward
            # (max_v_est_ff) holds the robot at this entry speed all the way down
            # the decline ("keep it slow from the brow"), so the downhill speed
            # stays close to the flat cruise instead of running away under gravity.
            "linear_vel":    0.05,
            "decel_zone":    2.0,
            "heading_kp":    0.4,
            "start_delay":   3.0,
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
        DeclareLaunchArgument("gui", default_value="true"),
        gazebo,
        rsp,
        spawn,
        after_spawn,
        after_jsb,
        after_effort,
    ])
