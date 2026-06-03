#!/usr/bin/env python3
"""Square-path autonomous follower for the self-balancing robot.

The robot starts near corner A (Red, world origin) and drives one full
lap of a 3 m × 3 m square, returning to the start:

    D (Blue,  0,3) ←——— C (Green, 3,3)
    |                          |
    |   leg 4 ↑    leg 2 ↓    |
    |                          |
    A (Red,   0,0) ———→ B (Yellow,3,0)
              leg 1 →    leg 3 ↑ (from C to D is leg 3, D to A is leg 4)

Uses /odom feedback (from odom_publisher.py) for accurate distance and
heading.  Sends /cmd_vel to the balance controller.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def wrap(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class SquarePathFollower(Node):

    WAIT  = 0   # waiting for initial stabilisation delay
    DRIVE = 1   # driving straight along current leg
    TURN  = 2   # turning 90° left at end of leg
    DONE  = 3   # lap complete, holding still

    def __init__(self):
        super().__init__("square_path_follower")

        self.declare_parameter("side_length",      3.0)   # m per side
        self.declare_parameter("linear_vel",       0.20)  # m/s cruise
        self.declare_parameter("angular_vel",      0.35)  # rad/s turn speed
        self.declare_parameter("dist_tolerance",   0.08)  # m  stop threshold
        self.declare_parameter("angle_tolerance",  0.03)  # rad (~1.7°)
        self.declare_parameter("decel_zone",       0.40)  # m  slow-down before stop
        self.declare_parameter("start_delay",      5.0)   # s  wait before moving

        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._odom_ready = False

        self._state       = self.WAIT
        self._leg         = 0          # 0–3
        self._leg_start_x = 0.0
        self._leg_start_y = 0.0
        # Target headings for legs 0-3: East(+X), North(+Y), West(-X), South(-Y)
        self._headings    = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        self._target_yaw  = 0.0
        self._start_time  = None

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_timer(0.05, self._loop)   # 20 Hz control loop

        self.get_logger().info(
            f"SquarePathFollower ready — waiting {self.get_parameter('start_delay').value:.0f} s "
            "for robot to stabilise before moving."
        )

    # ------------------------------------------------------------------ #

    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._odom_ready = True

    def _vel(self, linear: float = 0.0, angular: float = 0.0):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    # ------------------------------------------------------------------ #

    def _loop(self):
        if not self._odom_ready:
            return

        p = self.get_parameter
        side      = p("side_length").value
        lin_v     = p("linear_vel").value
        ang_v     = p("angular_vel").value
        dist_tol  = p("dist_tolerance").value
        ang_tol   = p("angle_tolerance").value
        decel     = p("decel_zone").value
        delay     = p("start_delay").value

        now = self.get_clock().now()

        # ---- WAIT ----
        if self._state == self.WAIT:
            if self._start_time is None:
                self._start_time = now
            elapsed = (now - self._start_time).nanoseconds * 1e-9
            if elapsed >= delay:
                self._begin_leg(0)
            return

        # ---- DONE ----
        if self._state == self.DONE:
            self._vel(0.0, 0.0)
            return

        # ---- DRIVE ----
        if self._state == self.DRIVE:
            dx   = self._x - self._leg_start_x
            dy   = self._y - self._leg_start_y
            dist = math.hypot(dx, dy)

            # Reached end of leg?
            if dist >= side - dist_tol:
                self._vel(0.0, 0.0)
                if self._leg == 3:
                    self._state = self.DONE
                    self.get_logger().info("Lap complete — back at start!")
                    return
                self._begin_turn(self._leg + 1)
                return

            # Proportional speed: slow down in decel zone
            remaining = side - dist
            if remaining < decel:
                speed = lin_v * max(0.25, remaining / decel)
            else:
                speed = lin_v

            # Gentle heading correction while driving
            yaw_err    = wrap(self._target_yaw - self._yaw)
            correction = 0.8 * yaw_err
            self._vel(speed, correction)

        # ---- TURN ----
        elif self._state == self.TURN:
            yaw_err = wrap(self._target_yaw - self._yaw)

            if abs(yaw_err) < ang_tol:
                self._vel(0.0, 0.0)
                self._begin_leg(self._leg)
                return

            # Proportional slow-down near target angle
            if abs(yaw_err) < 0.25:
                turn_speed = ang_v * max(0.25, abs(yaw_err) / 0.25)
            else:
                turn_speed = ang_v

            self._vel(0.0, math.copysign(turn_speed, yaw_err))

    # ------------------------------------------------------------------ #

    def _begin_leg(self, leg: int):
        self._leg         = leg
        self._state       = self.DRIVE
        self._target_yaw  = self._headings[leg]
        self._leg_start_x = self._x
        self._leg_start_y = self._y
        headings_name = ["East (+X)", "North (+Y)", "West (-X)", "South (-Y)"]
        self.get_logger().info(
            f"Leg {leg + 1}/4 — driving {headings_name[leg]} for "
            f"{self.get_parameter('side_length').value:.1f} m"
        )

    def _begin_turn(self, next_leg: int):
        self._state      = self.TURN
        self._target_yaw = self._headings[next_leg]
        self._leg        = next_leg
        self.get_logger().info(
            f"Turning to {math.degrees(self._target_yaw):.0f}° for leg {next_leg + 1}/4"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquarePathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
