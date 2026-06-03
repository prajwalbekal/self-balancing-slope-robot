#!/usr/bin/env python3
"""Straight-road driver for the self-balancing robot.

The robot starts at the red post (world origin, facing +X) and drives
slowly down the 18 m road to the blue end post, then stops.

Uses /odom for distance tracking and a gentle heading correction to stay
centred on the road.  Speed is deliberately slow (0.12 m/s) so the
balance controller has no trouble maintaining stability.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RoadDriver(Node):

    WAIT  = 0
    DRIVE = 1
    DONE  = 2

    def __init__(self):
        super().__init__("road_driver")

        self.declare_parameter("road_length",      16.5)  # stop 1.5 m before end post
        self.declare_parameter("linear_vel",       0.12)  # m/s — deliberately slow
        self.declare_parameter("decel_zone",        2.0)  # m — slow down in last 2 m
        self.declare_parameter("heading_kp",        0.5)  # heading correction gain
        self.declare_parameter("start_delay",       5.0)  # s — wait for robot to balance

        self._x     = 0.0
        self._y     = 0.0
        self._yaw   = 0.0
        self._odom_ready = False

        self._state      = self.WAIT
        self._start_time = None
        self._start_x    = None   # odom x when driving begins

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            f"RoadDriver ready — waiting {self.get_parameter('start_delay').value:.0f} s "
            "for robot to stabilise before driving."
        )

    def _on_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._odom_ready = True

    def _vel(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def _loop(self):
        if not self._odom_ready:
            return

        now   = self.get_clock().now()
        p     = self.get_parameter
        delay = p("start_delay").value

        if self._state == self.WAIT:
            if self._start_time is None:
                self._start_time = now
            elapsed = (now - self._start_time).nanoseconds * 1e-9
            if elapsed >= delay:
                self._start_x = self._x
                self._state   = self.DRIVE
                self.get_logger().info(
                    f"Starting drive from x={self._x:.2f} — target +{p('road_length').value:.1f} m"
                )
            return

        if self._state == self.DONE:
            self._vel(0.0, 0.0)
            return

        # ---- DRIVE ----
        dist_driven = self._x - self._start_x
        road_len    = p("road_length").value
        lin_v       = p("linear_vel").value
        decel       = p("decel_zone").value
        kp          = p("heading_kp").value

        if dist_driven >= road_len:
            self._vel(0.0, 0.0)
            self._state = self.DONE
            self.get_logger().info(
                f"Reached end of road at x={self._x:.2f} m — stopped."
            )
            return

        # Gradual deceleration in the last decel_zone metres
        remaining = road_len - dist_driven
        if remaining < decel:
            speed = lin_v * max(0.20, remaining / decel)
        else:
            speed = lin_v

        # Heading correction: target yaw = 0 (straight along +X road)
        yaw_err    = -self._yaw          # robot should keep yaw ≈ 0
        correction = kp * yaw_err
        # Clamp correction so it doesn't destabilise the balance
        correction = max(-0.30, min(0.30, correction))

        self._vel(speed, correction)


def main(args=None):
    rclpy.init(args=args)
    node = RoadDriver()
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
