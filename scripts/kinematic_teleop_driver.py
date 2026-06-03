#!/usr/bin/env python3
"""Kinematic Gazebo driver for keyboard teleop demos.

The balancing controller is still a physics/control problem. This node gives
the package a reliable teleop mode meanwhile: subscribe to /cmd_vel and move
the Gazebo model with set_entity_state so the robot can be driven manually.
"""
import math
import time

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class KinematicTeleopDriver(Node):

    def __init__(self):
        super().__init__("kinematic_teleop_driver")

        self.declare_parameter("entity_name", "two_wheel_robot")
        self.declare_parameter("update_rate", 30.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("z", 0.001)

        self.entity_name = self.get_parameter("entity_name").value
        self.command_timeout = self.get_parameter("command_timeout").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.z = self.get_parameter("z").value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.last_cmd_time = 0.0
        self.last_step_time = time.monotonic()
        self.pending = None

        self.client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.client.wait_for_service(timeout_sec=10.0)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        update_rate = self.get_parameter("update_rate").value
        self.create_timer(1.0 / update_rate, self.step)
        self.get_logger().info(
            f"Driving Gazebo entity '{self.entity_name}' from /cmd_vel"
        )

    def on_cmd_vel(self, msg):
        self.cmd_linear = clamp(
            msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.cmd_angular = clamp(
            msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_time = time.monotonic()

    def step(self):
        now = time.monotonic()
        dt = now - self.last_step_time
        self.last_step_time = now

        if self.pending is not None and not self.pending.done():
            return
        self.pending = None

        if now - self.last_cmd_time > self.command_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.cmd_linear
            angular = self.cmd_angular

        self.yaw += angular * dt
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.entity_name
        req.state.pose = Pose(
            position=Point(x=self.x, y=self.y, z=self.z),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(self.yaw * 0.5),
                w=math.cos(self.yaw * 0.5),
            ),
        )
        req.state.twist = Twist(linear=Vector3(), angular=Vector3())
        self.pending = self.client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicTeleopDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
