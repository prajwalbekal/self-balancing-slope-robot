#!/usr/bin/env python3
"""Wheel-encoder odometry for the self-balancing robot.

Subscribes: /joint_states (sensor_msgs/JointState)
Publishes:  /odom       (nav_msgs/Odometry)
Broadcasts: odom -> base_footprint TF
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")
        self.declare_parameter("wheel_radius",     0.075)
        self.declare_parameter("wheel_separation", 0.345)
        self.declare_parameter("left_joint",  "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")
        self.declare_parameter("odom_frame",  "odom")
        self.declare_parameter("base_frame",  "base_footprint")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._last_t = None

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(JointState, "joint_states", self._on_joints, be)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._tf_br   = TransformBroadcaster(self)

    def _on_joints(self, msg: JointState):
        r  = self.get_parameter("wheel_radius").value
        ws = self.get_parameter("wheel_separation").value
        lj = self.get_parameter("left_joint").value
        rj = self.get_parameter("right_joint").value

        vl = vr = None
        for name, vel in zip(msg.name, msg.velocity):
            if name == lj: vl = vel
            if name == rj: vr = vel
        if vl is None or vr is None:
            return

        now = self.get_clock().now()
        if self._last_t is None:
            self._last_t = now
            return
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            return

        v     = 0.5 * r * (vl + vr)
        omega = r * (vr - vl) / ws

        self.theta += omega * dt
        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt

        oframe = self.get_parameter("odom_frame").value
        bframe = self.get_parameter("base_frame").value
        stamp  = now.to_msg()
        q      = yaw_to_quat(self.theta)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = oframe
        odom.child_frame_id  = bframe
        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        self._odom_pub.publish(odom)

        # TF
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = oframe
        tf.child_frame_id  = bframe
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation      = q
        self._tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
