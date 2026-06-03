#!/usr/bin/env python3
"""Reset robot to upright spawn pose with a clean controller slate.

Sequence
--------
1. pause_physics         - stop ODE so no forces are applied during the reset
2. Zero effort_controllers commands - the killed controller may not have
   published a final 0.0 (SIGTERM skips Python finally blocks), so the
   JointGroupEffortController keeps the last stale torque.  Override it
   explicitly here while physics is still paused.
3. Kill balance_controller - launch's respawn=True will restart it in 0.1 s
4. reset_simulation       - snap all bodies back to spawn pose, zero ODE
   joint velocities (the only reliable way to zero wheel spin)
5. Sleep 0.20 s           - wait for the new balance_controller to boot and
   publish its first watchdog-0 command (so effort_controllers is at 0
   before physics resumes)
6. unpause_physics        - robot is upright, both controllers output 0,
   physics resumes; balance_controller catches it within one IMU cycle
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty


def call_empty(node, client, svc_name):
    client.wait_for_service(timeout_sec=10.0)
    fut = client.call_async(Empty.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    node.get_logger().info(f"{svc_name} OK")


def main():
    rclpy.init()
    node = Node("snap_upright")

    pause_cli   = node.create_client(Empty, "/pause_physics")
    reset_cli   = node.create_client(Empty, "/reset_simulation")
    unpause_cli = node.create_client(Empty, "/unpause_physics")
    cmd_pub     = node.create_publisher(Float64MultiArray,
                                        "/effort_controllers/commands", 10)

    node.get_logger().info("Waiting for Gazebo physics services …")
    pause_cli.wait_for_service(timeout_sec=30.0)

    # 1. Pause
    call_empty(node, pause_cli, "pause_physics")

    # 2. Zero torques before killing balance_controller (SIGTERM doesn't run
    #    Python finally blocks, so the old controller may leave stale torques)
    zero = Float64MultiArray()
    zero.data = [0.0, 0.0]
    for _ in range(5):          # publish a few times to ensure delivery
        cmd_pub.publish(zero)
        rclpy.spin_once(node, timeout_sec=0.02)

    # 3. Kill balance_controller → launch respawn=True restarts it in 0.1 s
    try:
        result = subprocess.run(["pgrep", "-f", "balance_controller.py"],
                                capture_output=True, text=True)
        for pid in result.stdout.strip().split():
            os.kill(int(pid), signal.SIGTERM)
        node.get_logger().info(f"Killed PIDs: {result.stdout.strip()}")
    except Exception as e:
        node.get_logger().warning(f"kill: {e}")

    # 4. Reset all bodies + joint velocities to spawn pose
    call_empty(node, reset_cli, "reset_simulation")

    # 5. Wait for the new balance_controller to respawn and publish 0 torque
    node.get_logger().info("Waiting 0.25 s for balance_controller respawn …")
    time.sleep(0.25)

    # 6. Resume — robot is upright, torque is 0, controller is ready
    call_empty(node, unpause_cli, "unpause_physics")

    node.get_logger().info("Snap done.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
