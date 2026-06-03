#!/usr/bin/env python3
"""Self-balancing cascade-PD controller for the two-wheel robot.

Improvements over the first cut:
  - Conditional anti-windup (only integrate when output is not saturated).
  - Active damping on tip-over (apply brake torque proportional to wheel
    velocity instead of just releasing -- keeps a fallen robot from sliding).
  - Watchdog on IMU / joint_state staleness -- fail-safe stop if sensor data
    goes silent for > 100 ms.
  - Explicit pitch_sign parameter so flipping IMU mounting doesn't require
    flipping every PD gain.
  - Publishes to /effort_controllers/commands (Float64MultiArray) which is
    the canonical ros2_control entry point on Humble.

Cascade architecture:
    v_err        = cmd_linear - v_robot
    target_pitch = clip( Kp_v * v_err + Ki_v * (integral with anti-windup), +/- max_lean )
    pitch_err    = pitch * pitch_sign - target_pitch
    tau_balance  = Kp_p * pitch_err + Kd_p * (pitch_rate * pitch_sign)
    tau_yaw      = K_yaw * (cmd_angular - yaw_rate)
    tau_left     = clip(tau_balance - tau_yaw, +/- max_torque)
    tau_right    = clip(tau_balance + tau_yaw, +/- max_torque)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quat_to_pitch(qw, qx, qy, qz):
    """Pitch (rotation about Y) in [-π, π] from a unit quaternion.

    Uses atan2(sinp, cosp) so the result is correct even past ±90°.
    asin-only approaches alias: a robot lying flat (pitch=180°) reads as
    pitch≈0, causing the controller to falsely exit the tipped state.
    """
    sinp = 2.0 * (qw * qy - qz * qx)
    cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.atan2(sinp, cosp)


class BalanceController(Node):

    def __init__(self):
        super().__init__("balance_controller")

        # ------------- Parameters -------------
        # Pitch loop (inner)
        # Iyy ≈ m*(sx²+sz²)/12 = 12.11*(0.50²+0.455²)/12 ≈ 0.46 kg·m²
        # gravity destabilising ≈ m*g*L = 12.11*9.81*0.293 ≈ 34.8 N·m/rad
        # kp=120 → ωn≈sqrt((120-34.8)/0.46)≈13.6 rad/s; kd=15 → ζ≈1.2
        self.declare_parameter("kp_pitch",       120.0)
        self.declare_parameter("kd_pitch",        15.0)
        # Velocity loop (outer)
        self.declare_parameter("kp_vel",           0.25)
        self.declare_parameter("ki_vel",           0.06)
        # Yaw loop
        self.declare_parameter("k_yaw",            0.35)
        # Robot geometry
        self.declare_parameter("wheel_radius",     0.075)
        self.declare_parameter("wheel_separation", 0.345)
        # Calibration / convention
        # cart-pole Lagrangian: for a backward lean (positive pitch),
        # forward wheel torque (positive) stabilises. pitch_sign=+1.
        self.declare_parameter("pitch_sign",       1.0)
        self.declare_parameter("pitch_offset",     0.0)    # IMU mounting bias
        # Saturation
        self.declare_parameter("max_lean",         0.20)   # rad  (~11.5 deg)
        # 8.0 N·m keeps gravity dominated up to ~25° lean; effort interface allows 10 N·m
        self.declare_parameter("max_torque",       8.0)    # N*m per wheel  (was 3.0 – too low)
        self.declare_parameter("safety_pitch",     0.60)   # rad  (~34 deg) tip-over threshold
        self.declare_parameter("recovery_pitch",   0.20)   # rad  needed to re-arm after tip
        # Loop / watchdog
        self.declare_parameter("loop_hz",        200.0)
        # 0.5 s gives IMU/joint_states time to start publishing before the
        # watchdog zeros out the torque commands at boot
        self.declare_parameter("sensor_timeout_s", 0.5)
        # Tip-over braking
        self.declare_parameter("brake_kv",        10.00)   # tau = -brake_kv * wheel_velocity when tipped
        # Joint names (must match URDF)
        self.declare_parameter("left_joint_name",  "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        # Complementary filter (gyro + accelerometer fusion)
        # Set use_complementary_filter=true for real hardware where the IMU
        # orientation quaternion is not available directly.  In Gazebo the
        # quaternion is ground-truth so the filter is off by default.
        # alpha=0.98 → gyro time-constant ≈ 49 × dt ≈ 0.25 s at 200 Hz,
        # meaning accelerometer drift correction kicks in over ~quarter-second.
        self.declare_parameter("use_complementary_filter", False)
        self.declare_parameter("cf_alpha", 0.98)

        # ------------- State -------------
        self.pitch             = 0.0
        self.pitch_rate        = 0.0
        self.wheel_vel_l       = 0.0
        self.wheel_vel_r       = 0.0
        self.cmd_linear        = 0.0
        self.cmd_angular       = 0.0
        self.vel_integral      = 0.0
        self.last_imu_t        = 0.0
        self.last_joints_t     = 0.0
        self.tipped            = False
        # Complementary filter state
        self._cf_pitch         = 0.0     # running estimate
        self._cf_last_stamp_s  = -1.0    # sim time of last IMU message
        # After recovering from tip, skip the velocity outer-loop for this
        # many control cycles so spinning wheels can't immediately cause a
        # second tip via a large velocity-error demand.
        self.recovery_cooldown = 0
        self._debug_countdown  = 0

        # ------------- I/O -------------
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu,        "imu",          self._on_imu,    be)
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self.create_subscription(Twist,      "cmd_vel",      self._on_cmd,    10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/effort_controllers/commands", 10)

        hz = self.get_parameter("loop_hz").value
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self._loop)

        self.get_logger().info(f"BalanceController running at {hz:.0f} Hz")

    # ------------- Callbacks -------------
    def _on_imu(self, msg):
        gyro_y = msg.angular_velocity.y
        self.pitch_rate = gyro_y

        use_cf = self.get_parameter("use_complementary_filter").value
        if use_cf:
            # Complementary filter: fuses accelerometer (long-term stable, noisy)
            # with gyroscope integration (short-term accurate, drifts over time).
            #   pitch = alpha*(pitch_prev + gyro_y*dt) + (1-alpha)*pitch_accel
            #
            # Accelerometer pitch derivation (body frame, gravity = +Z when upright):
            #   ax = -g*sin(θ),  az = g*cos(θ)  →  θ = atan2(-ax, az)
            alpha  = self.get_parameter("cf_alpha").value
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            ax = msg.linear_acceleration.x
            az = msg.linear_acceleration.z
            pitch_accel = math.atan2(-ax, az)

            if self._cf_last_stamp_s > 0:
                imu_dt = stamp_s - self._cf_last_stamp_s
                if 0.0 < imu_dt < 0.1:   # reject bad dt (e.g. after reset_simulation)
                    self._cf_pitch = (alpha * (self._cf_pitch + gyro_y * imu_dt)
                                      + (1.0 - alpha) * pitch_accel)
                else:
                    self._cf_pitch = pitch_accel   # re-initialise on discontinuity
            else:
                self._cf_pitch = pitch_accel       # first message: boot from accel

            self._cf_last_stamp_s = stamp_s
            self.pitch = self._cf_pitch
        else:
            q = msg.orientation
            self.pitch = quat_to_pitch(q.w, q.x, q.y, q.z)

        self.last_imu_t = time.monotonic()

    def _on_joints(self, msg):
        lname = self.get_parameter("left_joint_name").value
        rname = self.get_parameter("right_joint_name").value
        for n, v in zip(msg.name, msg.velocity):
            if n == lname:
                self.wheel_vel_l = v
            elif n == rname:
                self.wheel_vel_r = v
        self.last_joints_t = time.monotonic()

    def _on_cmd(self, msg):
        self.cmd_linear  = msg.linear.x
        self.cmd_angular = msg.angular.z

    # ------------- Output helpers -------------
    def _publish(self, tau_left, tau_right):
        m = Float64MultiArray()
        m.data = [float(tau_left), float(tau_right)]
        self.cmd_pub.publish(m)

    # ------------- Main control loop -------------
    def _loop(self):
        now = time.monotonic()
        timeout = self.get_parameter("sensor_timeout_s").value

        # Watchdog: refuse to act on stale data
        if (self.last_imu_t == 0.0 or self.last_joints_t == 0.0 or
                now - self.last_imu_t > timeout or
                now - self.last_joints_t > timeout):
            self._publish(0.0, 0.0)
            return

        # Snapshot parameters
        gp = self.get_parameter
        kp_p     = gp("kp_pitch").value
        kd_p     = gp("kd_pitch").value
        kp_v     = gp("kp_vel").value
        ki_v     = gp("ki_vel").value
        k_yaw    = gp("k_yaw").value
        r        = gp("wheel_radius").value
        ws       = gp("wheel_separation").value
        psign    = gp("pitch_sign").value
        offset   = gp("pitch_offset").value
        max_lean = gp("max_lean").value
        max_tau  = gp("max_torque").value
        safe_p   = gp("safety_pitch").value
        recov_p  = gp("recovery_pitch").value
        brake_kv = gp("brake_kv").value

        # Sign-corrected, bias-corrected pitch
        pitch = psign * self.pitch - offset
        pitch_rate = psign * self.pitch_rate

        # ----- Tip-over handling -----
        if not self.tipped and abs(pitch) > safe_p:
            self.tipped = True
            self.vel_integral = 0.0
            self.get_logger().warn(f"Tipped (pitch={pitch:+.2f} rad); braking")

        if self.tipped:
            # Active brake: damp wheel velocity to zero so the robot stops sliding
            tau_l = clamp(-brake_kv * self.wheel_vel_l, -max_tau, max_tau)
            tau_r = clamp(-brake_kv * self.wheel_vel_r, -max_tau, max_tau)
            self._publish(tau_l, tau_r)
            # Re-arm on pitch alone so an external set_entity_state teleport
            # can resume balance even while wheels are still spinning.
            if abs(pitch) < recov_p:
                self.tipped = False
                self.vel_integral = 0.0
                self.recovery_cooldown = 40
                self._debug_countdown = 20  # log first 20 cycles after recovery
                self.get_logger().info("Recovered; resuming balance")
            return

        # Debug logging for first N cycles after recovery
        if self._debug_countdown > 0:
            self._debug_countdown -= 1
            self.get_logger().info(
                f"[dbg] pitch={pitch:+.4f} wvel_l={self.wheel_vel_l:+.2f} "
                f"wvel_r={self.wheel_vel_r:+.2f} cd={self.recovery_cooldown}")

        # ----- Forward velocity and yaw rate from wheel encoders -----
        v_robot  = 0.5 * r * (self.wheel_vel_l + self.wheel_vel_r)
        yaw_rate = r * (self.wheel_vel_r - self.wheel_vel_l) / ws

        # ----- Outer loop: velocity error -> target pitch -----
        # Skip for the first N cycles after tip recovery so spinning wheels
        # cannot cause an immediate second tip via a large velocity demand.
        if self.recovery_cooldown > 0:
            self.recovery_cooldown -= 1
            target_pitch = 0.0
            pitch_err  = pitch - target_pitch
            tau_balance = kp_p * pitch_err + kd_p * pitch_rate
            tau_yaw     = k_yaw * (self.cmd_angular - yaw_rate)
            tau_l = clamp(tau_balance - tau_yaw, -max_tau, max_tau)
            tau_r = clamp(tau_balance + tau_yaw, -max_tau, max_tau)
            self._publish(tau_l, tau_r)
            return

        v_err = self.cmd_linear - v_robot

        # Pre-integration to compute candidate output; only commit integration
        # if the resulting target_pitch is NOT saturated (conditional anti-windup)
        candidate_integral = self.vel_integral + v_err * self.dt
        candidate_pitch = kp_v * v_err + ki_v * candidate_integral
        if -max_lean < candidate_pitch < max_lean:
            self.vel_integral = candidate_integral

        target_pitch = clamp(kp_v * v_err + ki_v * self.vel_integral,
                             -max_lean, max_lean)

        # ----- Inner loop: pitch error -> balancing torque -----
        pitch_err  = pitch - target_pitch
        tau_balance = kp_p * pitch_err + kd_p * pitch_rate

        # ----- Yaw mixing -----
        yaw_err = self.cmd_angular - yaw_rate
        tau_yaw = k_yaw * yaw_err

        tau_l = clamp(tau_balance - tau_yaw, -max_tau, max_tau)
        tau_r = clamp(tau_balance + tau_yaw, -max_tau, max_tau)
        self._publish(tau_l, tau_r)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
