#!/usr/bin/env python3
"""Self-balancing cascade-PD controller for the two-wheel robot.

SLOPE-CAPABLE VERSION — additions over the flat-road controller:

  - Gravity feed-forward via a torque-level disturbance integrator
    (`ki_tau`).  On an incline, gravity applies a constant force
    m*g*sin(slope) along the road.  The original controller could only
    cancel it indirectly (velocity integral -> extra lean -> pitch error
    -> torque), which is slow and steals lean authority.  The new
    integrator builds up a direct wheel torque that cancels the gravity
    pull, so the body returns to a small natural lean (~3 deg at 8 deg
    slope) instead of riding the lean limit.  Sign-convention free: it
    integrates velocity error, so it works uphill AND downhill.
  - Online slope estimate published on /slope_estimate (degrees), derived
    from the low-passed TOTAL commanded wheel torque: at constant speed
    the wheels must supply exactly m*g*sin(slope)*r, so
    slope = asin( lpf(tau_left+tau_right) / (m*g*r) ).
  - max_lean raised 0.20 -> 0.25 rad for ramp-entry transient headroom.
  - Feed-forward state is reset on tip-over and frozen during recovery.

Original features kept intact:
  - Conditional anti-windup (only integrate when output is not saturated).
  - Active damping on tip-over (brake torque proportional to wheel
    velocity instead of just releasing).
  - Watchdog on IMU / joint_state staleness — fail-safe stop if sensor
    data goes silent.
  - Explicit pitch_sign parameter.
  - Complementary filter (gyro + accelerometer fusion) for real hardware;
    in Gazebo the IMU quaternion is ground truth so it is off by default.

Cascade architecture:
    v_err        = cmd_linear - v_robot
    target_pitch = clip( Kp_v * v_err + Ki_v * integral, +/- max_lean )
    tau_ff      += Ki_tau * v_err * dt          (clamped, anti-windup)
    pitch_err    = pitch * pitch_sign - target_pitch
    tau_balance  = Kp_p * pitch_err + Kd_p * pitch_rate + 0.5 * tau_ff
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
from std_msgs.msg import Float64, Float64MultiArray


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
        # Hard clamp on the velocity integral (anti-windup for sustained slope
        # lag).  Large default = no effect on flat ground; set small (~0.5) on
        # slopes so the climbing lean stays bounded and can't cancel the
        # gravity feed-forward.
        self.declare_parameter("max_vel_integral", 50.0)
        # Yaw loop
        self.declare_parameter("k_yaw",            0.35)
        # ---- Slope / disturbance compensation (NEW) ----
        # tau_ff += ki_tau * v_err * dt   (total torque, split across wheels)
        # Holding an 8° slope needs ~1.36 N·m total; with ki_tau=3.0 and a
        # sustained ~0.1 m/s sag entering the ramp, that builds in ~4-5 s.
        self.declare_parameter("ki_tau",           3.0)    # N·m per (m/s)·s
        self.declare_parameter("max_tau_ff",       3.0)    # N·m total clamp (≈18° slope)
        # Slope-aware feed-forward decay: when the robot OVERSPEEDS (v_err
        # clearly negative) the feed-forward is exponentially bled toward zero
        # at rate ff_decay (1/s).  On the ramp the feed-forward winds up to the
        # climbing torque; when the robot crests onto the flat plateau the
        # slope vanishes and that torque becomes pure excess thrust → overspeed
        # → tip at the stop.  Bleeding it toward zero on real overspeed (not
        # climb noise) lets the robot settle to a clean cruise and stop.
        self.declare_parameter("ff_decay",         4.0)    # 1/s decay rate when overspeeding
        self.declare_parameter("ff_decay_v",       0.05)   # m/s; overspeed threshold to trigger decay
        # Commanded-speed gate on the feed-forward decay.  The decay (bleed
        # tau_ff -> 0) only fires when overspeeding AND |cmd_linear| < this gate.
        # Default 1e9 = gate always open, so the decay reverts to pure overspeed
        # (the uphill crest behaviour — unchanged).  On a DESCENT the robot
        # overspeeds the whole way down, so an ungated decay would erase the
        # braking feed-forward it needs; setting a small gate (e.g. 0.03) means
        # tau_ff BUILDS while the robot is driving down (cmd=0.05 > gate) and
        # only RELEASES when the robot is told to stop (cmd -> 0 < gate), so the
        # wound-up braking torque doesn't pitch the halted robot over backward.
        self.declare_parameter("ff_decay_cmd_gate", 1.0e9) # m/s
        # Velocity-integral release at stop.  When |cmd_linear| < this gate the
        # robot is being told to halt, so the velocity integral is bled toward
        # zero (at vint_stop_decay 1/s) instead of integrating.  Without this,
        # an integral wound up during a long descent (sustained v_err < 0) stays
        # pinned at its clamp after the stop command, holding a forward-lean
        # demand that drives the wheels and tips the halted robot.  Default gate
        # 0.0 = disabled (flat/uphill behaviour unchanged).
        self.declare_parameter("stop_cmd_gate",     0.0)   # m/s
        self.declare_parameter("vint_stop_decay",   3.0)   # 1/s
        # Slip-robust velocity estimate: the forward-velocity estimate may
        # change no faster than this (a plausible chassis acceleration).
        # Real controlled motion (<~2 m/s²) is tracked exactly; wheel SLIP
        # (which implies an absurd ~90 m/s² jump in the encoder reading) is
        # rejected smoothly.  Replaces the old hard ±0.3 m/s clamp, whose
        # discontinuity at the crest overspeed drove the plateau limit cycle.
        self.declare_parameter("vel_accel_limit",  5.0)    # m/s², slew limit on v_est
        # Hard ceiling on |v_est| (m/s).  Backstop against pathological sustained
        # wheel SLIP: when the wheels break traction and spin up, the encoder
        # reports an absurd forward speed that, uncapped, saturates v_err ->
        # target_pitch -> torque and drives a slip runaway (observed at a slope
        # brow).  Default 2.0 = effectively off on flat ground.  On a steep
        # descent set it just above the intended travel speed (e.g. 0.4) so a
        # slip reading cannot run v_err away — the proven fix from the uphill
        # ramp work, where clamping the wheel-derived speed killed the runaway.
        self.declare_parameter("max_v_est",         2.0)   # m/s
        # Speed-estimate ceiling used WHEN STOPPING (|cmd_linear| < stop_cmd_gate).
        # A tight max_v_est protects the DECLINE (where the controller must not
        # chase a lean it can't achieve), but that same tight clamp blinds the
        # controller to the real residual speed at the bottom, so it can't brake
        # to a halt.  At the stop the robot is on FLAT ground where leaning back
        # DOES brake, so open the clamp here to let it see and shed the real
        # speed.  Default 2.0 = effectively off (unchanged elsewhere).
        self.declare_parameter("max_v_est_stop",     2.0)   # m/s
        self.declare_parameter("robot_mass",      13.28)   # kg total, for slope estimate
        # Robot geometry
        self.declare_parameter("wheel_radius",     0.075)
        self.declare_parameter("wheel_separation", 0.345)
        # Calibration / convention
        # cart-pole Lagrangian: for a backward lean (positive pitch),
        # forward wheel torque (positive) stabilises. pitch_sign=+1.
        self.declare_parameter("pitch_sign",       1.0)
        self.declare_parameter("pitch_offset",     0.0)    # IMU mounting bias
        # Saturation
        # 0.25 rad (~14.3°): extra headroom over flat-road 0.20 for the
        # transient when the wheels hit the slope discontinuity.
        self.declare_parameter("max_lean",         0.25)   # rad
        # 8.0 N·m keeps gravity dominated up to ~25° lean; effort interface allows 10 N·m
        self.declare_parameter("max_torque",       8.0)    # N*m per wheel
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
        # NOTE for slopes: the accelerometer term measures pitch relative to
        # gravity, which is exactly what the balance loop needs — the filter
        # remains valid on inclines (transient bias while accelerating is
        # absorbed by the gyro-dominated alpha=0.98 blend).
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
        self.v_est             = 0.0     # slip-robust forward velocity estimate
        self.tau_ff            = 0.0     # NEW: learned gravity feed-forward (total N·m)
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
        self._slope_decim      = 0       # NEW: decimator for /slope_estimate
        self._tau_lpf          = 0.0     # NEW: low-passed total wheel torque
        # Startup grace: skip tip-check for first N IMU callbacks.  Stale DDS
        # messages from a previous killed run arrive in the very first callbacks
        # (often showing pitch≈π).  40 callbacks ≈ 0.2 s of real data at 200 Hz
        # — enough to flush the stale reading without leaving the robot
        # uncontrolled long enough to topple.
        self._imu_warmup       = 40

        # ------------- I/O -------------
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu,        "imu",          self._on_imu,    be)
        self.create_subscription(JointState, "joint_states", self._on_joints, 10)
        self.create_subscription(Twist,      "cmd_vel",      self._on_cmd,    10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, "/effort_controllers/commands", 10)
        # NEW: live slope estimate in degrees, ~10 Hz
        self.slope_pub = self.create_publisher(Float64, "slope_estimate", 10)

        hz = self.get_parameter("loop_hz").value
        self.dt = 1.0 / hz
        self.create_timer(self.dt, self._loop)

        self.get_logger().info(
            f"BalanceController (slope-capable) running at {hz:.0f} Hz")

    # ------------- Callbacks -------------
    def _on_imu(self, msg):
        if self._imu_warmup > 0:
            self._imu_warmup -= 1
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

    def _publish_slope_estimate(self, tau_total, r):
        """Slope angle implied by the sustained total wheel torque.

        At constant speed on an incline the wheels must supply exactly the
        gravity holding torque:  lpf(tau_l + tau_r) = m*g*sin(slope)*r
        =>  slope = asin( lpf / (m*g*r) ).  Published in degrees at 10 Hz.

        (Deriving this from tau_ff alone would over-read: the lean integral
        and the torque integrator share the same error signal, so the split
        between them is trajectory-dependent.  The total commanded torque
        is unambiguous.)
        """
        # 1st-order low-pass, time constant ~2 s at the 200 Hz loop rate
        beta = self.dt / 2.0
        self._tau_lpf += beta * (tau_total - self._tau_lpf)

        self._slope_decim += 1
        if self._slope_decim < 20:          # 200 Hz loop -> 10 Hz publish
            return
        self._slope_decim = 0
        m_tot = self.get_parameter("robot_mass").value
        s = clamp(self._tau_lpf / (m_tot * 9.81 * r), -1.0, 1.0)
        msg = Float64()
        msg.data = math.degrees(math.asin(s))
        self.slope_pub.publish(msg)

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

        # Startup warmup: skip tip-check until stale DDS messages are flushed
        if self._imu_warmup > 0:
            self._publish(0.0, 0.0)
            return

        # Snapshot parameters
        gp = self.get_parameter
        kp_p     = gp("kp_pitch").value
        kd_p     = gp("kd_pitch").value
        kp_v     = gp("kp_vel").value
        ki_v     = gp("ki_vel").value
        vint_max = gp("max_vel_integral").value
        ki_tau   = gp("ki_tau").value
        max_ff   = gp("max_tau_ff").value
        ff_decay = gp("ff_decay").value
        ff_dec_v = gp("ff_decay_v").value
        ff_dec_gate = gp("ff_decay_cmd_gate").value
        stop_gate   = gp("stop_cmd_gate").value
        vint_decay  = gp("vint_stop_decay").value
        a_lim    = gp("vel_accel_limit").value
        v_est_max = gp("max_v_est").value
        # When commanded to stop, open the speed-estimate ceiling so the loop
        # can see and brake the real residual speed (it's on flat ground here).
        if abs(self.cmd_linear) < gp("stop_cmd_gate").value:
            v_est_max = max(v_est_max, gp("max_v_est_stop").value)
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
            self.tau_ff = 0.0          # forget learned slope torque on tip
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
                self.v_est = 0.0
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

        # ----- Forward velocity (slip-robust) and yaw rate -----
        # 1-state estimator with a bounded-acceleration process model: the
        # wheel-derived velocity is the measurement, but v_est may slew no
        # faster than a_lim (a plausible chassis acceleration).  Real motion
        # (incl. the ~0.4 m/s² crest overspeed) is tracked exactly with no lag
        # and no ceiling, so the controller sees true speed and brakes the
        # plateau overspeed properly; wheel SLIP (an ~90 m/s² jump in the
        # encoder) is rejected smoothly.  This replaces the hard ±0.3 m/s clamp
        # whose discontinuity at the crest drove a plateau limit cycle.
        v_wheel  = 0.5 * r * (self.wheel_vel_l + self.wheel_vel_r)
        max_dv   = a_lim * self.dt
        self.v_est += clamp(v_wheel - self.v_est, -max_dv, max_dv)
        self.v_est = clamp(self.v_est, -v_est_max, v_est_max)   # backstop vs pathological sustained slip
        v_robot  = self.v_est
        yaw_rate = r * (self.wheel_vel_r - self.wheel_vel_l) / ws

        # ----- Outer loop: velocity error -> target pitch -----
        # Skip for the first N cycles after tip recovery so spinning wheels
        # cannot cause an immediate second tip via a large velocity demand.
        # tau_ff is APPLIED (helps hold position on a slope) but FROZEN
        # (not integrated) during the cooldown.
        if self.recovery_cooldown > 0:
            self.recovery_cooldown -= 1
            target_pitch = 0.0
            pitch_err  = pitch - target_pitch
            tau_balance = kp_p * pitch_err + kd_p * pitch_rate + 0.5 * self.tau_ff
            tau_yaw     = k_yaw * (self.cmd_angular - yaw_rate)
            tau_l = clamp(tau_balance - tau_yaw, -max_tau, max_tau)
            tau_r = clamp(tau_balance + tau_yaw, -max_tau, max_tau)
            self._publish(tau_l, tau_r)
            return

        v_err = self.cmd_linear - v_robot

        # ----- Gravity feed-forward (slope compensation, NEW) -----
        # Slow torque-level integrator with conditional anti-windup.  On an
        # incline gravity produces a persistent v_err; this term converges to
        # the exact holding torque m*g*sin(slope)*r and cancels it directly,
        # so the lean integral does not have to.
        #
        # Slope-aware decay: while climbing, integrate UP normally so the
        # feed-forward converges to the gravity holding torque.  But when the
        # robot clearly OVERSPEEDS (v_err < -ff_dec_v) it has crested onto
        # flatter ground and the wound-up climbing torque is now excess thrust
        # — exponentially bleed tau_ff TOWARD ZERO (time constant 1/ff_decay).
        # Decaying toward zero (not integrating past it) is deliberate: a
        # symmetric fast integrator overshot to -2.5 N·m and oscillated against
        # the velocity clamp, which pins v_err at -0.22 and can't tell the
        # robot is slowing.  Bleeding to zero just removes the excess so the
        # robot coasts down to a clean cruise/stop on the plateau.
        if v_err < -ff_dec_v and abs(self.cmd_linear) < ff_dec_gate:
            self.tau_ff *= max(0.0, 1.0 - ff_decay * self.dt)
        else:
            cand_ff = self.tau_ff + ki_tau * v_err * self.dt
            if -max_ff < cand_ff < max_ff:
                self.tau_ff = cand_ff

        # ----- Outer loop: velocity error -> target pitch -----
        # Pre-integration to compute candidate output; only commit integration
        # if the resulting target_pitch is NOT saturated (conditional anti-windup).
        # The integral is ALSO hard-clamped to +/- vint_max: on a slope the
        # robot lags the speed command indefinitely, so vel_integral would wind
        # up without bound, driving target_pitch above the pitch the body can
        # actually track.  Once target_pitch > pitch the kp_p*pitch_err term
        # cancels the gravity feed-forward, wheel torque collapses, and the
        # robot stalls mid-climb and slides back.  Clamping vel_integral lets
        # it supply the steady climbing lean (~0.04 rad) without over-winding.
        if abs(self.cmd_linear) < stop_gate:
            # Commanded to stop: release any wound-up integral so it can't keep
            # leaning the halted robot into a forward drive.
            self.vel_integral *= max(0.0, 1.0 - vint_decay * self.dt)
        else:
            candidate_integral = clamp(self.vel_integral + v_err * self.dt,
                                       -vint_max, vint_max)
            candidate_pitch = kp_v * v_err + ki_v * candidate_integral
            if -max_lean < candidate_pitch < max_lean:
                self.vel_integral = candidate_integral

        target_pitch = clamp(kp_v * v_err + ki_v * self.vel_integral,
                             -max_lean, max_lean)

        # ----- Inner loop: pitch error -> balancing torque (+ feed-forward) -----
        pitch_err  = pitch - target_pitch
        tau_balance = kp_p * pitch_err + kd_p * pitch_rate + 0.5 * self.tau_ff

        # ----- Yaw mixing -----
        yaw_err = self.cmd_angular - yaw_rate
        tau_yaw = k_yaw * yaw_err

        tau_l = clamp(tau_balance - tau_yaw, -max_tau, max_tau)
        tau_r = clamp(tau_balance + tau_yaw, -max_tau, max_tau)
        self._publish_slope_estimate(tau_l + tau_r, r)
        self._publish(tau_l, tau_r)

        # ---- High-rate diagnostic (ramp-jam investigation) ----
        # Fires at 20 Hz only while the robot is struggling (large velocity
        # error) so it captures the stall at the ramp foot without flooding
        # the flat-ground steady state.  Shows whether the wheels are blocked
        # (big tau, ~0 wheel vel), under-torqued, or commanded in reverse.
        if abs(v_err) > 0.03:
            self._diag_cnt = getattr(self, "_diag_cnt", 0) + 1
            if self._diag_cnt >= 10:
                self._diag_cnt = 0
                self.get_logger().info(
                    f"[diag] p={pitch:+.3f} tp={target_pitch:+.3f} "
                    f"verr={v_err:+.3f} vint={self.vel_integral:+.3f} "
                    f"tff={self.tau_ff:+.3f} tL={tau_l:+.2f} tR={tau_r:+.2f} "
                    f"wL={self.wheel_vel_l:+.2f} wR={self.wheel_vel_r:+.2f}")

        # Debug: log pitch every 200 loops (~1 s) so ramp dynamics are visible
        if not hasattr(self, '_dbg_cnt'):
            self._dbg_cnt = 0
        self._dbg_cnt += 1
        if self._dbg_cnt >= 200:
            self._dbg_cnt = 0
            self.get_logger().info(
                f"pitch={pitch:+.3f} v={v_robot:+.3f} tff={self.tau_ff:+.3f}"
            )


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
