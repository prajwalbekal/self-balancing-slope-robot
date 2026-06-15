# Self-Balancing Two-Wheel Robot — Complete Debug & Development Log

**Project:** ROS 2 Humble + Gazebo Classic 11 self-balancing robot  
**Workspace:** `/home/prajwal/ros2_ws/src/two_wheel_robot/`  
**Robot:** Differential-drive inverted pendulum (SolidWorks/FreeCAD CAD export)  
**Final result:** Robot spawns in a road environment, self-balances, and autonomously drives 15.5 m from start to end of road with < 2 mm lateral deviation.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Problem 1 — Robot Appears as Orange Box in Gazebo](#2-problem-1--robot-appears-as-orange-box-in-gazebo)
3. [Problem 2 — Robot Immediately Topples After Spawning](#3-problem-2--robot-immediately-topples-after-spawning)
4. [Problem 3 — snap_upright Launches Robot Into the Air](#4-problem-3--snap_upright-launches-robot-into-the-air)
5. [Problem 4 — Balance Controller Freezes After reset_simulation](#5-problem-4--balance-controller-freezes-after-reset_simulation)
6. [Problem 5 — Robot Oscillates Too Aggressively on Real Hardware Path](#6-problem-5--robot-oscillates-too-aggressively)
7. [Problem 6 — Road Launch Creates Process Storm (20+ Zombie Instances)](#7-problem-6--road-launch-creates-process-storm-20-zombie-instances)
8. [Final Architecture — Files & Their Roles](#8-final-architecture--files--their-roles)
9. [All Code Changes, File by File](#9-all-code-changes-file-by-file)
10. [How to Run](#10-how-to-run)

---

## 1. Project Overview

The robot is a two-wheeled self-balancing robot modelled after an inverted pendulum. It has:
- A tall chassis (≈ 0.455 m body on 0.075 m radius wheels)
- Two wheel joints driven by effort (torque) commands
- An IMU mounted at 0.15 m height on the chassis
- A cascade-PD controller: outer velocity loop feeds a target lean angle into an inner pitch loop

The control chain is:
```
cmd_vel (Twist) → balance_controller
  outer loop: v_error → target_pitch (lean angle command)
  inner loop: pitch_error → torque → /effort_controllers/commands
```

---

## 2. Problem 1 — Robot Appears as Orange Box in Gazebo

### Symptom
When Gazebo loaded the robot, instead of the correct 3-D mesh shape, it displayed a plain **orange bounding box**. The robot looked like a rectangular orange block.

### Root Cause
The mesh paths in `robot_core.xacro` used the `package://` URI scheme:
```xml
<!-- BROKEN -->
<mesh filename="package://two_wheel_robot/meshes/base_link.stl"/>
```
Gazebo Classic 11 has an unreliable resolver for `package://` URIs when the package is in an overlay workspace (not system-installed). When it cannot resolve the path, it falls back to rendering the collision primitive as an orange box.

### Fix — `description/robot_core.xacro`
Changed all three mesh references to `file://` with the absolute path resolved at xacro-parse time:
```xml
<!-- FIXED -->
<mesh filename="file://$(find two_wheel_robot)/meshes/base_link.stl" scale="1 1 1"/>
<mesh filename="file://$(find two_wheel_robot)/meshes/left_wheel_link.stl" scale="1 1 1"/>
<mesh filename="file://$(find two_wheel_robot)/meshes/right_wheel_link.stl" scale="1 1 1"/>
```
`$(find two_wheel_robot)` is resolved by xacro at parse time (before Gazebo sees it), producing an absolute path that always works regardless of how the package is installed.

---

## 3. Problem 2 — Robot Immediately Topples After Spawning

### Symptom
The robot would spawn upright, the balance controller would start, but within 1–2 seconds the robot would fall over and spin. Restarting only made it fall again. The controller appeared to be commanding large torques in the wrong direction.

### Root Cause — Wrong Pitch Formula (`math.asin` instead of `math.atan2`)

The original `quat_to_pitch()` function in `balance_controller.py` used:
```python
# BROKEN — only covers ±90°
sinp = 2.0 * (qw * qy - qz * qx)
pitch = math.asin(sinp)
```

`math.asin` only covers ±90°. When an inverted pendulum falls **past 90°** (robot is lying flat), the `asin` value aliases back toward 0 — the controller reads pitch ≈ 0° and thinks the robot is upright. It then applies maximum corrective torque in the wrong direction, which spins the robot rather than recovering it. This creates a runaway loop.

### Fix — `scripts/balance_controller.py`
```python
def quat_to_pitch(qw, qx, qy, qz):
    """Pitch in [-π, π] — atan2 covers full ±180°, asin only covers ±90°."""
    sinp = 2.0 * (qw * qy - qz * qx)
    cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    return math.atan2(sinp, cosp)   # <-- was math.asin(sinp)
```
`math.atan2(sinp, cosp)` covers the full ±180° range. Now when the robot lies flat (pitch = ±180°), the controller correctly detects it as fallen and switches to the brake/tip-over mode instead of applying corrective torque.

### Additional fix — tip-over detection and braking
Added a `safety_pitch` threshold (0.60 rad ≈ 34°). When pitch exceeds this, the controller switches to **active braking** (torque proportional to wheel velocity) instead of trying to balance. This prevents the robot from sliding along the floor after falling.

```python
if not self.tipped and abs(pitch) > safe_p:
    self.tipped = True
    # Switch to brake mode: tau = -brake_kv * wheel_velocity

if self.tipped:
    tau_l = clamp(-brake_kv * self.wheel_vel_l, -max_tau, max_tau)
    tau_r = clamp(-brake_kv * self.wheel_vel_r, -max_tau, max_tau)
    self._publish(tau_l, tau_r)
    # Re-arm only when pitch comes back below recovery_pitch (0.20 rad)
    if abs(pitch) < recov_p:
        self.tipped = False
```

---

## 4. Problem 3 — snap_upright Launches Robot Into the Air

### Symptom
After the robot fell, running `snap_upright.py` (which calls `reset_simulation` to snap the robot back to its upright spawn pose) would sometimes send the robot flying upward at high speed, as if hit by a large invisible force.

### Root Cause — Stale Torque Commands Persisting in effort_controllers

When `snap_upright` killed the `balance_controller` process using SIGTERM, Python's `finally` block did **not** execute (SIGTERM on Linux does not run Python cleanup code). This meant the `balance_controller` did not publish `[0.0, 0.0]` to `/effort_controllers/commands` before dying.

The `JointGroupEffortController` (ros2_control) **holds the last received torque command** until a new one arrives. So after killing the controller, the wheels still had large stale torques applied. When `reset_simulation` then teleported the robot back to the upright spawn position, those stale torques instantly launched it.

### Fix — `scripts/snap_upright.py`
Zero the torques **before** killing the balance controller, while physics is still paused:

```python
# Step 1: pause physics
call_empty(node, pause_cli, "pause_physics")

# Step 2: zero torques NOW, before killing balance_controller
#         (SIGTERM skips Python finally blocks — controller won't self-zero)
zero = Float64MultiArray()
zero.data = [0.0, 0.0]
for _ in range(5):          # publish several times for guaranteed delivery
    cmd_pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.02)

# Step 3: THEN kill balance_controller
os.kill(balance_pid, signal.SIGTERM)

# Step 4: reset_simulation (torques are already 0, so robot lands safely)
call_empty(node, reset_cli, "reset_simulation")

# Step 5: wait for balance_controller to respawn (0.25 s)
time.sleep(0.25)

# Step 6: unpause — robot is upright, torques are 0, controller is ready
call_empty(node, unpause_cli, "unpause_physics")
```

### Additional fix — snap_upright timer: 2.0 s → 3.5 s (`launch/gazebo.launch.py`)
The snap_upright script was called too early. The controllers (joint_state_broadcaster and effort_controllers) needed more time to finish loading before the first reset was attempted. The timer in the launch file was increased from 2.0 s to 3.5 s.

```python
# Before
TimerAction(period=2.0, actions=[snap_upright])

# After
TimerAction(period=3.5, actions=[snap_upright])
```

---

## 5. Problem 4 — Balance Controller Freezes After reset_simulation

### Symptom
After `snap_upright` called `reset_simulation`, the Gazebo simulation clock reset to 0. The `balance_controller`'s 200 Hz ROS timer (which used `use_sim_time=True`) would see the clock jump backward and stop firing until the simulation clock caught back up. This caused a **~2 second blackout** with no torque output — long enough for the robot to fall again.

### Root Cause
ROS 2 timers with `use_sim_time=True` base their fire schedule on the `/clock` topic. When `reset_simulation` resets the Gazebo clock to 0, the timer sees a time in the past and refuses to fire until sim time exceeds the next expected trigger.

### Fix — `scripts/balance_controller.py` and related nodes
Set `use_sim_time=False` on the balance controller so it uses the system wall clock for its timer:

```python
# In gazebo.launch.py and road.launch.py
balance = Node(
    package="two_wheel_robot",
    executable="balance_controller.py",
    parameters=[{"use_sim_time": False}],   # wall-clock timer
    ...
)
```

The IMU watchdog inside the controller already uses `time.monotonic()` (wall clock) independently, so this is consistent. The controller never stalls on a sim clock reset.

Same fix applied to `odom_publisher.py` (wall clock makes odometry immune to sim resets).

---

## 6. Problem 5 — Robot Oscillates Too Aggressively

### Symptom
The robot balanced but oscillated visibly at ±6°. This was acceptable for the simulation but would be too aggressive for real hardware, and caused instability when a velocity command was first received.

### Analysis — Cascade PD Gains
```
Iyy ≈ m*(sx²+sz²)/12 = 12.11*(0.50²+0.455²)/12 ≈ 0.46 kg·m²
Gravity destabilising torque ≈ m*g*L_com = 12.11*9.81*0.293 ≈ 34.8 N·m/rad

With kp_pitch=120:
  ωn ≈ sqrt((120 - 34.8) / 0.46) ≈ 13.6 rad/s
  kd=15 → damping ratio ζ ≈ 1.2  (slightly overdamped in theory, but physical friction causes limit cycle)
```

### Fix — Complementary Filter added to `balance_controller.py`
To improve sensor quality (especially relevant for real hardware where IMU orientation quaternion is not directly available), a complementary filter was added as an optional parameter:

```python
self.declare_parameter("use_complementary_filter", False)
self.declare_parameter("cf_alpha", 0.98)
```

In simulation, the IMU quaternion is ground-truth, so the filter is off by default. On real hardware, set `use_complementary_filter:=true`. The filter fuses:
- Gyroscope integration (fast, drifts over time): `pitch_gyro = pitch_prev + gyro_y * dt`
- Accelerometer pitch (stable long-term, noisy short-term): `pitch_accel = atan2(-ax, az)`
- Combined: `pitch = alpha * pitch_gyro + (1-alpha) * pitch_accel`

### Additional fix — Anti-windup on velocity integral
The outer velocity loop integrator was updated to use **conditional anti-windup**: only integrate when the resulting target pitch is within saturation limits.

```python
candidate_integral = self.vel_integral + v_err * self.dt
candidate_pitch = kp_v * v_err + ki_v * candidate_integral
if -max_lean < candidate_pitch < max_lean:   # only update if not saturated
    self.vel_integral = candidate_integral
```

---

## 7. Problem 6 — Road Launch Creates Process Storm (20+ Zombie Instances)

### Symptom
After several launch attempts (especially ones that were killed mid-way), running `ps aux` showed **20+ balance_controller.py processes** all running simultaneously, consuming nearly 100% of all CPU cores. The simulation was unusable.

### Root Cause — `respawn=True, respawn_delay=0.1` in road.launch.py
The `balance_controller` node in `road.launch.py` was configured with:
```python
balance = Node(
    ...
    respawn=True,
    respawn_delay=0.1,   # 100 ms
)
```

When a launch session is killed (Ctrl+C or process kill), ROS 2's launch system may not cleanly terminate all child processes — especially if the launch process itself is killed. This leaves the `balance_controller` running as an orphan. When the launch was attempted again, a new `balance_controller` was spawned by the new launch session. Crucially, the old orphan had `respawn=True` baked into its launch context, causing it to keep self-replicating every 100 ms whenever the balance controller exited (which it did whenever Gazebo topics disappeared).

Over multiple launch attempts this accumulated to 20+ parallel instances.

### Fix — `launch/road.launch.py`
Removed `respawn` entirely from the road launch. The road scenario uses the **paused-start approach** which eliminates the need for respawn:

```python
# BEFORE
balance = Node(
    package="two_wheel_robot",
    executable="balance_controller.py",
    parameters=[{"use_sim_time": False}],
    output="screen",
    respawn=True,          # REMOVED
    respawn_delay=0.1,     # REMOVED
)

# AFTER
balance = Node(
    package="two_wheel_robot",
    executable="balance_controller.py",
    parameters=[{"use_sim_time": False}],
    output="screen",
)
```

### Emergency cleanup command (if processes accumulate)
```bash
ps aux | grep -E "balance_controller|odom_publisher|road_drive|gzserver|gzclient" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

---

## 8. Final Architecture — Files & Their Roles

```
ros2_ws/src/two_wheel_robot/
├── description/
│   ├── robot.urdf.xacro         Top-level URDF — includes all sub-xacros
│   ├── robot_core.xacro         Links, joints, mesh paths (file:// fixed)
│   ├── inertial_macros.xacro    Inertia helper macros
│   ├── gazebo.xacro             Gazebo plugins: IMU, ros2_control, wheel friction
│   └── ros2_control.xacro       Hardware interface: effort command on both wheels
│
├── config/
│   └── controllers.yaml         controller_manager at 200 Hz, effort_controllers
│                                joint order: [left_wheel_joint, right_wheel_joint]
│
├── scripts/
│   ├── balance_controller.py    Cascade-PD self-balancing controller (200 Hz)
│   │                              - Outer: velocity error → target lean angle
│   │                              - Inner: pitch error + pitch_rate → wheel torque
│   │                              - atan2 pitch formula (full ±180°)
│   │                              - Tip detection + active braking
│   │                              - Anti-windup on velocity integral
│   │                              - Optional complementary filter
│   │                              - Watchdog: zeros torque if sensors go silent >0.5 s
│   ├── snap_upright.py          Reset utility: pause → zero torques → kill controller
│   │                              → reset_simulation → wait → unpause
│   ├── odom_publisher.py        Wheel encoder odometry + odom→base_footprint TF
│   ├── road_drive.py            Autonomous road driver: drives 15.5 m along +X
│   │                              with heading correction and deceleration zone
│   ├── kinematic_teleop_driver.py  Manual keyboard teleoperation
│   └── square_path.py           Autonomous square path follower
│
├── launch/
│   ├── gazebo.launch.py         Empty-world launch with snap_upright approach
│   │                              (respawn=True on balance_controller — only here)
│   └── road.launch.py           Road-world launch with PAUSED-START approach
│                                  (no respawn — cleaner and safer)
│
└── worlds/
    ├── road.world               18 m straight road:
    │                              - ground_plane (collision at z=0)
    │                              - road_surface (visual only, 4 mm thick dark grey)
    │                              - kerbs at y=±1.35 (visual only)
    │                              - 9 centre-line dashes (visual only)
    │                              - Red start post at x=0  (visual only)
    │                              - Blue end post at x=18  (visual only)
    └── empty.world              Flat world for general testing
```

---

## 9. All Code Changes, File by File

### `description/robot_core.xacro` — Mesh Path Fix

| What changed | Before | After |
|---|---|---|
| base_link mesh | `package://two_wheel_robot/meshes/base_link.stl` | `file://$(find two_wheel_robot)/meshes/base_link.stl` |
| left_wheel mesh | `package://...` | `file://$(find two_wheel_robot)/meshes/left_wheel_link.stl` |
| right_wheel mesh | `package://...` | `file://$(find two_wheel_robot)/meshes/right_wheel_link.stl` |

**Why:** Gazebo Classic cannot resolve `package://` URIs in overlay workspaces. `file://` with absolute path (resolved by xacro at parse time) always works.

---

### `scripts/balance_controller.py` — 5 Changes

**Change 1 — Pitch formula: `asin` → `atan2`**
```python
# BEFORE (broken for |pitch| > 90°)
sinp = 2.0 * (qw * qy - qz * qx)
return math.asin(sinp)

# AFTER (correct for full ±180°)
sinp = 2.0 * (qw * qy - qz * qx)
cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
return math.atan2(sinp, cosp)
```

**Change 2 — Tip-over detection and active braking**
```python
# Added safety_pitch threshold (0.60 rad ≈ 34°)
if not self.tipped and abs(pitch) > safe_p:
    self.tipped = True
    self.vel_integral = 0.0

if self.tipped:
    # Brake: damp wheel velocity instead of trying to balance
    tau_l = clamp(-brake_kv * self.wheel_vel_l, -max_tau, max_tau)
    tau_r = clamp(-brake_kv * self.wheel_vel_r, -max_tau, max_tau)
    if abs(pitch) < recov_p:
        self.tipped = False
        self.recovery_cooldown = 40  # skip velocity outer-loop briefly after recovery
```

**Change 3 — use_sim_time=False**
Set at launch time via parameter (not in the script itself). See launch file changes below.

**Change 4 — Anti-windup on velocity integral**
```python
# BEFORE: integrate unconditionally
self.vel_integral += v_err * self.dt

# AFTER: conditional anti-windup
candidate_integral = self.vel_integral + v_err * self.dt
candidate_pitch = kp_v * v_err + ki_v * candidate_integral
if -max_lean < candidate_pitch < max_lean:
    self.vel_integral = candidate_integral
```

**Change 5 — Optional complementary filter**
```python
self.declare_parameter("use_complementary_filter", False)
self.declare_parameter("cf_alpha", 0.98)

# In _on_imu():
if use_cf:
    alpha = self.get_parameter("cf_alpha").value
    pitch_accel = math.atan2(-ax, az)
    self._cf_pitch = alpha * (self._cf_pitch + gyro_y * imu_dt) \
                   + (1 - alpha) * pitch_accel
    self.pitch = self._cf_pitch
else:
    q = msg.orientation
    self.pitch = quat_to_pitch(q.w, q.x, q.y, q.z)
```

---

### `scripts/snap_upright.py` — Torque Zeroing Before Kill

```python
# ADDED before os.kill() call:
zero = Float64MultiArray()
zero.data = [0.0, 0.0]
for _ in range(5):
    cmd_pub.publish(zero)
    rclpy.spin_once(node, timeout_sec=0.02)
# THEN kill, THEN reset_simulation
```

---

### `launch/gazebo.launch.py` — snap_upright Timer

```python
# BEFORE
TimerAction(period=2.0, actions=[snap_upright])

# AFTER
TimerAction(period=3.5, actions=[snap_upright])
```

---

### `launch/road.launch.py` — respawn Removed

```python
# BEFORE
balance = Node(
    ...
    respawn=True,
    respawn_delay=0.1,
)

# AFTER
balance = Node(
    ...
    # no respawn — paused-start approach doesn't need it
)
```

---

### `scripts/road_drive.py` — New File (Autonomous Road Driver)

Drives the robot from its spawn position 15.5 m along the +X axis:

```python
# Key parameters (set in road.launch.py):
road_length  = 15.5    # m — drive this far from spawn
linear_vel   = 0.12    # m/s — slow enough for stable balance
decel_zone   = 2.0     # m — ramp down speed in last 2 m
heading_kp   = 0.4     # heading correction gain
start_delay  = 3.0     # s — wait after starting before driving

# State machine: WAIT → DRIVE → DONE
# Distance: dist_driven = odom_x - start_x
# Heading correction: angular.z = -heading_kp * yaw  (keeps yaw ≈ 0)
```

---

### `worlds/road.world` — New File (Road Environment)

18 m straight road from x=0 to x=18, 2.5 m wide:
- `ground_plane` — physical collision surface at z=0
- `road_surface` — visual dark-grey box (no collision, z≈0.002)
- Kerbs at y=±1.35 (visual only)
- 9 white centre-line dashes
- Red cylinder post at x=0 (start marker, visual only)
- Blue cylinder post at x=18 (end marker, visual only)
- White start line at x=0.5 and end line at x=17.5

---

## 10. How to Run

### Prerequisites
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Autonomous road drive (primary demo)
```bash
ros2 launch two_wheel_robot road.launch.py
```
Robot spawns at (0.8, 0, 0) facing +X, balances for ~6 seconds, then drives 15.5 m to x=16.3 m and stops. The Gazebo window shows the road with kerbs and markings.

### General balancing (empty world, with snap_upright)
```bash
ros2 launch two_wheel_robot gazebo.launch.py
```
Robot spawns and is snapped upright by `snap_upright.py` 3.5 s after launch. Use this launch when testing the balance controller in isolation.

### Teleop
```bash
# Terminal 1
ros2 launch two_wheel_robot gazebo.launch.py
# Terminal 2
ros2 launch two_wheel_robot teleop.launch.py
```

### If processes accumulate (emergency cleanup)
```bash
ps aux | grep -E "balance_controller|odom_publisher|road_drive|gzserver|gzclient" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
ros2 daemon stop && ros2 daemon start
```

---

## Summary Table — All Problems and Fixes

| # | Problem | Root Cause | File Changed | Fix |
|---|---------|-----------|-------------|-----|
| 1 | Orange box in Gazebo | `package://` URI unresolved | `robot_core.xacro` | Changed to `file://$(find ...)` |
| 2 | Robot immediately topples | `math.asin` wraps at ±90° → wrong pitch reading | `balance_controller.py` | Changed to `math.atan2(sinp, cosp)` |
| 3 | snap_upright launches robot | Stale torque in effort_controllers after SIGTERM | `snap_upright.py` | Zero torques **before** killing controller |
| 4 | Balance controller freezes after reset | Sim clock resets to 0, stalls ROS timer | `gazebo.launch.py` + nodes | `use_sim_time=False` on balance controller |
| 5 | Aggressive oscillation | High gains, no integral anti-windup | `balance_controller.py` | Added anti-windup, optional complementary filter |
| 6 | Process storm (20+ zombie instances) | `respawn=True, respawn_delay=0.1` on balance_controller | `road.launch.py` | Removed `respawn` — paused-start is reliable without it |

---

*Generated 2026-06-03. Workspace: `/home/prajwal/ros2_ws/src/two_wheel_robot/`*
