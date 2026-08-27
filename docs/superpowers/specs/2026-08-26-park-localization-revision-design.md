# Park localization revision — honest inputs, measured gates

**Date:** 2026-08-26
**Status:** design approved, not yet implemented
**Supersedes:** the localization sections (§4, §5) of
`2026-08-26-park-gps-nav2-design.md`. Nav2 sections of that spec stand.
**Scope:** rebuild the layer that feeds nav2 its position and heading in
`park`, and add the measurements that prove it, so that a goal sent to nav2
ends with the robot stopped at that goal.

---

## 1. Why this revision exists

The first implementation reached a goal only after four faults were found by
measurement, none of which produced an error anywhere in the stack:

| Fault | Mechanism | Found by |
|---|---|---|
| Robot drove ~150 deg off a valid path | gz-sim's IMU reports orientation relative to its **initial** pose (measured yaw 0.0000 at world yaw 149.72 deg). The global EKF fused it as an absolute map heading. | one-shot IMU read at spawn |
| Robot circled at the goal, stopped 4.4 m off | goal scripts defaulted yaw to 0 (= North in park); pose goal checker forced a 180 deg reversal MPPI performs as arcs | reading `nav_goal_ll.py` |
| Map servers never activated | two lifecycle managers racing at startup | lifecycle `get_state` query |
| Robot stopped 1.9 m short, nav2 reported success | localization estimate ran ahead of truth **while moving**, exact at rest (6 cm) | EKF vs Gazebo truth at the stop |

Every fault was in an *input* to nav2 — our localization layer or our goal
scripts. Nav2's planner and controller were correct in every check. The last
fault is not yet explained; §4 measures it before §3 fixes it.

Two lessons shape this design:

1. **Sim sensors must declare their conventions in the SDF**, not be corrected
   downstream. Every relay we added (`gps_covariance_relay`,
   `imu_map_relay`) is a constant hidden in Python that the next person will
   not find.
2. **A green lifecycle list is not readiness.** The only gate that means
   anything is *estimate vs truth, in motion*.

---

## 2. Non-goals

| Not doing | Why |
|---|---|
| Changing nav2 parameters | Every nav2 failure so far traced to an input. Nav2 stays at Clearpath defaults plus the documented deltas (keepout, fixed global costmap, `PositionGoalChecker`, single lifecycle manager, collision_monitor bypass, MPPI 30x600). |
| GPS fault detection / denial scenarios | Still out of scope. This revision makes the dead-reckoning fallback *measurable*, which is a prerequisite. |
| lake, 3D lidar | Unchanged from the original spec. |
| Editing `~/clearpath/` or Clearpath packages | Generated / vendor. All sensor overrides live in `urdf/`. |

---

## 3. Localization design

Structure is unchanged — dual EKF per REP-105 — because it is the standard and
it is what gives the GPS-loss dead-reckoning fallback. What changes is that
**every input arrives already in the frame and units the consumer expects.**

```
                 urdf/imu_world.urdf.xacro          urdf/gps_10hz.urdf.xacro
                 (one IMU, yaw in map frame)        (10 Hz, noise declared)
                          |                                   |
        +-----------------+-----------------+                 |
        v                                   v                 v
  ekf_node (Clearpath, local)        ekf_node_map (global)  navsat_transform
  wheel odom + IMU                   wheel vel + IMU yaw    lat/lon -> map x,y
  odom -> base_link                  + GPS x,y              (yaw_offset per §3.3)
                                     map -> odom
```

### 3.1 One IMU, world-referenced in the SDF

Replace both `imu_0` (stock, spawn-relative) and `imu_enu` (added today) with a
single custom sensor whose `<orientation_reference_frame>` makes its yaw equal
to the **map/world yaw** directly.

gz-sim's IMU supports `<localization>CUSTOM</localization>` with a
`<custom_rpy>` offset. The working hypothesis is
`<custom_rpy parent_frame="world">0 0 0</custom_rpy>`, i.e. reference the world
frame itself. **This is verified by the same probe used today** (spawn a box at
yaw 2.6132, read the gz topic): pass = yaw reads 2.6132 +/- 0.01. If CUSTOM
does not behave, fall back to ENU + `<custom_rpy>` of +pi/2, still in the SDF.
No relay in either case.

Both EKFs read this one IMU. The local EKF's odom frame is spawn-relative in
*position* but there is no requirement that its yaw be spawn-relative;
robot_localization fuses absolute IMU yaw into whatever `world_frame` it owns.

`imu_map_relay.py` and `imu_enu.urdf.xacro` are deleted.

**Why not keep stock `imu_0` and just add a reference-frame tag?** Clearpath's
`microstrain_imu` macro does not expose one, and `~/clearpath/` is generated.
The custom xacro replaces the sensor entry in `robot_configs/`, so the
generator never emits `imu_0`. The custom sensor must publish on the topic the
generated local EKF already subscribes to (`sensors/imu_0/data`) or the
generated `localization.yaml` will not see it — the custom xacro therefore
keeps the name `imu_0` and its topic, and is bridged by the explicit
`ros_gz_bridge` line in `scripts/run_husky_sim.sh` (gotcha #7). Verify at
launch that exactly one publisher exists on `sensors/imu_0/data`.

### 3.2 GPS at 10 Hz with declared noise

Clearpath's `garmin_18x` / `swiftnav_duro` xacros hardcode `update_rate 1` and
no `<noise>`. A custom `urdf/gps_10hz.urdf.xacro` (navsat, same mount as
today) sets:

- `update_rate 10` — the swiftnav/novatel class this models are 10–20 Hz
  receivers; 1 Hz is a Garmin 18x quirk, not a property of GPS.
- `<noise type="gaussian"><stddev>` on horizontal position, value from the
  receiver class being modelled (0.5 m is a reasonable non-RTK figure; the
  exact number is a config value, not a design decision).

Whether `ros_gz_bridge` then fills `position_covariance` from the declared
noise is **unknown** — verify with one `ros2 topic echo`. If it does,
`gps_covariance_relay.py` is deleted. If it does not, the relay stays, but it
reads its covariance from the same stddev so there is one number.

The sensor keeps the name `gps_0` and topic `sensors/gps_0/fix` for the same
reason as §3.1.

### 3.3 navsat_transform and the global EKF

- `navsat_transform_node` keeps `use_local_cartesian`, `datum [49.9, 8.9, 0]`
  and `yaw_offset 1.5708` (measured today: park world yaw = ENU yaw + pi/2).
  Its `imu` input is the single IMU; if §3.1 delivers map-frame yaw directly,
  `yaw_offset` becomes **0.0** — the plan carries both cases and the probe
  result selects one.
- `ekf_node_map` fuses: wheel vx/vy/vyaw, IMU yaw + yaw rate, GPS x/y. No
  change in structure. Its covariances are tuned **only** from §4's
  measurement, never by guess.

### 3.4 What is explicitly not decided yet

The 1.9 m in-motion error has a hypothesis (wheel-velocity integration between
GPS fixes, possibly with wheel slip) but no measurement. §4.1 runs before any
of §3.2–3.3's tuning. If 10 Hz GPS alone closes it, no EKF tuning happens.

---

## 4. Measurements (the deliverable that matters)

### 4.1 `tools/check_localization_drive.py`

Drives a fixed straight leg (default 15 m at 0.5 m/s via `cmd_vel`, no nav2)
and logs, at 10 Hz:

- EKF map pose (`map -> base_link` from TF)
- raw GPS map position (`odometry/gps`)
- Gazebo truth (`/world/park/pose/info` gz topic or `gz model -p`)

Reports: max and mean position error **in motion**, position error **at rest**
after stopping, heading error at rest and in motion. Pass thresholds are
parameters with defaults `0.5 m` motion, `0.2 m` rest, `2 deg` heading.

This is the test that would have caught every fault in §1. It becomes a gate in
`RUN_SIM.md` before any goal is sent.

### 4.2 `tools/check_route.py`

Sends `routes/park_route_1.yaml` through nav2 waypoint by waypoint and, at each
reported arrival, records Gazebo truth distance to the waypoint. Pass = every
stop within `0.5 m`. This is the acceptance test for "the robot navigates".

### 4.3 `tools/check_nav2_ready.py` replaced

The current script hangs under load and returns empty output. Replacement is
rclpy-only, bounded to 20 s wall time, prints one line per check and a final
`READY` / `NOT READY: <first failing check>`, and exits nonzero on NOT READY.
Checks: `/clock` has a publisher; all 13 lifecycle nodes `active`;
`map -> base_link` resolves; global costmap has nonzero occupied cells.

---

## 5. Goal tooling

`nav_goal.py` / `nav_goal_ll.py` / `nav_route.py`: orientation is no longer
consulted by the goal checker, so the `YAW_DEG` argument is removed from the
scripts and their docs rather than left as a trap. The action still requires a
pose; the scripts send the bearing from robot to goal so RViz displays
something sensible.

---

## 6. Runbooks

- `RUN_SIM.md` Step 7 = the new `check_nav2_ready.py` **and** §4.1's drive
  check. Step 9 = §4.2.
- `CLEAN_SIM.md` unchanged beyond today's daemon fix.
- `CLAUDE.md` gains gotchas for: IMU initial-pose reference; goal yaw default;
  dual lifecycle managers; the 1 Hz GPS dead-reckoning gap (once measured).

---

## 7. Order of work

1. §4.1 drive check (measure the 1.9 m fault before touching it)
2. §3.1 IMU probe -> custom IMU -> delete relay -> §4.1 must still pass
3. §3.2 GPS 10 Hz -> covariance check -> delete relay if possible -> §4.1
4. §3.3 tuning only if §4.1 still fails
5. §4.3 readiness replacement, §5 goal tooling
6. §4.2 route acceptance, runbooks, gotchas

Each step ends with §4.1 green. No step changes nav2 parameters.
