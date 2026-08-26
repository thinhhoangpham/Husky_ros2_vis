# Autonomous navigation in `park` — GPS localization + Nav2

**Date:** 2026-08-26
**Status:** design approved, not yet implemented
**Scope:** give the Husky A200 a goal in `worlds/park.sdf` and have it plan a
route and drive there autonomously, avoiding obstacles.

---

## 1. Goal and non-goals

### Goal

Send the robot a destination — by clicking in RViz, by running a script, by
latitude/longitude, or from a waypoint file — and have it drive there on its
own, planning a route around park's 97 static models and staying on the
terrain.

### Non-goals

| Not doing | Why |
|---|---|
| GPS fault detection | Explicitly out of scope (decided 2026-08-26). Dead reckoning is architecturally isolated, so a detector can be added later without changing this design. |
| Navigation in `lake` | Lake has 2.43 m of relief and up to 21 deg slopes; nav2's 2D costmap assumptions do not hold there. Separate project. |
| Navigation in stock Clearpath worlds | park only. |
| 3D lidar in the costmap | Phase 2, after the 2D stack is verified end to end. |
| Tuning nav2 recovery behaviours | Left at Clearpath defaults until we observe whether they trigger in park. |
| Editing anything in `~/clearpath/` | Generated build output; edits are silently overwritten on the next `apply_config.sh`. |

---

## 2. Verified facts this design rests on

Everything in this section was measured or read from source on 2026-08-26, not
assumed. Re-verify before trusting any of it after a package upgrade.

### Installed software

| Item | Value | How verified |
|---|---|---|
| nav2 | `nav2_util` 1.3.12 | `package.xml` |
| `clearpath_nav2_demos` | 2.8.1, ships a tuned A200 config | `package.xml` |
| `slam_toolbox`, `robot_localization` | present | `ls /opt/ros/jazzy/share` |
| `costmap_filter_info_server`, `map_server` | present | `ls /opt/ros/jazzy/lib/nav2_map_server/` |
| `navsat_transform_node` | present | `ls /opt/ros/jazzy/lib/robot_localization/` |
| `pointcloud_to_laserscan` | **absent** | not needed — nav2 costmap layers take `PointCloud2` natively |

`navsat_transform_node` parameters confirmed present in
`/opt/ros/jazzy/lib/librl_lib.so` (the executable is an 18 KB wrapper; the
logic is in that library, so grep the library, not the binary):
`use_local_cartesian`, `datum`, `wait_for_datum`, `yaw_offset`,
`magnetic_declination_radians`, `use_odometry_yaw`, `transform_timeout`,
plus the `fromLL` / `toLL` services the lat/lon goal script needs.

### Existing localization

`~/clearpath/platform/config/localization.yaml` runs **one** `ekf_node`:

- `world_frame: odom`, `two_d_mode: True`, 50 Hz
- inputs: `platform/odom` (wheel) + `sensors/imu_0/data` (IMU)
- output: the `odom -> base_link` transform

There is **no** global EKF and **no** `navsat_transform_node` today. The file is
generated, so it must not be edited.

### Clearpath's A200 nav2 config

`/opt/ros/jazzy/share/clearpath_nav2_demos/config/a200/nav2.yaml`:

- MPPI controller, NavFn planner, Simple smoother
- `odom_topic: platform/odom/filtered`
- global costmap: `static_layer`, `obstacle_layer`, `inflation_layer`
- local costmap: `static_layer`, `voxel_layer`, `inflation_layer`
- **only observation source is `sensors/lidar2d_0/scan`** on both costmaps and
  on the collision monitor
- `enable_stamped_cmd_vel: true` already set on all 13 relevant nodes

### park geometry

Terrain model `parque`: unit-normalised mesh (+/-1), `<scale>50 25 0.01</scale>`,
model pose `(0, -1.55121, 2.98891)`.

| axis | extent |
|---|---|
| x | -50.00 .. +50.00 m |
| y | -26.55 .. +23.45 m |
| z | 2.98891 +/- 0.0035 m — **flat to 3.5 mm** |

Spawn (`scripts/run_husky_sim.sh`): `x=45.64 y=0.02 z=3.3 yaw=2.6132`.
That is **4.36 m from the x=+50 edge**. Distance to the other edges:
95.64 m (x=-50), 23.43 m (y=+23.45), 26.57 m (y=-26.55).

park declares (`worlds/park.sdf` lines 60-67):
`EARTH_WGS84`, `world_frame_orientation ENU`, `latitude_deg -22.986687`,
`longitude_deg -43.202501`, `elevation 0`, `heading_deg 0`.

Per CLAUDE.md gotcha #25, park has **no ground plane** — the terrain mesh is
the only surface.

---

## 3. Map orientation change

**Decision:** the map edge nearest the spawn is defined as **north**, which
makes the robot's spawn heading **southwest**.

Implemented as a one-line change in `worlds/park.sdf`:

```xml
<heading_deg>90</heading_deg>   <!-- was 0; sign to be verified empirically -->
```

### What this does and does not change

**Does not change:** anything physical. Terrain, the 97 models, robot spawn
position `x=45.64 y=0.02`, and `SPAWN_YAW=2.6132` are all untouched. The robot
is not moved and not rotated. `run_husky_sim.sh` needs no edit.

**Does change:** the mapping from world position to geographic coordinates, and
therefore the GPS readings. At the spawn, 45.64 m along +x becomes 45.64 m
*north* of the datum (a latitude offset of ~0.00041 deg) instead of 45.64 m
*east* (a longitude offset of ~0.000445 deg). Same robot, same place, different
numbers on `sensors/gps_0/fix`.

A magnetometer would also change, but park carries no `Magnetometer` system
(CLAUDE.md), so `compass_0/mag` is silent there regardless. Every purely
mechanical measurement — physics, lidar, wheel odometry, IMU rates — is
geographically blind and therefore unaffected. The only previously recorded
value this invalidates is park's GPS fix.

### Resulting orientation

With `+x = North`, `+y` becomes **West**:

| | |
|---|---|
| park size | **100 m north-south** (x), **50 m east-west** (y) |
| north edge | x = +50 |
| south edge | x = -50 |
| west edge | y = +23.45 |
| east edge | y = -26.55 |
| spawn | 4.36 m inside the **north** edge, mid-park east-west |
| spawn heading | bearing ~210 deg — **southwest** (0.864 south, 0.504 west) |
| usable run | ~95 m down the long axis |

### Open verification (blocking, do first)

The SDF 1.10 spec for `heading_deg` is self-contradictory: it says "positive
angle indicates clockwise rotation (from east to north)", but east-to-north is
counter-clockwise. **90 vs -90 must be settled by running it**, not by reading
docs. Drive the robot in +x and observe whether latitude or longitude changes,
and in which direction.

Rotating the world frame also rotates what the IMU calls zero heading, so
`navsat_transform`'s `yaw_offset` must move by the same 90 deg. If the two
disagree the robot's position and heading are perpendicular and it will circle.
They are a matched pair.

---

## 4. Architecture

Four layers, each independently launchable and verifiable.

```
                  goal (4 front ends)
  RViz 2D Goal Pose | nav_goal.py | nav_goal_ll.py | nav_route.py
                          |
                          |  NavigateToPose / FollowWaypoints
                          v
              +-------------------------+
              |  nav2                   |  global costmap:
              |  Clearpath a200 config  |    static_layer   (park_map)
              |  + park overlay         |    keepout_filter (park_keepout)
              +-------------------------+    obstacle_layer (lidar2d)
                          |
                          |  needs map -> odom
                          v
              +-------------------------+
              |  GPS localization  NEW  |
              |  navsat_transform_node  |  publishes map -> odom
              |  + global ekf_node      |
              +-------------------------+
                          |
                          v
              +-------------------------+
              |  Clearpath generated    |
              |  UNTOUCHED              |
              |  ekf_node               |  publishes odom -> base_link
              |  diff drive, sensors    |
              +-------------------------+
```

### The two-EKF split

Both EKFs fuse wheel odometry and IMU. The **only** difference is whether GPS is
an input.

| | local EKF (existing, generated) | global EKF (new, this repo) |
|---|---|---|
| wheel odometry | yes | yes |
| IMU | yes | yes |
| GPS | **no** | **yes** |
| `world_frame` | `odom` | `map` |
| publishes | `odom -> base_link` | `map -> odom` |
| character | smooth, slowly drifting | absolute, jumps on fix |

They write to **different links in one transform chain**, so they cannot
conflict and neither can overwrite the other.

Rationale for keeping both rather than fusing into one estimate:

- Nav2's MPPI controller infers motion from successive positions. A 30 cm GPS
  jump reads as violent lateral displacement and it will lurch to correct
  motion that never happened. Steering needs continuity.
- The planner needs absolute position or the prior map is useless. Drift of a
  few metres sends it confidently to the wrong place. Planning needs truth.

Each consumer is given the estimate whose failure mode it can tolerate. This is
REP-105's `map` / `odom` split.

### Dead-reckoning fallback

The global EKF **subscribes to** the local EKF's output. Subscription does not
modify it, and the local EKF has no GPS input at all, so no GPS value can reach
the dead-reckoning chain. `platform/odom` and `platform/odom/filtered` keep
exactly their present meaning and values.

On GPS dropout: the global EKF stops receiving corrections and coasts on wheels
+ IMU; `map -> odom` freezes at its last offset; nav2 keeps navigating with
accuracy decaying slowly. **This is expected behaviour, not a fault.** Do not
"fix" it.

The failure this does *not* cover is GPS that keeps publishing wrong values —
the filter fuses them and grows confident while growing wrong. Dead reckoning
stays uncorrupted underneath, but nothing switches to it automatically. Out of
scope by decision; see non-goals.

---

## 5. Components

### 5.1 GPS localization — `config/gps_localization.yaml`

Two nodes.

**`navsat_transform_node`**

```yaml
use_local_cartesian: true
wait_for_datum: true
datum: [-22.986687, -43.202501, 0.0]    # verbatim from park.sdf
yaw_offset: 1.5708                       # matched to heading_deg 90; verify sign
magnetic_declination_radians: 0.0        # no magnetometer in park
use_odometry_yaw: false                  # heading from IMU orientation
publish_filtered_gps: true               # for check tooling
broadcast_utm_transform: false
```

**Why `use_local_cartesian: true`** rather than the UTM default: UTM zones carry
a 500,000 m false easting, and float32 costmap arithmetic loses centimetres at
that magnitude. A local tangent plane keeps coordinates near zero where
precision is dense. For a 100 m park, UTM buys nothing and costs accuracy.

**Why the datum is pinned** rather than taken from the first fix: an unpinned
datum places the map origin wherever the robot spawned, shifting every run. That
silently invalidates the pre-generated map, every saved goal, and every waypoint
file. Pinned to park's declared origin, the ROS `map` frame lands exactly on the
Gazebo world frame — same origin, same axes — because Gazebo's `NavSat` sensor
derives lat/lon from that same declaration. A tree at Gazebo `x=12.3, y=-4.1` is
at map `x=12.3, y=-4.1`. No registration step exists to get wrong.

**Global `ekf_node`** — same structure as the generated local EKF, with:

```yaml
world_frame: map
two_d_mode: true            # matches the local EKF; park is flat to 3.5 mm
frequency: 30.0
odom0: platform/odom
imu0: sensors/imu_0/data
odom1: odometry/gps         # from navsat_transform
publish_tf: true            # publishes map -> odom ONLY
```

### 5.2 Map generator — `tools/generate_park_maps.py`

Reads `worlds/park.sdf` and emits two aligned rasters.

| | value |
|---|---|
| resolution | 0.05 m/cell |
| extent | x -55 .. +55, y -31.55 .. +28.45 (terrain + 5 m margin) |
| size | 2200 x 1200 cells |
| `origin` in both `.yaml` | `[-55.0, -31.55, 0.0]` |

The 5 m margin exists so the keepout mask has cells *outside* the terrain to
mark lethal. A mask cropped exactly to the terrain has nowhere to draw the
border.

- **`maps/park_map.{pgm,yaml}`** — occupied where model collision geometry
  intersects the robot's height band, computed from each model's pose, mesh, and
  scale.
- **`maps/park_keepout.{pgm,yaml}`** — free inside the terrain rectangle,
  lethal outside.

Both are generated from **one origin constant in one script**, which is what
guarantees they stay aligned. Nav2 requires identical `origin` and `resolution`
in both `.yaml` files; a mask off by one cell silently shifts the no-go zone.

Committed as artifacts so runs are reproducible without regenerating.

**Why a prior map at all** (Approach A, chosen over sensor-only and over a
one-off SLAM pass): the keepout mask already forces map-raster tooling with
park's exact origin and resolution to exist, so emitting a second aligned layer
from the same collision geometry is a small increment rather than a new
capability. It is reproducible from the repo, unlike a hand-driven SLAM map.
Without it, the first path to any distant goal is planned through unknown space
and replans repeatedly as trees appear.

A ground-truth prior is optimistic — the robot "knows" things a real one could
not. Accepted deliberately: for the later adverse-conditions work, a fixed
correct map means the map is not a variable, so any localization degradation is
attributable to the GPS input alone.

### 5.3 Boundary — keepout filter

park has no ground plane. A lidar ray aimed past the terrain edge hits nothing
and returns max range, which the obstacle layer treats as **evidence of free
space** and actively clears. The void does not merely fail to register as an
obstacle — it is marked traversable. No sensor configuration can fix this; it
must be asserted as prior knowledge.

Nav2's mechanism is `KeepoutFilter`: a mask served like a map that stamps lethal
cost regardless of sensor input. Applied to **both** costmaps, served by a
`map_server` plus `costmap_filter_info_server`.

**No world edits for the boundary** (decided): no invisible collision walls.
The only change to `park.sdf` in this project is `heading_deg`.

### 5.4 Nav2 config — `config/nav2_park.yaml`

Copy of Clearpath's `a200/nav2.yaml`, **deltas only**, so upstream tuning stays
visible and a Clearpath upgrade is easy to re-diff:

1. `global_costmap`: `rolling_window: false`, sized to the map, `static_layer`
   against `park_map`.
2. Both costmaps gain a `keepout_filter` plugin pointing at the filter-info
   topic.
3. `robot_radius` / inflation reviewed against the A200 footprint
   (~0.99 x 0.67 m). The shipped value is tuned for warehouse aisles, which is
   unnecessarily timid in open parkland.
4. **Phase 2 only:** `observation_sources` gains `pointcloud` from
   `sensors/lidar3d_0/points`, with `min_obstacle_height` / `max_obstacle_height`
   gating so ground returns and canopy are not marked.

Unchanged: MPPI, NavFn, smoother, behaviours, collision monitor,
`odom_topic: platform/odom/filtered`.

### 5.5 Goal front ends

All four call the same nav2 action interface underneath.

| Front end | Interface |
|---|---|
| RViz "2D Goal Pose" | launch with `rviz:=true` |
| `tools/nav_goal.py X Y YAW` | `NavigateToPose` in the map frame |
| `tools/nav_goal_ll.py LAT LON` | `fromLL` service, then `NavigateToPose` |
| `tools/nav_route.py FILE` | `FollowWaypoints` |

All three scripts reject goals outside the terrain rectangle before sending.

### 5.6 Launch — `launch/nav_park.launch.py`

| Stage | Brings up | Gate before next stage |
|---|---|---|
| 0 | *(existing sim + Clearpath stack)* | `platform/odom/filtered` has a publisher |
| 1 | `navsat_transform_node` + global `ekf_node` | `map -> odom` resolves in TF |
| 2 | `map_server` x2 + `costmap_filter_info_server` | all lifecycle nodes `active` |
| 3 | nav2 with `config/nav2_park.yaml` | `NavigateToPose` action server available |

Stage 1 -> 2 is a real constraint. Nav2 transitions `planner_server` to active
and immediately looks up `map -> odom`; if `navsat_transform` has no fix yet the
lookup fails and nav2 comes up **healthy but useless** — no crash, no error, goals
silently ignored. park's GPS runs at **1 Hz** (verified rate table), so the
window is wide.

Gates are **one-shot checks, never sleeps or polling loops**, per CLAUDE.md.

---

## 6. Failure modes and guards

| # | Failure | Guard |
|---|---|---|
| 1 | Nav2 up but ignores goals (no `map -> odom`) | `tools/check_nav2_ready.py`: TF resolves, lifecycle nodes active, action server present |
| 2 | Robot never moves (cmd_vel type mismatch) | Explicit first-run check of the published type and subscriber on `/a200_0000/cmd_vel`. See open question 6.1 |
| 3 | Robot drives off the terrain | Keepout mask (planner) + goal-script rejection (input) + z reported in check output (diagnosis, gotcha #25) |
| 4 | Prior map misaligned with reality | `tools/check_map_alignment.py`: park, take a scan, compare mapped vs observed obstacle positions. A number, not a look at RViz |
| 5 | GPS dropout | Not a failure. Documented expected degradation |
| 6 | Leftovers from a previous run | `CLEAN_SIM.md` unchanged, but `kill_sim.sh`'s pattern list gains ~10 nav2 nodes. Per gotcha #21 that list has been wrong twice — verify with an independent sweep, never with its own success message |
| 7 | Nav2 spins recovering | Clearpath defaults for now; tune only if observed |

Most of these are "healthy but useless" rather than crashes. ROS nodes are
loosely coupled by topics, so a mismatch in type, frame, or timing produces
silence, not an error. Guards are therefore **positive checks** ("a publisher
exists and it is this type"), not error watching.

### 6.1 Open question — cmd_vel message type

CLAUDE.md gotcha #3: the Husky's `cmd_vel` is `TwistStamped`, not `Twist`.

Clearpath's `a200/nav2.yaml` sets `enable_stamped_cmd_vel: true` on all 13
relevant nodes, so nothing needs adding there. However, on this install:

- `nav2_util` 1.3.12 exposes only `validateTwist(const TwistStamped&)`
- the string `enable_stamped_cmd_vel` appears in nav2's **behavior** libraries
  but **not** in the controller, velocity-smoother, or collision-monitor ones

That is consistent with this nav2 version having moved to `TwistStamped`
unconditionally and dropped the toggle, but it **could not be settled from the
binaries** and is not asserted here. Verify empirically before tuning anything
else. If it does mismatch, the fix is a small relay node — cheap, but far better
known on day one than after a day of debugging "why won't it move".

---

## 7. Verification

Following the `tools/check_<what>.py` convention — each named after what it
proves.

| Script | Proves |
|---|---|
| `check_nav2_ready.py` | the stack is actually up, not merely launched |
| `check_map_alignment.py` | the generated map matches what the lidar sees |
| `check_nav_goal.py` | end to end: goal in, robot arrives within tolerance |

### Acceptance test

From the standard spawn (`45.64, 0.02`, yaw `2.6132` — 4.4 m inside the north
edge, heading southwest), send a goal ~40 m south down the park. The robot must:

- arrive within **0.5 m** of the goal
- collide with nothing
- never leave the terrain
- report success through nav2's **action result**, not by looking right on screen

### Runbook

`NAV_PARK.md` at the project root, in the style of `RUN_SIM.md`: numbered steps,
no explanation, verified gates — repeatable by the user or the `sim-operator`
agent without improvising.

---

## 8. Files

| Path | Purpose |
|---|---|
| `config/nav2_park.yaml` | Clearpath a200 config + park deltas |
| `config/gps_localization.yaml` | global `ekf_node` + `navsat_transform_node` |
| `launch/nav_park.launch.py` | staged bring-up with gates |
| `maps/park_map.{pgm,yaml}` | prior obstacle map, generated, committed |
| `maps/park_keepout.{pgm,yaml}` | terrain boundary mask, generated, committed |
| `tools/generate_park_maps.py` | rasterizes both from `park.sdf` |
| `tools/nav_goal.py` | goal by map x/y/yaw |
| `tools/nav_goal_ll.py` | goal by lat/lon via `fromLL` |
| `tools/nav_route.py` | waypoint sequence from a file |
| `tools/check_nav2_ready.py` | readiness gate, one-shot |
| `tools/check_map_alignment.py` | map vs lidar agreement |
| `tools/check_nav_goal.py` | end-to-end goal test |
| `NAV_PARK.md` | runbook |
| `worlds/park.sdf` | **modified:** `heading_deg` only |
| `scripts/kill_sim.sh` | **modified:** nav2 nodes added to the pattern list |
| `CLAUDE.md` | **modified:** `config/` and `maps/` rows added to the layout table; orientation and GPS notes |

`config/` and `maps/` are new top-level directories. Config deliberately does
**not** go in `robot_configs/`, which has a specific meaning in this project
(Clearpath robot descriptions consumed by `apply_config.sh`). Nav2 tuning files
are not that.

---

## 9. Build order

1. **Map generator** + the two rasters. No dependencies, and everything
   downstream needs it. Also the piece most likely to hide a subtle error — a
   wrong mesh scale produces a map that looks fine until the robot hits a tree —
   so it must be trustworthy before anything is debugged on top of it.
2. **`heading_deg` sign verification** and the GPS localization layer, verified
   by `map -> odom` existing and being sane.
3. **Nav2 + keepout filter**, verified by the cmd_vel check (6.1) and a short goal.
4. **The four goal front ends.**
5. **Acceptance test** (section 7).
6. **Phase 2, separately:** 3D lidar into the costmap.
