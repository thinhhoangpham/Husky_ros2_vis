# Demos

Steps only. Each demo is self-contained and runs verbatim, top to bottom.
Background and rationale live in `CLAUDE.md`.

Prerequisites for every demo: `CLEAN_SIM.md` has reported clean, and
`RUN_SIM.md` has completed through Step 5.

---

## Demo: uphill traction on lake

Proves the robot climbs a grade without sliding sideways off its heading.
Fails if `urdf/wheel_slip.urdf.xacro` is not reaching the robot.

**World:** `lake`
**Robot config:** `default`
**Spawn override:** none — the demo teleports to the test start instead

### Step 1 — Place the robot at the bottom of the longest climb

The dip at `(32, -16)` begins the longest sustained monotonic rise in the world:
20 m of travel in `-x` for 1.79 m of gain, a 5.1° mean grade. Yaw `π` faces it.

```bash
source /opt/ros/jazzy/setup.bash
gz service -s /world/lake/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'name: "a200_0000/robot" position { x: 32 y: -16 z: 4.36 } orientation { x: 0 y: 0 z: 1 w: 0 }'
gz model -m a200_0000/robot -p
```

Required: reported position within 0.1 m of `x=32, y=-16`, yaw ≈ `±3.14`.

`set_pose` returns `data: true` even for a nonexistent entity, so the
`gz model` read is the actual check — not the service response.

### Step 2 — Drive uphill and sample

```bash
ros2 topic pub -r 20 -t 1600 /a200_0000/cmd_vel \
  geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 1.0}}}' >/dev/null 2>&1 &
PUB=$!

for i in 1 2 3 4; do
  gz model -m a200_0000/robot -p 2>/dev/null | grep -A1 "Pose \[ XYZ" | tail -1 | tr -s ' '
done

kill $PUB 2>/dev/null    # stop driving as soon as sampling ends
```

The publisher outlives the four samples by a wide margin, so it must be stopped
explicitly — otherwise the robot keeps climbing for another minute after the
measurement window and whatever runs next inherits an unpredictable pose.

Capture the PID rather than reaching for `pkill`: a `pkill -f` pattern matching
`cmd_vel` also matches the shell running it (CLAUDE.md gotcha #9).

Required, comparing the first and last samples:

| quantity | required | meaning |
|---|---|---|
| `x` decreases | by > 2 m | it is moving, and uphill |
| `z` increases | by > 0.1 m | it is climbing, not stalling |
| **`y` drift** | **< 0.05 m per metre of `x` travelled** | it is not sliding sideways |

The drift figure is the point of the demo. Reference measurements on this
grade: **0.0045 m/m** with the wheel-slip override, **0.198 m/m** on Clearpath's
stock settings — a 44× difference, so the 0.05 threshold separates them
unambiguously rather than finely.

### Cleanup

Step 2 stops the drive. The robot is left partway up the slope — fine to leave,
since the demo teleports to a known start rather than assuming a pose. Tear the
sim down with `CLEAN_SIM.md` when finished with it.

### If it fails

A drift near 0.2 m/m means `urdf/wheel_slip.urdf.xacro` is not reaching the
robot. Check that the live config's `platform.extras.urdf` points at it —
`robot_default.yaml` and `robot_full.yaml` reach it by different paths. See
CLAUDE.md gotcha #24.

---

## Adding a demo

Copy the block below and fill it in. Keep it executable: commands only, with
required results stated so a step can pass or fail rather than be judged.

```markdown
## Demo: <short name>

**World:** <park | lake | warehouse_ext | warehouse_ramp>
**Robot config:** <default | full>
**Spawn override:** <SPAWN_X=… SPAWN_Y=… SPAWN_Z=… SPAWN_YAW=… | none>

### Step 1 — <what it does>

​```bash
<command>
​```

Required: <the observable result that decides pass or fail>

### Step 2 — <…>

​```bash
<command>
​```

Required: <…>

### Cleanup

<anything to restore, or "none — CLEAN_SIM.md handles it">
```

Two things that make a demo reproducible rather than merely repeatable:

- **State the required result, not the intent.** "Robot reaches the far bench"
  cannot be checked; "`gz model -m a200_0000/robot -p` reports x < 20" can.
- **Name the world and config explicitly.** A demo that silently assumes the
  previous demo's state breaks the moment it is run first, and that failure is
  hard to attribute.

If a demo needs a spawn pose other than the world default, put the `SPAWN_*`
override in the demo's own launch step rather than editing `RUN_SIM.md` — the
runbook's defaults are the authored poses and should stay that way.
