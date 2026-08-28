# Goal to (20, 0) in park — what I would do, and what I need from you first

**Nothing was executed.** This is a dry run, and the running sim was not touched.

## Short answer

I can't honestly tell you "it got there" from a goal sent into a sim I haven't
verified. `NAV_PARK.md` puts four gates in front of Step 7 (send a goal), and
each one exists because a sim has already passed a casual eyeball check while
being silently broken. I'd rather spend ~60 seconds on read-only gates than hand
you a number that means nothing.

The gates are read-only. They don't restart anything.

## What I would actually run

All of these only read state or drive the robot a short distance — none of them
kill, relaunch, or respawn anything.

### 1. Confirm nav2 is actually up and localized (NAV_PARK.md Step 4)

```bash
cd /home/thinhpham/Documents/Husky_viz
source /opt/ros/jazzy/setup.bash
python3 tools/check_nav2_ready.py
```

Gate: prints `READY`.

Why this one is non-negotiable: gotcha #34 — **nav2 comes up healthy but useless
if it started before `map -> odom` existed. No crash, no error, goals silently
ignored.** park's GPS is 2 Hz so the window is wide. If this is the state, my
report to you would be "it never moved" and that would look like a nav failure
when it's a startup ordering issue.

I'd also confirm the drive chain is alive, since gotcha #27 says the controller
spawner races and fails roughly half the time while every other gate looks green:

```bash
ros2 service call /a200_0000/controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers "{}"
ros2 topic info -v /a200_0000/platform/odom
```

Gate: `joint_state_broadcaster` and `platform_velocity_controller` both `active`;
`Publisher count: 1` on odom. (`ros2 control` CLI is not installed here —
gotcha #33 — hence the service call form.)

### 2. Where is the robot right now?

```bash
gz model -m a200_0000/robot -p | head -3
```

This is the one thing I genuinely need and cannot get any other way: I have no
idea whether a previous test left the robot at spawn (`45.64, 0.02`) or 30 m
downrange with an aborted goal in its history. It also tells me the goal's actual
distance and whether z is sane (~3.1, not a large negative — gotcha #25/#23).

### 3. Send the goal (NAV_PARK.md Step 7, verbatim)

```bash
python3 tools/nav_goal.py 20 0
```

Gate: exit code 0. Yaw is optional and not enforced — the checker is
`PositionGoalChecker`, position only.

### 4. Confirm arrival (Step 8, verbatim)

```bash
gz model -m a200_0000/robot -p | head -3
```

Gate: within 0.5 m of (20, 0), z near 3.1. Reached goals in this world land
essentially exactly on target (measured gap 0.000 m), so a 0.3 m miss is itself
a signal, not noise.

I would report the measured position against (20, 0), the distance driven, and
whether nav2 reported recoveries or an abort.

## Two things about (20, 0) specifically

Worth knowing before we spend the run:

- **It may be legitimately refused.** `NAV_PARK.md` Step 7: "A goal inside a
  mapped obstacle or off the terrain is correctly refused with `NO PATH`." park's
  terrain is finite and the keepout mask is the only thing keeping the planner on
  it (gotchas #25, #30). If (20, 0) lands in a mapped tree or outside the mask,
  `NO PATH` is the correct answer and not a bug.
- **From spawn (45.64, 0.02) this is a ~25 m run roughly along -x.** With
  `default` config RTF ~1.0 that's on the order of a minute of driving. Fine.

## What I skipped and why

`NAV_PARK.md` Steps 1–2 (clean + restart), Step 5 (`check_map_alignment.py`),
Step 6 (`check_localization_drive.py`) — you said don't restart and you're short
on time, and Steps 1–2 are exactly what you ruled out. I'm flagging them rather
than pretending the run is a clean-room result:

**Caveat on whatever number I come back with:** without a
`CLEAN_SIM.md` → `RUN_SIM.md` cycle, this is a measurement on a sim in unknown
prior state. If the robot has already driven somewhere, or a goal was aborted
earlier, the result is not reproducible and shouldn't go in the dataset or a
demo. It's fine as a "does it basically work right now" spot check. That's the
trade you're making, and it's a reasonable one — I just won't quietly upgrade it
to "verified."

If Step 1 comes back not-`READY`, I will **stop and report**, not relaunch nav2
by hand. Launching a second nav2 on top of one already running has cost a full
diagnostic cycle here before.

## What I need from you

Say the word and I'll run Steps 1→4 above, read-only gates first, and report the
measured arrival position against (20, 0). Roughly a minute of gates plus the
drive.

If you'd rather skip even the readiness gate: I can fire `nav_goal.py 20 0`
immediately, but then a robot that doesn't move is ambiguous between "nav2 was
never localized" and "the goal was unreachable," and I'd have to run the gates
anyway to tell you which.
