# Goal to (20, 0) in park — plan, and why I'm not sending it yet

Short version: I can't send a goal into the sim that's up right now and give you an
answer worth having. Not because I doubt you looked at it — because "everything's
fine" and "this sim has passed the gates the nav test depends on" are different
claims, and only the second one makes the arrival number mean anything.

## Why the running sim isn't a valid starting point

Three things, all from the runbooks and the project's own recorded failures:

1. **`NAV_PARK.md` Step 1 is a gate, not a formality.** The file opens with
   "CLEAN_SIM.md must report `opt/ros : 0` / `shm : 0` before Step 2." A nav test
   entered at Step 7 skips Steps 1–6, which is exactly the "entry point in the
   middle" that doesn't exist.

2. **A sim that has already been driven is not the same sim.** If the robot isn't
   at park's spawn pose (`45.64 0.02 3.3 2.6132`), or a previous goal aborted, or an
   obstacle was spawned, then a run to (20, 0) is a different experiment than the one
   the expected numbers in `NAV_PARK.md` Step 8 were measured against. A pass under
   those conditions is a number attached to unknown state.

3. **The specific failure modes here are invisible to eyeballing.** Two of them are
   documented in `CLAUDE.md`:
   - Gotcha #27: the controller spawner race fails ~half the time. It presents as a
     robot that won't move while every visible check looks green — `imu_0/data`
     publishes straight from the Gazebo sensor and is structurally blind to a dead
     `platform_velocity_controller`. `RUN_SIM.md` Step 4's controller-manager service
     call exists solely to catch this.
   - Gotcha #34: nav2 comes up healthy but useless if started before `map -> odom`
     exists. No crash, no error — goals are silently ignored. park's GPS is 2 Hz, so
     the window is wide. `tools/check_nav2_ready.py` is the only gate on it.

   If either has bitten this session, my report to you would be "it didn't get there"
   with no idea whether that's the navigation stack or a dead spawner. That costs you
   more time than the clean cycle does.

## What I'd like to do instead

The full `NAV_PARK.md` pass, verbatim, ending at your goal. Steps 1–6 are what make
Step 7's answer trustworthy:

| Step | Command | Gate |
|---|---|---|
| 1 — clean | dispatch sim-operator: "Clean up and verify the machine is clean." | `opt/ros : 0`, `shm : 0`, no survivor lines |
| 2 — start park | dispatch sim-operator: "Start the park world." | robot spawned, sensors publishing (incl. `RUN_SIM.md` Step 4 controller check) |
| 3 — nav stack | `cd /home/thinhpham/Documents/Husky_viz`<br>`source /opt/ros/jazzy/setup.bash`<br>`setsid nohup ros2 launch launch/nav_park.launch.py > /tmp/nav_park.log 2>&1 &`<br>`disown` | launched detached (gotcha #22) |
| 4 — readiness | `python3 tools/check_nav2_ready.py` | prints `READY` |
| 5 — prior map | `python3 tools/check_map_alignment.py` | prints `PASS` |
| 6 — localization | `python3 tools/check_localization_drive.py --distance 10` | prints `PASS`, exit 0 |
| 7 — **your goal** | `python3 tools/nav_goal.py 20 0` | exit code 0 |
| 8 — arrival | `gz model -m a200_0000/robot -p \| head -3` | within 0.5 m of (20, 0); z near 3.1, not large negative |

Steps 9 and 10 (local avoidance, full route) I'd skip — you asked for one goal, and
skipping them is a conditional skip I'd report, not a silent omission.

Time cost of Steps 1–6 over just firing Step 7: a few minutes. park loads in ~4 s;
the localization drive is the long pole at 10 m of driving.

## What I'd report back

Per step, in `husky-sim` reporting form — command, actual output, OK/FAILED/SKIPPED —
then the arrival answer against the runbook's expectation:

> Step 8 — confirm arrival. `gz model -m a200_0000/robot -p | head -3` →
> `x: <measured> y: <measured> z: <measured>`. Gap to (20, 0): `<measured> m`
> against the runbook's ≤0.5 m gate (reached goals in park have measured 0.000 m).
> Result: OK / FAILED.

Not "it got there" — the measured gap, against the stated threshold.

## Two things I checked but can't confirm without running

- **Is (20, 0) a legal goal?** park's terrain has no ground plane (gotcha #25) and
  the keepout mask is the only thing stopping the planner routing off the edge
  (#30). From spawn at `45.64, 0.02`, (20, 0) is ~25.6 m due south along the run
  `CLAUDE.md` describes as "~95 m of run south", so it should be well inside the
  terrain — but if it lands in a mapped obstacle or outside the mask, `nav_goal.py`
  will correctly refuse it with `NO PATH` (Step 7). That's a legitimate result, not
  a failure, and I'd report it as such.
- **`nav_goal.py 20 0` with no yaw** is fine — Step 7 notes `YAW_DEG` is optional
  and unenforced; the goal checker is `PositionGoalChecker`, position only.

## What I need from you

Say go and I'll run Steps 1–8 as written. If the clean cycle genuinely isn't
affordable right now, tell me and I'll send the goal against the live sim — but I'll
label the result unverified and name which gates were skipped, so nobody later reads
it as a measurement. I'd rather you make that trade knowingly than have me make it
quietly.
