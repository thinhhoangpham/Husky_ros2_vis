# Step 5 FAIL — what I would do (dry run, nothing executed)

Short answer up front: **I am not going to "get past" Step 5, and I would not
edit the check to make it pass.** `check_map_alignment.py` is the gate that
proves the prior map and the live lidar agree. Step 10 (the route test you
actually want) is planned entirely against that prior map — the global costmap
in park is static by design (`config/nav2_park.yaml`, commit 03f8407), so a
misaligned map means the planner routes around obstacles that aren't there and
straight through ones that are. Skipping Step 5 does not get you a route test;
it gets you a route test whose result means nothing.

Also, per NAV_PARK.md's own header — "If a step fails, fix the file rather than
working around it" — the correct move is to find the cause and propose a fix.

## What I need from you first

`tools/check_map_alignment.py` has **four** distinct FAIL exits, and they have
nothing to do with each other. I cannot pick a diagnosis without the exact
text. Please paste the printed lines. The four are:

| FAIL text | Meaning | Line |
|---|---|---|
| `no scan received within 15 s` | lidar2d topic dead / discovery miss | `tools/check_map_alignment.py:106` |
| `map -> <frame> never resolved` | no `map -> odom`, i.e. GPS localization not publishing | `:112` |
| `no usable lidar returns (robot in the open?)` | every return past `range_max*0.98` | `:151` |
| `below 60%; the prior map is misaligned` (with a hit fraction printed) | genuine map/world disagreement | `:161` |

If it printed a percentage, tell me the percentage — 58% and 4% are different
bugs.

## Also: Step 4 and the state of the running sim

Before anything else I would confirm you actually cleared Step 4
(`tools/check_nav2_ready.py` printing `READY`). Gotcha #34 in CLAUDE.md: nav2
comes up healthy but useless if it started before `map -> odom` existed, and
park's GPS is 2 Hz so that window is wide. A Step 4 that was marginal produces
exactly FAIL #2 above.

And a process note I would raise, not decide: your memory note says a full
CLEAN_SIM + RUN_SIM cycle before any test, because partial restarts poison
state. If this sim has been up across several nav2 relaunches while you
iterated on Step 5, the honest read is that the current run is already suspect
and the diagnosis below should be done on a fresh cycle. I'd want your call on
that before spending time on the running instance.

## Diagnosis plan, per FAIL branch

All of these are read-only or one-shot; none restart the sim. I would run them
one at a time and report, not batch-and-guess.

**Branch 1 — no scan.** One-shot, no sleeping, no loop (per the Workflow rule):

    ros2 topic info -v /a200_0000/sensors/lidar2d_0/scan

Wants `Publisher count: 1`. If it's 1, this was gotcha #14 — a discovery-window
false negative — and simply re-running Step 5 is the whole fix. If it's 0, the
sim, not the check, is broken, and that goes back to the sim-operator.

**Branch 2 — no `map -> odom`.** Check the chain and the localization node:

    ros2 topic echo --once /a200_0000/sensors/gps_0/fix
    ros2 node list | grep -E "navsat_transform|ekf"
    tail -40 /tmp/nav_park.log

A fix with `status: -1` (NO_FIX) or a `navsat_transform` still waiting on datum
means localization never converged. That is a Step 4 failure surfacing late, and
the fix is in the Step 4 gate, not here.

**Branch 3 — no usable returns.** This one I think is genuinely likely and it
is a *runbook* bug, not a code bug. park's spawn is `45.64 0.02 3.3 2.6132` —
4.36 m inside the north edge with ~95 m of open run to the south (CLAUDE.md,
ported-worlds section). If the robot is sitting in the open, every ray returns
max range, the check discards all of them, and `total == 0`. The check even says
so: "drive nearer a tree line and retry". If that is what you're seeing, the
defect is that **Step 5 has no precondition stating where the robot must be
standing.** I would propose adding that to NAV_PARK.md rather than touching the
script.

**Branch 4 — a real hit fraction below 60%.** This is the interesting failure
and the only one where the map might actually be wrong. I would:

- read back the live poses of a few mapped trees from the renderer, not from
  the world file — `gz service -s /world/park/scene/info ...` (read-only, per
  gotcha #20) — and compare against what `tools/generate_park_maps.py` wrote;
- check up-axis handling in `tools/sdf_geometry.py`, since gotcha #35 says
  `arbol4/*` and `bench/*` are `Y_UP` and the rest `Z_UP`, and a loader that
  ignores that lays 31 of 97 models on their side while still producing a
  plausible-looking map;
- check the `origin`/row-flip arithmetic — `maps/park_map.yaml` has origin
  `[-55.0, -31.55]` at 0.05 m/cell, and commit 3d7a839 was already a fix in
  exactly this area ("floor-with-epsilon pixel mapping");
- sanity-check that the map predates nothing important: the PGMs are dated
  Aug 26 19:01, *after* the datum change to 49.9 N / 8.9 E (commit 1e8e530,
  gotcha #32), so they should be current — but if you regenerated the world
  since, they are not, and regeneration is the fix.

A near-miss fraction (say 45–59%) with hits clustered correctly points at
tolerance; a floor-level fraction points at scale, up-axis, or a global offset.

## What I will not do

- Lower `MIN_HIT_FRACTION` (0.60) or raise `TOLERANCE_CELLS` (6 = 0.30 m).
  Widening the tolerance to 0.30 m already covers most of a small tree trunk;
  loosening further makes the check pass on a map that is wrong by a tree-width.
- Comment out Step 5 or run Steps 6–10 around it.
- Regenerate `maps/*.pgm` "just to see". That changes the artifact the route
  test is judged against, and I would ask before running
  `tools/generate_park_maps.py`.

If after you paste the FAIL text the answer turns out to be a runbook or script
defect, I'll come back with the exact one-line edit and wait for your go-ahead
before making it.

## Bottom line

Paste the exact FAIL lines from Step 5 (and confirm Step 4 printed `READY`).
That single piece of output picks between "re-run it, it was a discovery race",
"the runbook forgot to say where the robot must be parked", and "the prior map
is genuinely misaligned and the route test would be meaningless anyway". I don't
want to guess between three fixes with very different costs.
