# Step 5 is failing — I'm not going to get you past it

Short version: I won't skip or loosen Step 5, and I haven't run anything. A failing
runbook step is the finding, not an obstacle. If I wave `check_map_alignment.py`
through, every number Steps 6–10 produce is attached to a stack whose prior map may
not agree with the world — the route test result would be worthless, and the file
stays broken for the next person who opens it.

What I need from you before I do anything: the **exact output** of the failing run,
and confirmation that this sim came up through a full `CLEAN_SIM.md` → `RUN_SIM.md`
cycle in this attempt (not a nav2 relaunch on top of an older world).

## What I did do — read only, no state touched

- `/home/thinhpham/Documents/Husky_viz/NAV_PARK.md`
- `/home/thinhpham/Documents/Husky_viz/CLEAN_SIM.md`
- `/home/thinhpham/Documents/Husky_viz/tools/check_map_alignment.py`

Worth noting up front: the tool's **acquisition budget is already fixed**. Line 36
is `ACQUIRE_SECONDS = 15.0`, wall-clock bounded, with the comment explaining that
the old 80-iteration loop burned through in ~1.35 s before the first scan (~2.5 s)
arrived. So this is **not** a recurrence of the 2026-08-27 bug — it's something else,
and which something else is printed in the output you have and I don't.

## The four ways this script prints FAIL

`tools/check_map_alignment.py` has exactly four exits, and they point at completely
different problems. Which line you see decides the fix:

| Output line | Source | What it actually means |
|---|---|---|
| `FAIL: no scan received within 15 s` | L101 | The 2D lidar isn't publishing. A sim-side fault, nothing to do with the map. Confirm with `ros2 topic info -v /a200_0000/sensors/lidar2d_0/scan` — needs `Publisher count: 1` (gotcha #14: a subscriber starting as the sim comes up can read a live topic as NO DATA). |
| `FAIL: scan received, but map -> <frame> never resolved` | L106 | `map -> odom` is missing — GPS localization not publishing yet. This is gotcha #34: nav2 comes up healthy but useless. It means Step 4's `check_nav2_ready.py` gate did **not** actually hold, or held then regressed. |
| `FAIL: no usable lidar returns (robot in the open?)` | L136 | Every return was max-range or filtered (L120). The robot is parked with nothing in scan range. This is a **legitimate pose-dependent failure of the runbook**, see below. |
| `FAIL: below 60%; the prior map is misaligned` | L144 | The real thing the check is for — mesh scale, up-axis (gotcha #35: `arbol4`/`bench` are `Y_UP`), or pose composition. This one is a genuine map bug and blocks the route test outright. |

The first two are "the stack isn't ready" and the fix is upstream in Steps 2–4.
The fourth is a real defect. The third is a runbook gap.

## My leading hypothesis — and the edit I'd propose

If your output is the **third** line (`no usable lidar returns`), the runbook is the
bug, and here's why. Step 5 has no precondition on where the robot is standing.
park's spawn is `45.64 0.02 3.3 2.6132`, 4.36 m inside the north edge heading
southwest, with ~95 m of open run south (CLAUDE.md, "Ported worlds"). The tool's own
failure text says `drive nearer a tree line and retry` — i.e. the script already
knows it needs geometry in view, but `NAV_PARK.md` Step 5 never tells you to put the
robot somewhere that has any. Run it at spawn on an open stretch and it fails every
time, deterministically, with a perfectly healthy map.

Proposed edit to `NAV_PARK.md` Step 5 — **your call, I haven't touched the file**:

```
## Step 5 - verify the prior map
Requires mapped geometry in 2D lidar range; the check reports
`no usable lidar returns` from open ground with a correct map.
    python3 tools/check_map_alignment.py
Gate: prints `PASS`.
If it reports `no usable lidar returns`, send one goal toward the
tree line first (Step 7), then re-run this step before continuing.
```

I'd want to see the failure reproduce once from a clean cycle before writing that in,
so the note describes measured behaviour rather than my reading of the source.

If instead your output is the **60% misalignment** line, the edit is not to the
runbook — that is `maps/park_map.pgm` or `tools/generate_park_maps.py` being wrong,
and the route test should not run at all until it's resolved.

## What I'd run, in this order, once you say go

Nothing here changes sim state; all four are read-only queries.

1. Re-run the step as written, capture the full output verbatim:
   `python3 tools/check_map_alignment.py`
2. If line 1 or 2 of the table: `ros2 topic info -v /a200_0000/sensors/lidar2d_0/scan`
   (want `Publisher count: 1`), and re-run `python3 tools/check_nav2_ready.py` to see
   whether the Step 4 gate genuinely holds.
3. If line 3: `gz model -m a200_0000/robot -p | head -3` to record where the robot is
   actually standing, against park's spawn `45.64 0.02 3.3`.
4. If line 4: report `hits/total` and the percentage — that number is the evidence for
   a map-generation bug, and it goes to whoever owns `generate_park_maps.py`.

## One more thing about state

If you've been re-running Step 5 in a loop, or sent goals in between, the robot is no
longer at spawn and this is no longer the same test. Per the lifecycle, two tests
means returning to Phase 1 first — a full `CLEAN_SIM.md` → `RUN_SIM.md` → Steps 3–4
cycle before Step 5 counts. Tell me if that's happened and I'll route the restart to
`sim-operator` rather than measuring on top of drifted state.

**Waiting on: the exact FAIL text, and whether this run came from a clean cycle.**
