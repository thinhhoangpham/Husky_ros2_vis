# Open issue — `--config full` + nav2 stalls on park

Recorded 2026-08-28 from the live acceptance runs of `scripts/sim.py`.
Parked deliberately: this is a nav2 bring-up problem, not a `sim.py` defect.
`scripts/sim.py` reports it correctly (phase 6 FAIL, exit 16) rather than
falsely passing — which is the gate working as designed.

## Symptom

`python3 scripts/sim.py start park --config full` reaches READY through
phases 0-5 (gps_0/fix, compass_0/mag and camera_0/color/image each with
exactly 1 publisher) and then fails phase 6.

`map_server` activates; the lifecycle manager then stalls. Observed across
two runs, the nodes that never reach `Activating`:

- `filter_mask_server`
- `costmap_filter_info_server`
- `controller_server`

Their `get_state` services answer `label='inactive'` or stop answering.

## Two distinct observations, two different runs

| Run | RTF measured | Extra symptom in `/tmp/nav.log` |
|---|---|---|
| acceptance run 4 | 0.10-0.14 | repeated `ekf_node_map: Failed to meet update rate!` |
| acceptance run 5 | **1.00** | no update-rate warnings at all |

Run 4 looked like the documented RTF collapse under `full` (the RealSense
camera — see CLAUDE.md's park/lake section), and the nav2 budget is now
scaled by measured RTF because of it. Run 5 then stalled the same way at
RTF 1.00, so **RTF is not the whole cause**. Whatever stalls the lifecycle
manager under `full` is still unidentified.

## What is already ruled out

- Not a deadline problem alone: run 5 had a full budget at RTF 1.00 and
  still stalled.
- Not the gate's logic: the same gate passes park 5/5 under `default`.
- Not GPS bridging: `full` now declares `gps:` like `default`, and
  `gps_0/fix` was verified at exactly 1 publisher.

## Where to start

Compare a `default` and a `full` nav2 bring-up side by side and find what
differs for the three stalled servers specifically — they are the costmap
filter chain plus the controller, so the costmap filter info / mask topics
are the obvious first suspects under a config that adds a camera and its
point cloud.

`default` is the verified path and is unaffected (park 5/5 READY).
