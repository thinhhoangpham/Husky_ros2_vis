---
name: husky-sim
description: The runbooks at the project root (CLEAN_SIM.md, RUN_SIM.md, NAV_PARK.md, DEMO.md) are the only authority on stopping, starting, verifying, testing and demoing the Husky Gazebo simulation. Use this skill for EVERY sim operation without exception — "start park", "run the sim", "restart it", "kill it", "is it running", "test obstacle avoidance", "send a goal", "spawn a box", "run the demo", "check localization" — and for any `run_husky_sim.sh`, `ros2 launch`, `gz sim`, `gz service`, `nav_goal`, `nav_route` or `tools/check_*.py` invocation. Use it especially when a test is about to run against an already-running sim, because that is the single most common way results get silently invalidated here. Read the runbooks fresh every time and execute their numbered steps verbatim; when a step fails, the runbook is the bug — stop and propose the edit rather than working around it.
---

# Operating the Husky sim

Four runbooks at `/home/thinhpham/Documents/Husky_viz/` are the procedure:

| File | Covers |
|---|---|
| `CLEAN_SIM.md` | `python3 scripts/sim.py stop` — must print `CLEAN` |
| `RUN_SIM.md` | `python3 scripts/sim.py start <world> [--config full]` — must print `READY` |
| `NAV_PARK.md` | nav2 + GPS localization in park, goals, routes, avoidance |
| `DEMO.md` | demo scenarios, one self-contained block each |

`CLEAN_SIM.md` and `RUN_SIM.md` are each now a single `scripts/sim.py`
command rather than a numbered procedure — `start` cleans, launches and
verifies every gate itself, and `stop` cleans and verifies. Rationale lives
in `CLAUDE.md`. They are maintained for demos and **they change**. Your
memory of last session is not evidence of what they say now.

## Three laws

**1. No test begins without proof the machine is clean.**
Not a belief that it is clean. Not "I killed it a minute ago." A
`python3 scripts/sim.py start <world>` run ending in the `READY` verdict
line (it cleans first), or a `stop` run ending in `CLEAN` if no start
follows.

**2. Every command in a test comes from a runbook, verbatim.**
Not an equivalent you composed. If the sequence you need isn't in a runbook, that
is a gap to fill in the file first — not a reason to improvise it in chat.

**3. A failing step means the runbook is wrong.**
Stop, report the evidence, propose the specific edit, wait. Working around it
destroys the finding and leaves the file broken for whoever runs it next.

These exist because each was violated in a single session on 2026-08-27, and each
violation cost an hour or more. The details are in "What going wrong looks like".

## The lifecycle

Every sim operation is one pass through this. There is no entry point in the
middle.

```
sim.py start  →  [NAV_PARK.md]  →  [the test]
  READY            verify          measure
```

### Phase 0 — before anything

State which runbooks this operation needs and read each one **in full, now**.
Not a section, not from memory, not from this skill. For `NAV_PARK.md` /
`DEMO.md`, list the numbered steps you are about to run, in the file's order,
so a missing step is visible before it costs anything.

### Phase 1 — clean, launch and verify

Run `python3 scripts/sim.py start <world> [--config full]` (see `RUN_SIM.md`;
`CLEAN_SIM.md` is folded into this — it cleans before doing anything else, so
no separate pass is needed). Relay every phase line it prints and the final
verdict line verbatim. Every phase it checks is a gate, including the ones
that look redundant — several exist precisely because an earlier hand-run
procedure passed while the sim was broken:

- The `controllers` phase exists because `imu_0/data` publishes straight from
  the Gazebo sensor and is structurally blind to a dead controller spawner. A
  sim used to pass every gate with `platform_velocity_controller` absent —
  `sim.py` queries `list_controllers` directly and recovers with
  `--switch-timeout 30` if needed (CLAUDE.md gotcha #27).
- If it prints `FAIL <n> <phase>: <observation>`, report that line plus the
  last ~30 lines of the log it names (`/tmp/sim.log`, `/tmp/bridge.log`, or
  `/tmp/nav.log`). Do not re-run and do not investigate — that is the finding.

Gate: the verdict line reads `READY ...`. Do not proceed to a test until it
does. To stop without starting another sim, run
`python3 scripts/sim.py stop` — required last line `CLEAN`.

### Phase 2 — the test

Only now. And the test itself follows the same rules: its steps come from
`NAV_PARK.md` or `DEMO.md`, verbatim.

If the user asks for something no runbook covers — a new obstacle scenario, a
different goal, a sensor experiment — the honest move is to say so and offer to
add the block to `DEMO.md` first. A test that exists only in a chat message
cannot be reproduced by whoever opens the file next, and its result is worth
correspondingly less.

**Between two tests, return to Phase 1.** (re-run `sim.py start`) A second test on a world that already
has a spawned object, a robot 20 m off its spawn pose, or an aborted goal in its
history is not the same test. State-dependent results are how a passing number
gets attached to a broken system.

## What going wrong looks like

Measured on 2026-08-27. Each of these felt reasonable at the time.

| What happened | What it cost |
|---|---|
| Killed a running test mid-flight, then kept using the half-torn-down sim | Spawned an obstacle and sent goals into a stack whose gates never passed; every number from it was meaningless |
| Ran a hand-rolled kill loop instead of `CLEAN_SIM.md` | The loop's pattern matched its own command line and killed the invoking shell (gotcha #9); 29 processes survived and the next launch inherited them |
| Launched nav2 by hand instead of through the documented step | Started a second stack on top of one already shutting down; spent a cycle diagnosing "nav2 won't come up" |
| Treated a failing `check_map_alignment.py` as an obstacle | Nearly restarted the whole world. The actual bug was a 1.35 s acquisition budget in the tool, found only by reading it — a one-line runbook-adjacent fix |
| Improvised around a slow gate rather than reporting it | Missed that the gate had no check for `robot_description`, which was the real failure |

The pattern in all five: improvising produced a plausible-looking result and
destroyed the evidence that would have found the real problem.

## Reporting

Per step, always:

```
Step N — <heading from the runbook>
  command: <exactly what you ran>
  output:  <actual output, or the values that matter>
  result:  OK | FAILED | SKIPPED (condition not met: <why>)
```

Close with a count: steps in the file, steps run, steps conditionally skipped.
Any discrepancy must already be explained above.

Where a runbook states an expected value, quote yours against it. Where it states
a required result, that is a gate — do not continue past it.

## When a step fails

Stop. Do not continue to later steps. Do not improvise a fix.

Report:
- which step failed, quoted from the file
- the exact command and its actual output
- your reading of why
- **a concrete proposed edit to the runbook** that would make it correct

Then wait. These files are the user's demo material; changing them is their call.

Gathering more diagnostic information without asking is fine and encouraged —
it helps write the correct fix. The line is: reading a log, querying a topic, or
inspecting a tool's source to understand a failure is diagnosis. Launching the
sim a different way, or running a command the file doesn't specify, to get past
the failure is a workaround. Diagnose freely; never route around.

## Waiting

Never wait on a sim with a fixed `sleep`, and never delegate the wait to a script
that loops internally — that is the same thing wearing a different hat. Check
readiness yourself with one-shot commands and report immediately. If it isn't
ready, say so and check again with another one-shot command.
`tools/wait_for_ready.py` was tried and is explicitly rejected as a pattern.

One exception, and it is narrow: a check script bounding its own internal
message-acquisition window is not a readiness wrapper. `check_map_alignment.py`
legitimately waits up to 15 s for its first scan.

A topic appearing in `ros2 topic list` only means discovery found it, not that it
is publishing. Confirm `Publisher count: 1` on the topics that matter. And a
subscriber that starts as the sim comes up can read a live topic as "NO DATA" for
a few seconds — a false negative in the checking script, not a broken sensor
(gotcha #14). Confirm with a direct one-shot command before believing it.

## Scope

This governs stop, start, verify, test and demo. Editing worlds, writing tools,
and analysing results once measurements are in hand are ordinary work.

`CLAUDE.md` holds the gotchas and verified sensor semantics, and is where to look
when interpreting what a step's output means. Where the two describe the same
operational action, **the runbook wins.**
