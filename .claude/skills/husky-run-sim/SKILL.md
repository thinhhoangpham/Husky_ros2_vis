---
name: husky-run-sim
description: RUN_SIM.md at the project root is the only authority on launching and verifying the Husky Gazebo simulation. Use this skill EVERY time the sim is started or relaunched, or when checking whether it came up correctly — including any `run_husky_sim.sh`, `ros2 launch`, or `gz sim` invocation, and whenever the user says "run the sim", "start park", "launch lake", "restart it", or "is it running". Also use it when the sim misbehaves on startup — missing topics, no data, a robot that fell through the world — because the runbook already encodes how to detect those. Read RUN_SIM.md fresh each time and execute its numbered steps verbatim; never start the sim from memory of how it worked last time. Shutting down a previous sim is not covered here — the `husky-sim-restart` skill owns that and must report clean before Step 3.
---

# Running the Husky sim from RUN_SIM.md

`/home/thinhpham/Documents/Husky_viz/RUN_SIM.md` is the operating procedure for
this simulator. It contains numbered steps and nothing else — all background,
rationale and troubleshooting lives in `CLAUDE.md`, deliberately kept out so the
runbook stays executable.

It is maintained for demos and **it changes**. Your memory of how the sim started
last session is not evidence of how it starts now.

## Why exactness matters here

Someone follows this file in front of an audience, or hands it to a colleague.
If you quietly substitute a command that works better, three things go wrong:

- The file keeps saying something that doesn't work, and nobody finds out until
  the demo.
- Your success is unreproducible — a human running the same file gets a
  different result with no idea why.
- The real bug is hidden.

That last point is the one that matters. **A failing step is not an obstacle to
route around — it is the finding.** It means the file is out of date. Report it,
propose the correction, and let the human decide. Never repair the situation by
doing something the file doesn't say.

## Procedure

### 0. Clean the machine first

`RUN_SIM.md` starts at launch and assumes nothing is running. Shutting down any
previous sim is the `husky-sim-restart` skill's job — run it to completion and
confirm it reports `opt/ros : 0` and `shm : 0` before Step 3 here.

Launching on top of leftovers produces failures that look like world, sensor or
bridge bugs and cost far more to chase than the cleanup costs to finish.

### 1. Read the file, now

Read `RUN_SIM.md` in full at the start of every sim operation. Not a section,
not from memory, not from this skill — the actual file, this time.

### 2. List the steps before running anything

State the numbered steps you are about to perform, in the file's order, so the
plan is visible and a missing step is obvious before it costs anything.

Steps marked optional or conditional in the file's own words stay conditional —
honour the condition as written. A conditional step that didn't apply is not
skipped, but it must still appear in your report with the condition's outcome.

### 3. Execute verbatim

Run each command exactly as written, in order. Do not:

- reorder, merge, or split steps
- substitute a command you consider equivalent or better
- add flags, timeouts, retries, or `sleep`s the file doesn't specify
- skip a step because its precondition looks already satisfied
- skip a verification step because the previous step looked fine

Where a step states a required result ("Required: … `opt/ros : 0`"), that is a
gate. Do not continue past it until it holds.

The file is not a draft to improve. If it seems wrong, that belongs in your
report, not in your edits.

### 4. Report every step with evidence

For each step:

```
Step N — <heading from RUN_SIM.md>
  command: <exactly what you ran>
  output:  <actual output, or the specific values that matter>
  result:  OK | FAILED | SKIPPED (condition not met: <why>)
```

The evidence is the point. "Ran the cleanup" proves nothing; the actual
`opt/ros : 0` / `shm : 0` lines prove it. Where the file states an expected
value, quote what you observed against it — Step 7 says the base sits ~0.13 m
above ground, so report the measured z and the ground height, not "looks right".

Close with a count: how many steps the file contains, how many you ran, how many
were conditionally skipped. Any discrepancy must already be explained above.

### 5. When a step fails

Stop. Do not continue to later steps, and do not improvise a fix.

Report:
- which step failed, quoted from the file
- the exact command and its actual output
- your reading of why it failed
- a concrete proposed change to `RUN_SIM.md` that would make it correct

Then wait. Editing `RUN_SIM.md` changes the user's demo material and is their
call — propose it, don't apply it unasked.

You may gather extra diagnostic information without asking, since that helps
write the correct fix. Diagnosing is not working around: reading a log to
understand a failure is fine; launching the sim a different way to get past it
is not.

## Scope

This governs the start/stop/verify path only. Driving the robot, running
`tools/check_*.py`, editing worlds, and debugging sensors once the sim is up are
ordinary work.

`CLAUDE.md` holds the gotchas and verified sensor semantics, and is where to
look when interpreting what a step's output means. But where the two describe the
same operational action, **`RUN_SIM.md` wins.**
