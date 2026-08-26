---
name: husky-sim-restart
description: CLEAN_SIM.md at the project root is the procedure for killing every surviving Husky simulation process and clearing stale FastDDS shared memory. Use this skill EVERY time before starting or restarting the sim — it must complete and verify clean before RUN_SIM.md Step 3 (Launch) — and whenever the user says "kill the sim", "stop it", "restart it", "clean up", or asks whether the sim shut down cleanly. Also use it when a fresh sim shows a sensor with no data, an unexpected publisher count, or behaves differently between launches for no apparent reason, because those are almost always leftovers from a previous run rather than real faults. Read CLEAN_SIM.md fresh each time and execute its numbered steps verbatim; never judge cleanliness from `kill_sim.sh`'s own success message, which has reported "clean" with 73 processes still alive.
---

# Stopping the Husky sim, completely

`/home/thinhpham/Documents/Husky_viz/CLEAN_SIM.md` is the procedure. It contains
numbered steps and nothing else — rationale lives in `CLAUDE.md`, deliberately
kept out so the file stays executable.

Like `RUN_SIM.md`, it is maintained material and **it changes**. Read it fresh
every time; your memory of last session's commands is not evidence of what it
says now.

## Why this is a gate, not a formality

Launching on top of leftovers is the most expensive mistake available in this
project. Orphaned nodes keep publishing on the exact topic names the next sim
will use, and DDS participants killed with SIGKILL leave shared-memory segments
locked in `/dev/shm`. The next launch then fails as a *sensor* bug — one topic
silently empty, a different one each time, with nothing in the log pointing at a
process from twenty minutes ago.

Two structural traps the file's ordering exists to avoid:

- **`kill_sim.sh` verifies with the same node-name list it kills with**, so it
  cannot detect what it does not kill. Its list has been wrong at least twice
  here — it missed Clearpath's teleop stack, then the camera image bridges. That
  is why Step 3 checks on a deliberately broader pattern than Step 2 kills with.
- **Clearing `/dev/shm` before the processes are dead makes the count go up**,
  because survivors immediately recreate their segments — and the cleanup then
  reads as done when it is not.

Neither is obvious from reading the commands, which is why the step order is not
yours to optimise.

## Procedure

### 1. Read the file, now

Read `CLEAN_SIM.md` in full. Not from memory, not from this skill — the actual
file, this time.

### 2. Execute verbatim

Run each step exactly as written, in order. Do not reorder, merge, substitute an
equivalent command, add retries or `sleep`s, or skip a step because nothing looks
like it is running.

Step 3 states a required result. That is a gate: do not proceed to launch until
it holds.

### 3. Report every step with evidence

```
Step N — <heading from CLEAN_SIM.md>
  command: <exactly what you ran>
  output:  <actual output, or the values that matter>
  result:  OK | FAILED
```

Report the actual `opt/ros : 0` and `shm : 0` lines. "Ran the cleanup" proves
nothing; the numbers prove it. Finish with a count of steps in the file versus
steps executed.

### 4. When a step fails

Step 4 of the file already covers survivors — follow it, then repeat Step 3.

For anything else that fails, stop and report: which step, the exact command and
output, your reading of why, and a concrete proposed change to `CLEAN_SIM.md`.
A failing step means the file is out of date, which is a finding worth having —
working around it destroys that signal and leaves the file wrong for whoever
runs it next. Propose the edit; don't apply it unasked.

## After this

Once Step 3 reports clean, proceed to `RUN_SIM.md` (the `husky-run-sim` skill).

CLAUDE.md gotchas #11, #12 and #21 cover the mechanics, including the measured
case where 488 stale segments made camera and compass silently produce no data
while lidar and IMU kept working — twice, even after clearing the process tree.
Gotcha #9 covers why the `bash -c` guard in Step 2 matters: a `pkill`/`pgrep`
pattern that appears in your own command line matches the invoking shell.
