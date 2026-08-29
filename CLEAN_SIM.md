# Cleaning up before a new simulation

```bash
cd ~/Documents/Husky_viz
python3 scripts/sim.py stop
```

**Required: last line `CLEAN`.** `sim.py start` runs this itself, so a
separate clean pass is only needed to stop a sim without starting another.

## What it does

Kills every pid matching `scripts/kill_sim.sh`'s pattern list plus the
`a200_0000` / `gz sim` / `gz_tools_vendor` sweep (skipping `bash -c`
wrappers and itself), stops the `ros2` daemon, removes
`/dev/shm/fastrtps_*` and `sem.fastrtps_*`, then verifies no survivors and
a `fastrtps` count of 0 — re-reading once because a nonzero first read is
usually a transient release (CLAUDE.md #12).

## If it prints FAIL

It lists the survivors by full command line. A survivor means a node type
the pattern list does not cover — add it to `scripts/kill_sim.sh`'s
`PATTERNS` (the user decides), never kill it by hand and move on.
