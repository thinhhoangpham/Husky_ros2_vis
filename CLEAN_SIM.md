# Cleaning up before a new simulation

Steps only. Run all of them, in order, before every launch — including when you
are confident nothing is running.
Background and rationale live in `CLAUDE.md`.

Launch only after Step 3 passes. Then go to `RUN_SIM.md`.

---

## Step 1 — Go to the project

```bash
cd ~/Documents/Husky_viz
```

## Step 2 — Kill and clear

Run all three blocks, in this order. The shared memory must be cleared *after*
the processes are dead, not before.

```bash
./scripts/kill_sim.sh
```

```bash
for pid in $(pgrep -f "a200_0000|gz sim|gz_tools_vendor" 2>/dev/null); do
  grep -qa "bash -c" /proc/$pid/cmdline 2>/dev/null && continue
  kill -9 $pid 2>/dev/null
done
```

```bash
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
```

Ignore whatever `kill_sim.sh` prints about being clean. Step 3 decides that.

## Step 3 — Verify

```bash
ps -eo pid,cmd --no-headers | grep -viE "grep|bash -c" \
  | grep -iE "ros|gazebo|gz |a200|husky|clearpath|rviz"
echo "opt/ros : $(ps -eo cmd --no-headers | grep -c '^/opt/ros')"
echo "shm     : $(ls /dev/shm | grep -c fastrtps)"
```

**Required: no process lines, `opt/ros : 0`, `shm : 0`.**

Acceptable exceptions:
- a lone `ros2-daemon` python process — the ROS 2 CLI discovery daemon, not the sim
- `pgrep -c` returning 1–2 with nothing visible in `ps` — your own grep matching itself
- **a nonzero `shm` count on the first read** — see below

### A nonzero `shm` on the first read is usually transient

Participants killed in Step 2 release their segments as they tear down, which
takes a moment. A count read immediately after Step 2 can therefore be nonzero
and still be clearing on its own — observed at `38` twice, dropping to `0`
seconds later with no further action.

Run the same Step 3 command again — no sleep, no loop, just run it a second time:

- **drops to `0`** → transient, the gate has passed, continue
- **stays nonzero** → a real leak; go to Step 4

Do not clear `/dev/shm` again to force it. If segments are being recreated rather
than released, something is still alive and Step 4 will find it — a second `rm`
only hides that.

Do not launch until this passes.

## Step 4 — If survivors remain

Identify them by name before killing anything else.

```bash
ps -eo pid,etime,cmd --no-headers | grep -viE "grep|bash -c" \
  | grep -iE "ros|gazebo|gz |a200|husky|clearpath|rviz"
```

Read the full command lines, kill what you find, then repeat Step 3.

Report which node type leaked. A process that survived Step 2 means a new node
type has appeared that the sweep pattern does not cover — that is a change this
file needs, and the user decides it.
