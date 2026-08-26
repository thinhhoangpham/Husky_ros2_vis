#!/usr/bin/env python3
"""Restore authored model placements in an SDF world from the <state> block of
the original Gazebo Classic world it was ported from.

In Gazebo Classic a <state world_name='...'> block overrides each model's own
<pose> at load time, so the per-model <pose> values in such a world are stale
authoring leftovers. Harmonic has no such override, so the state poses must be
written into the model-level <pose> elements to reproduce the original layout.

Line-oriented on purpose (same precedent as offset_world_model_z.py): a full
ElementTree re-serialize would reformat the whole hand-authored file. Only the
model-level <pose> line of each top-level <model> is rewritten; link, visual,
collision and inertial poses are left byte-identical.

  target world: models at 4-space indent, model-level <pose> at 6-space indent
                (exactly one per model; not necessarily the first pose in the
                block -- e.g. camino_parque has it after its <link>)
  source state: models at 6-space indent, model-level <pose> at 8-space indent
                (the first pose in the block; link poses are at 10-space)

Usage:
  restore_world_state_poses.py TARGET.sdf --state SOURCE.world \
      [--skip NAME ...] [--backup PATH] [--dry-run]
"""
import argparse
import re
import shutil
import sys

FLOAT = r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?"
POSE_SIX = re.compile(r"^\s*(?:%s)(?:\s+(?:%s)){5}\s*$" % (FLOAT, FLOAT))

STATE_OPEN = re.compile(r"^\s{4}<state\b")
STATE_CLOSE = re.compile(r"^\s{4}</state>")
STATE_MODEL = re.compile(r"^\s{6}<model name=['\"]([^'\"]+)['\"]")
STATE_POSE = re.compile(r"^\s{8}<pose[^>]*>([^<]*)</pose>\s*$")

TGT_MODEL = re.compile(r"^\s{4}<model name=['\"]([^'\"]+)['\"]")
TGT_MODEL_CLOSE = re.compile(r"^\s{4}</model>")
TGT_POSE = re.compile(r"^(\s{6}<pose[^>]*>)([^<]*)(</pose>\s*)$")


def parse_state(path):
    """name -> 'x y z r p y' from the model-level pose inside <state>."""
    poses = {}
    inside = False
    current = None
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            if not inside:
                if STATE_OPEN.match(line):
                    inside = True
                continue
            if STATE_CLOSE.match(line):
                inside = False
                continue
            m = STATE_MODEL.match(line)
            if m:
                current = m.group(1)
                if current in poses:
                    raise SystemExit("%s:%d duplicate state model %s"
                                     % (path, lineno, current))
                continue
            if current is None:
                continue
            pm = STATE_POSE.match(line)
            if pm and poses.get(current) is None:
                text = ' '.join(pm.group(1).split())
                if not POSE_SIX.match(text):
                    raise SystemExit("%s:%d unparseable pose for %s: %r"
                                     % (path, lineno, current, text))
                poses[current] = text
                current = None  # only the first (model-level) pose
    if inside:
        raise SystemExit("%s: unterminated <state> block" % path)
    return poses


def rewrite(target, poses, skip, dry_run, backup):
    with open(target) as f:
        lines = f.read().split('\n')

    current = None
    pose_idx = None
    order = []
    updated, skipped, missing = [], [], []

    def finish():
        if current is None:
            return
        if current in skip:
            skipped.append((current, 'explicitly skipped', lines[pose_idx].strip()))
            return
        if current not in poses:
            missing.append(current)
            skipped.append((current, 'no <state> entry', lines[pose_idx].strip()))
            return
        m = TGT_POSE.match(lines[pose_idx])
        before = ' '.join(m.group(2).split())
        after = poses[current]
        lines[pose_idx] = m.group(1) + after + m.group(3)
        updated.append((current, before, after))

    for idx, line in enumerate(lines):
        m = TGT_MODEL.match(line)
        if m:
            if current is not None:
                raise SystemExit("line %d: nested <model> not supported" % (idx + 1))
            current = m.group(1)
            if current in order:
                raise SystemExit("line %d: duplicate model %s" % (idx + 1, current))
            order.append(current)
            pose_idx = None
            continue
        if TGT_MODEL_CLOSE.match(line):
            if pose_idx is None:
                raise SystemExit("model %s has no model-level <pose>" % current)
            finish()
            current = None
            pose_idx = None
            continue
        if current is not None and TGT_POSE.match(line):
            if pose_idx is not None:
                raise SystemExit("model %s has >1 model-level <pose>" % current)
            pose_idx = idx

    if current is not None:
        raise SystemExit("unterminated <model> %s" % current)

    for name, _, after in updated:
        z = abs(float(after.split()[2]))
        if z > 1e4:
            raise SystemExit("implausible z for %s: %s" % (name, after))

    if not dry_run:
        if backup:
            shutil.copyfile(target, backup)
        with open(target, 'w') as f:
            f.write('\n'.join(lines))

    unmatched = sorted(set(poses) - set(order))
    return order, updated, skipped, missing, unmatched


def main():
    p = argparse.ArgumentParser()
    p.add_argument('world')
    p.add_argument('--state', required=True, help='source Classic .world')
    p.add_argument('--skip', nargs='*', default=[],
                   help='model names to leave untouched')
    p.add_argument('--backup')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()

    poses = parse_state(a.state)
    order, updated, skipped, missing, unmatched = rewrite(
        a.world, poses, set(a.skip), a.dry_run, a.backup)

    print("state entries: %d   target models: %d" % (len(poses), len(order)))
    print("updated: %d   left alone: %d" % (len(updated), len(skipped)))
    for name, why, pose in skipped:
        print("  skip   %-34s %-18s %s" % (name, why, pose))
    if unmatched:
        print("  state entries with no model in target: %s" % ', '.join(unmatched))
    if a.dry_run:
        print("(dry run, nothing written)", file=sys.stderr)


if __name__ == '__main__':
    main()
