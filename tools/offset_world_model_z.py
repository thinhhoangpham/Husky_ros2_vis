#!/usr/bin/env python3
"""Apply a uniform z offset to the model-level <pose> of every top-level
<model> in an SDF world whose z exceeds a threshold.

Line-oriented on purpose: park.sdf is hand-authored SDF and a full
ElementTree re-serialize would reformat all 300 KB of it. Only the z field
of a model-level pose line is rewritten; x, y, roll, pitch, yaw and every
link/visual/collision/inertial pose are left byte-identical.

A model-level pose is the unique 6-space-indented <pose> inside a
4-space-indented <model> block (verified: exactly one per model in park.sdf).

Usage: offset_world_model_z.py WORLD.sdf --offset -2.97 --above 1.0 [--dry-run]
"""
import argparse
import re
import shutil
import sys

MODEL_OPEN = re.compile(r"^    <model name=['\"]([^'\"]+)['\"]")
MODEL_CLOSE = "    </model>"
MODEL_POSE = re.compile(r"^(      <pose>)(\S+)( +)(\S+)( +)(\S+)(.*</pose>)$")


def rewrite(path, offset, above, dry_run=False, backup=None):
    with open(path) as f:
        lines = f.read().split('\n')

    current = None
    seen = set()
    changed, kept = [], []

    for idx, line in enumerate(lines):
        m = MODEL_OPEN.match(line)
        if m:
            if current is not None:
                raise SystemExit(f"line {idx+1}: nested <model> not supported")
            current = m.group(1)
            if current in seen:
                raise SystemExit(f"line {idx+1}: duplicate model name {current}")
            seen.add(current)
            continue
        if line.startswith(MODEL_CLOSE):
            current = None
            continue
        if current is None:
            continue
        pm = MODEL_POSE.match(line)
        if not pm:
            continue
        z = float(pm.group(6))
        if z > above:
            new_z = z + offset
            # keep the original field's text width/format style simple
            text = f"{new_z:.6g}"
            lines[idx] = (pm.group(1) + pm.group(2) + pm.group(3) + pm.group(4)
                          + pm.group(5) + text + pm.group(7))
            changed.append((current, z, new_z))
        else:
            kept.append((current, z))

    if not dry_run:
        if backup:
            shutil.copyfile(path, backup)
        with open(path, 'w') as f:
            f.write('\n'.join(lines))

    return changed, kept, len(seen)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('world')
    p.add_argument('--offset', type=float, required=True)
    p.add_argument('--above', type=float, default=1.0,
                   help='only models with model-level pose z strictly above this move')
    p.add_argument('--backup')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()

    changed, kept, total = rewrite(a.world, a.offset, a.above, a.dry_run, a.backup)
    print(f"models: {total}  moved: {len(changed)}  left alone: {len(kept)}")
    for name, z in kept:
        print(f"  kept  {name:30s} z={z}")
    for name, z0, z1 in sorted(changed, key=lambda c: -c[1])[:5]:
        print(f"  moved {name:30s} {z0} -> {z1:.6g}")
    if a.dry_run:
        print("(dry run, nothing written)", file=sys.stderr)


if __name__ == '__main__':
    main()
