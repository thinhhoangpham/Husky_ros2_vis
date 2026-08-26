#!/usr/bin/env python3
"""Add flat (per-face) vertex normals to a triangle-only OBJ, in place.

Motivation: gz-sim/DART's ODE collision backend dereferences the normals array
of a mesh unconditionally. An OBJ with no `vn` lines (or with `f a b c` faces
that never index normals) presents a normal count of 0 to assimp and segfaults
DART on the first physics step.

Geometry is preserved byte-for-byte: `v` lines are copied verbatim and face
vertex indices are untouched. Only a `vn` block is inserted and the `f` lines
are rewritten into `v//vn` form so the normals are actually indexed.
"""
import sys
import numpy as np


def main(path: str, backup_path: str) -> int:
    with open(path, "r") as fh:
        lines = fh.read().splitlines()

    verts = []
    faces = []          # (0-based vertex indices)
    face_line_idx = []  # index into `lines` for each face
    last_v_line = -1

    for i, line in enumerate(lines):
        if line.startswith("v "):
            verts.append([float(t) for t in line.split()[1:4]])
            last_v_line = i
        elif line.startswith("f "):
            toks = line.split()[1:]
            if len(toks) != 3:
                raise SystemExit(f"{path}:{i+1}: non-triangular face, unsupported")
            idx = []
            for t in toks:
                if "/" in t:
                    raise SystemExit(f"{path}:{i+1}: face already indexes vt/vn")
                v = int(t)
                if v < 0:
                    v = len(verts) + v + 1  # relative index
                idx.append(v - 1)
            faces.append(idx)
            face_line_idx.append(i)

    if not faces:
        raise SystemExit(f"{path}: no faces found")

    V = np.asarray(verts, dtype=np.float64)
    F = np.asarray(faces, dtype=np.int64)
    if F.min() < 0 or F.max() >= len(V):
        raise SystemExit(f"{path}: face index out of range")

    n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    length = np.linalg.norm(n, axis=1)
    # Degenerate (zero-area) faces would normalize to NaN, which crashes ODE
    # just as surely as a missing normal. Substitute an arbitrary unit vector.
    degenerate = ~np.isfinite(length) | (length < 1e-20)
    n[degenerate] = (0.0, 0.0, 1.0)
    length[degenerate] = 1.0
    n /= length[:, None]
    if not np.isfinite(n).all():
        raise SystemExit(f"{path}: non-finite normal survived normalization")

    with open(backup_path, "w") as fh:
        fh.write("\n".join(lines) + "\n")

    for fi, li in enumerate(face_line_idx):
        a, b, c = F[fi] + 1
        vn = fi + 1
        lines[li] = f"f {a}//{vn} {b}//{vn} {c}//{vn}"

    vn_block = [f"vn {x:.10f} {y:.10f} {z:.10f}" for x, y, z in n]
    out = lines[: last_v_line + 1] + vn_block + lines[last_v_line + 1 :]

    with open(path, "w") as fh:
        fh.write("\n".join(out) + "\n")

    print(f"vertices={len(V)} faces={len(F)} normals={len(n)} "
          f"degenerate_faces={int(degenerate.sum())}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {sys.argv[0]} <mesh.obj> <backup.obj>")
    sys.exit(main(sys.argv[1], sys.argv[2]))
