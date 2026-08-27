"""Read park.sdf and emit collision geometry as world-space triangles.

Pure python: no ROS, no Gazebo. Used by tools/generate_park_maps.py to
rasterize the prior obstacle map and the terrain keepout mask.

Three dataset-specific traps are handled here, all of which corrupt geometry
silently rather than raising:
  * mixed COLLADA up_axis (arbol4 and bench are Y_UP, the rest Z_UP)
  * missing <unit> elements (COLLADA default is meter=1)
  * <matrix> node transforms in <library_visual_scenes>
"""

from __future__ import annotations

import os
from typing import Iterator, Sequence
from xml.etree import ElementTree as ET

import numpy as np

NS = "{http://www.collada.org/2005/11/COLLADASchema}"

# Every model:// URI in this world resolves under models/.
MODELS_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")


def _resolve_uri(uri: str) -> str:
    if uri.startswith("model://"):
        return os.path.join(MODELS_ROOT, uri[len("model://"):])
    return uri


# --------------------------------------------------------------------------- OBJ

def _load_obj(path: str) -> tuple[np.ndarray, np.ndarray]:
    verts: list[list[float]] = []
    faces: list[list[int]] = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("v "):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                # "f 1//1 2//2 3//3" -> vertex indices only; OBJ is 1-based and
                # allows negative (relative) indices.
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                for k in range(1, len(idx) - 1):
                    faces.append([idx[0], idx[k], idx[k + 1]])
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int64)


# --------------------------------------------------------------------------- DAE

def _dae_sources(mesh: ET.Element) -> dict[str, np.ndarray]:
    out: dict[str, np.ndarray] = {}
    for src in mesh.findall(f"{NS}source"):
        arr = src.find(f"{NS}float_array")
        if arr is None or not arr.text:
            continue
        vals = np.fromstring(arr.text, sep=" ", dtype=np.float64)
        acc = src.find(f"{NS}technique_common/{NS}accessor")
        stride = int(acc.get("stride", "3")) if acc is not None else 3
        out["#" + src.get("id", "")] = vals.reshape(-1, stride)
    return out


def _dae_position_source(mesh: ET.Element) -> str:
    """<vertices id="X"> indirects the VERTEX semantic to a POSITION source."""
    verts = mesh.find(f"{NS}vertices")
    if verts is None:
        return ""
    for inp in verts.findall(f"{NS}input"):
        if inp.get("semantic") == "POSITION":
            return inp.get("source", "")
    return ""


def _dae_geometries(root: ET.Element) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    geos: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for geo in root.iter(f"{NS}geometry"):
        mesh = geo.find(f"{NS}mesh")
        if mesh is None:
            continue
        sources = _dae_sources(mesh)
        pos_src = _dae_position_source(mesh)
        if pos_src not in sources:
            continue
        V = sources[pos_src][:, :3]
        faces: list[np.ndarray] = []
        for prim in list(mesh.findall(f"{NS}triangles")) + list(mesh.findall(f"{NS}polylist")):
            inputs = prim.findall(f"{NS}input")
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            offset = 0
            for i in inputs:
                if i.get("semantic") == "VERTEX":
                    offset = int(i.get("offset", "0"))
            p = prim.find(f"{NS}p")
            if p is None or not p.text:
                continue
            idx = np.fromstring(p.text, sep=" ", dtype=np.int64)
            idx = idx.reshape(-1, stride)[:, offset]
            vcount = prim.find(f"{NS}vcount")
            if vcount is not None and vcount.text:
                counts = np.fromstring(vcount.text, sep=" ", dtype=np.int64)
                pos = 0
                for c in counts:
                    fan = idx[pos:pos + c]
                    for k in range(1, c - 1):
                        faces.append(np.array([fan[0], fan[k], fan[k + 1]]))
                    pos += c
            else:
                faces.append(idx.reshape(-1, 3))
        if not faces:
            continue
        F = np.vstack([f.reshape(-1, 3) for f in faces])
        geos["#" + geo.get("id", "")] = (V, F)
    return geos


def _dae_node_transforms(root: ET.Element) -> list[tuple[str, np.ndarray]]:
    """Walk the visual scene, returning (geometry_url, 4x4) instance pairs."""
    out: list[tuple[str, np.ndarray]] = []

    def walk(node: ET.Element, parent: np.ndarray) -> None:
        M = parent
        for child in node:
            tag = child.tag[len(NS):] if child.tag.startswith(NS) else child.tag
            if tag == "matrix" and child.text:
                m = np.fromstring(child.text, sep=" ", dtype=np.float64).reshape(4, 4)
                M = M @ m
            elif tag == "translate" and child.text:
                t = np.eye(4)
                t[:3, 3] = np.fromstring(child.text, sep=" ", dtype=np.float64)[:3]
                M = M @ t
            elif tag == "scale" and child.text:
                s = np.eye(4)
                s[[0, 1, 2], [0, 1, 2]] = np.fromstring(child.text, sep=" ", dtype=np.float64)[:3]
                M = M @ s
        for child in node:
            tag = child.tag[len(NS):] if child.tag.startswith(NS) else child.tag
            if tag == "instance_geometry":
                out.append((child.get("url", ""), M))
            elif tag == "node":
                walk(child, M)

    for scene in root.iter(f"{NS}visual_scene"):
        for node in scene.findall(f"{NS}node"):
            walk(node, np.eye(4))
    return out


def _load_dae(path: str) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(path).getroot()

    unit = 1.0
    up = "Y_UP"  # COLLADA default
    asset = root.find(f"{NS}asset")
    if asset is not None:
        u = asset.find(f"{NS}unit")
        if u is not None:
            unit = float(u.get("meter", "1"))
        ua = asset.find(f"{NS}up_axis")
        if ua is not None and ua.text:
            up = ua.text.strip()

    geos = _dae_geometries(root)
    instances = _dae_node_transforms(root)
    if not instances:
        instances = [(url, np.eye(4)) for url in geos]

    all_V: list[np.ndarray] = []
    all_F: list[np.ndarray] = []
    base = 0
    for url, M in instances:
        if url not in geos:
            continue
        V, F = geos[url]
        H = np.hstack([V, np.ones((len(V), 1))])
        Vw = (H @ M.T)[:, :3]
        all_V.append(Vw)
        all_F.append(F + base)
        base += len(Vw)

    if not all_V:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)

    V = np.vstack(all_V) * unit
    F = np.vstack(all_F)

    # NOTE: deliberately NOT applying the COLLADA <up_axis> conversion.
    # Gazebo/gz-sim does not apply it for these assets, and this loader must
    # match what the simulator actually places in the world, not what the
    # COLLADA spec says. Verified against linea1/postes_lowpoly.dae, which
    # declares Y_UP: converting makes it 3.6 m tall and 16.5 m wide (a pylon
    # lying on its side), while Gazebo renders it 16.5 m tall as authored.
    # The same bug sank park's 16 benches below the terrain and flattened the
    # power lines into 60 m walls across the prior map.
    _ = up
    return V, F


def load_mesh(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (vertices, triangle_indices) in mesh-local metres, Z up."""
    if path.lower().endswith(".obj"):
        return _load_obj(path)
    return _load_dae(path)


# --------------------------------------------------------------------------- SDF

def pose_matrix(pose: Sequence[float]) -> np.ndarray:
    """4x4 homogeneous transform from an SDF pose [x y z roll pitch yaw]."""
    x, y, z, r, p, yw = (list(pose) + [0.0] * 6)[:6]
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(yw), np.sin(yw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [x, y, z]
    return M


def _pose_of(elem: ET.Element | None) -> np.ndarray:
    if elem is None:
        return np.eye(4)
    p = elem.find("pose")
    if p is None or not p.text:
        return np.eye(4)
    return pose_matrix([float(v) for v in p.text.split()])


def models_using_mesh_dir(sdf_path: str, dirname: str) -> set[str]:
    """Names of models whose collision meshes live under model://<dirname>/.

    Selecting a model family by the asset it instances is stable against the
    world's naming (arbolpartes4, arbolpartes4_clone, arbolpartes4_clone_12 ...)
    and cannot accidentally catch a different model with a similar name.
    """
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    out: set[str] = set()
    if world is None:
        return out
    prefix = f"model://{dirname}/"
    for model in world.findall("model"):
        for col in model.iter("collision"):
            for uri in col.iter("uri"):
                if uri.text and uri.text.strip().startswith(prefix):
                    out.add(model.get("name", ""))
    return out




def _primitive_triangles(col: ET.Element) -> np.ndarray | None:
    """Triangulate a <box>/<cylinder>/<sphere> collision, or None if it is a mesh.

    park's water_tower is built from cylinder primitives rather than a mesh, and
    a loader that only handles <mesh> drops it from the map entirely - it is a
    2 m diameter column from the ground to 6 m with a 5 m tank above, i.e. one of
    the largest obstacles in the world.
    """
    geom = col.find("geometry")
    if geom is None:
        return None
    local = _pose_of(col.find("geometry")) if False else np.eye(4)
    # the collision's own <pose> is applied by the caller; a primitive may also
    # carry a pose on the collision element, already handled there.

    def _emit(verts: np.ndarray, faces: list[tuple[int, int, int]]) -> np.ndarray:
        return np.asarray([[verts[a], verts[b], verts[c]] for a, b, c in faces], dtype=np.float64)

    box = geom.find("box")
    if box is not None:
        sz = box.find("size")
        sx, sy, sz_ = ([float(v) for v in sz.text.split()] if sz is not None and sz.text
                       else [1.0, 1.0, 1.0])
        hx, hy, hz = sx / 2, sy / 2, sz_ / 2
        V = np.array([[x, y, z] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)])
        F = [(0,1,3),(0,3,2),(4,6,7),(4,7,5),(0,4,5),(0,5,1),
             (2,3,7),(2,7,6),(0,2,6),(0,6,4),(1,5,7),(1,7,3)]
        return _emit(V, F)

    cyl = geom.find("cylinder")
    if cyl is not None:
        r_el, l_el = cyl.find("radius"), cyl.find("length")
        r = float(r_el.text) if r_el is not None and r_el.text else 1.0
        L = float(l_el.text) if l_el is not None and l_el.text else 1.0
        n = 24
        ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
        ring = np.column_stack([r * np.cos(ang), r * np.sin(ang)])
        bot = np.column_stack([ring, np.full(n, -L / 2)])
        top = np.column_stack([ring, np.full(n, L / 2)])
        V = np.vstack([bot, top, [[0, 0, -L / 2]], [[0, 0, L / 2]]])
        cb, ct = 2 * n, 2 * n + 1
        F = []
        for i in range(n):
            j = (i + 1) % n
            F += [(i, j, n + j), (i, n + j, n + i), (cb, j, i), (ct, n + i, n + j)]
        return _emit(V, F)

    sph = geom.find("sphere")
    if sph is not None:
        r_el = sph.find("radius")
        r = float(r_el.text) if r_el is not None and r_el.text else 1.0
        u = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        v = np.linspace(-np.pi / 2, np.pi / 2, 9)
        pts, F = [], []
        for iv, vv in enumerate(v):
            for iu, uu in enumerate(u):
                pts.append([r*np.cos(vv)*np.cos(uu), r*np.cos(vv)*np.sin(uu), r*np.sin(vv)])
        V = np.asarray(pts)
        for iv in range(len(v) - 1):
            for iu in range(len(u)):
                a = iv*len(u)+iu; b = iv*len(u)+(iu+1) % len(u)
                c = (iv+1)*len(u)+iu; d = (iv+1)*len(u)+(iu+1) % len(u)
                F += [(a, b, d), (a, d, c)]
        return _emit(V, F)

    return None


def world_triangles(sdf_path: str, skip_models: set[str]) -> Iterator[tuple[str, np.ndarray]]:
    """Yield (model_name, triangles) with triangles float64 (k, 3, 3) in world coords."""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        return
    cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    for model in world.findall("model"):
        name = model.get("name", "")
        if name in skip_models:
            continue
        M_model = _pose_of(model)
        chunks: list[np.ndarray] = []
        for link in model.findall("link"):
            M_link = M_model @ _pose_of(link)
            for col in link.findall("collision"):
                M_col = M_link @ _pose_of(col)
                prim = _primitive_triangles(col)
                if prim is not None:
                    H = np.hstack([prim.reshape(-1, 3), np.ones((prim.size // 3, 1))])
                    chunks.append((H @ M_col.T)[:, :3].reshape(-1, 3, 3))
                    continue
                mesh_el = col.find("geometry/mesh")
                if mesh_el is None:
                    continue
                uri_el = mesh_el.find("uri")
                if uri_el is None or not uri_el.text:
                    continue
                path = _resolve_uri(uri_el.text.strip())
                if not os.path.exists(path):
                    raise FileNotFoundError(f"{name}: collision mesh not found: {path}")
                if path not in cache:
                    cache[path] = load_mesh(path)
                V, F = cache[path]
                if len(F) == 0:
                    continue
                scale_el = mesh_el.find("scale")
                s = np.array([float(v) for v in scale_el.text.split()]) if (
                    scale_el is not None and scale_el.text) else np.ones(3)
                Vs = V * s
                H = np.hstack([Vs, np.ones((len(Vs), 1))])
                Vw = (H @ M_col.T)[:, :3]
                chunks.append(Vw[F])
        yield name, (np.vstack(chunks) if chunks else np.zeros((0, 3, 3)))
