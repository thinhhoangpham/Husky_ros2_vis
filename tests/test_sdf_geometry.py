import numpy as np

from tools.sdf_geometry import load_mesh, pose_matrix, world_triangles

REPO = "/home/thinhpham/Documents/Husky_viz"
SDF = f"{REPO}/worlds/park.sdf"


def test_load_mesh_terrain_is_unit_normalised():
    """terreno_parque is authored as a unit mesh: +/-1 in x and y."""
    V, F = load_mesh(f"{REPO}/models/terreno_parque/terreno_parque_lowpoly.dae")
    assert V.shape[1] == 3
    assert F.shape[1] == 3
    assert len(V) == 66049
    assert np.isclose(V[:, 0].min(), -1.0, atol=1e-6)
    assert np.isclose(V[:, 0].max(), 1.0, atol=1e-6)
    assert np.isclose(V[:, 1].min(), -1.0, atol=1e-6)
    assert np.isclose(V[:, 1].max(), 1.0, atol=1e-6)


def test_load_mesh_y_up_is_converted_to_z_up():
    """arbol4/tronco4.dae is Y_UP; a tree trunk must be tall in z, not in y."""
    V, _ = load_mesh(f"{REPO}/models/arbol4/tronco4.dae")
    extent = V.max(axis=0) - V.min(axis=0)
    assert extent[2] > extent[0], f"trunk not tallest in z: {extent}"
    assert extent[2] > extent[1], f"trunk not tallest in z: {extent}"


def test_pose_matrix_translation_and_yaw():
    M = pose_matrix([1.0, 2.0, 3.0, 0.0, 0.0, np.pi / 2])
    p = M @ np.array([1.0, 0.0, 0.0, 1.0])
    assert np.allclose(p[:3], [1.0, 3.0, 3.0], atol=1e-9)


def test_world_triangles_terrain_matches_measured_extent():
    """Ground truth from the spec: terrain spans x +/-50, y -26.55..23.45,
    z 2.98891 +/- 0.0035. This exercises unit, up-axis, scale and all three
    pose levels at once."""
    tris = {name: T for name, T in world_triangles(SDF, skip_models=set())}
    T = tris["parque"]
    P = T.reshape(-1, 3)
    assert np.isclose(P[:, 0].min(), -50.0, atol=0.01)
    assert np.isclose(P[:, 0].max(), 50.0, atol=0.01)
    assert np.isclose(P[:, 1].min(), -26.55, atol=0.01)
    assert np.isclose(P[:, 1].max(), 23.45, atol=0.01)
    assert np.isclose(P[:, 2].mean(), 2.98891, atol=0.01)
    assert P[:, 2].ptp() < 0.01


def test_world_triangles_covers_every_model_with_collision():
    names = [name for name, _ in world_triangles(SDF, skip_models=set())]
    assert len(names) == 97
    assert "parque" in names
    assert sum(1 for n in names if n.startswith("tree_8")) == 23


def test_models_using_mesh_dir_finds_the_small_trees():
    from tools.sdf_geometry import models_using_mesh_dir
    arb = models_using_mesh_dir(SDF, "arbol4")
    assert len(arb) == 15
    assert "arbolpartes4" in arb
    assert "arbolpartes4_clone_12" in arb
    # tree_8 is a different asset and must not be caught
    assert not any(n.startswith("tree_8") for n in arb)


def test_skip_models_is_honoured():
    names = [n for n, _ in world_triangles(SDF, skip_models={"parque", "camino_parque"})]
    assert "parque" not in names
    assert "camino_parque" not in names
    assert len(names) == 95
