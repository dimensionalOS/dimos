# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Contact invariants, persistent object round trips and bounded spatial loading."""

from concurrent.futures import Future
import json
from pathlib import Path
import sqlite3
from types import SimpleNamespace
from typing import Any

import mujoco
import numpy as np
import pytest

from dimos.experimental.gta_sim2.prefetch import (
    govern_velocity,
    predict_position,
    speed_for_distance,
)
from dimos.experimental.gta_sim2.world_compile import add_primitive, compile_world_window
from dimos.experimental.gta_sim2.world_geometry import SourceGeometry, WorldCatalog
from dimos.experimental.gta_sim2.world_index import (
    build,
    file_digest,
    instance_number,
    placed_bounds,
    placement_matrix,
)
from dimos.experimental.gta_sim2.world_objects import (
    ObjectStateStore,
    awake_objects,
    object_snapshot,
    restore_object,
    transfer_named_state,
)
from dimos.experimental.gta_sim2.world_streaming import WorldStreamingMujocoBackend, WorldWindow


def fixture_model(*, door: bool, cube: bool = False, floor_height: float = 0) -> mujoco.MjModel:
    door_xml = (
        '<body name="gta_object/door" pos="20 0 1"><joint name="gta_object/door/joint" type="hinge" axis="0 0 1"/><geom name="door" type="box" pos=".5 0 0" size=".5 .05 1" mass="10"/></body>'
        if door
        else ""
    )
    cube_xml = (
        '<body name="gta_object/cube" pos="8 0 .2"><freejoint name="gta_object/cube/joint"/><geom name="cube" type="box" size=".2 .2 .2" mass="2"/></body>'
        if cube
        else ""
    )
    return mujoco.MjModel.from_xml_string(f"""<mujoco><option timestep=".002"/><worldbody>
        <geom name="floor" type="plane" pos="0 0 {floor_height}" size="50 50 .1"/>
        <body name="robot" pos="0 0 .19"><freejoint name="robot/root"/><geom name="wheel" type="sphere" size=".2" mass="2"/></body>
        {door_xml}{cube_xml}</worldbody><actuator><motor name="drive" joint="robot/root" gear="1 0 0 0 0 0"/></actuator></mujoco>""")


def test_door_survives_unload_disk_reopen_and_revisit(tmp_path: Path) -> None:
    model = fixture_model(door=True)
    data = mujoco.MjData(model)
    data.qpos[model.joint("gta_object/door/joint").qposadr[0]] = 0.7
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    snapshot = object_snapshot(model, data, "door-instance", "gta_object/door", 2)
    store = ObjectStateStore(tmp_path / "objects.sqlite", "catalog")
    try:
        store.save([snapshot])
        no_door = fixture_model(door=False)
        unloaded = mujoco.MjData(no_door)
        assert transfer_named_state(model, data, no_door, unloaded, {"gta_object/door"}) == 0
        np.testing.assert_array_equal(data.qpos[:7], unloaded.qpos)
        assert unloaded.time == data.time
    finally:
        store.close()
    loaded = ObjectStateStore.read(tmp_path / "objects.sqlite", ["door-instance"])
    returning = fixture_model(door=True)
    restored = mujoco.MjData(returning)
    restore_object(returning, restored, loaded["door-instance"])
    assert transfer_named_state(no_door, unloaded, returning, restored, set()) == 0
    assert restored.qpos[returning.joint("gta_object/door/joint").qposadr[0]] == pytest.approx(0.7)
    with pytest.raises(ValueError, match="different world"):
        ObjectStateStore(tmp_path / "objects.sqlite", "other")


def test_loaded_contact_and_motor_state_match_resident_reference() -> None:
    old, new = fixture_model(door=True), fixture_model(door=False)
    before, after = mujoco.MjData(old), mujoco.MjData(new)
    before.qvel[0], before.ctrl[0], before.qfrc_applied[1] = 0.4, 0.3, 0.1
    before.xfrc_applied[1, 2] = 0.2
    old.actuator_gainprm[0, 0] = 0.8
    mujoco.mj_step(old, before)
    mujoco.mj_forward(old, before)
    assert before.ncon > 0
    assert transfer_named_state(old, before, new, after, {"gta_object/door"}) == 0
    np.testing.assert_allclose(after.qacc, before.qacc[:6], atol=1e-12)
    mujoco.mj_step(old, before)
    mujoco.mj_step(new, after)
    np.testing.assert_allclose(after.qpos, before.qpos[:7], atol=1e-12)
    np.testing.assert_allclose(after.qvel, before.qvel[:6], atol=1e-12)


def test_same_collider_name_cannot_hide_changed_contact_geometry() -> None:
    old, new = fixture_model(door=False), fixture_model(door=False, floor_height=0.01)
    before, after = mujoco.MjData(old), mujoco.MjData(new)
    mujoco.mj_forward(old, before)
    assert before.ncon > 0
    with pytest.raises(ValueError, match="geometry or material"):
        transfer_named_state(old, before, new, after, set())


def test_moving_and_robot_contacting_objects_remain_awake() -> None:
    model = fixture_model(door=False, cube=True)
    data = mujoco.MjData(model)
    objects = {"cube": {"body": "gta_object/cube"}}
    settled: dict[str, float] = {}
    for _ in range(1500):
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        awake_objects(model, data, objects, settled)
    assert awake_objects(model, data, objects, settled) == set()
    joint = model.joint("gta_object/cube/joint")
    data.qvel[joint.dofadr[0]] = 0.2
    mujoco.mj_forward(model, data)
    assert awake_objects(model, data, objects, settled) == {"cube"}
    other = fixture_model(door=False)
    with pytest.raises(ValueError, match="awake"):
        transfer_named_state(model, data, other, mujoco.MjData(other), set())
    data.qvel[:] = 0
    data.qpos[joint.qposadr[0]] = 0.3
    data.time += 2
    settled["cube"] = data.time - 2
    mujoco.mj_forward(model, data)
    assert awake_objects(model, data, objects, settled) == {"cube"}


def test_moved_object_is_found_at_current_location(tmp_path: Path) -> None:
    store = ObjectStateStore(tmp_path / "states.sqlite", "world")
    try:
        store.save([{"identity": "crate", "position": [100, 20, 1], "radius": 1, "joints": {}}])
    finally:
        store.close()
    assert ObjectStateStore.nearby(tmp_path / "states.sqlite", np.zeros(3), 12) == {}
    assert set(ObjectStateStore.nearby(tmp_path / "states.sqlite", np.array([100, 20, 1]), 12)) == {
        "crate"
    }
    assert ObjectStateStore.generation(tmp_path / "states.sqlite") == 1


def test_stopping_envelope_accounts_for_loading_and_braking() -> None:
    speed = speed_for_distance(3, 1.5)
    assert speed * 1.5 + speed**2 / (2 * 0.75) == pytest.approx(3)
    assert speed_for_distance(3, 3) < speed
    velocity = govern_velocity(
        np.array([1.5, 0.5]), np.array([8.5, 0]), np.array([-9, -9]), np.array([9, 9]), 2
    )
    assert 0 <= velocity[0] < 0.25
    assert velocity[1] / velocity[0] == pytest.approx(1 / 3)
    inward = govern_velocity(
        np.array([-1.5, 0.0]), np.array([8.5, 0]), np.array([-9, -9]), np.array([9, 9]), 2
    )
    np.testing.assert_array_equal(inward, [-1.5, 0])


def test_route_prefetch_looks_around_corner() -> None:
    predicted = predict_position(
        np.array([0.0, 0, 0]),
        np.array([2.0, 0, 0]),
        2,
        np.array([[0.0, 0, 0], [2, 0, 0], [2, 10, 0]]),
        8,
    )
    np.testing.assert_allclose(predicted, [2, 3 + 4 / 1.5, 0])


def test_placed_bounds_include_rotation_and_nonuniform_scale() -> None:
    matrix = placement_matrix(
        {
            "position": [20, 30, 1],
            "rotation": [0, 0, np.sqrt(0.5), np.sqrt(0.5)],
            "scale": [2, 1, 3],
        }
    )
    result = placed_bounds([[-1, -2, 0], [1, 2, 1]], matrix)
    np.testing.assert_allclose(result, [[18, 28, 1], [22, 32, 4]])
    assert instance_number("map/a/ordinal/1") == instance_number("map/a/ordinal/1")
    assert instance_number("map/a/ordinal/1") != instance_number("map/b/ordinal/1")


def test_round_primitives_are_native_without_triangle_duplicates() -> None:
    spec = mujoco.MjSpec()
    assert add_primitive(
        spec,
        "capsule",
        {"type": "capsule", "a": [0, 0, 0], "b": [0, 0, 1], "radius": 0.1},
        np.eye(4),
    )
    assert add_primitive(
        spec,
        "cylinder",
        {"type": "cylinder", "a": [2, 0, 0], "b": [2, 0, 1], "radius": 0.2},
        np.eye(4),
    )
    model = spec.compile()
    assert model.ngeom == 2
    assert model.nmesh == 0
    assert model.geom("capsule").type == mujoco.mjtGeom.mjGEOM_CAPSULE
    assert model.geom("cylinder").type == mujoco.mjtGeom.mjGEOM_CYLINDER


def test_source_cache_is_bounded_and_reloads_evicted_geometry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    archive = tmp_path / "levels.rpf"
    archive.write_bytes(b"fixture")
    decoder = tmp_path / "decoder"
    decoder.write_bytes(b"fixture decoder")
    database = tmp_path / "world.sqlite"
    sqlite3.connect(database).close()
    manifest = {
        "version": 1,
        "database": database.name,
        "database_sha256": file_digest(database),
        "decoder": str(decoder),
        "decoder_sha256": file_digest(decoder),
        "game": str(tmp_path),
        "workspace": str(tmp_path),
        "origin_rage_m": [0, 0, 0],
        "coverage_bounds": [[-100] * 3, [100] * 3],
        "archive_size": archive.stat().st_size,
        "archive_mtime_ns": archive.stat().st_mtime_ns,
        "geometry_cache_bytes": 1200,
        "disk_cache_bytes": 100000,
        "include_mask": 1,
    }
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(manifest))
    calls = []

    def decode(self: WorldCatalog, source: str, output: Path) -> dict[str, Any]:
        calls.append(source)
        output.write_bytes(source.encode())
        return {"id": source, "glb_sha256": file_digest(output), "primitives": [], "stats": {}}

    monkeypatch.setattr(WorldCatalog, "_decode", decode)
    monkeypatch.setattr(
        "dimos.experimental.gta_sim2.world_geometry.glb_triangles",
        lambda path: [("0", np.zeros((4, 3, 3)))],
    )
    catalog = WorldCatalog(path)
    try:
        catalog._source("a")
        catalog._source("b")
        catalog._source("c")
        catalog._source("a")
        assert catalog.stats["decoded_cache_bytes"] <= 1200
        assert catalog.stats["evictions"] >= 2
        assert calls == ["a", "b", "c"]
        assert catalog.stats["disk_hits"] == 1
    finally:
        catalog.close()


def test_authored_parent_and_compiled_object_revisit(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Exercise the actual index, window compiler and disk state together."""
    archive, decoder = tmp_path / "levels.rpf", tmp_path / "decoder"
    archive.write_bytes(b"fixture")
    decoder.write_bytes(b"fixture")
    dependencies, collision, interiors = (
        tmp_path / name for name in ("deps.jsonl", "bounds.jsonl", "interiors.jsonl")
    )
    bounds = [[-0.5, -0.05, -1], [0.5, 0.05, 1]]
    entity = {
        "ordinal": 7,
        "offset": 100,
        "archetype": 10,
        "archetype_key": "key",
        "name": "door",
        "position": [8, 0, 1],
        "rotation": [0, 0, 0, 1],
        "scale": [1, 1, 1],
        "parent_map": 2,
        "lod": {"level": "HD", "parent_index": 3},
    }
    records = [
        {"kind": "archetype", "name": 10, "bbox": bounds, "physics_dictionary": 10},
        {
            "kind": "model",
            "id": "door-source",
            "hash": 10,
            "key": "key",
            "name": "door",
            "type": "ft",
        },
        {
            "kind": "map",
            "id": "child-map",
            "name": 1,
            "space": "world",
            "parent_maps": [2],
            "entities": [entity],
        },
        {
            "kind": "map",
            "id": "parent-map",
            "name": 2,
            "space": "world",
            "parent_maps": [],
            "entities": [
                {
                    **entity,
                    "ordinal": 3,
                    "parent_map": None,
                    "lod": {"level": "LOD", "parent_index": None},
                }
            ],
        },
    ]
    dependencies.write_text("".join(json.dumps(row) + "\n" for row in records))
    floor_box = [[-100, -100, -0.1], [100, 100, 0]]
    collision.write_text(json.dumps({"id": "floor-source", "bbox": floor_box}) + "\n")
    interiors.write_text("")
    definitions = tmp_path / "objects.json"
    identity = "child-map/entity/7"
    definitions.write_text(
        json.dumps(
            {
                identity: {
                    "kind": "hinge",
                    "provenance": "synthetic regression fixture",
                    "mass": 10,
                    "inertia": [4, 4, 1],
                    "inertia_quaternion_wxyz": [0.9238795325112867, 0.3826834323650898, 0, 0],
                    "pivot": [-0.5, 0, 0],
                    "axis": [0, 0, 1],
                    "range": [-1.2, 1.2],
                }
            }
        )
    )
    output = tmp_path / "catalog"
    result = build(
        dependencies,
        collision,
        interiors,
        output,
        [0, 0, 0],
        decoder,
        tmp_path,
        tmp_path,
        definitions,
    )
    assert result["counts"]["resolved_parent_links"] == 1
    assert result["counts"]["indexed_entity_instances"] == 1
    with sqlite3.connect(output / "world.sqlite") as db:
        assert db.execute("SELECT * FROM parent_links").fetchall() == [
            (identity, "parent-map/entity/3")
        ]

    def geometry(self: WorldCatalog, source: str) -> SourceGeometry:
        box = floor_box if source == "floor-source" else bounds
        return SourceGeometry(
            np.empty((0, 3, 3)),
            [{"type": "box", "bbox": box, "matrix": np.eye(4).ravel().tolist()}],
            {},
            100,
        )

    monkeypatch.setattr(WorldCatalog, "_source", geometry)
    catalog = WorldCatalog(output / "catalog.json")
    state_path = tmp_path / "state.sqlite"
    store = ObjectStateStore(state_path, catalog.digest)
    spec = mujoco.MjSpec.from_string(
        '<mujoco><worldbody><body name="robot" pos="0 0 .2"><freejoint/><geom name="wheel" type="sphere" size=".2" mass="2"/></body></worldbody></mujoco>'
    )
    try:
        model, data, _, metadata = compile_world_window(spec, catalog, np.zeros(3), 12, state_path)
        actor = metadata["actors"][identity]
        joint = model.joint(actor["body"] + "/joint")
        np.testing.assert_allclose(
            model.body(actor["body"]).iquat, [0.9238795325112867, 0.3826834323650898, 0, 0]
        )
        np.testing.assert_allclose(joint.range, [-1.2, 1.2])
        data.qpos[joint.qposadr[0]] = 0.7
        mujoco.mj_forward(model, data)
        store.save([object_snapshot(model, data, identity, actor["body"], actor["radius"])])
        store.close()
        away, away_data, _, metadata = compile_world_window(
            spec, catalog, np.array([32.0, 0, 0]), 12, state_path
        )
        assert metadata["actors"] == {}
        assert transfer_named_state(model, data, away, away_data, {actor["body"]}) == 0
        back, back_data, _, _ = compile_world_window(spec, catalog, np.zeros(3), 12, state_path)
        transfer_named_state(away, away_data, back, back_data, set())
        assert back_data.qpos[back.joint(actor["body"] + "/joint").qposadr[0]] == pytest.approx(0.7)
    finally:
        store.close()
        catalog.close()


def test_route_projection_does_not_prefetch_behind_robot() -> None:
    result = predict_position(
        np.array([40.0, 0, 0]), np.array([2.0, 0, 0]), 1, np.array([[0.0, 0, 0], [100.0, 0, 0]]), 8
    )
    np.testing.assert_allclose(result, [43 + 4 / 1.5, 0, 0])


def test_compiler_exit_preserves_current_physics(tmp_path: Path) -> None:
    backend = WorldStreamingMujocoBackend(
        tmp_path / "catalog.json", "robot", tmp_path / "metrics.json"
    )
    backend.model = fixture_model(door=False)
    backend.data = mujoco.MjData(backend.model)
    mujoco.mj_forward(backend.model, backend.data)
    backend._window = WorldWindow(np.zeros(3), backend.model, backend.data, {}, 0)
    backend._catalog = SimpleNamespace(coverage=np.array([[-100.0] * 3, [100.0] * 3]))  # type: ignore[assignment]
    backend._tick = 1
    backend._pending = Future()
    backend._pending.set_exception(BrokenPipeError("fixture compiler exited"))
    backend.step()
    assert backend.data.time > 0
    assert "compiler exited" in backend._load_error
    assert backend._pending is None


def test_visual_state_survives_reopen_at_both_original_and_moved_locations(tmp_path: Path) -> None:
    model = fixture_model(door=True)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    snapshot = object_snapshot(model, data, "prop", "gta_object/door", 2)
    snapshot["position"] = [100, 0, 1]
    snapshot["visual"] = {"source_position_local": [20, 0, 1], "pivot": [0.5, 0, 0]}
    store = ObjectStateStore(tmp_path / "objects.sqlite", "catalog")
    store.save([snapshot])
    store.close()
    for center in (np.array([20.0, 0, 1]), np.array([100.0, 0, 1])):
        result = ObjectStateStore.visual_nearby(store.path, center, 5)
        assert result["prop"]["position"] == [100, 0, 1]
        assert result["prop"]["visual"]["pivot"] == [0.5, 0, 0]
    assert not ObjectStateStore.visual_nearby(store.path, np.array([50.0, 0, 1]), 5)
    assert not ObjectStateStore.nearby(store.path, np.array([20.0, 0, 1]), 5)
