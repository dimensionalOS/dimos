# Copyright 2025-2026 Dimensional Inc.
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

"""Four-room scene constants, scene XML, and the sqlite-backed PlacesMemory."""

from __future__ import annotations

from collections.abc import Iterator
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from dimos.robot.microduck.places import (
    ARENA_HALF_EXTENT,
    BALL_BODY,
    BALL_FREEJOINT,
    BALL_GEOM,
    BALL_GEOM_GROUP,
    BALL_MASS,
    BALL_RADIUS,
    BALL_START,
    DEFAULT_SCENE,
    FOUR_ROOM_XML,
    HUB_HALF_EXTENT,
    MICRODUCK_OBJECTS,
    MICRODUCK_ROOMS,
    PlaceRecord,
    PlacesMemory,
    RoomSpec,
    add_ball_body,
    normalize_place_query,
    place_key,
    scene_id,
)

# What the pre-existing agentic-sim blueprint seeds: two objects, no rooms.
LEGACY_OBJECTS = {"red_box": (1.5, 0.8), "blue_box": (-1.5, -0.8)}

_ROBOT_MJCF = Path("~/.cache/dimos/microduck/robot/robot_allcollisions.xml").expanduser()


def _floats(text: str | None) -> tuple[float, ...]:
    return tuple(float(v) for v in (text or "").split())


def _geoms() -> dict[str, ET.Element]:
    root = ET.parse(FOUR_ROOM_XML).getroot()
    return {g.get("name", ""): g for g in root.iter("geom") if g.get("name")}


# --------------------------------------------------------------------------
# Scene XML <-> constants
# --------------------------------------------------------------------------


def test_scene_file_exists() -> None:
    assert FOUR_ROOM_XML.is_file()
    assert FOUR_ROOM_XML.name == "four_room_scene.xml"


def test_objects_match_scene_positions() -> None:
    geoms = _geoms()
    for name, (x, y) in MICRODUCK_OBJECTS.items():
        assert name in geoms, f"{name} missing from {FOUR_ROOM_XML.name}"
        pos = _floats(geoms[name].get("pos"))
        assert pos[:2] == pytest.approx((x, y)), name
        assert geoms[name].get("group") == "0", f"{name} must be lidar-visible (group 0)"
        size = _floats(geoms[name].get("size"))
        # Sized 6..14 cm (half-extents 0.03..0.14 depending on the shape).
        assert 0.03 <= min(size) and max(size) <= 0.14, (name, size)


def test_scene_lists_no_unknown_landmarks() -> None:
    """Every coloured object geom in the scene is in MICRODUCK_OBJECTS."""
    geoms = _geoms()
    structural = {"floor"}
    landmarks = {
        n
        for n, g in geoms.items()
        if g.get("rgba") is not None and n not in structural and not n.startswith("floor_")
    }
    assert landmarks == set(MICRODUCK_OBJECTS)


def test_walls_and_stubs_match_design() -> None:
    geoms = _geoms()
    stubs = {
        "stub_east": ((1.4, 0.0, 0.25), (0.6, 0.05, 0.25)),
        "stub_west": ((-1.4, 0.0, 0.25), (0.6, 0.05, 0.25)),
        "stub_north": ((0.0, 1.4, 0.25), (0.05, 0.6, 0.25)),
        "stub_south": ((0.0, -1.4, 0.25), (0.05, 0.6, 0.25)),
    }
    for name, (pos, size) in stubs.items():
        assert _floats(geoms[name].get("pos")) == pytest.approx(pos), name
        assert _floats(geoms[name].get("size")) == pytest.approx(size), name
    for name in ("wall_north", "wall_south", "wall_east", "wall_west"):
        size = _floats(geoms[name].get("size"))
        assert size[2] == pytest.approx(0.25)
        assert min(size[:2]) == pytest.approx(0.05)
    # The hub opening between stub tips is 1.6 m wide; the inner wall faces
    # are at +/- ARENA_HALF_EXTENT.
    assert (1.4 - 0.6) * 2 == pytest.approx(2 * HUB_HALF_EXTENT)
    for name, sign, axis in (
        ("wall_north", 1, 1),
        ("wall_south", -1, 1),
        ("wall_east", 1, 0),
        ("wall_west", -1, 0),
    ):
        pos = _floats(geoms[name].get("pos"))
        half = _floats(geoms[name].get("size"))
        assert pos[axis] - sign * half[axis] == pytest.approx(sign * ARENA_HALF_EXTENT), name
    for name, g in geoms.items():
        assert g.get("group") == "0", name


def test_room_floor_patches_are_visual_only() -> None:
    geoms = _geoms()
    for room in MICRODUCK_ROOMS.values():
        patch = geoms[f"floor_{room.name}"]
        assert patch.get("contype") == "0" and patch.get("conaffinity") == "0"
        cx, cy = room.center
        assert _floats(patch.get("pos"))[:2] == pytest.approx((cx, cy))


def test_scene_declares_no_bodies_or_joints() -> None:
    """The engine takes joint 0 as the robot's root free joint and MuJoCo
    numbers joints in body order, so nothing jointed (the ball included) may
    be declared in the scene ahead of the attached robot."""
    root = ET.parse(FOUR_ROOM_XML).getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None
    assert worldbody.findall("body") == []
    assert list(root.iter("freejoint")) == [] and list(root.iter("joint")) == []
    assert not any(b.get("name") == BALL_BODY for b in root.iter("body"))


def test_ball_constants_match_design() -> None:
    assert BALL_BODY == "ball"
    assert BALL_FREEJOINT == f"{BALL_BODY}_freejoint"
    assert BALL_GEOM == f"{BALL_BODY}_geom"
    assert BALL_START == pytest.approx((1.2, -0.6, 0.035))
    assert BALL_START[2] == pytest.approx(BALL_RADIUS)  # resting on the floor
    assert BALL_RADIUS == pytest.approx(0.035)
    assert BALL_MASS == pytest.approx(0.05)
    assert BALL_GEOM_GROUP == 1  # not group 0: the raycast lidar must not see it
    # It starts inside the office, well clear of the stubs and walls.
    assert MICRODUCK_ROOMS["office"].contains(BALL_START[0], BALL_START[1])


def test_visual_offscreen_size() -> None:
    root = ET.parse(FOUR_ROOM_XML).getroot()
    glob = root.find("visual/global")
    assert glob is not None
    assert glob.get("offwidth") == "1280" and glob.get("offheight") == "720"


def test_room_table_matches_design() -> None:
    assert list(MICRODUCK_ROOMS) == ["kitchen", "living", "bedroom", "office"]
    expected = {
        "kitchen": (("space A",), (0, 2, 0, 2), (1.2, 1.0, 0.0)),
        "living": (("space B", "living room", "lounge"), (-2, 0, 0, 2), (-1.2, 1.0, 3.14159)),
        "bedroom": (("space C",), (-2, 0, -2, 0), (-1.2, -1.0, 3.14159)),
        "office": (("space D", "study"), (0, 2, -2, 0), (1.2, -1.0, 0.0)),
    }
    for name, (aliases, bounds, target) in expected.items():
        room = MICRODUCK_ROOMS[name]
        assert isinstance(room, RoomSpec)
        assert room.name == name
        assert room.aliases == aliases
        assert room.bounds == pytest.approx(bounds)
        assert room.target == pytest.approx(target)
        # Target lies inside the room, past the hub, and its yaw looks into
        # the room (away from the hub at the origin) so the head camera sees
        # the room's landmark after arriving.
        assert room.contains(target[0], target[1])
        assert abs(target[0]) > HUB_HALF_EXTENT or abs(target[1]) > HUB_HALF_EXTENT
        outward = math.atan2(target[1], target[0])
        assert math.cos(outward - target[2]) > 0.7


def test_every_object_sits_in_a_room() -> None:
    for name, (x, y) in MICRODUCK_OBJECTS.items():
        assert any(r.contains(x, y) for r in MICRODUCK_ROOMS.values()), name


# --------------------------------------------------------------------------
# MuJoCo compile / compose (explicit: pytest -m mujoco)
# --------------------------------------------------------------------------


@pytest.mark.mujoco
def test_scene_compiles_standalone() -> None:
    mujoco = pytest.importorskip("mujoco")
    model = mujoco.MjModel.from_xml_path(str(FOUR_ROOM_XML))
    assert model.njnt == 0 and model.nbody == 1  # world only: nothing ahead of the robot
    for name in MICRODUCK_OBJECTS:
        gid = model.geom(name).id
        assert model.geom_pos[gid][:2] == pytest.approx(MICRODUCK_OBJECTS[name])
        assert model.geom_group[gid] == 0
    assert (model.vis.global_.offwidth, model.vis.global_.offheight) == (1280, 720)


def _attach_robot(mujoco, spec_scene):  # type: ignore[no-untyped-def]
    """Same recipe as ``MicroduckSimModule._compose_model``: MjSpec attach."""
    if not _ROBOT_MJCF.is_file():
        pytest.skip(f"robot MJCF not cached at {_ROBOT_MJCF}")
    spec_robot = mujoco.MjSpec.from_file(str(_ROBOT_MJCF))
    if not spec_robot.meshdir or not os.path.isabs(spec_robot.meshdir):
        spec_robot.meshdir = str(_ROBOT_MJCF.parent / (spec_robot.meshdir or "assets"))
    assert spec_robot.body("trunk_base") is not None
    spec_scene.option.timestep = 0.005
    spec_robot.option.timestep = 0.005
    frame = spec_scene.worldbody.add_frame(pos=[0.0, 0.0, 0.0], quat=[1.0, 0.0, 0.0, 0.0])
    spec_scene.attach(spec_robot, prefix="", frame=frame)


@pytest.mark.mujoco
def test_scene_composes_with_robot() -> None:
    """The robot's root free joint must be joint 0 of the composed model:
    ``MujocoEngine._find_first_freejoint_adrs`` takes ``jnt_qposadr[0]`` as
    the robot root (odom, spawn, IMU fallback, lidar exclusion)."""
    mujoco = pytest.importorskip("mujoco")
    spec_scene = mujoco.MjSpec.from_file(str(FOUR_ROOM_XML))
    _attach_robot(mujoco, spec_scene)
    model = spec_scene.compile()
    data = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, data)
    assert model.body("trunk_base").id > 0
    assert model.nu == 14
    assert model.joint(0).name == "trunk_base_freejoint"
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.jnt_qposadr[0] == model.jnt_qposadr[model.joint("trunk_base_freejoint").id] == 0


@pytest.mark.mujoco
def test_ball_added_after_robot_keeps_trunk_as_joint_zero() -> None:
    """``add_ball_body`` (called by the sim module after ``attach``) yields
    the design's ball without displacing the robot from joint 0."""
    mujoco = pytest.importorskip("mujoco")
    spec_scene = mujoco.MjSpec.from_file(str(FOUR_ROOM_XML))
    _attach_robot(mujoco, spec_scene)
    body = add_ball_body(spec_scene)
    assert body.name == BALL_BODY
    model = spec_scene.compile()
    data = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, data)

    assert model.joint(0).name == "trunk_base_freejoint"
    ball_joint = model.joint(BALL_FREEJOINT)
    assert ball_joint.id > 0
    assert ball_joint.type == mujoco.mjtJoint.mjJNT_FREE
    adr = model.jnt_qposadr[ball_joint.id]
    assert adr >= 7
    assert data.qpos[adr : adr + 3] == pytest.approx(BALL_START, abs=2e-3)  # rests in place
    gid = model.geom(BALL_GEOM).id
    assert model.geom_group[gid] == BALL_GEOM_GROUP
    assert model.geom_size[gid][0] == pytest.approx(BALL_RADIUS)
    assert model.body_mass[model.body(BALL_BODY).id] == pytest.approx(BALL_MASS)


@pytest.mark.mujoco
def test_add_ball_body_custom_name_and_pos() -> None:
    mujoco = pytest.importorskip("mujoco")
    spec = mujoco.MjSpec.from_file(str(FOUR_ROOM_XML))
    add_ball_body(spec, name="football", pos=(-0.5, 0.5, 0.035))
    model = spec.compile()
    assert model.joint("football_freejoint").type == mujoco.mjtJoint.mjJNT_FREE
    assert model.geom("football_geom").id >= 0
    assert model.body_pos[model.body("football").id] == pytest.approx((-0.5, 0.5, 0.035))


# --------------------------------------------------------------------------
# PlacesMemory
# --------------------------------------------------------------------------


@pytest.fixture
def memory(tmp_path: Path) -> Iterator[PlacesMemory]:
    mem = PlacesMemory(tmp_path / "places.db")
    mem.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
    try:
        yield mem
    finally:
        mem.close()


def test_seed_is_idempotent(tmp_path: Path) -> None:
    mem = PlacesMemory(tmp_path / "places.db")
    try:
        assert mem.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS) == 9
        assert mem.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS) == 0
        records = mem.all()
        assert len(records) == 9
        assert len({(r.kind, r.name) for r in records}) == 9
        assert [r.name for r in records if r.kind == "room"] == list(MICRODUCK_ROOMS)
        assert [r.name for r in records if r.kind == "object"] == list(MICRODUCK_OBJECTS)
    finally:
        mem.close()


def test_seed_rewrites_moved_object(tmp_path: Path) -> None:
    mem = PlacesMemory(tmp_path / "places.db")
    try:
        mem.seed({}, {"red_box": (1.0, 1.0)})
        assert mem.seed({}, {"red_box": (1.5, 1.5)}) == 1
        rec = mem.find("red_box")
        assert rec is not None and (rec.x, rec.y) == (1.5, 1.5)
        assert len(mem.all()) == 1
    finally:
        mem.close()


def test_persists_across_reopen(tmp_path: Path) -> None:
    db = tmp_path / "places.db"
    mem = PlacesMemory(db)
    mem.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
    mem.add("charger", 0.3, -0.2, 0.0)
    mem.close()

    reopened = PlacesMemory(db)
    try:
        assert reopened.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS) == 0
        records = reopened.all()
        assert len(records) == 10
        charger = reopened.find("charger")
        assert charger == PlaceRecord("tagged", "charger", (), 0.3, -0.2, 0.0, None)
    finally:
        reopened.close()


def test_scene_id_is_stable_and_table_sensitive() -> None:
    cockpit = scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
    assert cockpit == scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
    assert cockpit == scene_id(dict(reversed(list(MICRODUCK_ROOMS.items()))), MICRODUCK_OBJECTS)
    assert cockpit.startswith("scene-") and len(cockpit) == len("scene-") + 12
    assert scene_id({}, LEGACY_OBJECTS) != cockpit
    assert scene_id({}, {}) != cockpit
    assert scene_id(MICRODUCK_ROOMS, {**MICRODUCK_OBJECTS, "red_box": (1.5, 0.8)}) != cockpit
    assert scene_id() == scene_id({}, {}) == scene_id(None, None)
    assert scene_id() != DEFAULT_SCENE


def test_default_scene_and_explicit_scene(tmp_path: Path) -> None:
    mem = PlacesMemory(tmp_path / "places.db")
    try:
        assert mem.scene == DEFAULT_SCENE == "default"
    finally:
        mem.close()
    mem = PlacesMemory(tmp_path / "places.db", scene="")
    try:
        assert mem.scene == DEFAULT_SCENE
    finally:
        mem.close()
    mem = PlacesMemory(tmp_path / "places.db", scene="cockpit-a")
    try:
        assert mem.scene == "cockpit-a"
        loc = mem.add("charger", 0.0, 0.0)
        assert loc.metadata["scene"] == "cockpit-a"
    finally:
        mem.close()


def test_scenes_sharing_one_db_do_not_leak(tmp_path: Path) -> None:
    """Cockpit (4 rooms, 5 objects) and the old agentic sim (2 objects) share
    ``~/.cache/dimos/microduck/places.db``; each must only see its own world."""
    db = tmp_path / "places.db"
    cockpit = PlacesMemory(db, scene=scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS))
    legacy = PlacesMemory(db, scene=scene_id({}, LEGACY_OBJECTS))
    try:
        assert cockpit.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS) == 9
        assert legacy.seed({}, LEGACY_OBJECTS) == 2
        cockpit.add("charger", 0.3, -0.2)
        legacy.add("nest", -0.3, 0.2)

        assert len(cockpit.all()) == 10
        assert legacy.rooms() == []
        assert [r.name for r in legacy.all()] == ["red_box", "blue_box", "nest"]

        # Same object name, different position per scene.
        red_cockpit = cockpit.find("red_box")
        red_legacy = legacy.find("red_box")
        assert red_cockpit is not None and (red_cockpit.x, red_cockpit.y) == (1.5, 1.5)
        assert red_legacy is not None and (red_legacy.x, red_legacy.y) == (1.5, 0.8)

        # Rooms, tagged spots and the spatial index are scoped too.
        assert cockpit.room_at(1.5, 0.8) is not None
        assert legacy.room_at(1.5, 0.8) is None
        assert legacy.find("kitchen") is None and legacy.find("space A") is None
        assert legacy.find("charger") is None and cockpit.find("nest") is None
        assert [r.name for r in legacy.near(1.5, 0.8, 0.2)] == ["red_box"]
        assert [r.name for r in cockpit.near(1.5, 0.8, 0.2)] == []
        assert [r.name for r in cockpit.near(0.3, -0.2, 0.05)] == ["charger"]
        assert legacy.near(0.3, -0.2, 0.05) == []

        legacy_json = json.loads(legacy.to_json())
        assert legacy_json["rooms"] == [] and legacy_json["tagged"] == [
            {"name": "nest", "x": -0.3, "y": 0.2, "yaw": 0.0}
        ]
        assert [o["name"] for o in json.loads(cockpit.to_json())["objects"]] == list(
            MICRODUCK_OBJECTS
        )

        # Re-seeding either scene writes nothing (idempotent per scene).
        assert cockpit.seed(MICRODUCK_ROOMS, MICRODUCK_OBJECTS) == 0
        assert legacy.seed({}, LEGACY_OBJECTS) == 0
    finally:
        cockpit.close()
        legacy.close()

    # Reopening sees the same partition; the default scene sees nothing.
    reopened = PlacesMemory(db, scene=scene_id({}, LEGACY_OBJECTS))
    default = PlacesMemory(db)
    try:
        assert [r.name for r in reopened.all()] == ["red_box", "blue_box", "nest"]
        assert default.all() == []
    finally:
        reopened.close()
        default.close()


def test_expands_home_and_creates_parents(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("HOME", str(tmp_path))
    mem = PlacesMemory("~/nested/dir/places.db")
    try:
        assert mem.db_path == tmp_path / "nested" / "dir" / "places.db"
        assert mem.db_path.parent.is_dir()
    finally:
        mem.close()


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        ("kitchen", "kitchen"),
        ("space A", "kitchen"),
        ("Space a", "kitchen"),
        ("SPACE-A", "kitchen"),
        ("the kitchen", "kitchen"),
        ("kitchen please", "kitchen"),
        ("go to the kitchen", "kitchen"),
        ("room A", "kitchen"),
        ("space B", "living"),
        ("living room", "living"),
        ("the living room", "living"),
        ("lounge", "living"),
        ("living", "living"),
        ("space C", "bedroom"),
        ("bedroom", "bedroom"),
        ("space D", "office"),
        ("study", "office"),
        ("the office please", "office"),
    ],
)
def test_find_rooms_by_name_and_alias(memory: PlacesMemory, query: str, expected: str) -> None:
    rec = memory.find(query)
    assert rec is not None and rec.kind == "room" and rec.name == expected


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        ("red_box", "red_box"),
        ("red box", "red_box"),
        ("Red Box", "red_box"),
        ("the red box", "red_box"),
        ("blue box", "blue_box"),
        ("green cylinder", "green_cylinder"),
        ("yellow pillar", "yellow_pillar"),
        ("yellow", "yellow_pillar"),
        ("orange crate", "orange_crate"),
    ],
)
def test_find_objects(memory: PlacesMemory, query: str, expected: str) -> None:
    rec = memory.find(query)
    assert rec is not None and rec.kind == "object" and rec.name == expected


def test_find_unknown_and_kind_filter(memory: PlacesMemory) -> None:
    assert memory.find("garage") is None
    assert memory.find("") is None
    assert memory.find("   ") is None
    assert memory.find("red box", kind="room") is None
    assert memory.find("space A", kind="object") is None
    kitchen = memory.find("space A", kind="room")
    assert kitchen is not None and kitchen.name == "kitchen"


def test_find_prefers_exact_name_over_substring(memory: PlacesMemory) -> None:
    memory.add("box", 0.1, 0.2)
    rec = memory.find("box")
    assert rec is not None and rec.kind == "tagged" and rec.name == "box"
    assert memory.matches("box") == [rec]


@pytest.mark.parametrize("query", ["box", "the box", "space", "room", "the room please"])
def test_find_is_none_when_ambiguous_or_filler_only(memory: PlacesMemory, query: str) -> None:
    assert memory.find(query) is None


def test_matches_lists_candidates_in_table_order(memory: PlacesMemory) -> None:
    assert [r.name for r in memory.matches("box")] == ["red_box", "blue_box"]
    assert [r.name for r in memory.matches("box", kind="object")] == ["red_box", "blue_box"]
    assert memory.matches("box", kind="room") == []
    assert [r.name for r in memory.matches("kitchen")] == ["kitchen"]
    assert [r.name for r in memory.matches("space A")] == ["kitchen"]
    assert [r.name for r in memory.matches("yellow")] == ["yellow_pillar"]
    # Filler-only queries hit nothing by containment...
    assert memory.matches("space") == []
    assert memory.matches("room") == []
    assert memory.matches("") == []
    # ...but still resolve a place that is literally called that.
    memory.add("spot", 0.0, 0.5)
    assert [r.name for r in memory.matches("spot")] == ["spot"]


def test_normalize_place_query() -> None:
    assert normalize_place_query("Space a") == ("space a", "a")
    assert normalize_place_query("the kitchen please") == ("the kitchen please", "kitchen")
    assert normalize_place_query("go to space B") == ("go to space b", "b")
    assert normalize_place_query("red_box") == ("red box",)
    assert normalize_place_query("") == ()
    assert normalize_place_query("the") == ("the",)


def test_room_records_carry_target_and_bounds(memory: PlacesMemory) -> None:
    living = memory.find("living")
    assert living is not None
    assert living.bounds == pytest.approx((-2, 0, 0, 2))
    assert (living.x, living.y, living.yaw) == pytest.approx((-1.2, 1.0, 3.14159))
    assert living.aliases == ("space B", "living room", "lounge")


@pytest.mark.parametrize(
    ("x", "y", "expected"),
    [
        (1.5, 1.5, "kitchen"),
        (0.9, 0.1, "kitchen"),
        (-1.0, 1.9, "living"),
        (-1.5, -1.5, "bedroom"),
        (1.2, -1.0, "office"),
        (0.0, 0.0, None),
        (0.5, -0.5, None),
        (0.79, 0.79, None),
        (2.5, 0.0, None),
        (0.0, -2.5, None),
    ],
)
def test_room_at(memory: PlacesMemory, x: float, y: float, expected: str | None) -> None:
    rec = memory.room_at(x, y)
    assert (rec.name if rec is not None else None) == expected


def test_near_uses_spatial_index(memory: PlacesMemory) -> None:
    near = memory.near(1.4, 1.4, 0.6)
    names = [r.name for r in near]
    assert names[0] == "red_box"
    assert "kitchen" in names  # its target (1.2, 1.0) is 0.45 m away
    assert "blue_box" not in names
    assert memory.near(0.0, 0.0, 0.1) == []
    memory.add("charger", 0.05, 0.0)
    assert [r.name for r in memory.near(0.0, 0.0, 0.1)] == ["charger"]


def test_add_updates_existing_tagged_place(memory: PlacesMemory) -> None:
    memory.add("charger", 0.3, -0.2, 0.0)
    memory.add("charger", 0.4, -0.2, 1.0)
    tagged = [r for r in memory.all() if r.kind == "tagged"]
    assert tagged == [PlaceRecord("tagged", "charger", (), 0.4, -0.2, 1.0, None)]


def test_place_key_folds_case_punctuation_and_spacing() -> None:
    assert place_key("Charger") == place_key("charger") == place_key(" charger! ") == "charger"
    assert place_key("Front  Door") == place_key("front_door") == "front door"
    assert place_key("Space A") == "space a"
    assert place_key("kitchen") != place_key("the kitchen")  # different names, both kept
    assert place_key("") == ""


def test_add_is_case_insensitive_and_keeps_latest_spelling(memory: PlacesMemory) -> None:
    """'Charger' and 'charger' are one place: re-tagging overwrites instead of
    duplicating (find() would otherwise report the pair as ambiguous)."""
    memory.add("charger", 0.3, -0.2, 0.0)
    memory.add("Charger", 0.5, -0.1, 1.0)
    memory.add("CHARGER!", 0.6, -0.1, 2.0)
    tagged = [r for r in memory.all() if r.kind == "tagged"]
    assert tagged == [PlaceRecord("tagged", "CHARGER!", (), 0.6, -0.1, 2.0, None)]
    found = memory.find("charger")
    assert found is not None and (found.x, found.yaw) == (0.6, 2.0)
    assert len(memory.matches("Charger")) == 1
    payload = json.loads(memory.to_json())
    assert [t["name"] for t in payload["tagged"]] == ["CHARGER!"]


def test_seed_treats_case_variants_as_one_place(tmp_path: Path) -> None:
    """A respelled table entry renames the stored place (the blueprint's
    table is ground truth) instead of leaving two records behind."""
    mem = PlacesMemory(tmp_path / "places.db")
    try:
        assert mem.seed({}, {"Red Box": (1.0, 1.0)}) == 1
        assert mem.seed({}, {"red_box": (1.0, 1.0)}) == 1  # respelled: rewritten once...
        assert mem.seed({}, {"red_box": (1.0, 1.0)}) == 0  # ...then idempotent again
        objects = [r for r in mem.all() if r.kind == "object"]
        assert [(r.name, r.x) for r in objects] == [("red_box", 1.0)]
        assert len(mem.matches("red box")) == 1
    finally:
        mem.close()


def test_add_returns_robot_location(memory: PlacesMemory) -> None:
    loc = memory.add("charger", 0.3, -0.2, 0.5, metadata={"note": "usb-c"})
    assert loc.name == "charger"
    assert loc.position == (0.3, -0.2, 0.0)
    assert loc.rotation == (0.0, 0.0, 0.5)
    assert loc.frame_id == "world"
    assert loc.metadata["kind"] == "tagged"
    assert loc.metadata["note"] == "usb-c"


def test_to_json_shape(memory: PlacesMemory) -> None:
    memory.add("charger", 0.3, -0.2, 0.0)
    payload = json.loads(memory.to_json(t=123.0))
    assert set(payload) == {"frame", "rooms", "objects", "tagged", "t"}
    assert payload["frame"] == "world"
    assert payload["t"] == 123.0
    assert payload["rooms"][0] == {
        "name": "kitchen",
        "aliases": ["space A"],
        "bounds": [0.0, 2.0, 0.0, 2.0],
        "target": [1.2, 1.0, 0.0],
    }
    assert [r["name"] for r in payload["rooms"]] == list(MICRODUCK_ROOMS)
    assert payload["objects"][0] == {"name": "red_box", "x": 1.5, "y": 1.5}
    assert [o["name"] for o in payload["objects"]] == list(MICRODUCK_OBJECTS)
    assert payload["tagged"] == [{"name": "charger", "x": 0.3, "y": -0.2, "yaw": 0.0}]
    # Compact separators, as every JSON stream in the cockpit.
    assert ", " not in memory.to_json() and ": " not in memory.to_json()
    assert "t" in json.loads(memory.to_json())
