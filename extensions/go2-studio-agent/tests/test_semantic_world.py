"""Persistence and resolution tests for the confirmed semantic-place world."""

import json
from pathlib import Path

from dimos_go2_studio.mission_contracts import MissionKind, TaskSpec
from dimos_go2_studio.semantic_world import (
    MapIdentityUnavailableError,
    ResolutionStatus,
    SemanticFrameTransformError,
    SemanticWorld,
    SemanticWorldStorageError,
)
from dimos_lcm.std_msgs import String
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.tf import MultiTBuffer


class _TestTF(MultiTBuffer):
    def stop(self) -> None:
        return None


def _pose(x: float, y: float, *, frame_id: str = "map") -> PoseStamped:
    return PoseStamped(
        ts=1234.5,
        frame_id=frame_id,
        position=[x, y, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )


def _task(destination: str) -> TaskSpec:
    return TaskSpec(
        task_id="task-semantic-0001",
        kind=MissionKind.GO_TO_PLACE,
        destination=destination,
    )


def _world(
    path: Path,
    *,
    map_id: str = "venue-hall",
    map_version: str = "2026-07-25T01",
    canonical_frame_id: str = "",
    navigation_frame_id: str = "",
) -> SemanticWorld:
    return SemanticWorld(
        storage_path=path,
        map_id=map_id,
        map_version=map_version,
        canonical_frame_id=canonical_frame_id,
        navigation_frame_id=navigation_frame_id,
    )


def _close(*worlds: SemanticWorld) -> None:
    for world in worlds:
        world.stop()


def test_two_confirmed_places_survive_restart_and_aliases_resolve(tmp_path: Path) -> None:
    store = tmp_path / "semantic-world.json"
    first = _world(store)
    second: SemanticWorld | None = None
    try:
        door = first.confirm_place(
            name="会场正门",
            aliases=["正门", "入口"],
            pose=_pose(1.25, -0.5),
        )
        stage = first.confirm_place(
            name="主舞台",
            aliases=["舞台"],
            pose=_pose(4.0, 2.5),
        )

        second = _world(store)
        places = second.list_places()
        assert [place.name for place in places] == ["主舞台", "会场正门"]
        assert {place.entity_id for place in places} == {
            door.entity_id,
            stage.entity_id,
        }

        goal = second.resolve(_task("入口"))
        assert goal is not None
        assert goal.frame_id == "map"
        assert goal.ts == pytest.approx(1234.5)
        assert goal.x == pytest.approx(1.25)
        assert goal.y == pytest.approx(-0.5)

        payload = json.loads(second.list_semantic_places())
        assert payload["map_id"] == "venue-hall"
        assert payload["map_version"] == "2026-07-25T01"
        assert len(payload["places"]) == 2
    finally:
        _close(first, *([second] if second is not None else []))


def test_confirmed_place_publishes_current_map_snapshot(tmp_path: Path) -> None:
    world = _world(tmp_path / "semantic-world.json")
    snapshots: list[dict[str, object]] = []
    world.semantic_places_snapshot.subscribe(
        lambda message: snapshots.append(json.loads(message.data))
    )
    try:
        world.confirm_place(
            name="会场正门",
            aliases=["入口"],
            pose=_pose(1.0, 2.0),
        )

        assert snapshots[-1]["map_id"] == "venue-hall"
        assert snapshots[-1]["map_version"] == "2026-07-25T01"
        places = snapshots[-1]["places"]
        assert isinstance(places, list)
        assert [place["name"] for place in places] == ["会场正门"]
    finally:
        _close(world)


def test_start_publishes_places_loaded_from_persistent_store(tmp_path: Path) -> None:
    store = tmp_path / "semantic-world.json"
    first = _world(store)
    second: SemanticWorld | None = None
    try:
        first.confirm_place(
            name="会场正门",
            aliases=["入口"],
            pose=_pose(1.0, 2.0),
        )
        second = _world(store)
        snapshots: list[dict[str, object]] = []
        second.semantic_places_snapshot.subscribe(
            lambda message: snapshots.append(json.loads(message.data))
        )

        second.start()

        places = snapshots[-1]["places"]
        assert isinstance(places, list)
        assert [place["name"] for place in places] == ["会场正门"]
    finally:
        _close(first, *([second] if second is not None else []))


def test_map_version_mismatch_fails_closed(tmp_path: Path) -> None:
    store = tmp_path / "semantic-world.json"
    first = _world(store)
    mismatched: SemanticWorld | None = None
    try:
        first.confirm_place(
            name="会场正门",
            aliases=["入口"],
            pose=_pose(1.0, 2.0),
        )
        mismatched = _world(store, map_version="2026-07-25T02")

        result = mismatched.resolve_detail(_task("会场正门"))
        assert result.status is ResolutionStatus.MAP_MISMATCH
        assert result.place is None
        assert mismatched.resolve(_task("会场正门")) is None
    finally:
        _close(first, *([mismatched] if mismatched is not None else []))


def test_same_name_update_preserves_id_and_conflicts_are_rejected(tmp_path: Path) -> None:
    world = _world(tmp_path / "semantic-world.json")
    try:
        original = world.confirm_place(
            name="会场正门",
            aliases=["入口"],
            pose=_pose(1.0, 2.0),
        )
        updated = world.confirm_place(
            name=" 会场正门 ",
            aliases=["大门"],
            pose=_pose(2.0, 3.0),
        )

        assert updated.entity_id == original.entity_id
        assert updated.aliases == ("大门",)
        assert updated.pose.x == pytest.approx(2.0)
        assert world.resolve(_task("入口")) is None
        assert world.resolve(_task("大门")) is not None

        with pytest.raises(ValueError, match="conflicts"):
            world.confirm_place(
                name="服务台",
                aliases=["大门"],
                pose=_pose(3.0, 4.0),
            )
        with pytest.raises(ValueError, match="conflicts"):
            world.confirm_place(
                name="大门",
                aliases=[],
                pose=_pose(3.0, 4.0),
            )
    finally:
        _close(world)


def test_unknown_place_is_distinct_from_map_mismatch(tmp_path: Path) -> None:
    world = _world(tmp_path / "semantic-world.json")
    try:
        result = world.resolve_detail(_task("不存在的地点"))
        assert result.status is ResolutionStatus.UNRESOLVED
        assert result.place is None
        assert world.resolve(_task("不存在的地点")) is None
    finally:
        _close(world)


def test_corrupt_store_fails_without_overwriting_source(tmp_path: Path) -> None:
    store = tmp_path / "semantic-world.json"
    original = "{ definitely-not-json"
    store.write_text(original, encoding="utf-8")

    with pytest.raises(SemanticWorldStorageError, match="could not load"):
        _world(store)

    assert store.read_text(encoding="utf-8") == original


def test_missing_map_identity_fails_closed(tmp_path: Path) -> None:
    world = _world(
        tmp_path / "semantic-world.json",
        map_id="",
        map_version="",
    )
    try:
        with pytest.raises(MapIdentityUnavailableError):
            world.confirm_place(
                name="会场正门",
                aliases=[],
                pose=_pose(1.0, 2.0),
            )

        result = world.resolve_detail(_task("会场正门"))
        assert result.status is ResolutionStatus.MAP_UNAVAILABLE
        assert world.resolve(_task("会场正门")) is None
    finally:
        _close(world)


def test_live_world_pose_is_persisted_in_map_and_resolved_back_to_world(
    tmp_path: Path,
) -> None:
    world = _world(
        tmp_path / "semantic-world.json",
        canonical_frame_id="map",
        navigation_frame_id="world",
    )
    transforms = _TestTF(buffer_size=10.0)
    transforms.receive_transform(
        Transform(
            frame_id="world",
            child_frame_id="map",
            ts=1234.5,
            translation=Vector3(10.0, 0.0, 0.0),
        )
    )
    world._tf = transforms
    try:
        place = world.confirm_place(
            name="会场正门",
            aliases=["门口"],
            pose=_pose(12.0, 3.0, frame_id="world"),
        )

        assert place.pose.frame_id == "map"
        assert place.pose.x == pytest.approx(2.0)
        assert place.pose.y == pytest.approx(3.0)

        goal = world.resolve(_task("门口"))
        assert goal is not None
        assert goal.frame_id == "world"
        assert goal.x == pytest.approx(12.0)
        assert goal.y == pytest.approx(3.0)
    finally:
        _close(world)


def test_live_confirmation_and_resolution_fail_closed_without_relocalization_tf(
    tmp_path: Path,
) -> None:
    world = _world(
        tmp_path / "semantic-world.json",
        canonical_frame_id="map",
        navigation_frame_id="world",
    )
    world._tf = _TestTF(buffer_size=10.0)
    try:
        with pytest.raises(SemanticFrameTransformError, match="world.*map"):
            world.confirm_place(
                name="会场正门",
                aliases=[],
                pose=_pose(12.0, 3.0, frame_id="world"),
            )

        assert world.list_places() == ()
        assert world.resolve(_task("会场正门")) is None
    finally:
        _close(world)


def test_operator_clicked_map_candidate_uses_canonical_confirmation(
    tmp_path: Path,
) -> None:
    world = _world(
        tmp_path / "semantic-world.json",
        canonical_frame_id="map",
        navigation_frame_id="world",
    )
    transforms = _TestTF(buffer_size=10.0)
    transforms.receive_transform(
        Transform(
            frame_id="world",
            child_frame_id="map",
            ts=1234.5,
            translation=Vector3(10.0, 0.0, 0.0),
        )
    )
    world._tf = transforms
    confirmations: list[dict[str, object]] = []
    unsubscribe = world.semantic_place_confirmation.subscribe(
        lambda message: confirmations.append(json.loads(message.data))
    )
    try:
        world._confirm_map_candidate(
            String(
                json.dumps(
                    {
                        "request_id": "marker-request-1",
                        "place": {
                            "name": "会场正门",
                            "aliases": [],
                            "pose": {
                                "frame_id": "world",
                                "ts": 1234.5,
                                "x": 12.0,
                                "y": 3.0,
                                "z": 0.0,
                                "qx": 0.0,
                                "qy": 0.0,
                                "qz": 0.0,
                                "qw": 1.0,
                            },
                        },
                    },
                    ensure_ascii=False,
                )
            )
        )

        assert confirmations[-1]["request_id"] == "marker-request-1"
        assert confirmations[-1]["accepted"] is True
        place = world.list_places()[0]
        assert place.name == "会场正门"
        assert place.pose.frame_id == "map"
        assert place.pose.x == pytest.approx(2.0)
        assert place.pose.y == pytest.approx(3.0)
    finally:
        unsubscribe()
        _close(world)
