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

from __future__ import annotations

from pathlib import Path
import pickle
import re
from typing import Any

import pytest
import rerun as rr
import rerun.blueprint as rrb

from dimos.manipulation.data_collection.piper_blueprint_config import (
    CAMERA_RECORDED_ENTITY_PATH,
    CAMERA_TOPIC,
    _piper_data_collection_topic_to_entity,
    default_episode_path_factory,
    default_recording_id_factory,
    default_session_name,
    joint_state_to_rerun_scalars,
    piper_data_collection_joint_short_names,
    piper_data_collection_rerun_blueprint,
    piper_data_collection_rerun_config,
    piper_episode_metadata,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def _walk(node: Any) -> list[Any]:
    """Depth-first walk of a Rerun blueprint container/view tree."""
    out: list[Any] = [node]
    contents = getattr(node, "contents", None)
    if isinstance(contents, list | tuple):
        for child in contents:
            out.extend(_walk(child))
    return out


# ── Override entity-path scheme tests ────────────────────────────────────────


def test_joint_state_to_rerun_scalars_emits_one_entry_per_joint() -> None:
    names = [
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "arm/gripper",
    ]
    positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.05]
    msg = JointState(name=names, position=positions)

    measured = joint_state_to_rerun_scalars("measured")(msg)
    commanded = joint_state_to_rerun_scalars("commanded")(msg)

    assert len(measured) == len(names)
    assert len(commanded) == len(names)

    measured_paths = [path for path, _ in measured]
    commanded_paths = [path for path, _ in commanded]
    assert measured_paths == [
        "/observation/state/joint1",
        "/observation/state/joint2",
        "/observation/state/joint3",
        "/observation/state/joint4",
        "/observation/state/joint5",
        "/observation/state/joint6",
        "/observation/state/gripper",
    ]
    assert commanded_paths == [
        "/action/joint1",
        "/action/joint2",
        "/action/joint3",
        "/action/joint4",
        "/action/joint5",
        "/action/joint6",
        "/action/gripper",
    ]
    # Gripper specifically lands under .../gripper, not a special-cased path.
    assert measured_paths[-1].endswith("/gripper")
    assert commanded_paths[-1].endswith("/gripper")
    # Archetypes are Rerun Scalars carrying the position values.
    for (_, arch), _pos in zip(measured, positions, strict=True):
        assert isinstance(arch, rr.Scalars)


def test_joint_state_to_rerun_scalars_skips_missing_joints() -> None:
    """Helper must not raise on partial JointStates — emit only what's present."""
    msg = JointState(name=["arm/joint1", "arm/gripper"], position=[0.0, 0.5])
    result = joint_state_to_rerun_scalars("measured")(msg)
    paths = [p for p, _ in result]
    assert paths == [
        "/observation/state/joint1",
        "/observation/state/gripper",
    ]


def test_camera_topic_maps_to_observation_namespace() -> None:
    """The shared topic_to_entity callback rewrites the camera LCM topic
    onto the LeRobot-aligned recorded entity path."""

    class _Topic:
        def __init__(self, name: str) -> None:
            self.name = name

    assert _piper_data_collection_topic_to_entity(_Topic(CAMERA_TOPIC)) == (
        CAMERA_RECORDED_ENTITY_PATH
    )
    # LCM "#Type" suffix is stripped.
    assert (
        _piper_data_collection_topic_to_entity(
            _Topic("/piper_data_collection/color_image#sensor_msgs.Image")
        )
        == CAMERA_RECORDED_ENTITY_PATH
    )
    # Joint state topics fall through to the bare topic name.
    assert _piper_data_collection_topic_to_entity(_Topic("/coordinator/joint_state")) == (
        "/coordinator/joint_state"
    )


# ── Preset layout tests ──────────────────────────────────────────────────────


def test_preset_factory_returns_blueprint_with_one_view_per_joint() -> None:
    bp = piper_data_collection_rerun_blueprint()
    assert isinstance(bp, rrb.Blueprint)

    nodes = _walk(bp.root_container)
    time_series_views = [n for n in nodes if isinstance(n, rrb.TimeSeriesView)]
    spatial_views = [n for n in nodes if isinstance(n, rrb.Spatial2DView)]

    expected_joints = piper_data_collection_joint_short_names()
    # One TimeSeriesView per joint, named by short name, in order.
    assert [v.name for v in time_series_views] == expected_joints
    # Gripper is the last joint plot.
    assert time_series_views[-1].name == "gripper"

    # Each TimeSeriesView pairs measured + commanded paths for that joint.
    for view, short in zip(time_series_views, expected_joints, strict=True):
        contents = [str(c) for c in (view.contents or [])]
        assert any(f"/observation/state/{short}" in c for c in contents), (short, contents)
        assert any(f"/action/{short}" in c for c in contents), (short, contents)

    # Camera Spatial2DView uses the LeRobot-aligned recorded entity path.
    assert len(spatial_views) == 1
    assert str(spatial_views[0].origin) == CAMERA_RECORDED_ENTITY_PATH


def test_rerun_config_is_picklable() -> None:
    """The override + blueprint factory must pickle so the bridge / recorder
    can be deployed into a multiprocessing worker. A bare local closure would
    silently break ``dimos run teleop-quest-piper-data-collection`` at startup.
    """
    cfg = piper_data_collection_rerun_config()
    restored = pickle.loads(pickle.dumps(cfg))

    msg = JointState(name=["arm/joint1", "arm/gripper"], position=[0.1, 0.5])
    measured = restored["visual_override"]["/coordinator/joint_state"](msg)
    assert [p for p, _ in measured] == [
        "/observation/state/joint1",
        "/observation/state/gripper",
    ]
    assert callable(restored["blueprint"])
    assert isinstance(restored["blueprint"](), rrb.Blueprint)


# ── Session / episode helpers tests ──────────────────────────────────────────


def test_default_session_name_matches_utc_timestamp_format() -> None:
    name = default_session_name()
    assert re.fullmatch(r"\d{8}T\d{6}Z", name), name


def test_default_episode_path_factory_increments_monotonically(tmp_path: Path) -> None:
    factory = default_episode_path_factory(session_name="20260514T120000Z")
    p1 = factory()
    p2 = factory()
    p3 = factory()
    assert p1.name == "episode_001.rrd"
    assert p2.name == "episode_002.rrd"
    assert p3.name == "episode_003.rrd"
    assert p1.parent.name == "20260514T120000Z"
    assert p1.parent.parent.name == "piper_data_collection"


def test_default_episode_path_factory_generates_session_name_when_missing() -> None:
    factory = default_episode_path_factory()
    p1 = factory()
    p2 = factory()
    # Same session directory across calls.
    assert p1.parent == p2.parent
    # Matches the timestamp format.
    assert re.fullmatch(r"\d{8}T\d{6}Z", p1.parent.name)


def test_default_recording_id_factory_uses_session_and_episode_stem() -> None:
    path = Path("/data/piper_data_collection/20260514T120000Z/episode_007.rrd")
    assert default_recording_id_factory(path) == "20260514T120000Z/episode_007"


def test_piper_episode_metadata_omits_operator_when_unset() -> None:
    metadata = piper_episode_metadata("sess1", operator=None)
    out = metadata(3)
    assert out == {
        "episode_id": "sess1/episode_003",
        "episode_index": "3",
        "session_id": "sess1",
    }
    assert "operator" not in out


def test_piper_episode_metadata_includes_operator_when_set() -> None:
    metadata = piper_episode_metadata("sess1", operator="alice")
    out = metadata(5)
    assert out["operator"] == "alice"
    assert out["episode_id"] == "sess1/episode_005"


def test_piper_data_collection_rerun_config_propagates_operator_env(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("DIMOS_OPERATOR", "bob")
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")
    metadata = cfg["episode_metadata"]
    assert metadata(1)["operator"] == "bob"
    # And when the env var is unset, operator is absent.
    monkeypatch.delenv("DIMOS_OPERATOR", raising=False)
    cfg2 = piper_data_collection_rerun_config(session_name="20260514T120000Z")
    assert "operator" not in cfg2["episode_metadata"](1)


def test_piper_data_collection_rerun_config_exposes_recorder_fields() -> None:
    cfg = piper_data_collection_rerun_config(session_name="20260514T120000Z")
    assert callable(cfg["record_path_factory"])
    assert callable(cfg["recording_id_factory"])
    assert callable(cfg["episode_metadata"])
    assert cfg["entity_prefix"] == ""
    assert callable(cfg["topic_to_entity"])
