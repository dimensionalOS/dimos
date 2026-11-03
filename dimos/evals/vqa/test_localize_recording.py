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

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from dimos.evals.vqa import localize_recording as localize_recording_module
from dimos.evals.vqa.localize_recording import (
    discover_object_names,
    discover_object_names_vlm,
    proposals_from_localize_scene,
    scene_at_timestamp,
)
from dimos.evals.vqa.primitives.localize_scene import (
    LocalizeInstanceEvidence,
    LocalizeSceneEvidence,
)
from dimos.perception.localize.types import Localization


def _hit(xyz: tuple[float, float, float]) -> Localization:
    # Avoid PointCloud2.from_numpy (open3d) — scene_at_timestamp only needs xyz.
    return Localization(
        instance_id="g0",
        semantic_score=0.9,
        ambiguity_margin=1.0,
        position_world_xyz=xyz,
        frame_id="world",
        last_seen_timestamp=1.0,
        point_cloud=SimpleNamespace(),  # type: ignore[arg-type]
        n_views=2,
    )


def test_scene_at_timestamp_returns_none_without_base_link(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(localize_recording_module, "robot_pose_xyz_yaw", lambda rig, ts: None)
    assert scene_at_timestamp((("chair", _hit((1.0, 0.0, 0.5))),), SimpleNamespace(), 1.0) is None


def test_scene_at_timestamp_builds_named_evidence(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        localize_recording_module,
        "robot_pose_xyz_yaw",
        lambda rig, ts: ((0.0, 0.0, 0.0), 0.0),
    )
    scene = scene_at_timestamp((("chair", _hit((2.0, 0.5, 0.4))),), SimpleNamespace(), 1.0)

    assert scene is not None
    assert scene.robot_position_xyz == (0.0, 0.0, 0.0)
    assert scene.robot_yaw_rad == 0.0
    assert scene.instances == (
        LocalizeInstanceEvidence(
            object_name="chair",
            position_world_xyz=(2.0, 0.5, 0.4),
            n_views=2,
            semantic_score=0.9,
            instance_id="g0",
            last_seen_timestamp=1.0,
        ),
    )


def test_scene_at_timestamp_uses_time_local_position(monkeypatch: pytest.MonkeyPatch) -> None:
    """Later sightings must not rewrite object position at an earlier question time."""
    from dimos.evals.vqa.localize_recording import position_at_timestamp

    monkeypatch.setattr(
        localize_recording_module,
        "robot_pose_xyz_yaw",
        lambda rig, ts: ((0.0, 0.0, 0.0), 0.0),
    )
    # Localization.latest is behind the robot; earlier sighting was ahead.
    hit = _hit((-2.0, 0.0, 0.5))
    timeline = {
        "g0": (
            (1.0, 2.0, 0.0, 0.5),
            (5.0, -2.0, 0.0, 0.5),
        )
    }
    scene = scene_at_timestamp(
        (("person", hit),),
        SimpleNamespace(),
        1.0,
        position_timeline_by_instance=timeline,
    )
    assert scene is not None
    assert scene.instances[0].position_world_xyz == (2.0, 0.0, 0.5)
    assert position_at_timestamp(timeline["g0"], 1.0) == (2.0, 0.0, 0.5)


def test_scene_at_timestamp_drops_mover_without_nearby_sighting(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        localize_recording_module,
        "robot_pose_xyz_yaw",
        lambda rig, ts: ((0.0, 0.0, 0.0), 0.0),
    )
    hit = _hit((2.0, 0.0, 0.5))
    # Moved 4 m; question time has no nearby sample → omit.
    timeline = {"g0": ((1.0, 2.0, 0.0, 0.5), (5.0, -2.0, 0.0, 0.5))}
    scene = scene_at_timestamp(
        (("person", hit),),
        SimpleNamespace(),
        8.0,
        position_timeline_by_instance=timeline,
    )
    assert scene is not None
    assert scene.instances == ()


def test_scene_at_timestamp_keeps_static_track_with_sparse_sightings(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Sparse static detections must not erase a chair and flip existential GT."""
    monkeypatch.setattr(
        localize_recording_module,
        "robot_pose_xyz_yaw",
        lambda rig, ts: ((0.0, 0.0, 0.0), 0.0),
    )
    ahead = _hit((2.0, 0.0, 0.5))
    ahead = replace(ahead, instance_id="ahead")
    behind = _hit((-2.0, 0.0, 0.5))
    behind = replace(behind, instance_id="behind")
    # Ahead last seen at t=2 (>2s before question); behind seen near t=4.
    timeline = {
        "ahead": ((2.0, 2.0, 0.0, 0.5),),
        "behind": ((4.0, -2.0, 0.0, 0.5),),
    }
    scene = scene_at_timestamp(
        (("chair", ahead), ("chair", behind)),
        SimpleNamespace(),
        5.0,
        position_timeline_by_instance=timeline,
    )
    assert scene is not None
    by_id = {inst.instance_id: inst.position_world_xyz for inst in scene.instances}
    assert by_id["ahead"] == (2.0, 0.0, 0.5)
    assert by_id["behind"] == (-2.0, 0.0, 0.5)


def test_first_seen_by_instance_for_camera_scopes_to_optical_frame() -> None:
    from dimos.evals.vqa.localize_recording import first_seen_by_instance_for_camera
    from dimos.evals.vqa.primitives.cross_view_identity import SightingView
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat

    front = Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="front", ts=2.0
    )
    rear = Image.from_numpy(
        np.zeros((8, 8, 3), dtype=np.uint8), format=ImageFormat.RGB, frame_id="rear", ts=1.0
    )
    box = (1.0, 1.0, 4.0, 4.0)

    def view(*, camera: str, ts: float, image: object) -> SightingView:
        return SightingView(
            query="chair",
            instance_id="a",
            image=image,  # type: ignore[arg-type]
            bbox=box,
            confidence=0.9,
            timestamp=ts,
            n_views=2,
            semantic_score=0.9,
            camera_frame=camera,
        )

    views = {
        "chair": {
            "a": (
                view(camera="rear", ts=1.0, image=rear),
                view(camera="front", ts=2.0, image=front),
            )
        }
    }
    assert first_seen_by_instance_for_camera(views, "front") == {"a": 2.0}
    assert first_seen_by_instance_for_camera(views, "rear") == {"a": 1.0}


def test_track_horizontal_displacement_flags_movers() -> None:
    from dimos.evals.vqa.localize_recording import (
        _MOVING_TRACK_DISPLACEMENT_M,
        track_horizontal_displacement_m,
    )

    static = ((1.0, 1.0, 0.0, 0.5), (2.0, 1.05, 0.02, 0.5))
    moving = ((1.0, 2.0, 0.0, 0.5), (5.0, -2.0, 0.0, 0.5))
    assert track_horizontal_displacement_m(static) < _MOVING_TRACK_DISPLACEMENT_M
    assert track_horizontal_displacement_m(moving) > _MOVING_TRACK_DISPLACEMENT_M


def _rig_with_one_frame() -> SimpleNamespace:
    from unittest.mock import Mock

    frame_image = Mock()
    observation = SimpleNamespace(data=frame_image, ts=1.0)

    class _Color:
        def after(self, _after: float) -> _Color:
            return self

        def before(self, _before: float) -> _Color:
            return self

        def transform(self, _op: object) -> list[SimpleNamespace]:
            return [observation]

    return SimpleNamespace(cameras={"cam": SimpleNamespace(color=_Color())})


def test_discover_object_names_uses_query_score_rows_on_shared_detector() -> None:
    from unittest.mock import Mock

    import numpy as np

    detector = Mock()
    # one box: chair=0.05 (below floor), potted plant=0.40
    detector.query_score_rows.return_value = (
        np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float64),
        np.asarray([[0.05, 0.40]], dtype=np.float64),
    )

    names = discover_object_names(
        _rig_with_one_frame(),  # type: ignore[arg-type]
        0.0,
        10.0,
        detector=detector,
        discovery_vocab=("chair", "potted plant"),
        score_threshold=0.18,
        max_frames=1,
    )

    assert names == ("potted plant",)
    detector.query_score_rows.assert_called_once()
    detector.stop.assert_not_called()


def test_discover_object_names_vlm_merges_per_frame_lists() -> None:
    from unittest.mock import Mock

    vl_model = Mock()
    vl_model.query_json.return_value = {"objects": ["green backpack", "object", "blue mug"]}

    names = discover_object_names_vlm(
        _rig_with_one_frame(),  # type: ignore[arg-type]
        0.0,
        10.0,
        vl_model=vl_model,
        max_frames=1,
    )

    assert names == ("green backpack", "blue mug")
    vl_model.query_json.assert_called_once()
    vl_model.stop.assert_not_called()


def test_proposals_from_localize_scene_cover_singles_and_pairs() -> None:
    scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(object_name="door", position_world_xyz=(1.0, 1.0, 1.0)),
            LocalizeInstanceEvidence(object_name="table", position_world_xyz=(1.0, 0.0, 0.7)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.0, 0.5)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, -1.0, 0.5)),
        ),
    )
    proposals = proposals_from_localize_scene(scene)
    families = {(p.family, p.object_names) for p in proposals}
    assert ("ego_fore_aft", ("door",)) in families
    assert ("ego_turn", ("door",)) in families
    assert ("ego_fore_aft", ("table",)) in families
    assert ("ego_any_fore_aft", ("door",)) in families
    assert ("height_order", ("door", "table")) in families
    # multi-instance chair → nearest / any-ahead only (no left/right wording)
    assert ("ego_fore_aft", ("nearest chair",)) in families
    assert ("ego_turn", ("nearest chair",)) in families
    assert ("ego_fore_aft", ("left chair",)) not in families
    assert ("ego_fore_aft", ("right chair",)) not in families
    assert ("ego_turn", ("left chair",)) not in families
    assert ("ego_turn", ("right chair",)) not in families
    assert ("ego_any_fore_aft", ("chair",)) in families
    assert ("ego_fore_aft", ("chair",)) not in families


def test_proposals_include_next_to_relational_referents() -> None:
    scene = LocalizeSceneEvidence(
        robot_position_xyz=(0.0, 0.0, 0.0),
        robot_yaw_rad=0.0,
        instances=(
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.0, 0.5)),
            LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(5.0, -2.0, 0.5)),
            LocalizeInstanceEvidence(object_name="table", position_world_xyz=(2.2, 1.1, 0.7)),
        ),
    )
    proposals = proposals_from_localize_scene(scene)
    families = {(p.family, p.object_names) for p in proposals}
    assert ("ego_fore_aft", ("chair next to table",)) in families
    assert ("ego_turn", ("chair next to table",)) in families
