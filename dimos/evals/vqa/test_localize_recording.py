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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.types import Localization


def _hit(xyz: tuple[float, float, float]) -> Localization:
    cloud = PointCloud2.from_numpy(
        np.zeros((1, 3), dtype=np.float32), frame_id="world", timestamp=0.0
    )
    return Localization(
        instance_id="g0",
        semantic_score=0.9,
        ambiguity_margin=1.0,
        position_world_xyz=xyz,
        frame_id="world",
        last_seen_timestamp=1.0,
        point_cloud=cloud,
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
