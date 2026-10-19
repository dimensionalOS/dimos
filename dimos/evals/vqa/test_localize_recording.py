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
    monkeypatch.setattr(
        localize_recording_module, "robot_pose_xyz_yaw", lambda rig, ts: None
    )
    assert scene_at_timestamp((("chair", _hit((1.0, 0.0, 0.5))),), SimpleNamespace(), 1.0) is None


def test_scene_at_timestamp_builds_named_evidence(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        localize_recording_module,
        "robot_pose_xyz_yaw",
        lambda rig, ts: ((0.0, 0.0, 0.0), 0.0),
    )
    scene = scene_at_timestamp(
        (("chair", _hit((2.0, 0.5, 0.4))),), SimpleNamespace(), 1.0
    )

    assert scene is not None
    assert scene.robot_position_xyz == (0.0, 0.0, 0.0)
    assert scene.robot_yaw_rad == 0.0
    assert scene.instances == (
        LocalizeInstanceEvidence(
            object_name="chair",
            position_world_xyz=(2.0, 0.5, 0.4),
            n_views=2,
            semantic_score=0.9,
        ),
    )


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
    # multi-instance chair → nearest / sides / any-ahead, not bare unique
    assert ("ego_fore_aft", ("nearest chair",)) in families
    assert ("ego_turn", ("nearest chair",)) in families
    assert ("ego_fore_aft", ("left chair",)) in families
    assert ("ego_fore_aft", ("right chair",)) in families
    assert ("ego_any_fore_aft", ("chair",)) in families
    assert ("ego_fore_aft", ("chair",)) not in families
