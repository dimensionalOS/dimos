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

import math

import pytest

from dimos.evals.vqa.contracts import InsufficientEvidenceError
from dimos.evals.vqa.primitives.localize_scene import (
    LocalizeInstanceEvidence,
    LocalizeSceneEvidence,
    any_ahead,
    ego_turn_to,
    fore_aft_of,
    higher_object,
    next_to_referents,
    parse_referent,
    resolve_referent,
    robot_relative_xy,
    spatial_side_referents,
)


def _scene(
    *instances: LocalizeInstanceEvidence,
    yaw: float = 0.0,
    robot_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> LocalizeSceneEvidence:
    return LocalizeSceneEvidence(
        robot_position_xyz=robot_xyz,
        robot_yaw_rad=yaw,
        instances=instances,
    )


def test_robot_relative_xy_rotates_into_body_frame() -> None:
    scene = _scene(yaw=math.pi / 2)
    forward, left = robot_relative_xy(scene, (1.0, 0.0, 0.0))
    assert forward == pytest.approx(0.0, abs=1e-9)
    assert left == pytest.approx(-1.0, abs=1e-9)


@pytest.mark.parametrize(
    ("xyz", "expected"),
    (
        ((2.0, 0.0, 0.0), "ahead"),
        ((-2.0, 0.0, 0.0), "behind"),
    ),
)
def test_fore_aft_uses_forward_axis_sign(xyz: tuple[float, float, float], expected: str) -> None:
    scene = _scene(LocalizeInstanceEvidence(object_name="chair", position_world_xyz=xyz))
    assert fore_aft_of(scene, "chair") == expected


def test_fore_aft_rejects_near_lateral_plane() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(0.1, 1.0, 0.0))
    )
    with pytest.raises(InsufficientEvidenceError, match="fore/aft"):
        fore_aft_of(scene, "chair")


@pytest.mark.parametrize(
    ("xyz", "expected"),
    (
        ((2.0, 0.0, 0.0), "ahead"),
        ((1.0, 1.0, 0.0), "left"),
        ((1.0, -1.0, 0.0), "right"),
    ),
)
def test_ego_turn_buckets_bearing(xyz: tuple[float, float, float], expected: str) -> None:
    scene = _scene(LocalizeInstanceEvidence(object_name="door", position_world_xyz=xyz))
    assert ego_turn_to(scene, "door") == expected


def test_higher_object_compares_world_z() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="shelf", position_world_xyz=(1.0, 0.0, 1.5)),
        LocalizeInstanceEvidence(object_name="table", position_world_xyz=(1.0, 1.0, 0.8)),
    )
    assert higher_object(scene, "shelf", "table") == "shelf"


def test_higher_object_rejects_small_margin() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="shelf", position_world_xyz=(1.0, 0.0, 1.0)),
        LocalizeInstanceEvidence(object_name="table", position_world_xyz=(1.0, 1.0, 0.95)),
    )
    with pytest.raises(InsufficientEvidenceError, match="height difference"):
        higher_object(scene, "shelf", "table")


def test_ego_turn_rejects_near_robot() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="door", position_world_xyz=(0.05, 0.05, 0.0))
    )
    with pytest.raises(InsufficientEvidenceError, match="ego-turn"):
        ego_turn_to(scene, "door")


def test_resolve_nearest_picks_closest_instance() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(3.0, 0.0, 0.0)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(1.0, 0.0, 0.0)),
    )
    nearest = resolve_referent(scene, "nearest chair")
    assert nearest.position_world_xyz == (1.0, 0.0, 0.0)
    assert fore_aft_of(scene, "nearest chair") == "ahead"


def test_spatial_side_referents_for_left_and_right_instances() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.2, 0.0)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, -1.2, 0.0)),
    )
    assert spatial_side_referents(scene, "chair") == ("left chair", "right chair")
    assert resolve_referent(scene, "left chair").position_world_xyz[1] == pytest.approx(1.2)
    assert resolve_referent(scene, "right chair").position_world_xyz[1] == pytest.approx(-1.2)


def test_any_ahead_true_when_one_instance_ahead() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(-2.0, 0.0, 0.0)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 0.5, 0.0)),
    )
    assert any_ahead(scene, "chair") is True


def test_any_ahead_false_when_all_behind() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(-2.0, 0.0, 0.0)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(-1.0, 0.5, 0.0)),
    )
    assert any_ahead(scene, "chair") is False


def test_next_to_picks_anchor_nearest_landmark() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(1.0, 2.0, 0.5)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(5.0, -2.0, 0.5)),
        LocalizeInstanceEvidence(object_name="table", position_world_xyz=(1.1, 2.1, 0.7)),
    )
    parsed = parse_referent("chair next to table")
    assert parsed.selector == "next_to"
    assert parsed.category == "chair"
    assert parsed.landmark == "table"
    inst = resolve_referent(scene, "chair next to table")
    assert inst.position_world_xyz == (1.0, 2.0, 0.5)
    assert ego_turn_to(scene, "chair next to table") == "left"


def test_next_to_rejects_ambiguous_pairs() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 0.0, 0.5)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.05, 0.2, 0.5)),
        LocalizeInstanceEvidence(object_name="table", position_world_xyz=(2.1, 0.1, 0.7)),
    )
    with pytest.raises(InsufficientEvidenceError, match="ambiguous"):
        resolve_referent(scene, "chair next to table")


def test_next_to_referents_lists_unambiguous_pairs() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 1.0, 0.5)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(5.0, -2.0, 0.5)),
        LocalizeInstanceEvidence(object_name="table", position_world_xyz=(2.2, 1.1, 0.7)),
    )
    assert "chair next to table" in next_to_referents(scene)


def test_bare_name_still_requires_unique_instance() -> None:
    scene = _scene(
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(2.0, 0.0, 0.0)),
        LocalizeInstanceEvidence(object_name="chair", position_world_xyz=(3.0, 0.0, 0.0)),
    )
    with pytest.raises(InsufficientEvidenceError, match="exactly one verified"):
        fore_aft_of(scene, "chair")


def test_from_localization_maps_query_name_not_instance_id() -> None:
    from types import SimpleNamespace

    hit = SimpleNamespace(
        position_world_xyz=(1.0, 2.0, 0.5),
        n_views=3,
        semantic_score=0.9,
        instance_id="group-7",
    )
    evidence = LocalizeInstanceEvidence.from_localization("chair", hit)  # type: ignore[arg-type]
    assert evidence.object_name == "chair"
    assert evidence.position_world_xyz == (1.0, 2.0, 0.5)
    assert evidence.n_views == 3
    assert evidence.semantic_score == 0.9
