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

"""Offline scene-loading and reviewed-answer regression checks."""

from importlib import import_module
from types import SimpleNamespace

import pytest

SCENES = [
    ("hm3d_CFVBbU9Rsyb", "00337-CFVBbU9Rsyb", 13),
    ("hm3d_GLAQ4DNUx5U", "00861-GLAQ4DNUx5U", 14),
    ("hm3d_NBg5UqG3di3", "00770-NBg5UqG3di3", 9),
    ("habitat_test_apartment_1", None, 7),
    ("replicacad_apt_1", "apt_1", 13),
    ("replicacad_apt_5", "apt_5", 13),
    ("replicacad_v3_sc1_staging_00", "v3_sc1_staging_00", 14),
    ("replicacad_v3_sc2_staging_00", "v3_sc2_staging_00", 14),
]


def suite_module(name):
    family = "test" if name.startswith("habitat_test_") else name.split("_", 1)[0]
    filename = {
        "hm3d_CFVBbU9Rsyb": "hm3d_scene_1",
        "hm3d_GLAQ4DNUx5U": "hm3d_scene_2",
        "hm3d_NBg5UqG3di3": "hm3d_scene_3",
        "habitat_test_apartment_1": "habitat_test_scene_1",
        "replicacad_apt_1": "replicacad_scene_1",
        "replicacad_apt_5": "replicacad_scene_2",
        "replicacad_v3_sc1_staging_00": "replicacad_scene_3",
        "replicacad_v3_sc2_staging_00": "replicacad_scene_4",
    }[name]
    return f"dimos.evals.suites.habitat.{family}.{filename}"


@pytest.mark.parametrize("name,scene_id,size", SCENES)
def test_scene_contract(name, scene_id, size):
    suite = import_module(suite_module(name)).SUITE
    assert len(suite) == len({c.id for c in suite}) == size
    assert len({id(c.environment) for c in suite}) == size
    for c in suite:
        assert c.id.startswith(name + "_")
        habitat = c.environment.config
        if scene_id is not None:
            assert habitat.scene_id == scene_id
        assert habitat.seed == 0
        assert c.timeout_s == 1200
        for invalid in ("", "unknown"):
            assert c.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer=invalid))) == 0


@pytest.mark.parametrize(
    "name,suffix,answer,expected",
    [
        ("hm3d_CFVBbU9Rsyb", "location_elevation_order", "B,C,A", 1),
        ("hm3d_CFVBbU9Rsyb", "location_elevation_order", "CBA", 2 / 3),
        ("hm3d_CFVBbU9Rsyb", "location_elevation_order", "BBC", 0),
        ("hm3d_CFVBbU9Rsyb", "location_elevation_order", "BC", 0),
        ("hm3d_CFVBbU9Rsyb", "bunk_utility_height_difference", "5.6", 1),
        ("hm3d_CFVBbU9Rsyb", "bunk_utility_height_difference", "6.2", 0.5),
        ("hm3d_CFVBbU9Rsyb", "bunk_utility_height_difference", "6.7", 0),
        ("hm3d_CFVBbU9Rsyb", "sofa_color", " c ", 1),
        ("hm3d_GLAQ4DNUx5U", "every_bedroom_tv", "no", 1),
        ("hm3d_GLAQ4DNUx5U", "every_bedroom_tv", "yes", 0),
        ("hm3d_GLAQ4DNUx5U", "every_desk_chair", "yes", 1),
        ("hm3d_GLAQ4DNUx5U", "mural_location", "C", 1),
        ("hm3d_NBg5UqG3di3", "blue_room_windows", "3", 1),
        ("hm3d_NBg5UqG3di3", "floor_pattern", "B", 1),
        ("habitat_test_apartment_1", "serving_stand_tiers", "2", 1),
        ("habitat_test_apartment_1", "dining_door_state", "B", 1),
        ("replicacad_apt_1", "stools", "2", 1),
        ("replicacad_apt_5", "stools", "1", 1),
        ("replicacad_apt_1", "books", "21", 1),
        ("replicacad_apt_5", "books", "19", 1),
        ("replicacad_apt_1", "sofa_stand_distance", "6.04", 1),
        ("replicacad_apt_5", "sofa_stand_distance", "2.61", 1),
        ("replicacad_apt_5", "sofa_stand_distance", "6.04", 0),
        ("replicacad_v3_sc1_staging_00", "bicycles", "1", 1),
        ("replicacad_v3_sc2_staging_00", "bicycles", "2", 1),
        ("replicacad_v3_sc1_staging_00", "beanbags", "2", 1),
        ("replicacad_v3_sc2_staging_00", "beanbag_exists", "no", 1),
        ("replicacad_v3_sc1_staging_00", "books_exists", "no", 1),
        ("replicacad_v3_sc2_staging_00", "books", "0", 1),
        ("replicacad_v3_sc1_staging_00", "height_order", "ACB", 1),
        ("replicacad_v3_sc2_staging_00", "height_order", "BCA", 1),
    ],
)
def test_reference_transcription_and_scores(name, suffix, answer, expected):
    suite = import_module(suite_module(name)).SUITE
    c = next(c for c in suite if c.id == f"{name}_{suffix}")
    assert c.grade(
        SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))
    ) == pytest.approx(expected)
