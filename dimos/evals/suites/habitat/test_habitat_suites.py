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
    ("replicacad_v3_sc1_staging_00", "v3_sc1_staging_00", 13),
    ("replicacad_v3_sc2_staging_00", "v3_sc2_staging_00", 13),
    ("hssd_102344193", "102344193", 11),
    ("hssd_102344403", "102344403", 14),
    ("hssd_103997424_171030444", "103997424_171030444", 13),
    ("hssd_103997970_171031287", "103997970_171031287", 12),
    ("hssd_104348463_171513588", "104348463_171513588", 10),
    ("hssd_106366410_174226806", "106366410_174226806", 12),
    ("hssd_106878858_174886965", "106878858_174886965", 11),
    ("hssd_107734110_175999914", "107734110_175999914", 10),
    ("hssd_108736851_177263586", "108736851_177263586", 11),
    ("hssd_108736884_177263634", "108736884_177263634", 9),
]

START_OVERRIDES = {
    "hssd_102344193": (3.0, 5.5, 0.124386),
    "hssd_102344403": (3.713, 6.3, 0.159347),
}


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
        "hssd_102344193": "hssd_scene_1",
        "hssd_102344403": "hssd_scene_2",
        "hssd_103997424_171030444": "hssd_scene_3",
        "hssd_103997970_171031287": "hssd_scene_4",
        "hssd_104348463_171513588": "hssd_scene_5",
        "hssd_106366410_174226806": "hssd_scene_6",
        "hssd_106878858_174886965": "hssd_scene_7",
        "hssd_107734110_175999914": "hssd_scene_8",
        "hssd_108736851_177263586": "hssd_scene_9",
        "hssd_108736884_177263634": "hssd_scene_10",
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
        assert "point-nav-skill-container" in habitat.blueprint  # agents must be able to move
        if name in START_OVERRIDES:
            assert habitat.start_position_ros_override == START_OVERRIDES[name]
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
        ("hm3d_CFVBbU9Rsyb", "sofa_color", "Answer: C", 1),
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
        ("hssd_102344193", "bedrooms", "1", 1),
        ("hssd_102344193", "bathroom_count", "1", 1),
        ("hssd_102344193", "largest_room", "B", 1),
        ("hssd_102344193", "living_area", "47.23", 1),
        ("hssd_102344193", "bedroom_perimeter", "15.55", 1),
        ("hssd_102344193", "laptop_location", "C", 1),
        ("hssd_102344193", "laundry_exists", "yes", 1),
        ("hssd_102344193", "fridge_height", "1.68", 1),
        ("hssd_102344193", "room_area_order", "ACB", 1),
        ("hssd_102344193", "fridge_state", "B", 1),
        ("hssd_102344193", "laptop_tv_distance", "11.46", 1),
        ("hssd_102344403", "garage_cars", "3", 1),
        ("hssd_102344403", "every_bedroom_tv", "no", 1),
        ("hssd_102344403", "arcade_exists", "no", 1),
        ("hssd_102344403", "arcade_exists", "yes", 0),
        ("hssd_102344403", "dumbbells", "6", 1),
        ("hssd_102344403", "smallest_car_color", "C", 1),
        ("hssd_102344403", "lounge_path_order", "A,C,B", 1),
        ("hssd_102344403", "lounge_path_order", "ABC", 2 / 3),
        ("hssd_103997970_171031287", "room_count", "3", 1),
        ("hssd_103997970_171031287", "dining_exists", "no", 1),
        ("hssd_103997970_171031287", "largest_room", "C", 1),
        ("hssd_103997970_171031287", "smallest_room", "B", 1),
        ("hssd_103997970_171031287", "laptop_exists", "no", 1),
        ("hssd_103997970_171031287", "dining_table_diameter", "1.60", 1),
        ("hssd_103997970_171031287", "tv_location", "C", 1),
        ("hssd_103997970_171031287", "every_room_plants", "yes", 1),
        ("hssd_103997424_171030444", "computer_bed_wall", "yes", 1),
        ("hssd_103997424_171030444", "adjacent_red_objects", "D", 1),
        ("hssd_103997424_171030444", "adjacent_red_objects", "A", 0),
        ("hssd_103997424_171030444", "dining_table_diagonal", "2.43", 1),
        ("hssd_103997424_171030444", "dining_table_diagonal", "2.55", 1),
        ("hssd_103997424_171030444", "dining_table_diagonal", "2.88", 0),
        ("hssd_104348463_171513588", "island_chairs", "3", 1),
        ("hssd_104348463_171513588", "room_count", "3", 1),
        ("hssd_107734110_175999914", "office_doorway_radius", "0.49", 1),
        ("hssd_107734110_175999914", "office_doorway_radius", "0.7", 0),
        ("hssd_107734110_175999914", "piano_computer_distance", "11.64", 1),
        ("hssd_106366410_174226806", "laundry_appliances", "3", 1),
        ("hssd_106366410_174226806", "red_trash_bin_location", "B", 1),
        ("hssd_106366410_174226806", "red_trash_bin_location", "A", 0),
        ("hssd_106366410_174226806", "bed_relative_to_sofa", "A", 1),
        ("hssd_106366410_174226806", "bed_relative_to_sofa", "B", 0),
        ("hssd_106366410_174226806", "toilet_room_bathtub", "yes", 1),
        ("hssd_106366410_174226806", "bedroom_farthest_object", "A", 1),
        ("hssd_106366410_174226806", "bedroom_farthest_object", "C", 0),
        ("hssd_106878858_174886965", "beds", "4", 1),
        ("hssd_106878858_174886965", "garage_bedroom_doorways", "4", 1),
        ("hssd_106878858_174886965", "entryway_path_order", "A B C", 1),
        ("hssd_106878858_174886965", "garage_car_color", "The car is blue.\n\n**B**", 1),
        ("hssd_106878858_174886965", "exterior_opening", "yes", 1),
        ("hssd_106878858_174886965", "bathroom_floor_pattern_match", "yes", 1),
        ("hssd_108736851_177263586", "beds", "4", 1),
        ("hssd_108736851_177263586", "dining_chairs", "8", 1),
        ("hssd_108736851_177263586", "curved_sofa_table_shape", "A", 1),
        ("hssd_108736851_177263586", "side_table_sides", "C", 1),
        ("hssd_108736851_177263586", "side_table_sides", "6", 0),
        ("hssd_108736851_177263586", "office_nearest_room", "C", 1),
        ("hssd_108736851_177263586", "office_nearest_room", "B", 0),
        ("hssd_108736884_177263634", "toilets", "3", 1),
        ("hssd_108736884_177263634", "red_potted_plant_location", "D", 1),
        ("hssd_108736884_177263634", "kitchen_counter_windows", "3", 1),
        ("hssd_108736884_177263634", "bathtub_shape_match", "no", 1),
        ("hssd_108736884_177263634", "bathroom_plant_exists", "yes", 1),
        ("hssd_108736884_177263634", "area_order", "CAB", 1),
    ],
)
def test_reference_transcription_and_scores(name, suffix, answer, expected):
    suite = import_module(suite_module(name)).SUITE
    c = next(c for c in suite if c.id == f"{name}_{suffix}")
    assert c.grade(
        SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))
    ) == pytest.approx(expected)


def test_hssd_dataset_config_follows_environment(monkeypatch, tmp_path):
    from dimos.evals.suites.habitat.hssd.hssd_scene_1 import _environment

    dataset = tmp_path / "hssd-hab.scene_dataset_config.json"
    monkeypatch.setenv("HSSD_DATASET_CONFIG", str(dataset))
    assert _environment().config.scene_dataset_config == str(dataset)
