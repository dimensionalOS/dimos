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

from types import SimpleNamespace

import pytest

from dimos.evals.suites.habitat.hssd.hssd_scene_1 import SUITE, _environment
from dimos.evals.suites.habitat.hssd.hssd_scene_2 import SUITE as LARGE_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_3 import SUITE as OFFICE_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_4 import SUITE as COMPACT_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_5 import SUITE as FURNISHED_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_6 import SUITE as GYM_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_7 import SUITE as GARAGE_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_8 import SUITE as PIANO_HOME_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_9 import SUITE as TWO_KITCHEN_SUITE
from dimos.evals.suites.habitat.hssd.hssd_scene_10 import SUITE as THREE_BEDROOM_SUITE


def test_reviewed_scene_contract(monkeypatch, tmp_path):
    dataset = tmp_path / "hssd-hab.scene_dataset_config.json"
    monkeypatch.setenv("HSSD_DATASET_CONFIG", str(dataset))
    env = _environment()
    assert env.config.scene_dataset_config == str(dataset)
    assert env.config.scene_id == "102344193"
    assert env.config.start_position_ros_override == (3.0, 5.5, 0.124386)
    assert len(SUITE) == len({case.id for case in SUITE}) == 11
    assert len({id(case.environment) for case in SUITE}) == 11
    for case in SUITE:
        assert case.environment.config.scene_id == "102344193"
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suffix,answer",
    [
        ("bedrooms", "1"),
        ("bathroom_count", "1"),
        ("largest_room", "B"),
        ("living_area", "47.23"),
        ("bedroom_perimeter", "15.55"),
        ("laptop_location", "C"),
        ("laundry_exists", "yes"),
        ("fridge_height", "1.68"),
        ("room_area_order", "ACB"),
        ("fridge_state", "B"),
        ("laptop_tv_distance", "11.46"),
    ],
)
def test_reviewed_answers_score_full_credit(suffix, answer):
    case = next(case for case in SUITE if case.id == f"hssd_102344193_{suffix}")
    assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))) == 1


def test_large_home_reviewed_contract():
    assert len(LARGE_HOME_SUITE) == len({case.id for case in LARGE_HOME_SUITE}) == 14
    assert len({id(case.environment) for case in LARGE_HOME_SUITE}) == 14
    for case in LARGE_HOME_SUITE:
        assert case.environment.config.scene_id == "102344403"
        assert case.environment.config.start_position_ros_override == (3.713, 6.3, 0.159347)
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suffix,answer,expected",
    [
        ("garage_cars", "3", 1),
        ("every_bedroom_tv", "no", 1),
        ("arcade_exists", "no", 1),
        ("arcade_exists", "yes", 0),
        ("dumbbells", "6", 1),
        ("smallest_car_color", "C", 1),
        ("lounge_path_order", "A,C,B", 1),
        ("lounge_path_order", "ABC", 2 / 3),
    ],
)
def test_large_home_human_corrections(suffix, answer, expected):
    case = next(c for c in LARGE_HOME_SUITE if c.id == f"hssd_102344403_{suffix}")
    assert case.grade(
        SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))
    ) == pytest.approx(expected)


def test_compact_home_contract():
    assert len(COMPACT_HOME_SUITE) == len({c.id for c in COMPACT_HOME_SUITE}) == 12
    assert len({id(c.environment) for c in COMPACT_HOME_SUITE}) == 12
    for case in COMPACT_HOME_SUITE:
        assert case.environment.config.scene_id == "103997970_171031287"
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suffix,answer",
    [
        ("room_count", "3"),
        ("dining_exists", "no"),
        ("largest_room", "C"),
        ("smallest_room", "B"),
        ("laptop_exists", "no"),
        ("dining_table_diameter", "1.60"),
        ("tv_location", "C"),
        ("every_room_plants", "yes"),
    ],
)
def test_compact_home_human_corrections(suffix, answer):
    case = next(c for c in COMPACT_HOME_SUITE if c.id == f"hssd_103997970_171031287_{suffix}")
    assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))) == 1


def test_office_home_contract():
    assert len(OFFICE_HOME_SUITE) == len({c.id for c in OFFICE_HOME_SUITE}) == 13
    assert len({id(c.environment) for c in OFFICE_HOME_SUITE}) == 13
    for case in OFFICE_HOME_SUITE:
        assert case.environment.config.scene_id == "103997424_171030444"
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suffix,answer,score",
    [
        ("computer_bed_wall", "yes", 1),
        ("adjacent_red_objects", "D", 1),
        ("adjacent_red_objects", "A", 0),
        ("dining_table_diagonal", "2.43", 1),
        ("dining_table_diagonal", "2.55", 1),
        ("dining_table_diagonal", "2.88", 0),
    ],
)
def test_office_home_reviewed_answers(suffix, answer, score):
    case = next(c for c in OFFICE_HOME_SUITE if c.id == f"hssd_103997424_171030444_{suffix}")
    assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))) == score


def test_furnished_home_contract():
    assert len(FURNISHED_HOME_SUITE) == len({c.id for c in FURNISHED_HOME_SUITE}) == 10
    for case in FURNISHED_HOME_SUITE:
        assert case.environment.config.scene_id == "104348463_171513588"
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suffix,answer",
    [("island_chairs", "3"), ("room_count", "3")],
)
def test_furnished_home_corrections(suffix, answer):
    case = next(c for c in FURNISHED_HOME_SUITE if c.id == f"hssd_104348463_171513588_{suffix}")
    assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))) == 1


@pytest.mark.parametrize(
    "scene_id,suite,count",
    [
        ("106366410_174226806", GYM_HOME_SUITE, 12),
        ("106878858_174886965", GARAGE_HOME_SUITE, 11),
        ("107734110_175999914", PIANO_HOME_SUITE, 10),
        ("108736851_177263586", TWO_KITCHEN_SUITE, 11),
        ("108736884_177263634", THREE_BEDROOM_SUITE, 9),
    ],
)
def test_remaining_scene_contracts(scene_id, suite, count):
    assert len(suite) == len({c.id for c in suite}) == count
    assert len({id(c.environment) for c in suite}) == count
    for case in suite:
        assert case.id.startswith(f"hssd_{scene_id}_")
        assert case.environment.config.scene_id == scene_id
        assert case.environment.config.seed == 0
        assert case.grade(SimpleNamespace(trajectory=SimpleNamespace(final_answer="unknown"))) == 0


@pytest.mark.parametrize(
    "suite,suffix,answer,score",
    [
        (PIANO_HOME_SUITE, "office_doorway_radius", "0.49", 1),
        (PIANO_HOME_SUITE, "office_doorway_radius", "0.7", 0),
        (PIANO_HOME_SUITE, "piano_computer_distance", "11.64", 1),
        (GYM_HOME_SUITE, "laundry_appliances", "3", 1),
        (GYM_HOME_SUITE, "red_trash_bin_location", "B", 1),
        (GYM_HOME_SUITE, "red_trash_bin_location", "A", 0),
        (GYM_HOME_SUITE, "bed_relative_to_sofa", "A", 1),
        (GYM_HOME_SUITE, "bed_relative_to_sofa", "B", 0),
        (GYM_HOME_SUITE, "toilet_room_bathtub", "yes", 1),
        (GYM_HOME_SUITE, "bedroom_path_order", "B,C,A", 1),
        (GYM_HOME_SUITE, "bedroom_path_order", "CBA", 2 / 3),
        (GYM_HOME_SUITE, "bedroom_path_order", "BBA", 0),
        (GARAGE_HOME_SUITE, "beds", "4", 1),
        (GARAGE_HOME_SUITE, "garage_bedroom_doorways", "4", 1),
        (GARAGE_HOME_SUITE, "entryway_path_order", "A B C", 1),
        (GARAGE_HOME_SUITE, "garage_car_color", " b ", 1),
        (GARAGE_HOME_SUITE, "exterior_opening", "yes", 1),
        (GARAGE_HOME_SUITE, "bathroom_floor_pattern_match", "yes", 1),
        (TWO_KITCHEN_SUITE, "beds", "4", 1),
        (TWO_KITCHEN_SUITE, "dining_chairs", "8", 1),
        (TWO_KITCHEN_SUITE, "curved_sofa_table_shape", "A", 1),
        (TWO_KITCHEN_SUITE, "side_table_sides", "C", 1),
        (TWO_KITCHEN_SUITE, "side_table_sides", "6", 0),
        (TWO_KITCHEN_SUITE, "office_path_order", "CBA", 1),
        (TWO_KITCHEN_SUITE, "office_path_order", "CB", 0),
        (THREE_BEDROOM_SUITE, "toilets", "3", 1),
        (THREE_BEDROOM_SUITE, "red_potted_plant_location", "D", 1),
        (THREE_BEDROOM_SUITE, "kitchen_counter_windows", "3", 1),
        (THREE_BEDROOM_SUITE, "bathtub_shape_match", "no", 1),
        (THREE_BEDROOM_SUITE, "bathroom_plant_exists", "yes", 1),
        (THREE_BEDROOM_SUITE, "area_order", "CAB", 1),
    ],
)
def test_remaining_review_corrections_and_scoring(suite, suffix, answer, score):
    scene_id = suite[0].environment.config.scene_id
    case = next(c for c in suite if c.id == f"hssd_{scene_id}_{suffix}")
    assert case.grade(
        SimpleNamespace(trajectory=SimpleNamespace(final_answer=answer))
    ) == pytest.approx(score)
