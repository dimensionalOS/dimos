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

import copy

import pytest
from microduck_world.scene import WorldScene, load_world
from pydantic import ValidationError


@pytest.fixture
def scene_data():
    return {
        "id": "test-world",
        "spawn_xy": [10.0, 20.0],
        "rooms": {
            "studio": {
                "name": "studio",
                "aliases": ["workspace"],
                "bounds": [9.0, 12.0, 19.0, 22.0],
                "target": [11.0, 21.0, 0.0],
            }
        },
        "objects": {"bench": [10.5, 21.5]},
    }


def test_world_metadata_supports_a_relocated_scene(scene_data):
    scene = WorldScene.model_validate(scene_data)
    assert scene.spawn_xy == (10.0, 20.0)
    assert scene.rooms["studio"].target == (11.0, 21.0, 0.0)
    assert scene.objects == {"bench": (10.5, 21.5)}


@pytest.mark.parametrize("name", ["studio", "WORKSPACE", ""])
def test_landmarks_cannot_shadow_room_names_or_aliases(scene_data, name):
    scene_data["objects"] = {name: [10.5, 21.5]}
    with pytest.raises(ValidationError, match="empty or ambiguous"):
        WorldScene.model_validate(scene_data)


def test_two_rooms_cannot_share_an_alias(scene_data):
    second = copy.deepcopy(scene_data["rooms"]["studio"])
    second["name"] = "office"
    scene_data["rooms"]["office"] = second
    with pytest.raises(ValidationError, match="empty or ambiguous"):
        WorldScene.model_validate(scene_data)


def test_room_key_must_match_the_displayed_name(scene_data):
    scene_data["rooms"]["studio"]["name"] = "office"
    with pytest.raises(ValidationError, match="must match"):
        WorldScene.model_validate(scene_data)


def test_navigation_target_must_be_in_its_room(scene_data):
    scene_data["rooms"]["studio"]["target"] = [0.0, 0.0, 0.0]
    with pytest.raises(ValidationError, match="inside its bounds"):
        WorldScene.model_validate(scene_data)


def test_room_bounds_cannot_be_reversed(scene_data):
    scene_data["rooms"]["studio"]["bounds"] = [12.0, 9.0, 19.0, 22.0]
    with pytest.raises(ValidationError, match="increasing"):
        WorldScene.model_validate(scene_data)


@pytest.mark.parametrize("coordinate", [float("nan"), float("inf")])
def test_non_finite_coordinates_are_rejected(scene_data, coordinate):
    scene_data["objects"]["bench"] = [coordinate, 21.5]
    with pytest.raises(ValidationError, match="finite"):
        WorldScene.model_validate(scene_data)


def test_shipped_scene_loads_with_its_artifacts():
    package, scene = load_world()
    assert package.mujoco_scene_path.is_file()
    assert scene.id == "football-club-v3"
    assert set(scene.rooms) == {
        "kitchen",
        "living",
        "bedroom",
        "office",
        "football",
        "red_lockers",
        "blue_lockers",
        "player_tunnel",
        "benchmark_corridor",
    }


def test_compact_lockers_keep_half_the_previous_area_and_midfield_spawns_are_clear():
    import json

    from microduck_world.roster import SETTINGS
    from microduck_world.scene import PROJECT_ROOT

    places = json.loads((PROJECT_ROOT / "assets/scenes/apartment/places.json").read_text())
    for team in ("red", "blue"):
        x0, x1, y0, y1 = places["rooms"][team + "_lockers"]["bounds"]
        assert abs((x1 - x0) * (y1 - y0) - (2.55 * 3.25 / 2)) < 1e-6
        bays = [r["spawn"] for r in SETTINGS["robots"].values() if r["team"] == team]
        assert len(bays) == 3
        expected_y = 3.25 if team == "red" else 5.35
        assert sorted(x for x, _ in bays) == [-0.6, 0.0, 0.6]
        assert all(abs(y - expected_y) < 1e-6 for _, y in bays)
    import math
    from itertools import combinations

    assert all(
        math.dist(a, b) >= SETTINGS["clearance"] for a, b in combinations(SETTINGS["spawns"], 2)
    )
