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

"""Historical identity must not become a destination in a new map."""

import importlib.util
from pathlib import Path

SPEC = importlib.util.spec_from_file_location("frank_world", Path(__file__).with_name("world.py"))
world_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(world_module)


def test_restored_sighting_never_supplies_map_coordinates():
    world = world_module.World()
    world.load({"p1": {"last_seen_ts": 90, "x": 2.5, "y": -6, "in_view": True}})
    people = [{"person_id": "p1", "name": "Henry", "last_seen_pose": {"x": 2.5, "y": -6}}]
    row = world_module.snapshot(world, people, now=100)["people"][0]
    assert (row["name"], row["last_seen_ts"], row["in_view"], row["x"], row["y"]) == (
        "Henry",
        90,
        False,
        None,
        None,
    )


def test_current_sighting_position_expires():
    world = world_module.World()
    world.record("p1", {"x": 1, "y": 2}, now=100)
    people = [{"person_id": "p1", "name": "Henry"}]
    assert world_module.snapshot(world, people, now=110)["people"][0]["x"] == 1
    assert world_module.snapshot(world, people, now=131)["people"][0]["x"] is None
