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

import json
from pathlib import Path

import numpy as np
import pytest

from dimos.evals.sandbox.scene_grading import GroundTruth, Zone, round_trips, visits

A = Zone("a", center=(0.0, 0.0), radius=1.0)
B = Zone("b", box=(9.0, -1.0, 11.0, 1.0))


def _poses(xs: list[float]) -> np.ndarray:
    return np.array([[x, 0.0, 0.3, 0, 0, 0, 1] for x in xs], dtype=np.float64)


def test_visits_collapse_repeats_and_ignore_the_space_between() -> None:
    assert visits(_poses([0, 0.5, 5, 10, 10.5, 5, 0, 0]), [A, B]) == ["a", "b", "a"]


def test_a_round_trip_is_two_moves_between_the_zones() -> None:
    assert round_trips(["a", "b"], "a", "b") == 0
    assert round_trips(["a", "b", "a"], "a", "b") == 1
    assert round_trips(["b", "a", "b", "a", "b"], "a", "b") == 2


def test_collisions_count_bumps_once_and_skip_feet_on_floor(tmp_path: Path) -> None:
    lines = [
        {"t": 0.0, "contacts": [["foot", "floor"]]},
        {"t": 1.0, "contacts": [["foot", "floor"], ["lidar", "clutter"]]},  # a bump
        {"t": 1.1, "contacts": [["foot", "floor"]]},
        {"t": 1.3, "contacts": [["lidar", "clutter"]]},  # the same bump, chattering
        {"t": 1.4, "contacts": [["foot", "floor"]]},
        {"t": 0.5, "pose": [0, 0, 0.3, 0, 0, 0, 1]},
        {"t": 8.9, "pose": [1, 0, 0.3, 0, 0, 0, 1]},
        {"t": 9.0, "contacts": [["foot", "clutter"]]},  # stepped on a shelf base: a second one
    ]
    path = tmp_path / "ground_truth.jsonl"
    path.write_text("\n".join(json.dumps(line) for line in lines))
    truth = GroundTruth.load(path)
    assert truth.collisions() == [1.0, 9.0]
    assert truth.collisions(start=5.0) == [9.0]
    assert len(truth.poses) == 2


def test_a_robot_parked_against_furniture_bumps_it_once(tmp_path: Path) -> None:
    lines = [{"t": 0.0, "pose": [0, 0, 0.3, 0, 0, 0, 1]}]
    for k in range(10):  # a leg resting on a sofa, flickering in and out for a minute
        lines += [
            {"t": 6.0 * k, "contacts": [["leg", "clutter"]]},
            {"t": 6.0 * k + 3, "contacts": [["foot", "floor"]]},
        ]
    path = tmp_path / "ground_truth.jsonl"
    path.write_text("\n".join(json.dumps(line) for line in lines))
    assert GroundTruth.load(path).collisions() == [0.0]


def test_touching_a_person_is_its_own_kind_of_collision(tmp_path: Path) -> None:
    lines = [
        {"t": 0.0, "contacts": [["foot", "floor"]]},
        {"t": 1.0, "contacts": [["foot", "floor"], ["trunk", "person"]]},
        {"t": 1.2, "contacts": [["foot", "floor"]]},
        {"t": 5.0, "contacts": [["foot", "person"], ["leg", "clutter"]]},  # one of each
        {"t": 5.1, "contacts": [["foot", "floor"]]},
        {"t": 2.0, "pose": [0, 0, 0.3, 0, 0, 0, 1]},
        {"t": 2.0, "people": [[1.0, 0.0, 0.0], [3.0, 0.0, 1.0]]},
        {"t": 2.1, "pose": [0.5, 0, 0.3, 0, 0, 0, 1]},
        {"t": 2.1, "people": [[0.8, 0.0, 0.0], [3.0, 0.0, 1.0]]},
    ]
    path = tmp_path / "ground_truth.jsonl"
    path.write_text("\n".join(json.dumps(line) for line in lines))
    truth = GroundTruth.load(path)
    assert truth.collisions() == [5.0]
    assert truth.collisions(people=True) == [1.0, 5.0]
    assert truth.people.shape == (2, 2, 3)
    assert truth.closest_person() == pytest.approx(0.3)
    assert truth.closest_person(start=3.0) == float("inf")
