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

import numpy as np
import pytest

from dimos.teleop.memory_world.tf_tree import (
    TfTree,
    pose_matrix,
    quaternion_from_matrix,
    slerp,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)
YAW_90 = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
YAW_180 = (0.0, 0.0, 1.0, 0.0)


def test_static_chain_composes_parent_to_child() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (1.0, 0.0, 0.0), YAW_90)
    tree.add("body", "camera", 0.0, (1.0, 0.0, 0.0), IDENTITY)

    matrix = tree.lookup("world", "camera", 5.0)

    # The camera sits 1 m along the body's x, which yaw 90 turns into world +y.
    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.0, 1.0, 0.0))


def test_two_routes_of_equal_length_are_not_chosen_by_the_hash_seed() -> None:
    """The same recording has to place its map the same way on every run.

    `_path` walked `self._neighbours[frame]`, a set, and a set of strings iterates in an
    order that depends on hash randomisation -- which Python re-rolls per PROCESS. A tf
    tree with two equally short routes between the same pair of frames therefore answered
    with one route on one run and the other on the next, with nothing in the recording or
    the code having changed, and each route composes to a different matrix.

    So this asks several interpreters, each with a different PYTHONHASHSEED, and requires
    that they agree. A single process cannot see this: within one run the order is fixed.
    """
    import json
    import os
    import subprocess
    import sys

    program = """
import json
from dimos.teleop.memory_world.tf_tree import TfTree

tree = TfTree()
# A diamond: world -> left -> tool and world -> right -> tool, both two hops, and the
# two routes put `tool` in very different places.
tree.add("world", "left", 0.0, (0.0, 10.0, 0.0), (0.0, 0.0, 0.0, 1.0))
tree.add("left", "tool", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
tree.add("world", "right", 0.0, (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
tree.add("right", "tool", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
matrix = tree.lookup("world", "tool", 0.0)
print(json.dumps([round(float(v), 6) for v in matrix[:3, 3]]))
"""
    answers = set()
    for seed in ("1", "2", "3", "4", "5", "6", "7", "8"):
        environment = {**os.environ, "PYTHONHASHSEED": seed}
        out = subprocess.run(
            [sys.executable, "-c", program],
            capture_output=True,
            text=True,
            check=True,
            env=environment,
        )
        answers.add(json.dumps(json.loads(out.stdout.strip())))

    assert len(answers) == 1, f"the route depends on the hash seed: {sorted(answers)}"


def test_lookup_walks_edges_backwards_too() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (1.0, 2.0, 0.0), IDENTITY)
    tree.add("body", "camera", 0.0, (0.5, 0.0, 0.0), IDENTITY)

    matrix = tree.lookup("camera", "world", 0.0)

    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((-1.5, -2.0, 0.0))


def test_dynamic_edge_interpolates_between_samples() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), YAW_180)

    matrix = tree.lookup("world", "body", 11.0)

    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.0, 0.0, 0.0))
    # Halfway to a 180 degree yaw is a 90 degree yaw: body x points along world y.
    assert matrix[:3, 0] == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)


def test_lookup_holds_the_ends_within_tolerance_and_refuses_beyond() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), IDENTITY)

    held = tree.lookup("world", "body", 12.05, tolerance_s=0.1)
    assert held is not None
    assert held[:3, 3] == pytest.approx((2.0, 0.0, 0.0))
    assert tree.lookup("world", "body", 12.5, tolerance_s=0.1) is None
    assert tree.lookup("world", "body", 9.5, tolerance_s=0.1) is None


def test_unconnected_frames_have_no_transform() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("other", "thing", 0.0, (0.0, 0.0, 0.0), IDENTITY)
    assert tree.lookup("world", "thing", 0.0) is None
    assert tree.lookup("world", "nowhere", 0.0) is None


def test_samples_may_arrive_out_of_order() -> None:
    tree = TfTree()
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    matrix = tree.lookup("world", "body", 11.5)
    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.5, 0.0, 0.0))


def test_span_is_where_every_edge_on_the_path_has_data() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 20.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 12.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 30.0, (0.0, 0.0, 0.0), IDENTITY)
    assert tree.span("world", "camera") == (12.0, 20.0)
    assert tree.span("world", "nowhere") is None


def test_an_edge_published_once_spans_from_then_on() -> None:
    """A camera hung on the body by one tf message (no tf_static) is held afterwards,
    so the span of the path is bounded by the moving edge, not collapsed to an instant."""
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 20.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 12.0, (0.0, 0.0, 1.0), IDENTITY)
    assert tree.span("world", "camera") == (12.0, 20.0)
    assert tree.lookup("world", "camera", 19.0) is not None


def test_quaternion_round_trips_through_the_matrix() -> None:
    for quat in [IDENTITY, (0.0, 0.218, 0.0, 0.976), (0.5, 0.5, 0.5, 0.5), YAW_90]:
        unit = np.array(quat) / np.linalg.norm(quat)
        back = np.array(quaternion_from_matrix(pose_matrix((0, 0, 0), tuple(unit))[:3, :3]))
        assert np.allclose(back, unit, atol=1e-6) or np.allclose(back, -unit, atol=1e-6)


def test_slerp_halfway_between_identity_and_a_quarter_turn() -> None:
    halfway = slerp(np.array(IDENTITY), np.array(YAW_90), 0.5)
    angle = 2 * np.degrees(np.arccos(halfway[3]))
    assert angle == pytest.approx(45.0, abs=1e-6)
    # ...and about the RIGHT AXIS. The angle comes from w alone, so swapping x and z in
    # the result left this green while a roll was returned as halfway to a yaw.
    assert halfway[:3] == pytest.approx([0.0, 0.0, np.sin(np.radians(22.5))], abs=1e-6)
