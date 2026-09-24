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

"""FlexTf against MultiTBuffer: the same answers, a batch at a time."""

from __future__ import annotations

import math

import numpy as np
import pytest

from dimos.mapping.hyperspace.flextf import WIDTH, Edge, FlexTf
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.protocol.tf.tf import MultiTBuffer

CHAIN = [("odom", "base"), ("base", "arm"), ("arm", "camera")]


def transform(parent: str, child: str, ts: float, angle: float, shift: float) -> Transform:
    return Transform(
        translation=Vector3(shift, shift / 2, 0.0),
        rotation=Quaternion(0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def walk(count: int, seed: int = 0) -> list[list[Transform]]:
    """A message per tick, each carrying the whole chain -- the shape tf really arrives in."""
    rng = np.random.default_rng(seed)
    messages = []
    for step in range(count):
        ts = 100.0 + step * 0.05
        messages.append(
            [
                transform(
                    parent, child, ts, float(rng.uniform(-0.4, 0.4)), float(rng.uniform(-1, 1))
                )
                for parent, child in CHAIN
            ]
        )
    return messages


def filled(count: int, seed: int = 0) -> tuple[FlexTf, MultiTBuffer, list[float]]:
    flex, plain = FlexTf(), MultiTBuffer(buffer_size=math.inf)
    for message in walk(count, seed):
        flex.receive_transform(*message)
        plain.receive_transform(*message)
    return flex, plain, [100.0 + i * 0.05 for i in range(count)]


def test_the_array_doubles_and_only_the_written_rows_count() -> None:
    edge = Edge()
    assert len(edge.rows) == 16 and edge.count == 0
    for step in range(10):
        edge.extend([[float(step * 7 + i)] + [0.0] * (WIDTH - 1) for i in range(7)])
    assert edge.count == 70
    assert len(edge.rows) >= 70, "the buffer has to grow"
    assert len(edge.rows) == 128, "and grow by doubling, not by exactly enough"
    # The mask separates transforms from reserved space.
    assert edge.mask.sum() == 70
    assert edge.mask[:70].all() and not edge.mask[70:].any()
    assert len(edge.used) == 70


def test_a_batch_matches_one_at_a_time_on_the_samples() -> None:
    """Same answers as MultiTBuffer where the two agree: at the recorded stamps.

    Between them they differ on purpose -- see the interpolation test below.
    """
    flex, plain, stamps = filled(200)
    asked = np.asarray(stamps)

    poses, valid = flex.batch_get("odom", "camera", asked)
    assert valid.all()

    for index in np.random.default_rng(1).choice(len(asked), 40, replace=False):
        one = plain.get("odom", "camera", float(asked[index]), warn=False)
        assert one is not None
        expected = [one.translation.x, one.translation.y, one.translation.z]
        assert np.allclose(poses[index][:3, 3], expected, atol=1e-6), index


def test_between_samples_it_interpolates_where_the_old_buffer_snapped() -> None:
    """A deliberate difference, and the reason to say so out loud.

    `MultiTBuffer` answers with the NEAREST recorded transform -- `TBuffer.get` calls
    `find_closest`, which is a zero-order hold. At 20 Hz that is up to 25 ms of stale
    pose for a camera whose shutter did not land on a tf tick. Jeff chose linear
    interpolation over zero-order hold in the June design; this is that choice.
    """
    flex = FlexTf()
    for ts, shift in ((100.0, 0.0), (100.2, 10.0)):
        flex.receive_transform(transform("odom", "base", ts, 0.0, shift))

    poses, valid = flex.batch_get("odom", "base", [100.05, 100.1, 100.15])
    assert valid.all()
    quarter, half, threequarters = poses[:, 0, 3]
    assert np.isclose(half, 5.0), "the midpoint is the midpoint, not the nearer sample"
    assert quarter < half < threequarters
    assert np.isclose(quarter, 2.5) and np.isclose(threequarters, 7.5)

    plain = MultiTBuffer(buffer_size=math.inf)
    for ts, shift in ((100.0, 0.0), (100.2, 10.0)):
        plain.receive_transform(transform("odom", "base", ts, 0.0, shift))
    snapped = plain.get("odom", "base", 100.05, warn=False)
    assert snapped is not None and np.isclose(snapped.translation.x, 0.0), (
        "the buffer this replaces snaps to the nearest sample"
    )


def test_a_stamp_outside_the_range_fails_alone() -> None:
    """A batch is not all-or-nothing: one bad stamp must not lose its neighbours."""
    flex = FlexTf(buffer_size=1.0)
    for message in walk(50):
        flex.receive_transform(*message)
    asked = np.array([100.5, 1e9, 101.0])
    poses, valid = flex.batch_get("odom", "camera", asked)
    assert list(valid) == [True, False, True]
    assert np.isfinite(poses).all()


def test_sources_can_differ_per_entry() -> None:
    flex, _, stamps = filled(60)
    asked = np.linspace(stamps[0], stamps[-1], 6)
    sources = ["camera", "arm", "camera", "base", "arm", "camera"]
    poses, valid = flex.batch_get("odom", sources, asked)
    assert valid.all()
    alone, _ = flex.batch_get("odom", "arm", asked[[1]])
    assert np.allclose(poses[1], alone[0])


def test_reform_edge_changes_answers_already_being_given() -> None:
    """A loop closure amends the past; every later answer must reflect it."""
    flex, _, stamps = filled(80)
    at = stamps[40]
    before, valid = flex.batch_get("odom", "camera", [at])
    assert valid[0]

    moved = transform("base", "arm", at, 0.0, 99.0)
    assert flex.reform_edge("base", "arm", moved, tolerance=1e-3) == 1

    after, valid = flex.batch_get("odom", "camera", [at])
    assert valid[0]
    assert not np.allclose(before[0], after[0]), "the correction did not reach the answer"

    # It amended, it did not append.
    assert flex.edges[("base", "arm")].count == 80


def test_reform_edge_refuses_a_stamp_it_does_not_have() -> None:
    flex, _, _ = filled(20)
    stray = transform("base", "arm", 5000.0, 0.0, 1.0)
    assert flex.reform_edge("base", "arm", stray, tolerance=1e-3) == 0
    assert flex.reform_edge("base", "nowhere", stray) == 0


def test_out_of_order_arrivals_are_still_answered_in_time_order() -> None:
    flex = FlexTf()
    for ts in (100.0, 100.2, 100.1):
        flex.receive_transform(transform("odom", "base", ts, 0.0, ts - 100.0))
    rows = flex.edges[("odom", "base")].used
    assert list(rows[:, 0]) == [100.0, 100.1, 100.2]


def test_the_interface_matches_the_buffer_it_replaces() -> None:
    flex, plain, stamps = filled(30)
    assert flex.get_frames() == plain.get_frames()
    assert flex.get_connections("base") == plain.get_connections("base")
    one = flex.get("odom", "camera", stamps[10])
    assert one is not None and one.frame_id == "odom" and one.child_frame_id == "camera"


@pytest.mark.parametrize("count", [1, 2])
def test_a_chain_shorter_than_the_walk(count: int) -> None:
    flex, _, stamps = filled(40)
    poses, valid = flex.batch_get("odom", CHAIN[count - 1][1], stamps[:5])
    assert valid.all()
