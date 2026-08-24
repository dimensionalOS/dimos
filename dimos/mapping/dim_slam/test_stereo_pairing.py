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

from dataclasses import dataclass

from reactivex.subject import Subject

from dimos.mapping.dim_slam.stereo_pairing import stamp_matched_pairs


@dataclass
class Frame:
    ts: float


def pair_stamps(*arrivals: tuple[str, float]) -> list[tuple[float, float]]:
    left, right = Subject(), Subject()
    pairs: list[tuple[float, float]] = []
    stamp_matched_pairs(left, right).subscribe(
        on_next=lambda pair: pairs.append((pair[0].ts, pair[1].ts))
    )
    for side, ts in arrivals:
        (left if side == "left" else right).on_next(Frame(ts))
    return pairs


def test_frames_within_tolerance_pair_left_first_whichever_arrives_first():
    assert pair_stamps(("right", 1.0), ("left", 1.0005)) == [(1.0005, 1.0)]
    assert pair_stamps(("left", 1.0), ("right", 1.0005)) == [(1.0, 1.0005)]


def test_a_dropped_frame_does_not_desynchronize_the_rest():
    arrivals = [("left", 0.0), ("left", 1.0), ("right", 1.0), ("left", 2.0), ("right", 2.0)]
    assert pair_stamps(*arrivals) == [(1.0, 1.0), (2.0, 2.0)]


def test_a_frame_beyond_the_tolerance_never_pairs():
    assert pair_stamps(("left", 0.0), ("right", 0.5)) == []
