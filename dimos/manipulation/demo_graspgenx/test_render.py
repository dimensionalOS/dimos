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

"""Focused checks for static grasp proposal annotations."""

from types import SimpleNamespace

import numpy as np

from .render import _fork_strips, _fork_strips_local, _score_colors


def _candidate(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> SimpleNamespace:
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
            orientation=SimpleNamespace(
                x=orientation[0],
                y=orientation[1],
                z=orientation[2],
                w=orientation[3],
            ),
        )
    )


def _gripper() -> SimpleNamespace:
    return SimpleNamespace(
        extents_open=(0.2, 0.3, 0.4),
        offset_open=(0.1, 0.0, 0.3),
        extents_half_open=(0.1, 0.15, 0.2),
        offset_half_open=(0.0, 0.2, 0.1),
    )


def test_wireframe_opens_along_local_positive_z() -> None:
    strips = _fork_strips_local(_gripper())
    rear_bridge, left_rail, right_rail = strips[1:]
    rear_width = np.ptp(rear_bridge[:, 0])
    mouth_width = abs(right_rail[-1, 0] - left_rail[-1, 0])

    assert all(np.allclose(strip[:, 1], 0.0) for strip in strips)
    assert mouth_width > rear_width
    assert left_rail[-1, 2] > left_rail[0, 2]
    assert right_rail[-1, 2] > right_rail[0, 2]


def test_wireframe_uses_candidate_full_rigid_pose() -> None:
    first = _fork_strips(_candidate((1.0, 2.0, 3.0)), _gripper())
    second = _fork_strips(
        _candidate((4.0, 5.0, 6.0), (2**-0.5, 0.0, 0.0, 2**-0.5)),
        _gripper(),
    )
    rotation_x_90 = np.asarray([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    for first_strip, second_strip in zip(first, second, strict=True):
        local = first_strip - [1.0, 2.0, 3.0]
        expected = (rotation_x_90 @ local.T).T + np.asarray([4.0, 5.0, 6.0])
        np.testing.assert_allclose(second_strip, expected, rtol=0.0, atol=1e-6)


def test_score_colors_are_relative_and_monotonic() -> None:
    colors = _score_colors(np.asarray([0.1, 0.5, 0.9]))
    assert len({tuple(color) for color in colors}) == 3
    assert np.all(np.diff(colors.mean(axis=1)) > 0)
