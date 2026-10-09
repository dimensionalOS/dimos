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

"""Render-only marker removal does not alter the official toggle state/geometry."""

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.simulation.behavior.policy_visuals import hide_toggle_markers


class ToggleState:
    pass


def test_hide_only_visibility_preserves_goal_state_and_overlap_extent():
    marker = SimpleNamespace(visible=True, extent=np.array([0.01, 0.02, 0.03]), scale=[1, 1, 1])
    state = SimpleNamespace(visual_marker=marker, value=False)
    obj = SimpleNamespace(states={ToggleState: state})
    count = hide_toggle_markers([obj], ToggleState)
    assert count == 1 and marker.visible is False
    assert state.value is False
    assert marker.extent.tolist() == [0.01, 0.02, 0.03] and marker.scale == [1, 1, 1]


def test_missing_marker_fails_closed_instead_of_allowing_truth_hint():
    obj = SimpleNamespace(states={ToggleState: SimpleNamespace(visual_marker=None)})
    with pytest.raises(RuntimeError, match="marker unavailable"):
        hide_toggle_markers([obj], ToggleState)


def test_visibility_backend_that_changes_overlap_extent_is_rejected():
    class BrokenMarker:
        extent = np.array([1, 1, 1])

        @property
        def visible(self):
            return True

        @visible.setter
        def visible(self, value):
            self.extent = np.zeros(3)

    obj = SimpleNamespace(states={ToggleState: SimpleNamespace(visual_marker=BrokenMarker())})
    with pytest.raises(RuntimeError, match="overlap geometry"):
        hide_toggle_markers([obj], ToggleState)
