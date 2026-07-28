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

"""Tests for manipulation planning value models."""

from dataclasses import FrozenInstanceError

import pytest

from dimos.manipulation.planning.spec.models import CartesianDelta


def test_cartesian_delta_defaults_to_identity_in_world_frame() -> None:
    delta = CartesianDelta()

    assert delta.translation == (0.0, 0.0, 0.0)
    assert delta.rotation_rpy == (0.0, 0.0, 0.0)
    assert delta.frame_id == "world"


def test_cartesian_delta_is_immutable() -> None:
    delta = CartesianDelta(translation=(0.1, 0.0, 0.0))

    with pytest.raises(FrozenInstanceError):
        delta.frame_id = "tool"  # type: ignore[misc]
