# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

import pytest

from dimos.simulation import scene_controls
from dimos.simulation.scene_controls import NavigationSceneControl, PlanarBounds


class _Control:
    provider_name = "fake"

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_agent_position(
        self,
        x: float,
        y: float,
        z: float | None = None,
    ) -> None:
        pass

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        pass

    def publish_goal(self, x: float, y: float) -> None:
        pass

    def semantic_object_bounds(self, query: str) -> PlanarBounds:
        return PlanarBounds(0.0, 0.0, 1.0, 1.0)


class _EntryPoint:
    name = "fake"

    def load(self) -> Any:
        return _Control


def test_planar_bounds_distance_uses_nearest_point() -> None:
    bounds = PlanarBounds(min_x=1.0, min_y=2.0, max_x=3.0, max_y=4.0)

    assert bounds.distance_to(2.0, 3.0) == 0.0
    assert bounds.distance_to(4.0, 5.0) == pytest.approx(2**0.5)


def test_load_external_navigation_scene_control(monkeypatch: pytest.MonkeyPatch) -> None:
    def entry_points(*, group: str, name: str | None = None) -> list[_EntryPoint]:
        assert group == scene_controls.ENTRY_POINT_GROUP
        return [_EntryPoint()] if name in (None, "fake") else []

    monkeypatch.setattr(scene_controls.importlib_metadata, "entry_points", entry_points)

    control = scene_controls.load_scene_control("fake")

    assert isinstance(control, NavigationSceneControl)
    assert control.provider_name == "fake"


def test_load_scene_control_rejects_provider_identity_change(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class WrongControl(_Control):
        provider_name = "other"

    class WrongEntryPoint(_EntryPoint):
        def load(self) -> Any:
            return WrongControl

    monkeypatch.setattr(
        scene_controls.importlib_metadata,
        "entry_points",
        lambda **kwargs: [WrongEntryPoint()],
    )

    with pytest.raises(ValueError, match="returned provider 'other'"):
        scene_controls.load_scene_control("fake")
