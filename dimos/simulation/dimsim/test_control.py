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

from dimos.simulation.dimsim import control as control_module
from dimos.simulation.dimsim.control import DimSimSceneControl
from dimos.simulation.scene_controls import NavigationSceneControl, PlanarBounds


def test_dimsim_control_maps_canonical_world_coordinates(mocker) -> None:
    goal_transport = mocker.Mock()
    browser_client = mocker.Mock()
    browser_client.get_semantic_object_bounds.return_value = {
        "min": {"x": -2.0, "y": 0.0, "z": 3.0},
        "max": {"x": 1.0, "y": 2.0, "z": 5.0},
    }
    mocker.patch.object(control_module, "make_transport", return_value=goal_transport)
    mocker.patch.object(control_module, "SceneClient", return_value=browser_client)
    control = DimSimSceneControl()

    control.set_agent_position(3.0, -2.0)
    control.add_wall(2.0, -2.5, 12.0, 3.5)
    bounds = control.semantic_object_bounds("bed")

    assert isinstance(control, NavigationSceneControl)
    browser_client.start.assert_called_once_with()
    browser_client.set_agent_position.assert_called_once_with(-2.0, 0.52, 3.0)
    browser_client.add_wall.assert_called_once_with(-2.5, 2.0, 3.5, 12.0)
    browser_client.get_semantic_object_bounds.assert_called_once_with("bed")
    assert bounds == PlanarBounds(min_x=3.0, min_y=-2.0, max_x=5.0, max_y=1.0)


def test_dimsim_control_does_not_open_browser_during_unused_cleanup(mocker) -> None:
    goal_transport = mocker.Mock()
    scene_client_type = mocker.patch.object(control_module, "SceneClient")
    mocker.patch.object(control_module, "make_transport", return_value=goal_transport)
    control = DimSimSceneControl()

    control.start()
    control.stop()

    goal_transport.start.assert_called_once_with()
    goal_transport.stop.assert_called_once_with()
    scene_client_type.assert_not_called()
