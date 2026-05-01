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

from dimos.hardware.manipulators.registry import adapter_registry
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.catalog.a750 import a750


def test_a750_adapter_is_registered() -> None:
    assert "a750" in adapter_registry.available()

    adapter = adapter_registry.create("a750", address="/dev/ttyACM0", dof=6)

    assert type(adapter).__name__ == "A750Adapter"
    assert adapter.get_info().model == "A-750"


def test_keyboard_teleop_a750_points_at_a750_blueprint() -> None:
    assert (
        all_blueprints["keyboard-teleop-a750"]
        == "dimos.robot.manipulators.a750.blueprints:keyboard_teleop_a750"
    )


def test_a750_catalog_uses_a750_adapter_when_requested() -> None:
    hardware = a750(
        name="arm",
        adapter_type="a750",
        device_path="/dev/ttyACM0",
    ).to_hardware_component()

    assert hardware.adapter_type == "a750"
    assert hardware.address == "/dev/ttyACM0"
