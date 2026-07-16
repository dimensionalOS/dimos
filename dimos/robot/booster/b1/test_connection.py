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

from pathlib import Path

from dimos.core.native_module import LogFormat
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.booster.b1.blueprints.basic.booster_b1_keyboard_teleop import (
    booster_b1_keyboard_teleop,
)
from dimos.robot.booster.b1.blueprints.basic.booster_b1_keyboard_teleop_camera import (
    booster_b1_keyboard_teleop_camera,
)
from dimos.robot.booster.b1.camera import BoosterCamera
from dimos.robot.booster.b1.connection import (
    BoosterB1Connection,
    BoosterB1ConnectionConfig,
)
from dimos.robot.get_all_blueprints import get_blueprint_by_name
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop


def test_connection_configures_native_cmake_build(monkeypatch) -> None:
    monkeypatch.setenv("ROBOT_INTERFACE", "eth-test")

    config = BoosterB1ConnectionConfig()

    assert config.network_interface == "eth-test"
    assert config.command_timeout_sec == 0.25
    assert config.cwd == "cpp"
    assert config.executable == "result/bin/booster_b1_native"
    assert config.log_format == LogFormat.TEXT
    assert config.build_command == "nix build .#booster-b1-native"


def test_cmake_links_self_contained_arch_specific_booster_sdk_archive() -> None:
    cmake_path = Path(__file__).parent / "cpp" / "CMakeLists.txt"

    cmake = cmake_path.read_text()

    assert '"${BOOSTER_SDK_ROOT}/lib/${BOOSTER_SDK_ARCH}/libbooster_robotics_sdk.a"' in cmake
    assert '"${BOOSTER_SDK_ROOT}/include"' in cmake
    assert "libfastrtps.so" not in cmake
    assert "libfastcdr.so" not in cmake
    assert "libfoonathan_memory" not in cmake
    assert "must be provided by the Nix build" in cmake
    assert '"${DIMOS_LCM_SOURCE_DIR}/generated/cpp_lcm_msgs"' in cmake
    assert "find_package(Python3" not in cmake
    assert "lcm_wrap_types(" not in cmake
    assert "ENV{BOOSTER_SDK_ROOT}" not in cmake


def test_nix_flake_pins_booster_sdk_for_native_build() -> None:
    flake_path = Path(__file__).parent / "cpp" / "flake.nix"

    flake = flake_path.read_text()

    assert (
        'url = "github:BoosterRobotics/booster_robotics_sdk/'
        'd5d8f7ae76d3e9f8cc224e682216f4681003ca46"' in flake
    )
    assert "flake = false" in flake
    assert '"-DBOOSTER_SDK_ROOT=${booster-sdk-package}"' in flake
    assert '"-DDIMOS_LCM_SOURCE_DIR=${dimos-lcm}"' in flake
    assert '"-DDIMOS_NATIVE_COMMON_DIR=${dimos-native-common}"' in flake
    assert "booster-sdk}/third_party" not in flake
    assert "tinyxml2" not in flake


def test_keyboard_teleop_blueprint_wires_twist_to_native_connection() -> None:
    atoms = booster_b1_keyboard_teleop.blueprints

    assert [atom.module for atom in atoms] == [KeyboardTeleop, BoosterB1Connection]
    assert atoms[0].streams[0].name == "cmd_vel"
    assert atoms[0].streams[0].type is Twist
    assert atoms[0].streams[0].direction == "out"
    assert atoms[1].streams[0].name == "cmd_vel"
    assert atoms[1].streams[0].type is Twist
    assert atoms[1].streams[0].direction == "in"
    assert atoms[1].kwargs == {}


def test_keyboard_teleop_blueprint_is_discoverable() -> None:
    registry_path = all_blueprints["booster-b1-keyboard-teleop"]

    assert registry_path.endswith(":booster_b1_keyboard_teleop")
    assert get_blueprint_by_name("booster-b1-keyboard-teleop") is booster_b1_keyboard_teleop


def test_keyboard_teleop_camera_blueprint_composes_separate_modules() -> None:
    atoms = booster_b1_keyboard_teleop_camera.blueprints

    assert [atom.module for atom in atoms] == [
        KeyboardTeleop,
        BoosterB1Connection,
        BoosterCamera,
    ]


def test_keyboard_teleop_camera_blueprint_is_discoverable() -> None:
    registry_path = all_blueprints["booster-b1-keyboard-teleop-camera"]

    assert registry_path.endswith(":booster_b1_keyboard_teleop_camera")
    assert (
        get_blueprint_by_name("booster-b1-keyboard-teleop-camera")
        is booster_b1_keyboard_teleop_camera
    )
