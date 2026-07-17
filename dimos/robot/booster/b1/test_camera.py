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

from dimos.core.native_module import LogFormat
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.booster.b1.blueprints.basic.booster_b1_camera import booster_b1_camera
from dimos.robot.booster.b1.camera import BoosterCamera, BoosterCameraConfig
from dimos.robot.get_all_blueprints import get_blueprint_by_name


def test_camera_configures_booster_sdk_native_bridge(monkeypatch) -> None:
    monkeypatch.setenv("ROBOT_INTERFACE", "eth-test")

    config = BoosterCameraConfig()

    assert config.network_interface == "eth-test"
    assert config.cwd == "cpp"
    assert config.executable == "result/bin/booster_camera_native"
    assert config.build_command == "nix build .#booster-camera-native"
    assert config.log_format == LogFormat.TEXT
    assert config.depth_scale == 0.001
    assert config.publish_rate_hz is None
    assert config.image_reliable is False
    assert config.depth_enabled is False
    assert config.depth_compressed is True
    assert (
        config.depth_topic
        == "rt/boostercamera/camera/aligned_depth_to_color/image_raw/compressedDepth"
    )


def test_camera_defaults_to_sdk_transports_without_robot_interface(monkeypatch) -> None:
    monkeypatch.delenv("ROBOT_INTERFACE", raising=False)

    config = BoosterCameraConfig()

    assert config.network_interface is None
    assert "--network_interface" not in config.to_cli_args()


def test_camera_forwards_publish_rate_limit_to_native_bridge() -> None:
    config = BoosterCameraConfig(publish_rate_hz=10)
    args = config.to_cli_args()

    assert args[args.index("--publish_rate_hz") + 1] == "10.0"


def test_camera_forwards_depth_enabled_to_native_bridge() -> None:
    config = BoosterCameraConfig(depth_enabled=True)
    args = config.to_cli_args()

    assert args[args.index("--depth_enabled") + 1] == "true"


def test_camera_can_disable_compressed_depth_for_native_bridge() -> None:
    config = BoosterCameraConfig(depth_compressed=False)
    args = config.to_cli_args()

    assert args[args.index("--depth_compressed") + 1] == "false"


def test_camera_blueprint_exposes_rgb_depth_and_calibration_streams() -> None:
    [atom] = booster_b1_camera.blueprints

    assert atom.module is BoosterCamera
    assert atom.kwargs == {"depth_enabled": False}
    assert {(stream.name, stream.type, stream.direction) for stream in atom.streams} == {
        ("color_image", Image, "out"),
        ("depth_image", Image, "out"),
        ("camera_info", CameraInfo, "out"),
        ("depth_camera_info", CameraInfo, "out"),
    }


def test_camera_blueprint_is_discoverable() -> None:
    registry_path = all_blueprints["booster-b1-camera"]

    assert registry_path.endswith(":booster_b1_camera")
    assert get_blueprint_by_name("booster-b1-camera") is booster_b1_camera
