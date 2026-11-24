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

from dataclasses import replace
import math

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect, config_defaults
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, HardwareSpec, Mounts
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.assets.model import RobotModel, UsdModel

_ARM_URDF = """<robot name="arm">
  <link name="base"/>
  <link name="forearm"/>
  <joint name="elbow" type="revolute">
    <parent link="base"/><child link="forearm"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"""

_CAMERA_URDF = """<robot name="camera">
  <link name="camera_link"/>
  <link name="optical"/>
  <joint name="optical_joint" type="fixed">
    <parent link="camera_link"/><child link="optical"/><origin xyz="0.02 0 0"/>
  </joint>
</robot>"""


def _arm_usd() -> UsdModel:
    model = UsdModel.from_robot_model(RobotModel.from_urdf_string(_ARM_URDF))
    return model.with_frames({"wrist": ("forearm", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))})


class ArmConfig(HardwareModuleConfig):
    usd: UsdModel | None = _arm_usd()


class Arm(HardwareModule):
    config: ArmConfig
    mounts = Mounts("wrist")


class WristCameraConfig(HardwareModuleConfig):
    usd: UsdModel | None = UsdModel.from_robot_model(RobotModel.from_urdf_string(_CAMERA_URDF))


class WristCamera(HardwareModule):
    config: WristCameraConfig


def _verify(blueprint: MetaBlueprint) -> None:
    HardwareModule.blueprint_checks(
        [
            replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
            for atom in blueprint.solidify().active_blueprints
        ]
    )


def test_mounts_are_named_on_the_class() -> None:
    assert Arm.mounts.wrist == "wrist"
    with pytest.raises(AttributeError, match="the mounts are \\['wrist'\\]"):
        _ = Arm.mounts.elbow


def test_mount_namespaces_under_the_mount_name() -> None:
    (atom,) = WristCamera.mount(Arm.mounts.wrist).active_blueprints
    assert atom.name == "wrist/wristcamera"
    assert atom.kwargs["_mount"] == "wrist"


def test_one_unmounted_with_mounted_hardware_passes() -> None:
    _verify(autoconnect(Arm.blueprint(), WristCamera.mount(Arm.mounts.wrist)))


def test_two_unmounted_in_one_namespace_is_refused() -> None:
    with pytest.raises(ValueError, match=r"the top level: arm, wristcamera"):
        _verify(autoconnect(Arm.blueprint(), WristCamera.blueprint()))


def test_each_namespace_gets_its_own_unmounted_hardware() -> None:
    robot = autoconnect(Arm.blueprint(), WristCamera.mount(Arm.mounts.wrist))
    _verify(autoconnect(robot.namespace("left"), robot.namespace("right")))
    with pytest.raises(ValueError, match=r"left: left/arm, left/wristcamera"):
        _verify(autoconnect(Arm.blueprint(), WristCamera.blueprint()).namespace("left"))


def _build(blueprint: MetaBlueprint) -> ModuleCoordinator:
    solid = blueprint.solidify()
    parsed = BlueprintConfigParser(solid).parse(environ={}, overrides={"g": {"viewer": "none"}})
    return ModuleCoordinator.build(solid, parsed)


def test_build_refuses_two_unmounted() -> None:
    with pytest.raises(ValueError, match="more than one unmounted HardwareModule"):
        _build(autoconnect(Arm.blueprint(), WristCamera.blueprint()))


def test_each_hardware_serves_its_own_usd_named_as_on_tf() -> None:
    from dimos.robot.assets.usd_utils.frames import usd_frame_names

    coordinator = _build(
        autoconnect(Arm.blueprint(), WristCamera.mount(Arm.mounts.wrist)).namespace("left")
    )
    try:
        arm = coordinator.get_instance("left/arm")
        camera = coordinator.get_instance("left/wrist/wristcamera")
        assert arm.mounted_on() is None
        assert camera.mounted_on() == "left/wrist"
        arm_frames = usd_frame_names(arm.get_usd())
        assert {"left/base", "left/forearm", "left/wrist"} <= arm_frames
        assert not any("camera" in frame for frame in arm_frames)
        assert {"left/wrist/camera_link", "left/wrist/optical"} <= usd_frame_names(camera.get_usd())
    finally:
        coordinator.stop()


def test_hardware_publishes_its_own_tf_and_its_mount() -> None:
    camera = WristCamera(_mount="wrist", instance_name="left/wrist/wristcamera")
    arm = Arm(instance_name="left/arm")
    try:
        edges = {
            (t.frame_id, t.child_frame_id): t for t in arm.usd_transforms({"elbow": math.pi / 2})
        }
        assert set(edges) == {("left/base", "left/forearm"), ("left/forearm", "left/wrist")}
        elbow = edges["left/base", "left/forearm"].to_matrix()
        np.testing.assert_allclose(elbow[:3, 3], [0.0, 0.0, 0.5], atol=1e-9)
        np.testing.assert_allclose(elbow[:3, 0], [0.0, 1.0, 0.0], atol=1e-9)

        edges = {(t.frame_id, t.child_frame_id) for t in camera.usd_transforms(ts=1.0)}
        assert edges == {
            ("left/wrist/camera_link", "left/wrist/optical"),
            ("left/wrist", "left/wrist/camera_link"),
        }
    finally:
        arm.stop()
        camera.stop()


class Assembler(Module):
    hardware: list[HardwareSpec]

    @rpc
    def mounted(self) -> list[str | None]:
        return sorted(str(part.mounted_on()) for part in self.hardware)


def test_a_list_ref_gets_every_hardware_module() -> None:
    robot = autoconnect(Arm.blueprint(), WristCamera.mount(Arm.mounts.wrist))
    coordinator = _build(
        autoconnect(robot.namespace("left"), robot.namespace("right"), Assembler.blueprint())
    )
    try:
        assert coordinator.get_instance("assembler").mounted() == [
            "None",
            "None",
            "left/wrist",
            "right/wrist",
        ]
    finally:
        coordinator.stop()


class SharedCamera(WristCamera):
    mount_shared = frozenset({"color_image"})
    color_image: Out[Image]


def test_a_mount_keeps_its_shared_streams_outside_its_namespace() -> None:
    blueprint = (
        autoconnect(Arm.blueprint(), SharedCamera.mount(Arm.mounts.wrist))
        .namespace("left", shared={"color_image"})
        .solidify()
    )
    (camera,) = [atom for atom in blueprint.active_blueprints if atom.module is SharedCamera]
    assert blueprint.remapping_map.get((camera.name, "color_image"), "color_image") == "color_image"
