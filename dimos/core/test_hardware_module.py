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
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
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
    usd: UsdModel = _arm_usd()


class Arm(HardwareModule):
    config: ArmConfig
    mounts = Mounts("wrist")


class WristCameraConfig(HardwareModuleConfig):
    usd: UsdModel = UsdModel.from_robot_model(RobotModel.from_urdf_string(_CAMERA_URDF))


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
        Arm.mounts.elbow  # noqa: B018


def test_mount_namespaces_under_the_mount_name() -> None:
    (atom,) = WristCamera.mount(Arm.mounts.wrist).active_blueprints
    assert atom.name == "wrist/wristcamera"
    assert atom.kwargs["mount"] == "wrist"


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


def test_a_mount_no_hardware_offers_is_refused() -> None:
    with pytest.raises(
        ValueError, match=r"wrist/wristcamera is mounted on 'wrist', which no other"
    ):
        _verify(autoconnect(WristCamera.mount(Arm.mounts.wrist)))


def test_a_sensor_namespaced_without_its_parent_is_refused() -> None:
    # The camera is under left/, the arm it mounts on is not: left/wrist is offered by nobody.
    with pytest.raises(
        ValueError, match=r"(?s)offered: 'wrist'.*left/wrist/wristcamera is mounted on 'left/wrist'"
    ):
        _verify(autoconnect(Arm.blueprint(), WristCamera.mount(Arm.mounts.wrist).namespace("left")))


class Gripper(HardwareModule):
    config: WristCameraConfig
    mounts = Mounts("palm")


class Palm(HardwareModule):
    config: WristCameraConfig
    mounts = Mounts("finger")


def test_hardware_mounted_on_each_other_in_a_loop_is_refused() -> None:
    # Each names the other's mount, but a mount namespaces under itself, so neither frame exists.
    with pytest.raises(
        ValueError,
        match=r"(?s)finger/gripper is mounted on 'finger'.*palm/palm is mounted on 'palm'",
    ):
        _verify(autoconnect(Gripper.mount(Palm.mounts.finger), Palm.mount(Gripper.mounts.palm)))


def test_two_hardware_on_one_mount_is_refused() -> None:
    with pytest.raises(ValueError, match=r"wrist holds wrist/gripper, wrist/wristcamera"):
        _verify(
            autoconnect(
                Arm.blueprint(),
                WristCamera.mount(Arm.mounts.wrist),
                Gripper.mount(Arm.mounts.wrist),
            )
        )


def test_a_chain_of_mounts_under_a_namespace_passes() -> None:
    robot = autoconnect(
        Arm.blueprint(),
        Gripper.mount(Arm.mounts.wrist),
        # The camera mounts on the gripper, so it goes under the gripper's namespace.
        WristCamera.mount(Gripper.mounts.palm).namespace(Arm.mounts.wrist),
    )
    _verify(robot.namespace("left"))


def _build(blueprint: MetaBlueprint) -> ModuleCoordinator:
    solid = blueprint.solidify()
    parsed = BlueprintConfigParser(solid).parse(environ={}, overrides={"g": {"viewer": "none"}})
    return ModuleCoordinator.build(solid, parsed)


def test_build_refuses_two_unmounted() -> None:
    with pytest.raises(ValueError, match="more than one unmounted HardwareModule"):
        _build(autoconnect(Arm.blueprint(), WristCamera.blueprint()))


def test_each_hardware_serves_its_own_usd_named_as_on_tf() -> None:
    pytest.importorskip("pxr")
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
    camera = WristCamera(mount="wrist", instance_name="left/wrist/wristcamera")
    arm = Arm(instance_name="left/arm")
    try:
        edges = {
            (t.frame_id, t.child_frame_id): t for t in arm.usd_transforms({"elbow": math.pi / 2})
        }
        assert set(edges) == {("left/base", "left/forearm"), ("left/forearm", "left/wrist")}
        elbow = edges["left/base", "left/forearm"].to_matrix()
        np.testing.assert_allclose(elbow[:3, 3], [0.0, 0.0, 0.5], atol=1e-9)
        np.testing.assert_allclose(elbow[:3, 0], [0.0, 1.0, 0.0], atol=1e-9)

        camera_edges = {(t.frame_id, t.child_frame_id) for t in camera.usd_transforms(ts=1.0)}
        assert camera_edges == {
            ("left/wrist/camera_link", "left/wrist/optical"),
            ("left/wrist", "left/wrist/camera_link"),
        }
    finally:
        arm.stop()
        camera.stop()


_LINKAGE_URDF = """<robot name="linkage">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="a"/>
    <origin xyz="0.1 -0.2 0.3" rpy="0.4 -0.5 0.6"/><axis xyz="1 2 -3"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="a"/><child link="b"/>
    <origin xyz="0 0.2 0" rpy="-0.3 0.2 1.1"/><axis xyz="0 1 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="follower" type="revolute">
    <parent link="b"/><child link="c"/>
    <origin xyz="0 0 0.1" rpy="3.0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
    <mimic joint="hinge" multiplier="-2" offset="0.25"/>
  </joint>
  <joint name="weld" type="fixed">
    <parent link="c"/><child link="d"/><origin xyz="0.3 0 0" rpy="0 1.2 0"/>
  </joint>
</robot>"""


class LinkageConfig(HardwareModuleConfig):
    usd: UsdModel = UsdModel.from_robot_model(RobotModel.from_urdf_string(_LINKAGE_URDF))


class Linkage(HardwareModule):
    config: LinkageConfig


def test_every_edge_is_where_its_joint_puts_it_at_any_position() -> None:
    from dimos.core.hardware_module import _position

    linkage = Linkage(instance_name="linkage")
    try:
        for positions in ({}, {"hinge": 2.9, "slide": -0.7}, {"hinge": -1.3, "slide": 0.4}):
            edges = {t.child_frame_id: t.to_matrix() for t in linkage.usd_transforms(positions)}
            for joint in linkage._posable_joints:
                expected = joint.parent_from_child(_position(joint, positions))
                np.testing.assert_allclose(edges[joint.child_link], expected, atol=1e-9)
    finally:
        linkage.stop()


def test_a_frame_id_prefix_names_the_usd_and_its_tf_alike() -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.usd_utils.frames import usd_frame_names

    arm = Arm(instance_name="left/arm", frame_id_prefix="spare")
    try:
        edges = {(t.frame_id, t.child_frame_id) for t in arm.usd_transforms()}
        frames = usd_frame_names(arm.get_usd())
    finally:
        arm.stop()
    assert edges == {
        ("left/spare/base", "left/spare/forearm"),
        ("left/spare/forearm", "left/spare/wrist"),
    }
    assert {frame for edge in edges for frame in edge} <= frames


def test_hardware_without_a_usd_is_refused() -> None:
    class Bare(HardwareModule):
        config: HardwareModuleConfig

    with pytest.raises(ValueError, match="usd"):
        Bare(instance_name="bare")
