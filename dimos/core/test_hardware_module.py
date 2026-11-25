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

import asyncio
from dataclasses import replace
import math
import threading
from typing import Any

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import MetaBlueprint, autoconnect, config_defaults
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
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


class _ClockedCamera(WristCamera):
    """A camera on a clock that moves only when slept on, and stops on its third sleep."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.now = 100.0
        self.slept: list[float] = []
        self.stopped = threading.Event()

    def time(self) -> float:
        return self.now

    async def sleep(self, seconds: float) -> None:
        self.slept.append(seconds)
        self.now += seconds
        if len(self.slept) == 3:
            self.stopped.set()
            await asyncio.Future()


def _listening(module: HardwareModule) -> tuple[list[TFMessage], list[list[Transform]]]:
    """What ``module`` publishes on tf_static, and on tf (through its own tfbuffer)."""
    module._tf = module._create_tfbuffer(None)
    on_static: list[TFMessage] = []
    on_tf: list[list[Transform]] = []
    module.tf_static.subscribe(on_static.append)
    publish = module._tf.publish

    def recorded(*transforms: Transform) -> None:
        on_tf.append(list(transforms))
        publish(*transforms)

    module._tf.publish = recorded  # type: ignore[method-assign]
    return on_static, on_tf


_CAMERA_STATIC_EDGES = {
    ("left/wrist/camera_link", "left/wrist/optical"),
    ("left/wrist", "left/wrist/camera_link"),
}


def test_static_edges_go_out_once_on_tf_static_then_on_tf_at_the_rate() -> None:
    camera = _ClockedCamera(
        mount="wrist", instance_name="left/wrist/wristcamera", static_tf_rate_hz=4.0
    )
    try:
        on_static, on_tf = _listening(camera)
        camera.publish_static_tf()
        assert camera.stopped.wait(5.0)
    finally:
        camera.stop()

    (static,) = on_static
    assert {(t.frame_id, t.child_frame_id) for t in static.transforms} == _CAMERA_STATIC_EDGES
    assert {t.ts for t in static.transforms} == {100.0}
    assert camera.slept == [0.25, 0.25, 0.25]
    assert [{t.ts for t in published} for published in on_tf] == [{100.0}, {100.25}, {100.5}]
    assert all(
        {(t.frame_id, t.child_frame_id) for t in published} == _CAMERA_STATIC_EDGES
        for published in on_tf
    )


def test_a_static_tf_rate_of_zero_publishes_static_edges_only_on_tf_static() -> None:
    camera = _ClockedCamera(
        mount="wrist", instance_name="left/wrist/wristcamera", static_tf_rate_hz=0.0
    )
    try:
        on_static, on_tf = _listening(camera)
        camera.publish_static_tf()
    finally:
        camera.stop()

    assert len(on_static) == 1
    assert on_tf == []
    assert camera.slept == []


def test_publish_usd_tf_publishes_only_the_edges_that_move() -> None:
    arm = Arm(instance_name="left/arm")
    camera = WristCamera(mount="wrist", instance_name="left/wrist/wristcamera")
    try:
        _, arm_tf = _listening(arm)
        _, camera_tf = _listening(camera)
        arm.publish_usd_tf({"elbow": 1.0}, ts=2.0)
        camera.publish_usd_tf(ts=2.0)
    finally:
        arm.stop()
        camera.stop()

    ((elbow,),) = arm_tf
    assert (elbow.frame_id, elbow.child_frame_id, elbow.ts) == ("left/base", "left/forearm", 2.0)
    assert camera_tf == []


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


class WorldReader(Module):
    world_observations: In[Image]
    world_actions: Out[Image]
    color_image: Out[Image]
    tf_static: Out[TFMessage]


def test_the_simulated_world_stays_global_under_any_namespace() -> None:
    blueprint = WorldReader.blueprint().namespace("lidar").namespace("robot1")

    for stream in ("world_observations", "world_actions"):
        assert ("robot1/lidar/worldreader", stream) not in blueprint.remapping_map
    assert (
        blueprint.remapping_map[("robot1/lidar/worldreader", "color_image")]
        == "robot1/lidar/color_image"
    )
    assert ("robot1/lidar/worldreader", "tf_static") not in blueprint.remapping_map

    private = WorldReader.blueprint().namespace("lidar", share_tf=False)
    assert private.remapping_map[("lidar/worldreader", "tf_static")] == "lidar/tf_static"


def test_a_module_names_its_namespace_over_rpc() -> None:
    reader = WorldReader(instance_name="robot1/lidar/worldreader")
    try:
        assert reader.get_namespace() == "robot1/lidar"
    finally:
        reader.stop()
