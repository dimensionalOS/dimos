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
from typing import Any, TypeVar

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import (
    BlueprintProtocol,
    MetaBlueprint,
    autoconnect,
    config_defaults,
)
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.core.stream import IO
from dimos.core.tf_module import TfModule, TfModuleConfig
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.joint_transforms import joint_position
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
    return model.with_frames({"wrist": ("arm/base/forearm", (0.0, 0.0, 0.1), (0.0, 0.0, 0.0))})


class ArmConfig(HardwareModuleConfig):
    usd: UsdModel = _arm_usd()


class Arm(HardwareModule):
    config: ArmConfig
    mounts = Mounts("arm/base/forearm/wrist")


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
    assert Arm.mounts.wrist == "arm/base/forearm/wrist"
    with pytest.raises(AttributeError, match="the mounts are \\['wrist'\\]"):
        Arm.mounts.elbow  # noqa: B018
    with pytest.raises(ValueError, match="both end in 'wrist'"):
        Mounts("arm/wrist", "leg/wrist")


def test_mount_namespaces_under_the_mount_name() -> None:
    (atom,) = WristCamera.attach(Arm.mounts.wrist).active_blueprints
    assert atom.name == "arm/base/forearm/wrist/wristcamera"
    assert atom.kwargs["attached_to"] == "arm/base/forearm/wrist"
    # blueprint() fills in the USD's frames, plus the mount: the namespace frame itself (role and frame ""), hung over the USD's top prim.
    assert atom.kwargs["tf_frames_local"][""] == ""
    assert "camera/camera_link/optical" in atom.kwargs["tf_frames_local"]
    assert ("", "camera") in atom.kwargs["tf_edges"]


def test_the_mount_frame_is_the_namespace_and_needs_one() -> None:
    from dimos.core.tf_module import _frame_ids

    assert _frame_ids({"": ""}, {}, "left/wrist") == {"": "left/wrist"}
    with pytest.raises(ValueError, match="has no namespace"):
        _frame_ids({"": ""}, {}, "")


def test_one_unmounted_with_mounted_hardware_passes() -> None:
    _verify(autoconnect(Arm.blueprint(), WristCamera.attach(Arm.mounts.wrist)))


def test_two_unmounted_in_one_namespace_is_refused() -> None:
    with pytest.raises(ValueError, match=r"the top level: arm, wristcamera"):
        _verify(autoconnect(Arm.blueprint(), WristCamera.blueprint()))


def test_each_namespace_gets_its_own_unmounted_hardware() -> None:
    robot = autoconnect(Arm.blueprint(), WristCamera.attach(Arm.mounts.wrist))
    _verify(autoconnect(robot.namespace("left"), robot.namespace("right")))
    with pytest.raises(ValueError, match=r"left: left/arm, left/wristcamera"):
        _verify(autoconnect(Arm.blueprint(), WristCamera.blueprint()).namespace("left"))


def test_a_mount_no_hardware_offers_is_refused() -> None:
    with pytest.raises(
        ValueError,
        match=r"arm/base/forearm/wrist/wristcamera is mounted on 'arm/base/forearm/wrist', which no other",
    ):
        _verify(autoconnect(WristCamera.attach(Arm.mounts.wrist)))


def test_a_sensor_namespaced_without_its_parent_is_refused() -> None:
    # The camera is under left/, the arm it mounts on is not: left/arm/... is offered by nobody.
    with pytest.raises(
        ValueError,
        match=r"(?s)offered: 'arm/base/forearm/wrist'.*left/arm/base/forearm/wrist/wristcamera is mounted on 'left/arm/base/forearm/wrist'",
    ):
        _verify(
            autoconnect(Arm.blueprint(), WristCamera.attach(Arm.mounts.wrist).namespace("left"))
        )


def _camera_with(frame: str) -> UsdModel:
    camera = UsdModel.from_robot_model(RobotModel.from_urdf_string(_CAMERA_URDF))
    return camera.with_frames({frame: ("camera/camera_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))})


class GripperConfig(HardwareModuleConfig):
    usd: UsdModel = _camera_with("palm")


class Gripper(HardwareModule):
    config: GripperConfig
    mounts = Mounts("camera/camera_link/palm")


class PalmConfig(HardwareModuleConfig):
    usd: UsdModel = _camera_with("finger")


class Palm(HardwareModule):
    config: PalmConfig
    mounts = Mounts("camera/camera_link/finger")


BP = TypeVar("BP", bound=BlueprintProtocol)


def _under(blueprint: BP, path: str) -> BP:
    """``blueprint`` under the namespace ``path``, one segment at a time (namespace() takes no '/')."""
    for segment in reversed(path.split("/")):
        blueprint = blueprint.namespace(segment)
    return blueprint


def test_hardware_mounted_on_each_other_in_a_loop_is_refused() -> None:
    # Each names the other's mount, but a mount namespaces under itself, so neither frame exists.
    with pytest.raises(
        ValueError,
        match=r"(?s)finger/gripper is mounted on 'camera/camera_link/finger'.*palm/palm is mounted on 'camera/camera_link/palm'",
    ):
        _verify(autoconnect(Gripper.attach(Palm.mounts.finger), Palm.attach(Gripper.mounts.palm)))


def test_two_hardware_on_one_mount_is_refused() -> None:
    with pytest.raises(
        ValueError,
        match=r"arm/base/forearm/wrist holds arm/base/forearm/wrist/gripper, arm/base/forearm/wrist/wristcamera",
    ):
        _verify(
            autoconnect(
                Arm.blueprint(),
                WristCamera.attach(Arm.mounts.wrist),
                Gripper.attach(Arm.mounts.wrist),
            )
        )


def test_a_chain_of_mounts_under_a_namespace_passes() -> None:
    robot = autoconnect(
        Arm.blueprint(),
        Gripper.attach(Arm.mounts.wrist),
        # The camera mounts on the gripper, so it goes under the gripper's namespace.
        _under(WristCamera.attach(Gripper.mounts.palm), Arm.mounts.wrist),
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
        autoconnect(Arm.blueprint(), WristCamera.attach(Arm.mounts.wrist)).namespace("left")
    )
    try:
        arm = coordinator.get_instance("left/arm")
        camera = coordinator.get_instance("left/arm/base/forearm/wrist/wristcamera")
        assert arm.attached_to() is None
        assert camera.attached_to() == "left/arm/base/forearm/wrist"
        # Served under the namespace, so each frame's prim path is its tf frame.
        arm_frames = {"left/arm", "left/arm/base", "left/arm/base/forearm", _WRIST}
        assert usd_frame_names(arm.get_usd()) == {"left", *arm_frames}
        camera_frames = {
            f"{_WRIST}/camera",
            f"{_WRIST}/camera/camera_link",
            f"{_WRIST}/camera/camera_link/optical",
        }
        assert usd_frame_names(camera.get_usd()) == {
            "left",
            *(arm_frames - {_WRIST}),
            _WRIST,
            *camera_frames,
        }
    finally:
        coordinator.stop()


def test_a_usd_with_no_source_path_is_served_and_published() -> None:
    pytest.importorskip("pxr")
    from dimos.robot.assets.usd_utils.frames import usd_frame_names

    # A layer handed over as text: no file it came from, so no folder its assets resolve from.
    usd = UsdModel.from_layer(_arm_usd().usd)
    assert usd.source_path is None
    coordinator = _build(autoconnect(Arm.blueprint(usd=usd)))
    try:
        arm = coordinator.get_instance("arm")
        assert usd_frame_names(arm.get_usd()) == {
            "arm",
            "arm/base",
            "arm/base/forearm",
            "arm/base/forearm/wrist",
        }
        assert ("arm/base", "arm/base/forearm") in {
            tuple(edge) for edge in arm.get_hardware_transform_ids()
        }
    finally:
        coordinator.stop()


def test_hardware_publishes_its_own_tf_and_its_mount() -> None:
    camera = WristCamera(
        attached_to="arm/base/forearm/wrist",
        instance_name="left/arm/base/forearm/wrist/wristcamera",
    )
    arm = Arm(instance_name="left/arm")
    try:
        edges = {
            (t.frame_id, t.child_frame_id): t for t in arm.moving_transforms({"elbow": math.pi / 2})
        }
        edges |= {(t.frame_id, t.child_frame_id): t for t in arm.static_transforms()}
        assert set(edges) == {
            ("left/arm", "left/arm/base"),
            ("left/arm/base", "left/arm/base/forearm"),
            ("left/arm/base/forearm", "left/arm/base/forearm/wrist"),
        }
        elbow = edges["left/arm/base", "left/arm/base/forearm"].to_matrix()
        np.testing.assert_allclose(elbow[:3, 3], [0.0, 0.0, 0.5], atol=1e-9)
        np.testing.assert_allclose(elbow[:3, 0], [0.0, 1.0, 0.0], atol=1e-9)

        camera_edges = {
            (t.frame_id, t.child_frame_id)
            for t in [*camera.static_transforms(ts=1.0), *camera.moving_transforms(ts=1.0)]
        }
        assert camera_edges == _CAMERA_STATIC_EDGES
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


_WRIST = "left/arm/base/forearm/wrist"
_CAMERA_STATIC_EDGES = {
    (_WRIST, f"{_WRIST}/camera"),
    (f"{_WRIST}/camera", f"{_WRIST}/camera/camera_link"),
    (f"{_WRIST}/camera/camera_link", f"{_WRIST}/camera/camera_link/optical"),
}


def test_static_edges_go_out_once_on_tf_static_then_on_tf_at_the_rate() -> None:
    camera = _ClockedCamera(
        attached_to="arm/base/forearm/wrist",
        instance_name=f"{_WRIST}/wristcamera",
        static_tf_rate_hz=4.0,
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
        attached_to="arm/base/forearm/wrist",
        instance_name=f"{_WRIST}/wristcamera",
        static_tf_rate_hz=0.0,
    )
    try:
        on_static, on_tf = _listening(camera)
        camera.publish_static_tf()
    finally:
        camera.stop()

    assert len(on_static) == 1
    assert on_tf == []
    assert camera.slept == []


def test_a_subclass_publishes_its_moving_edges_through_the_checked_tf_port(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sent: list[TFMessage] = []
    monkeypatch.setattr(IO, "publish", lambda _port, msg: sent.append(msg))
    arm = Arm(instance_name="left/arm")
    try:
        arm.tf.publish(TFMessage(*arm.moving_transforms({"elbow": 1.0}, ts=2.0)))
        with pytest.raises(ValueError, match="undeclared tf edge"):
            arm.tf.publish(
                TFMessage(Transform(frame_id="left/arm/base", child_frame_id="elsewhere"))
            )
    finally:
        arm.stop()

    ((elbow,),) = [message.transforms for message in sent]
    assert (elbow.frame_id, elbow.child_frame_id, elbow.ts) == (
        "left/arm/base",
        "left/arm/base/forearm",
        2.0,
    )


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
    linkage = Linkage(instance_name="linkage_ns/linkage")
    try:
        for positions in ({}, {"hinge": 2.9, "slide": -0.7}, {"hinge": -1.3, "slide": 0.4}):
            edges = {
                t.child_frame_id: t.to_matrix()
                for t in [*linkage.static_transforms(), *linkage.moving_transforms(positions)]
            }
            for joint in linkage.config.usd.joints:
                expected = joint.parent_from_child(joint_position(joint, positions))
                np.testing.assert_allclose(
                    edges[f"linkage_ns/{joint.child_link}"], expected, atol=1e-9
                )
    finally:
        linkage.stop()


def test_a_frame_id_prefix_is_refused_since_tf_is_the_namespace_and_the_prim_path() -> None:
    with pytest.raises(ValueError, match="frame_id_prefix"):
        Arm(instance_name="left/arm", frame_id_prefix="spare")


def test_the_usd_fills_the_tf_config_so_every_edge_is_declared() -> None:
    camera = WristCamera(
        attached_to="arm/base/forearm/wrist", instance_name=f"{_WRIST}/wristcamera"
    )
    try:
        assert camera.config.tf_frames_local == {
            frame: frame
            for frame in ("", "camera", "camera/camera_link", "camera/camera_link/optical")
        }
        assert camera.config.tf_frames_global == {}
        assert camera.tf_frames[""] == _WRIST
        assert camera.tf_edge_ids == _CAMERA_STATIC_EDGES
        assert camera.attach_frame == f"{_WRIST}/camera"
    finally:
        camera.stop()


def test_the_hardware_transform_ids_are_exactly_what_it_publishes_on_tf_and_tf_static() -> None:
    arm = Arm(instance_name="left/arm")
    camera = _ClockedCamera(
        attached_to="arm/base/forearm/wrist",
        instance_name=f"{_WRIST}/wristcamera",
        static_tf_rate_hz=4.0,
    )
    try:
        on_static, on_tf = _listening(camera)
        camera.publish_static_tf()
        assert camera.stopped.wait(5.0)
        static_edges = {(t.frame_id, t.child_frame_id) for m in on_static for t in m.transforms}
        static_edges |= {(t.frame_id, t.child_frame_id) for ts in on_tf for t in ts}
        moving = camera.moving_transforms()
        published = static_edges | {(t.frame_id, t.child_frame_id) for t in moving}
        assert set(camera.get_hardware_transform_ids()) == published == _CAMERA_STATIC_EDGES

        arm_edges = {
            (t.frame_id, t.child_frame_id)
            for t in [*arm.static_transforms(), *arm.moving_transforms()]
        }
        assert set(arm.get_hardware_transform_ids()) == arm_edges
    finally:
        arm.stop()
        camera.stop()


class AliasedArmConfig(ArmConfig):
    tf_frames_local: dict[str, str] = {"body": "arm/base", "elbow": "arm/base/forearm"}
    tf_edges: list[tuple[str, str]] = [("body", "elbow")]


class AliasedArm(HardwareModule):
    config: AliasedArmConfig


def test_a_subclass_may_name_its_usd_s_frames_and_edges_by_role() -> None:
    arm = AliasedArm(instance_name="left/arm")
    try:
        assert (arm.frame.body, arm.frame.elbow) == ("left/arm/base", "left/arm/base/forearm")
    finally:
        arm.stop()


@pytest.mark.parametrize(
    "estimate",
    [
        {"tf_frames_global": {"map": "map"}},
        {"tf_frames_local": {"body": "arm/base"}, "tf_edges": [("arm/base/forearm", "body")]},
    ],
)
def test_a_hardware_module_never_publishes_an_estimate(estimate: dict[str, Any]) -> None:
    with pytest.raises(
        ValueError, match="belongs in a separate module that is not a HardwareModule"
    ):
        Arm(instance_name="left/arm", **estimate)


def test_a_hardware_module_cannot_skip_the_check_that_keeps_it_to_its_usd() -> None:
    with pytest.raises(ValueError, match="cannot skip_tf_checks"):
        Arm(instance_name="left/arm", skip_tf_checks=True)


def test_a_hardware_module_publishes_no_edge_but_its_usd_s_and_its_mount(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(IO, "publish", lambda _port, msg: None)
    arm = Arm(instance_name="left/arm")
    try:
        estimate = Transform(frame_id="left/arm/base/forearm", child_frame_id="left/arm/base")
        with pytest.raises(ValueError, match="undeclared tf edge"):
            arm.tf.publish(TFMessage(estimate))
        with pytest.raises(ValueError, match="undeclared tf edge"):
            arm._create_tfbuffer(None).publish(estimate)
    finally:
        arm.stop()


def test_a_local_frame_its_usd_does_not_have_is_refused() -> None:
    with pytest.raises(
        ValueError, match=r"tf_frames_local\['odom'\] is 'odom', which is not a frame"
    ):
        Arm(instance_name="arm", tf_frames_local={"odom": "odom"})


def test_a_mount_its_usd_does_not_have_is_refused_in_the_blueprint() -> None:
    class Elsewhere(HardwareModule):
        config: ArmConfig
        mounts = Mounts("arm/base/nowhere")

    with pytest.raises(ValueError, match="the mount 'arm/base/nowhere' is not a frame of the USD"):
        _check_tf(autoconnect(Elsewhere.blueprint()))


def _check_tf(blueprint: MetaBlueprint) -> None:
    """TfModule's check, which sees each HardwareModule's tf as its config fills it in."""
    TfModule.blueprint_checks(
        [
            replace(atom, kwargs={**config_defaults(atom.module), **atom.kwargs})
            for atom in blueprint.solidify().active_blueprints
        ]
    )


def test_an_attached_assembly_is_one_tf_tree() -> None:
    _check_tf(autoconnect(Arm.blueprint(), WristCamera.attach(Arm.mounts.wrist)).namespace("left"))


class _Estimator(TfModule):
    config: TfModuleConfig


def test_the_mount_edge_goes_through_the_normal_tf_check() -> None:
    # camera -> arm/base closes a loop only through the mount edge wrist -> wrist/camera.
    looped = autoconnect(
        Arm.blueprint(),
        WristCamera.attach(Arm.mounts.wrist),
        _Estimator.blueprint(
            tf_frames_global={
                "camera": "arm/base/forearm/wrist/camera",
                "base": "arm/base",
            },
            tf_edges=[("camera", "base")],
        ),
    )
    with pytest.raises(
        ValueError,
        match=r"form a cycle.*arm/base/forearm/wrist/camera -> arm/base/forearm/wrist ->",
    ):
        _check_tf(looped)


def test_every_frame_on_tf_is_its_prim_path_in_the_composed_usd() -> None:
    """Served layers overlay as they are: every frame published is a prim of the assembly, and every frame prim is published (bar the namespace's own)."""
    pytest.importorskip("pxr")
    from pxr import Sdf, Usd

    from dimos.robot.assets.usd_utils.frames import frame_prims
    from dimos.robot.assets.usd_utils.pxr import frame_path

    arm = Arm(instance_name="left/arm")
    camera = WristCamera(
        attached_to="arm/base/forearm/wrist", instance_name=f"{_WRIST}/wristcamera"
    )
    try:
        layers = []
        for module in (arm, camera):
            layer = Sdf.Layer.CreateAnonymous(".usda")
            assert layer.ImportFromString(module.get_usd())
            layers.append(layer)
        assembly = Usd.Stage.CreateInMemory()
        assembly.GetRootLayer().subLayerPaths = [layer.identifier for layer in layers]
        assembly.SetDefaultPrim(assembly.GetPrimAtPath("/left"))
        composed = {frame_path(prim) for prim in frame_prims(assembly)}
        published = {
            frame
            for module in (arm, camera)
            for t in [*module.static_transforms(), *module.moving_transforms()]
            for frame in (t.frame_id, t.child_frame_id)
        }
    finally:
        arm.stop()
        camera.stop()

    assert f"{_WRIST}/camera/camera_link/optical" in published
    assert composed == published | {"left"}


def test_a_hardware_namespace_is_a_prim_name_so_it_cannot_start_with_a_digit() -> None:
    with pytest.raises(ValueError, match="'2nd' starts with '2'"):
        _build(autoconnect(Arm.blueprint().namespace("2nd")))


def test_hardware_without_a_usd_is_refused() -> None:
    class Bare(HardwareModule):
        config: HardwareModuleConfig

    with pytest.raises(ValueError, match="usd"):
        Bare(instance_name="bare")
