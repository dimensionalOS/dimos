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

"""Private primitive worlds for physics contracts; no runnable robot or blueprint.

SI metres/kg/radians; X forward, Y left, Z up. Two orthogonal +X/+Y sliders,
grounded base, tool zero (-.4, 0, .08). Solid-box inertias and identical visual/
collision boxes. Passive +Z hinge, .4 kg panel, COM (.225,0,.15), +/-1.5 rad stops.
These Python builders own temporary URDFs; they are not distributed robot assets.
"""

from contextlib import ExitStack
import math
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand, JointCommand
from dimos_lcm.sim_msgs import WorldState
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.robot.assets.model import RobotModel
from dimos.sim2.description import Drive, Instance, SceneConfig
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend


def _box_link(
    root: ET.Element,
    name: str,
    size: tuple[float, float, float],
    mass: float,
    color: str,
    offset: str = "0 0 0",
) -> None:
    link = ET.SubElement(root, "link", name=name)
    inertia = ET.SubElement(link, "inertial")
    ET.SubElement(inertia, "origin", xyz=offset)
    ET.SubElement(inertia, "mass", value=str(mass))
    x, y, z = size
    ET.SubElement(
        inertia,
        "inertia",
        ixx=str(mass * (y * y + z * z) / 12),
        iyy=str(mass * (x * x + z * z) / 12),
        izz=str(mass * (x * x + y * y) / 12),
        ixy="0",
        ixz="0",
        iyz="0",
    )
    for kind in ("visual", "collision"):
        shape = ET.SubElement(link, kind)
        ET.SubElement(shape, "origin", xyz=offset)
        ET.SubElement(ET.SubElement(shape, "geometry"), "box", size=" ".join(map(str, size)))
        if kind == "visual":
            ET.SubElement(ET.SubElement(shape, "material", name=name), "color", rgba=color)


def _pusher_urdf() -> ET.Element:
    root = ET.Element("robot", name="cartesian_pusher")
    _box_link(root, "base_link", (0.12, 0.3, 0.10), 2.0, ".25 .3 .4 1", "-.55 0 .05")
    _box_link(root, "carriage", (0.10, 0.10, 0.04), 0.5, ".2 .5 .8 1", "0 0 .15")
    _box_link(root, "tool", (0.10, 0.10, 0.10), 0.3, ".1 .7 .9 1")
    for name, parent, child, origin, axis, lower, upper in (
        ("slide_x", "base_link", "carriage", "-.4 0 .08", "1 0 0", "0", ".8"),
        ("slide_y", "carriage", "tool", "0 0 0", "0 1 0", "-.3", ".3"),
    ):
        joint = ET.SubElement(root, "joint", name=name, type="prismatic")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=origin)
        ET.SubElement(joint, "axis", xyz=axis)
        ET.SubElement(joint, "limit", lower=lower, upper=upper, effort="80", velocity=".5")
        ET.SubElement(joint, "dynamics", damping="2", friction="0")
    return root


def _door_urdf() -> ET.Element:
    root = ET.Element("robot", name="door")
    _box_link(root, "base", (0.04, 0.04, 0.32), 1.0, ".3 .3 .3 1", "0 0 .16")
    _box_link(root, "panel", (0.4, 0.025, 0.3), 0.4, ".7 .6 .3 1", ".225 0 .15")
    joint = ET.SubElement(root, "joint", name="hinge", type="revolute")
    ET.SubElement(joint, "parent", link="base")
    ET.SubElement(joint, "child", link="panel")
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="-1.5", upper="1.5", effort="10", velocity="2")
    ET.SubElement(joint, "dynamics", damping=".1")
    return root


@pytest.fixture
def world(tmp_path: Path) -> SceneConfig:
    pusher_path, door_path, box_path = (
        tmp_path / name for name in ("pusher.urdf", "door.urdf", "box.urdf")
    )
    ET.ElementTree(_pusher_urdf()).write(pusher_path, encoding="unicode")
    ET.ElementTree(_door_urdf()).write(door_path, encoding="unicode")
    # 10 cm, 200 g solid box, at its COM; same fixture geometry as the former demo asset.
    box = ET.Element("robot", name="box")
    _box_link(box, "box", (0.1, 0.1, 0.1), 0.2, ".9 .45 .12 1")
    ET.ElementTree(box).write(box_path, encoding="unicode")
    pusher = (
        RobotModel.from_file(pusher_path)
        .with_fixed_frame(
            "camera", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
        )
        .with_fixed_frame("lidar", "tool", xyz=(0, 0, 0.12))
        .with_fixed_frame("imu", "tool")
    )

    robots = tuple(
        Instance(
            name,
            pusher.with_prefix(name + "/"),
            xyz=(0, y, 0),
            drives=tuple(Drive(name + "/" + joint) for joint in ("slide_x", "slide_y")),
        )
        for name, y in (("left", 0.0), ("right", 1.2))
    )
    return SceneConfig(
        (
            *robots,
            Instance(
                "box",
                RobotModel.from_file(box_path),
                xyz=(0, 0, 0.055),
                floating=True,
            ),
            Instance(
                "door",
                RobotModel.from_file(door_path)
                .with_fixed_frame("imu", "panel", xyz=(0.225, 0, 0.15))
                .with_prefix("door/"),
                xyz=(0.6, 0.4, 0),
            ),
        ),
    )


@pytest.fixture
def backend(world):
    backend = MujocoBackend()
    backend.load(world, 2_000_000)
    try:
        yield backend
    finally:
        backend.close()


@pytest.fixture
def poses(backend, world):
    def capture(time_ns=0, epoch=0):
        bodies = backend.read_bodies()
        header = SampleHeader(
            clock_id="test",
            epoch=epoch,
            time_ns=time_ns,
            source_id="sim/observations",
            frame_id="world",
        )
        joints = backend.read_joints(backend.joint_ids)
        joints.header = header
        return WorldState(
            header=header,
            model_revision=world.revision,
            bodies_length=len(bodies),
            bodies=bodies,
            joint_truth=joints,
        )

    return capture


@pytest.fixture
def make_sim(world, tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    with ExitStack() as cleanup:

        def deploy(
            *, on_observations=None, backend_type=MujocoBackend, with_actions=False, **settings
        ):
            blueprint = SimModule.blueprint(
                scene=world, backend=backend_type, **settings
            ).global_config(
                viewer="none",
                transport="zenoh",
                n_workers=1,
                zenoh_domain="sim2-owner-test-" + uuid.uuid4().hex,
            )
            coordinator = ModuleCoordinator.build(blueprint)
            cleanup.callback(coordinator.stop)
            if on_observations is not None:
                transport = coordinator.transports[("observations", WorldState)]
                cleanup.callback(transport.stop)
                cleanup.callback(transport.subscribe(on_observations))
            sim = coordinator.get_instance(SimModule)
            if with_actions:
                actions = coordinator.transports[("actions", ActionCommand)]
                cleanup.callback(actions.stop)
                return sim, actions
            return sim

        yield deploy


@pytest.fixture
def command():
    def make(
        header, names=("left/slide_x",), positions=(0.2,), ttl=2.0, kp=200.0, kd=20.0, effort=0.0
    ):
        return ActionCommand(
            header=SampleHeader(
                clock_id=header.clock_id,
                epoch=header.epoch,
                time_ns=header.time_ns,
                source_id="test",
                sequence=header.sequence,
            ),
            based_on_time_ns=header.time_ns,
            apply_at_ns=header.time_ns,
            valid_until_ns=header.time_ns + round(ttl * 1e9),
            joint_commands_length=len(names),
            joint_commands=[
                JointCommand(joint_id=n, position=q, velocity=0, kp=kp, kd=kd, feedforward=effort)
                for n, q in zip(names, positions, strict=True)
            ],
        )

    return make
