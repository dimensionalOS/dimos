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

"""UrdfManagerNative against the Python UrdfManager: the same tf edges for the same joint state, the same files served."""

from __future__ import annotations

from collections.abc import Callable, Iterator
import contextlib
from pathlib import Path
import threading
import time
from typing import Any
import uuid

import numpy as np
import pytest

from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.File import File
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_native.native import UrdfManagerNative

# Every joint kind UrdfManager handles: revolute (off-axis, rotated origin), continuous, prismatic, a mimic, fixed,
# and a floating joint it leaves to tf; plus a mesh and a texture named through package://.
_URDF = """<?xml version="1.0"?>
<robot name="kit">
  <link name="world"/>
  <link name="base"><visual><geometry><mesh filename="package://kit/meshes/base.stl"/></geometry>
    <material name="skin"><texture filename="package://kit/textures/skin.png"/></material></visual></link>
  <link name="upper"/>
  <link name="wheel"/>
  <link name="slider"/>
  <link name="finger"/>
  <link name="camera"/>
  <joint name="odom" type="floating"><parent link="world"/><child link="base"/></joint>
  <joint name="shoulder" type="revolute">
    <origin xyz="0.1 -0.2 0.5" rpy="0.3 -0.2 1.1"/><parent link="base"/><child link="upper"/>
    <axis xyz="0.2 1 -0.4"/><limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="wheel_joint" type="continuous">
    <origin xyz="0 0.3 0" rpy="1.5707963 0 0"/><parent link="base"/><child link="wheel"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="slide" type="prismatic">
    <origin xyz="1 0 0"/><parent link="upper"/><child link="slider"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="finger_joint" type="prismatic">
    <parent link="slider"/><child link="finger"/><mimic joint="slide" multiplier="-2" offset="0.1"/>
  </joint>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.05 0 0.1" rpy="0 0.4 0"/><parent link="upper"/><child link="camera"/>
  </joint>
</robot>
"""

_TIMEOUT_S = 10.0


@pytest.fixture
def model(tmp_path: Path) -> RobotModel:
    package = tmp_path / "kit"
    (package / "meshes").mkdir(parents=True)
    (package / "meshes" / "base.stl").write_bytes(b"solid base\nendsolid base\n")
    (package / "textures").mkdir()
    (package / "textures" / "skin.png").write_bytes(b"png bytes")
    urdf = tmp_path / "kit.urdf"
    urdf.write_text(_URDF)
    return RobotModel.from_file(urdf, package_paths={"kit": package})


class _Wired:
    """A running UrdfManagerNative on private topics, with what it publishes collected."""

    def __init__(
        self, module: UrdfManagerNative, transports: dict[str, PubSubTransport[Any]]
    ) -> None:
        self.module = module
        self.transports = transports
        self.edges: list[Transform] = []
        self.files: list[File] = []
        self._lock = threading.Lock()
        transports["tf"].subscribe(self._on_tf)
        transports["file_info"].subscribe(self._on_file)

    def _on_tf(self, message: TFMessage) -> None:
        with self._lock:
            self.edges.extend(message.transforms)

    def _on_file(self, file: File) -> None:
        with self._lock:
            self.files.append(file)

    def send(self, port: str, message: Any) -> None:
        self.transports[port].broadcast(None, message)

    def take_files(self) -> list[File]:
        with self._lock:
            files, self.files = self.files, []
        return files

    def edges_at(self, ts: float) -> dict[tuple[str, str], Transform]:
        with self._lock:
            return {
                (edge.frame_id, edge.child_frame_id): edge for edge in self.edges if edge.ts == ts
            }


@pytest.fixture
def wire(model: RobotModel) -> Iterator[Callable[..., _Wired]]:
    started: list[_Wired] = []

    def make(**overrides: Any) -> _Wired:
        module = UrdfManagerNative(model=model, **overrides)
        prefix = f"/urdf_native_test/{uuid.uuid4().hex[:8]}"
        transports: dict[str, PubSubTransport[Any]] = {
            "tf": make_transport(f"{prefix}/tf", TFMessage),
            "coordinator_joint_state": make_transport(f"{prefix}/joint_state", JointState),
            "render_file_request": make_transport(f"{prefix}/render_file_request", String),
            "file_info": make_transport(f"{prefix}/file_info", File),
        }
        for port, transport in transports.items():
            module.set_transport(port, transport)
        wired = _Wired(module, transports)
        started.append(wired)
        module.start()
        return wired

    yield make
    for wired in started:
        wired.module.stop()
        for transport in wired.transports.values():
            with contextlib.suppress(Exception):
                transport.stop()


def _wait_for(condition: Callable[[], bool], what: str) -> None:
    deadline = time.monotonic() + _TIMEOUT_S
    while not condition():
        assert time.monotonic() < deadline, f"timed out waiting for {what}"
        time.sleep(0.02)


def _send_until_edges(
    wired: _Wired, state: JointState, count: int
) -> dict[tuple[str, str], Transform]:
    """Send the state until its edges come back: the native process subscribes a moment after it is spawned."""
    deadline = time.monotonic() + _TIMEOUT_S
    while len(edges := wired.edges_at(state.ts)) < count:
        assert time.monotonic() < deadline, f"timed out waiting for the edges at {state.ts}"
        wired.send("coordinator_joint_state", state)
        time.sleep(0.1)
    return edges


def _request_until_served(wired: _Wired, name: str) -> File:
    """Request the file until it comes back (the process subscribes a moment after it is spawned), then forget it."""
    deadline = time.monotonic() + _TIMEOUT_S
    while not wired.files:
        assert time.monotonic() < deadline, f"timed out waiting for {name}"
        wired.send("render_file_request", String(name))
        time.sleep(0.1)
    time.sleep(0.2)
    return wired.take_files()[0]


def _python_edges(model: RobotModel, state: JointState) -> dict[tuple[str, str], Transform]:
    python = UrdfManager(model=model)
    try:
        return {(t.frame_id, t.child_frame_id): t for t in python.joint_transforms(state)}
    finally:
        python.dispose()


def test_the_config_crosses_as_the_materialized_urdf(model: RobotModel) -> None:
    module = UrdfManagerNative(model=model, joint_tf_hz=10.0)
    config = module.config
    module.dispose()

    assert config.to_cli_args() == []
    assert config.to_config_dict() == {
        "urdf_xml": model.load().xml,
        "publish_joint_tf": True,
        "joint_tf_hz": 10.0,
        "unchanged_tf_hz": 4.0,
    }


@pytest.mark.native_e2e
def test_tf_edges_match_the_python_urdf_manager(
    wire: Callable[..., _Wired], model: RobotModel
) -> None:
    # fast enough that the rate limit never drops a state here, slow keepalive so unchanged edges stay quiet
    wired = wire(joint_tf_hz=1000.0, unchanged_tf_hz=0.1)
    first = JointState(ts=1.0, name=["shoulder", "wheel_joint", "slide"], position=[0.7, -2.5, 0.3])
    # only the shoulder moves; the wheel and slide (and its mimic) and the fixed camera keep their last pose
    second = JointState(
        ts=2.0, name=["shoulder", "wheel_joint", "slide"], position=[-1.2, -2.5, 0.3]
    )

    expected_first = _python_edges(model, first)
    native_first = _send_until_edges(wired, first, len(expected_first))
    wired.send("coordinator_joint_state", second)
    _wait_for(lambda: bool(wired.edges_at(2.0)), "the moved edge")
    time.sleep(0.2)
    native_second = wired.edges_at(2.0)

    # the floating joint is tf's, never the manager's
    assert set(native_first) == set(expected_first)
    assert ("world", "base") not in native_first
    assert set(native_second) == {("base", "upper")}
    for native, expected in [
        (native_first, expected_first),
        (native_second, _python_edges(model, second)),
    ]:
        for key, edge in native.items():
            np.testing.assert_allclose(
                edge.to_matrix(), expected[key].to_matrix(), atol=1e-9, err_msg=str(key)
            )


@pytest.mark.native_e2e
def test_no_tf_when_publish_joint_tf_is_off(wire: Callable[..., _Wired]) -> None:
    wired = wire(publish_joint_tf=False)
    # a file round trip shows the process is up and subscribed before the silence counts
    _request_until_served(wired, "x.urdf")
    wired.send("coordinator_joint_state", JointState(ts=1.0, name=["shoulder"], position=[0.3]))
    time.sleep(0.5)

    assert wired.edges_at(1.0) == {}


@pytest.mark.native_e2e
def test_render_file_request_serves_the_urdf_and_only_files_it_names(
    wire: Callable[..., _Wired], model: RobotModel, tmp_path: Path
) -> None:
    wired = wire()
    package = tmp_path / "kit"
    mesh = str((package / "meshes" / "base.stl").resolve())
    texture = str((package / "textures" / "skin.png").resolve())
    urdf = str(tmp_path / "kit.urdf")
    served_urdf = _request_until_served(wired, urdf)

    # anything else on disk, or the unresolved URI, gets nothing
    for request in (str(package / "meshes"), "package://kit/meshes/base.stl", mesh, texture):
        wired.send("render_file_request", String(request))
    _wait_for(lambda: len(wired.files) >= 2, "the mesh and texture")
    time.sleep(0.2)

    assert served_urdf.uri == urdf
    assert served_urdf.data.decode() == model.load().xml
    assert f'filename="{mesh}"' in served_urdf.data.decode()
    assert [file.uri for file in wired.files] == [mesh, texture]
    assert wired.files[0].data == Path(mesh).read_bytes()
    assert wired.files[1].data == b"png bytes"
