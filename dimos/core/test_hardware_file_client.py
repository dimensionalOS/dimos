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
import threading
import uuid

import pytest

from dimos.core.hardware_file_client import fetch_hardware_files
from dimos.core.hardware_files import HARDWARE_USD
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig
from dimos.core.test_hardware_files import _robot
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.model import UsdModel

pytest.importorskip("pxr")


class _RobotConfig(HardwareModuleConfig):
    usd: UsdModel


class _Robot(HardwareModule):
    config: _RobotConfig


def _serving(robot: Path, namespace: str) -> _Robot:
    module = _Robot(usd=UsdModel.from_file(robot), instance_name=f"{namespace}/robot")
    module.hardware_file_request.transport = make_transport(
        f"/{namespace}/hardware_file_request", FileInfo
    )
    module.hardware_file_response.transport = make_transport(
        f"/{namespace}/hardware_file_response", FileInfo
    )
    return module


def test_a_client_lays_the_files_out_so_its_layers_resolve_and_fetches_a_kept_file_once(
    tmp_path: Path,
) -> None:
    namespace = f"files_{uuid.uuid4().hex[:8]}"
    module = _serving(_robot(tmp_path / "robot"), namespace)
    cache = tmp_path / "cache"
    fetched: list[str] = []
    try:
        # Asked before the module starts: the request is sent again when it announces itself.
        result: list[Path] = []
        fetching = threading.Thread(
            target=lambda: result.append(fetch_hardware_files(namespace, cache))
        )
        fetching.start()
        module.start()
        fetching.join()
        (usd,) = result
        assert usd.read_text() == module.get_usd()

        from pxr import Usd

        stage = Usd.Stage.Open(str(usd))
        # Served under the namespace, so the prim path is the tf frame.
        forearm = stage.GetPrimAtPath(f"/{namespace}/robot/arm/forearm")
        assert forearm, "the part layer resolved from the cache"
        skin = forearm.GetAttribute("skin").Get().resolvedPath
        assert Path(skin).read_bytes() == b"png"

        listening = make_transport(f"/{namespace}/hardware_file_response", FileInfo)
        listening.start()
        unsubscribe = listening.subscribe(
            lambda reply: fetched.append(reply.uri) if reply.content is not None else None
        )
        try:
            assert fetch_hardware_files(namespace, cache) == usd
        finally:
            unsubscribe()
            listening.stop()
    finally:
        module.stop()
    # Only hardware.usda comes whole again; each mesh is kept by the sha256 in its assetInfo.
    assert set(fetched) <= {HARDWARE_USD}


def test_a_client_fetches_a_binary_layer_the_robot_references(tmp_path: Path) -> None:
    from pxr import Sdf, Usd

    root = tmp_path / "robot"
    root.mkdir()
    crate = Sdf.Layer.CreateNew(str(root / "base.usdc"))
    Sdf.CreatePrimInLayer(crate, "/base").specifier = Sdf.SpecifierDef
    crate.defaultPrim = "base"
    crate.Save()
    robot = root / "robot.usda"
    robot.write_text(
        """#usda 1.0
(
    defaultPrim = "robot"
)

def Xform "robot"
{
    def Xform "base" (
        prepend references = @base.usdc@
    )
    {
    }
}
"""
    )
    namespace = f"files_{uuid.uuid4().hex[:8]}"
    module = _serving(robot, namespace)
    try:
        module.start()
        usd = fetch_hardware_files(namespace, tmp_path / "cache")
        assert Usd.Stage.Open(str(usd)).GetPrimAtPath(f"/{namespace}/robot/base")
    finally:
        module.stop()
