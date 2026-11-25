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
import posixpath
import threading
import uuid

import pytest

from dimos.core import hardware_files
from dimos.core.hardware_file_client import fetch_hardware_files
from dimos.core.hardware_files import HARDWARE_USD, HardwareFiles
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import asset_hash, referenced_asset_paths

pytest.importorskip("pxr")

_HULL = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
_FOREARM = b"v 0 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n"


def _robot(root: Path) -> Path:
    """A robot whose layer references a part layer, which names a mesh above it and a texture."""
    (root / "meshes").mkdir(parents=True)
    (root / "parts").mkdir()
    (root / "textures").mkdir()
    (root / "meshes" / "hull.obj").write_bytes(_HULL)
    (root / "meshes" / "forearm.obj").write_bytes(_FOREARM)
    (root / "textures" / "skin.png").write_bytes(b"png")
    (root / "parts" / "arm.usda").write_text(
        f"""#usda 1.0
(
    defaultPrim = "arm"
)

def Xform "arm"
{{
    def Mesh "forearm" (
        assetInfo = {{
            asset identifier = @../meshes/forearm.obj@
            string version = "{asset_hash(_FOREARM)}"
        }}
    )
    {{
        asset skin = @../textures/skin.png@
    }}
}}
"""
    )
    robot = root / "robot.usda"
    robot.write_text(
        f"""#usda 1.0
(
    defaultPrim = "robot"
)

def Xform "robot"
{{
    def Xform "arm" (
        prepend references = @parts/arm.usda@
    )
    {{
    }}

    def Mesh "hull" (
        assetInfo = {{
            asset identifier = @meshes/hull.obj@
            string version = "{asset_hash(_HULL)}"
        }}
    )
    {{
    }}
}}
"""
    )
    return robot


def _files(tmp_path: Path) -> HardwareFiles:
    from dimos.robot.assets.usd_utils.assets import resolve_usd_assets

    robot = _robot(tmp_path / "robot")
    return HardwareFiles(resolve_usd_assets(robot.read_text(), str(robot.parent)))


def _served(files: HardwareFiles) -> dict[str, bytes]:
    served = {}
    for uri in files.uris:
        reply = files.reply(FileInfo(uri))
        assert reply is not None and reply.content is not None, uri
        served[uri] = reply.content
    return served


def test_a_request_holding_the_current_sha256_gets_only_the_header(tmp_path: Path) -> None:
    files = _files(tmp_path)
    for uri in files.uris:
        full = files.reply(FileInfo(uri))
        assert full is not None and full.content is not None
        assert full.sha256 == asset_hash(full.content)

        header = files.reply(FileInfo(uri, full.sha256))
        assert header is not None
        assert (header.uri, header.sha256, header.mtime, header.content) == (
            uri,
            full.sha256,
            full.mtime,
            None,
        )

        stale = files.reply(FileInfo(uri, "sha256:" + "0" * 64))
        assert stale is not None and stale.content == full.content


@pytest.mark.parametrize(
    "uri", ["/etc/passwd", "../robot.usda", "robot/../../robot.usda", "", "not/served.obj"]
)
def test_a_uri_it_did_not_publish_gets_no_reply(tmp_path: Path, uri: str) -> None:
    files = _files(tmp_path)
    absolute = tmp_path / "robot" / "meshes" / "hull.obj"

    assert files.reply(FileInfo(uri)) is None
    assert files.reply(FileInfo(str(absolute))) is None


def test_no_served_file_or_uri_holds_a_host_path_nested_layers_included(tmp_path: Path) -> None:
    served = _served(_files(tmp_path))

    host = str(tmp_path.resolve())
    for uri, content in served.items():
        assert not uri.startswith("/") and ".." not in uri.split("/")
        assert host not in uri
        assert host.encode() not in content and str(tmp_path).encode() not in content, uri
    layers = {uri: content for uri, content in served.items() if uri.endswith(".usda")}
    assert len(layers) == 2, "the part layer is served rewritten too"
    for uri, content in layers.items():
        for name in referenced_asset_paths(content.decode()):
            reached = posixpath.normpath(posixpath.join(posixpath.dirname(uri), name))
            assert reached in served, f"{uri} names {name}, which is not served"
    assert sorted(content for uri, content in served.items() if uri.endswith(".obj")) == sorted(
        [_HULL, _FOREARM]
    )


def test_a_file_is_hashed_again_only_once_it_changes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    mesh = tmp_path / "mesh.obj"
    mesh.write_bytes(_HULL)
    hashed: list[bytes] = []

    def counted(data: bytes) -> str:
        hashed.append(data)
        return asset_hash(data)

    monkeypatch.setattr(hardware_files, "asset_hash", counted)

    first = hardware_files.file_hash(mesh)
    assert hardware_files.file_hash(mesh) == first
    assert hashed == [_HULL]

    mesh.write_bytes(_FOREARM + b"# longer")
    assert hardware_files.file_hash(mesh)[0] == asset_hash(_FOREARM + b"# longer")
    assert len(hashed) == 2


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
        forearm = stage.GetPrimAtPath("/robot/arm/forearm")
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
