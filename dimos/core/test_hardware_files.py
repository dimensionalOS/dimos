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

import pytest

from dimos.core import hardware_files
from dimos.core.hardware_files import HardwareFiles
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.assets import asset_hash, referenced_asset_paths

pytest.importorskip("pxr")

_HULL = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
_FOREARM = b"v 0 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n"
_SKIN = b"png"


def _robot(root: Path) -> Path:
    """A robot whose layer references a part layer, which names a mesh above it and a texture."""
    (root / "meshes").mkdir(parents=True)
    (root / "parts").mkdir()
    (root / "textures").mkdir()
    (root / "meshes" / "hull.obj").write_bytes(_HULL)
    (root / "meshes" / "forearm.obj").write_bytes(_FOREARM)
    (root / "textures" / "skin.png").write_bytes(_SKIN)
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
        def Shader "skin" (
            assetInfo = {{
                asset identifier = @../textures/skin.png@
                string version = "{asset_hash(_SKIN)}"
            }}
        )
        {{
            asset inputs:file = @../textures/skin.png@
        }}
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


def _provided(files: HardwareFiles) -> dict[str, bytes]:
    provided = {}
    for uri in files.uris:
        reply = files.reply(FileInfo(uri))
        assert reply is not None and reply.content is not None, uri
        provided[uri] = reply.content
    return provided


def test_only_meshes_written_inside_the_layer_count_as_inline() -> None:
    from dimos.core.hardware_files import inline_meshes
    from dimos.robot.assets.usd_utils.pxr import load_pxr

    usd = """#usda 1.0
def Xform "robot"
{
    def Mesh "inline"
    {
        point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    }
    def Mesh "referenced" (
        prepend references = @meshes/hull.usda@
    )
    {
    }
    def Xform "link"
    {
    }
}
"""
    assert inline_meshes(load_pxr(), usd) == ["/robot/inline"]


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
    "uri", ["/etc/passwd", "../robot.usda", "robot/../../robot.usda", "", "not/provided.obj"]
)
def test_a_uri_it_did_not_publish_gets_no_reply(tmp_path: Path, uri: str) -> None:
    files = _files(tmp_path)
    absolute = tmp_path / "robot" / "meshes" / "hull.obj"

    assert files.reply(FileInfo(uri)) is None
    assert files.reply(FileInfo(str(absolute))) is None


def test_no_provided_file_or_uri_holds_a_host_path_nested_layers_included(tmp_path: Path) -> None:
    provided = _provided(_files(tmp_path))

    host = str(tmp_path.resolve())
    for uri, content in provided.items():
        assert not uri.startswith("/") and ".." not in uri.split("/")
        assert host not in uri
        assert host.encode() not in content and str(tmp_path).encode() not in content, uri
    layers = {uri: content for uri, content in provided.items() if uri.endswith(".usda")}
    assert len(layers) == 2, "the part layer is provided rewritten too"
    for uri, content in layers.items():
        for name in referenced_asset_paths(content.decode()):
            reached = posixpath.normpath(posixpath.join(posixpath.dirname(uri), name))
            assert reached in provided, f"{uri} names {name}, which is not provided"
    assert sorted(content for uri, content in provided.items() if uri.endswith(".obj")) == sorted(
        [_HULL, _FOREARM]
    )


def test_only_the_rewritten_layers_are_hashed_never_a_file_with_a_declared_sha256(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    hashed: list[bytes] = []

    def counted(data: bytes) -> str:
        hashed.append(data)
        return asset_hash(data)

    monkeypatch.setattr(hardware_files, "asset_hash", counted)
    files = _files(tmp_path)
    provided = _provided(files)

    assert sorted(hashed) == sorted(
        content for uri, content in provided.items() if uri.endswith(".usda")
    )
    for uri in ("meshes/hull.obj", "meshes/forearm.obj", "textures/skin.png"):
        (reached,) = [name for name in provided if name.endswith(uri.rpartition("/")[2])]
        assert files.reply(FileInfo(reached)).sha256 == asset_hash(provided[reached])


def test_a_file_with_no_declared_sha256_is_refused_when_the_hardware_starts(tmp_path: Path) -> None:
    (tmp_path / "hull.obj").write_bytes(_HULL)
    usd = f"""#usda 1.0
def Xform "robot"
{{
    def Mesh "hull"
    {{
        asset mesh = @{tmp_path / "hull.obj"}@
    }}
}}
"""
    with pytest.raises(ValueError, match=r"no sha256 declared[\s\S]*hull\.obj"):
        HardwareFiles(usd)
