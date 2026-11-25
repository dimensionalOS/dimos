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
from pathlib import Path
import posixpath
from typing import Any

import pytest

pytest.importorskip("pxr")

_PART = """#usda 1.0
def Cube "part"
{
}
"""


def _world(part: Path) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "room"
)
def Xform "room"
{{
    def Xform "desk" (
        prepend references = @{part}@
    )
    {{
    }}
}}
"""


def test_a_usd_request_is_the_world_with_its_assets_under_names_that_are_answered_too(
    tmp_path: Path,
) -> None:
    from dimos.sim2.world_files import WorldFiles

    part = tmp_path / "part.usda"
    part.write_text(_PART)
    secret = tmp_path / "secret.txt"
    secret.write_text("not the world's")
    files = WorldFiles(_world(part))

    async def ask() -> tuple[bytes | None, bytes | None, bytes | None]:
        layer = await files.file("*.usd")
        assert layer is not None
        named = [line for line in layer.decode().splitlines() if "references" in line]
        published = named[0].split("@")[1]
        return layer, await files.file(published), await files.file(str(secret))

    layer, asset, elsewhere = asyncio.run(ask())
    assert layer is not None and layer.startswith(b"#usda")
    assert str(tmp_path).encode() not in layer, "a viewer cannot open this machine's paths"
    assert asset == _PART.encode()
    assert elsewhere is None, "only names the world published are answered"


def test_a_request_repeated_while_its_reply_is_on_the_way_gets_that_one_reply() -> None:
    from dimos.sim2.world_files import WorldFiles

    files = WorldFiles('#usda 1.0\ndef Xform "room"\n{\n}\n')
    published: list[Any] = []

    async def ask_three_times() -> None:
        await asyncio.gather(*(files.answer("*.usd", published.append) for _ in range(3)))

    asyncio.run(ask_three_times())
    assert [reply.uri for reply in published] == ["*.usd"]


def test_a_texture_a_referenced_layer_names_beside_itself_is_answered_and_an_escape_is_not(
    tmp_path: Path,
) -> None:
    from dimos.sim2.world_files import WorldFiles

    scene = tmp_path / "scene"
    (scene / "deeper").mkdir(parents=True)
    (scene / "wood.png").write_bytes(b"wood")
    (scene / "deeper" / "grain.png").write_bytes(b"grain")
    (tmp_path / "outside.png").write_bytes(b"not the scene's")
    (scene / "deeper" / "more.usda").write_text(
        '#usda 1.0\ndef Shader "grain"\n{\n    asset inputs:file = @grain.png@\n}\n'
    )
    mesh = scene / "mesh.usda"
    mesh.write_text(
        "#usda 1.0\n(\n    subLayers = [@./deeper/more.usda@]\n)\n"
        'def Shader "wood"\n{\n    asset inputs:file = @wood.png@\n'
        "    asset inputs:other = @../outside.png@\n}\n"
    )
    files = WorldFiles(_world(mesh))

    async def ask() -> tuple[str, dict[str, bytes | None]]:
        layer = await files.file("*.usd")
        assert layer is not None
        named = [line for line in layer.decode().splitlines() if "references" in line]
        published = named[0].split("@")[1]
        folder = published.rsplit("/", 1)[0]
        asked = [
            f"{folder}/wood.png",
            f"{folder}/deeper/grain.png",
            posixpath.normpath(f"{folder}/../outside.png"),
        ]
        return folder, {name: await files.file(name) for name in asked}

    folder, answers = asyncio.run(ask())
    assert answers[f"{folder}/wood.png"] == b"wood"
    assert answers[f"{folder}/deeper/grain.png"] == b"grain", "a sublayer's texture, at depth"
    assert answers[posixpath.normpath(f"{folder}/../outside.png")] is None, (
        "a path out of the scene is refused"
    )
