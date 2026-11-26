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

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path
import threading
import uuid

import pytest

from dimos.core.hardware_file_client import fetch_hardware_files
from dimos.core.hardware_files import HARDWARE_USD, HardwareFiles, hardware_file_topics
from dimos.core.test_hardware_files import _robot
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.assets.usd_utils.assets import asset_hash, resolve_usd_assets
from dimos.robot.assets.usd_utils.layers import usd_under_namespace

pytest.importorskip("pxr")


@contextmanager
def _providing(robot: Path, namespace: str) -> Iterator[tuple[HardwareFiles, Callable[[], None]]]:
    """``robot`` provided as a HardwareModule in ``namespace`` would provide it, over the same two topics; and what announces it."""
    root = str(robot.parent)
    files = HardwareFiles(
        usd_under_namespace(resolve_usd_assets(robot.read_text(), root), namespace, root)
    )
    request_topic, response_topic = hardware_file_topics(namespace)
    requests, responses = (
        make_transport(request_topic, FileInfo),
        make_transport(response_topic, FileInfo),
    )
    requests.start()
    responses.start()

    def answer(request: FileInfo) -> None:
        reply = files.reply(request)
        if reply is not None:
            responses.publish(reply)

    unsubscribe = requests.subscribe(answer)
    try:
        yield files, lambda: responses.publish(files.announcement())
    finally:
        unsubscribe()
        requests.stop()
        responses.stop()


def test_a_client_lays_the_files_out_so_its_layers_resolve_and_fetches_a_kept_file_once(
    tmp_path: Path,
) -> None:
    namespace = f"files_{uuid.uuid4().hex[:8]}"
    cache = tmp_path / "cache"
    fetched: list[str] = []
    with _providing(_robot(tmp_path / "robot"), namespace) as (files, announce):
        # Asked before the files are announced: the request is sent again when they are.
        result: list[Path] = []
        fetching = threading.Thread(
            target=lambda: result.append(fetch_hardware_files(namespace, cache))
        )
        fetching.start()
        announce()
        fetching.join()
        (usd,) = result
        assert usd.read_text() == files.usd

        from pxr import Usd

        stage = Usd.Stage.Open(str(usd))
        # Provided under the namespace, so the prim path is the tf frame.
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
    with _providing(robot, namespace) as (files, announce):
        announce()
        usd = fetch_hardware_files(namespace, tmp_path / "cache")
        assert Usd.Stage.Open(str(usd)).GetPrimAtPath(f"/{namespace}/robot/base")


def test_a_sublayer_also_called_hardware_usda_is_fetched_as_itself(tmp_path: Path) -> None:
    from pxr import Usd

    root = tmp_path / "robot"
    (root / "base").mkdir(parents=True)
    (root / "base" / HARDWARE_USD).write_text('#usda 1.0\n\ndef Xform "base"\n{\n}\n')
    robot = root / "robot.usda"
    robot.write_text(
        '#usda 1.0\n(\n    subLayers = [@base/hardware.usda@]\n)\n\ndef Xform "robot"\n{\n}\n'
    )
    namespace = f"files_{uuid.uuid4().hex[:8]}"
    with _providing(robot, namespace) as (_files, announce):
        announce()
        usd = fetch_hardware_files(namespace, tmp_path / "cache")
        assert Usd.Stage.Open(str(usd)).GetPrimAtPath("/base")


def test_a_relative_cache_still_links_to_its_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from pxr import Usd

    namespace = f"files_{uuid.uuid4().hex[:8]}"
    monkeypatch.chdir(tmp_path)
    with _providing(_robot(tmp_path / "robot"), namespace) as (_files, announce):
        announce()
        usd = fetch_hardware_files(namespace, Path("cache"))
        assert Usd.Stage.Open(str(usd)).GetPrimAtPath(f"/{namespace}/robot/arm/forearm")


def test_a_layer_that_names_itself_is_fetched_once(tmp_path: Path) -> None:
    layers = {
        HARDWARE_USD: b'#usda 1.0\n\ndef Xform "robot" (\n    prepend references = @a.usda@\n)\n{\n}\n',
        "a.usda": b'#usda 1.0\n\ndef Xform "a" (\n    prepend references = @a.usda@\n)\n{\n}\n',
    }
    namespace = f"files_{uuid.uuid4().hex[:8]}"
    request_topic, response_topic = hardware_file_topics(namespace)
    requests, responses = (
        make_transport(request_topic, FileInfo),
        make_transport(response_topic, FileInfo),
    )
    requests.start()
    responses.start()
    asked: list[str] = []

    def answer(request: FileInfo) -> None:
        asked.append(request.uri)
        content = layers[request.uri]
        responses.publish(FileInfo(request.uri, asset_hash(content), None, content))

    unsubscribe = requests.subscribe(answer)
    try:
        fetch_hardware_files(namespace, tmp_path / "cache")
    finally:
        unsubscribe()
        requests.stop()
        responses.stop()
    assert asked == [HARDWARE_USD, "a.usda"]
