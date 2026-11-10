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

"""Python and the Rust SDK calling each other over zenoh.

Builds and runs ``native/rust/dimos-module/examples/rpc_echo.rs``. Needs cargo,
so it carries the native_e2e marker and stays out of the fast suite.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
from pathlib import Path
import shutil
import socket
import subprocess

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.rpc.rpc_utils import RemoteError
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool

pytestmark = pytest.mark.native_e2e

_SDK = DIMOS_PROJECT_ROOT / "native" / "rust" / "dimos-module"
_TIMEOUT = 20.0


@pytest.fixture(scope="module")
def rpc_echo() -> Path:
    if shutil.which("cargo") is None:
        pytest.skip("cargo is not installed")
    subprocess.run(["cargo", "build", "--release", "--example", "rpc_echo"], cwd=_SDK, check=True)
    metadata = subprocess.run(
        ["cargo", "metadata", "--format-version", "1", "--no-deps"],
        cwd=_SDK,
        capture_output=True,
        check=True,
    ).stdout
    return Path(json.loads(metadata)["target_directory"]) / "release" / "examples" / "rpc_echo"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture
def python_side() -> Iterator[tuple[ZenohRPC, str]]:
    endpoint = f"tcp/127.0.0.1:{_free_port()}"
    rpc = ZenohRPC(
        rpc_timeouts={},
        default_rpc_timeout=_TIMEOUT,
        session_pool=ZenohSessionPool(),
        mode="peer",
        connect=[],
        listen=[endpoint],
        scouting=False,
        multicast=False,
    )
    rpc.start()

    def boom(who: str) -> None:
        raise ValueError(f"boom {who}")

    rpc.serve_rpc(lambda who: f"hello {who}", "PyEcho/greet")
    rpc.serve_rpc(boom, "PyEcho/boom")
    try:
        yield rpc, endpoint
    finally:
        rpc.stop()


@pytest.fixture
def rust_side(rpc_echo: Path, python_side: tuple[ZenohRPC, str]) -> Iterator[ZenohRPC]:
    rpc, endpoint = python_side
    session = ZenohConfig(
        mode="peer", connect=[endpoint], listen=[], scouting=False, multicast=False
    )
    launch = {"session": session.to_wire(), "name": "RustEcho", "python": "PyEcho"}
    process = subprocess.Popen([str(rpc_echo)], stdin=subprocess.PIPE)
    assert process.stdin is not None
    process.stdin.write(json.dumps(launch).encode() + b"\n")
    process.stdin.flush()
    try:
        yield rpc
    finally:
        process.stdin.close()
        process.wait(timeout=10)


def _call(rpc: ZenohRPC, name: str, args: tuple[object, ...], kwargs: dict[str, object]) -> object:
    result, _ = rpc.call_sync(name, (args, kwargs), rpc_timeout=_TIMEOUT)
    return result


def test_python_calls_rust(rust_side: ZenohRPC) -> None:
    assert _call(rust_side, "RustEcho/add", (1, 2.5), {"offset": 10}) == 13.5
    pose = PoseStamped(position=[1.0, 2.0, 3.0], ts=1.5, frame_id="map")
    assert _call(rust_side, "RustEcho/frame", (pose,), {}) == "map"
    with pytest.raises(RemoteError) as raised:
        _call(rust_side, "RustEcho/fail", (), {})
    assert raised.value.remote_type == "rust.ValueError"
    assert raised.value.args == ("nope",)


def test_rust_calls_python(rust_side: ZenohRPC) -> None:
    assert _call(rust_side, "RustEcho/call_python", (), {}) == "hello rust"
    with pytest.raises(ValueError, match="boom rust"):
        _call(rust_side, "RustEcho/call_python", (), {"fail": True})
