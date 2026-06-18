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

"""Throwaway end-to-end check for the TF-tree demo. Run:

    python examples/tf_tree/demo_smoke.py

Verifies: the bridge serves the @web_module page + client (one server), the binary
(LCM) fast path, transient_local QoS replay, allow-all reception (subscribeAll),
dynamic set_qos, and the backend topic-exposure gate.
"""

import asyncio
import json
import os
import time

import requests
import websockets

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.web.ts_bridge.ts_bridge_module import DimosWebsocket
from examples.tf_tree.tf_tree import FakeTfPublisher, TfWebUi

PORT = global_config.ts_api_port
BASE = f"http://127.0.0.1:{PORT}"
WS = f"ws://127.0.0.1:{PORT}/ws"


def check_web_host() -> None:
    page = requests.get(f"{BASE}/TfWebUi", timeout=5)
    assert page.status_code == 200 and 'import { Dimos } from "/dimos.js"' in page.text
    print(f"PAGE /TfWebUi served ({len(page.text)} bytes)")
    assert requests.get(f"{BASE}/dimos.js", timeout=5).text.count("class Dimos")
    icon = requests.get(f"{BASE}/TfWebUi/icon", timeout=5)
    assert icon.status_code == 200 and icon.headers["content-type"] == "image/svg+xml"
    print("CLIENT /dimos.js + bundled /TfWebUi/icon served (svg, sniffed)")


def parse_binary_frame(frame: bytes) -> tuple[str, bytes]:
    name_len = int.from_bytes(frame[:2], "big")
    return frame[2 : 2 + name_len].decode("utf-8"), frame[2 + name_len :]


async def _hello(ws: object, **extra: object) -> None:
    await ws.send(json.dumps({"type": "hello", "encoding": "binary", **extra}))  # type: ignore[attr-defined]
    assert json.loads(await ws.recv())["type"] == "ready"  # type: ignore[attr-defined]


async def check_streaming() -> None:
    # binary fast path + transient_local replay
    async with websockets.connect(WS) as ws:
        await _hello(
            ws, whitelist=["/tf"], qos={"/tf": {"rate": 5, "durability": "transient_local"}}
        )
        frame = await asyncio.wait_for(ws.recv(), timeout=5)
        assert isinstance(frame, bytes)
        stream, payload = parse_binary_frame(frame)
        decoded = TFMessage.lcm_decode(payload)  # the browser does this via @dimos/msgs
        print(f"BINARY {stream} bytes={len(payload)} transforms={len(decoded.transforms)}")

        # dynamic set_qos — change the rate at runtime, keep streaming
        await ws.send(json.dumps({"type": "set_qos", "qos": {"/tf": {"rate": 2}}}))
        assert isinstance(await asyncio.wait_for(ws.recv(), timeout=5), bytes)
        print("SET_QOS accepted, stream continues")

    # allow-all (no whitelist) — what subscribeAll consumes: streams not named up front
    async with websockets.connect(WS) as ws:
        await _hello(ws)
        frame = await asyncio.wait_for(ws.recv(), timeout=5)
        assert isinstance(frame, bytes)
        stream, _ = parse_binary_frame(frame)
        assert stream == "/tf"
        print(f"ALLOW-ALL received {stream} (no whitelist) — subscribeAll path")


async def check_backend_gate() -> None:
    # bridge configured to block /tf — client asking for it gets nothing
    async with websockets.connect(WS) as ws:
        await _hello(ws, whitelist=["/tf"])
        try:
            await asyncio.wait_for(ws.recv(), timeout=2)
            raise AssertionError("expected /tf to be blocked by the backend blacklist")
        except asyncio.TimeoutError:
            print("BACKEND GATE blocked /tf (blacklist)")


def _run(blueprint: object, coro: object) -> None:
    coordinator = ModuleCoordinator.build(blueprint)  # type: ignore[arg-type]
    try:
        time.sleep(2.0)
        asyncio.run(coro)  # type: ignore[arg-type]
    finally:
        coordinator.stop()


def main() -> None:
    os.environ["DIMOS_NO_BROWSER"] = "1"  # keep the smoke test headless

    async def phase1() -> None:
        check_web_host()
        await check_streaming()

    _run(
        autoconnect(FakeTfPublisher.blueprint(), DimosWebsocket.blueprint(), TfWebUi.blueprint()),
        phase1(),
    )
    time.sleep(1.0)
    _run(
        autoconnect(
            FakeTfPublisher.blueprint(),
            DimosWebsocket.blueprint(blacklist=["/tf"]),
        ),
        check_backend_gate(),
    )
    print("OK")


if __name__ == "__main__":
    main()
