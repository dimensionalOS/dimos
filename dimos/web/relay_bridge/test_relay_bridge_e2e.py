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

"""End-to-end: a real RelayBridgeModule spawning a real relay child.

The module runs standalone with hand-wired LCM transports (no coordinator);
a Python viewer drives the full session flow: robots -> watch -> manifest ->
sub -> decoded frames, plus lazy-encode stop on unsub and relay-child-death
recovery. One file on purpose: --dist=loadfile keeps the module-scoped
module + relay on a single xdist worker. The child-kill test gets its own
function-scoped bridge: it destroys the relay it is given.

Tests are sync and drive their viewer flows via asyncio.run: constructing a
Module rebinds the constructing thread's current event loop (module.py
get_loop), which corrupts pytest-asyncio's function-scoped loop teardown.
"""

import asyncio
from collections.abc import Iterator
import json
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.relay_bridge.e2e_support import attach_viewer, collect_until, stop_module
from dimos.web.relay_bridge.protocol import Unsub
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule
from dimos.web.relay_bridge.wt_client import RelayClient

ROBOT_ID = "bridge-e2e"
POSE = PoseStamped(ts=42.5, position=[1.5, -2.5, 0.25], orientation=[0.0, 0.0, 0.0, 1.0])
_LISTENER = """
import socket
import time

listener = socket.socket()
listener.bind(("127.0.0.1", 0))
listener.listen()
print(listener.getsockname()[1], flush=True)
time.sleep(60)
"""


class _Publisher:
    """Publishes odom + color_image on their LCM topics from a daemon thread
    (frames flow only once the bridge lazily subscribes, so a single publish
    is never enough - keep them coming like a robot would)."""

    def __init__(self) -> None:
        self.odom = pLCMTransport("/rb_e2e/odom")
        self.image = pLCMTransport("/rb_e2e/color_image")
        self.stop = threading.Event()
        arr = np.zeros((48, 64, 3), dtype=np.uint8)
        arr[:, :, 1] = np.linspace(0, 255, 64, dtype=np.uint8)
        self._image = Image.from_numpy(arr)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.odom.start()
        self.image.start()
        self._thread.start()

    def _run(self) -> None:
        while not self.stop.is_set():
            self.odom.publish(POSE)
            self.image.publish(self._image)
            time.sleep(0.05)

    def close(self) -> None:
        self.stop.set()
        self._thread.join(timeout=2)
        self.odom.stop()
        self.image.stop()


def _start_bridge() -> tuple[RelayBridgeModule, pLCMTransport, pLCMTransport]:
    # cockpit_build=False: tests must never trigger the npm-downloading build.
    module = RelayBridgeModule(
        local_port=0, open_browser=False, cockpit_build=False, robot_id=ROBOT_ID
    )
    odom_tr = pLCMTransport("/rb_e2e/odom")
    image_tr = pLCMTransport("/rb_e2e/color_image")
    odom_tr.start()
    image_tr.start()
    module.odom.transport = odom_tr
    module.color_image.transport = image_tr
    module.start()  # spawns the Deno relay, connects, registers
    return module, odom_tr, image_tr


@pytest.fixture(scope="module")
def bridge() -> Iterator[RelayBridgeModule]:
    module, _, _ = _start_bridge()
    try:
        yield module
    finally:
        module.stop()


@pytest.fixture
def respawn_bridge() -> Iterator[RelayBridgeModule]:
    """Function-scoped bridge for the relay-child-kill test.

    The respawn creates a fresh relay child mid-test whose reader threads live
    as long as the bridge, so a shared module-scoped bridge would trip the
    conftest thread-leak check. Owning the bridge scopes those threads to the
    test, with everything reaped here.
    """
    module, odom_tr, image_tr = _start_bridge()
    try:
        yield module
    finally:
        stop_module(module)
        odom_tr.stop()
        image_tr.stop()


@pytest.fixture(scope="module")
def publisher() -> Iterator[_Publisher]:
    pub = _Publisher()
    pub.start()
    try:
        yield pub
    finally:
        pub.close()


def test_local_relay_port_collision_does_not_kill_listener() -> None:
    listener = subprocess.Popen(
        [sys.executable, "-c", _LISTENER],
        stdout=subprocess.PIPE,
        text=True,
    )
    module: RelayBridgeModule | None = None
    try:
        assert listener.stdout is not None
        port = int(listener.stdout.readline())
        module = RelayBridgeModule(local_port=port, open_browser=False, robot_id="collision-test")

        with pytest.raises(RuntimeError, match=rf"port {port} is unavailable"):
            module._spawn_relay(False)

        assert listener.poll() is None
    finally:
        if module is not None:
            module.stop()
        listener.terminate()
        try:
            listener.wait(timeout=5)
        except subprocess.TimeoutExpired:
            listener.kill()
            listener.wait(timeout=5)
        if listener.stdout is not None:
            listener.stdout.close()


def test_full_session_flow_and_lazy_encode(
    bridge: RelayBridgeModule, publisher: _Publisher
) -> None:
    async def flow() -> None:
        assert bridge._url is not None
        async with await RelayClient.connect(bridge._url, "viewer") as viewer:
            await viewer.hello()
            # robots push carries the bridge's identity (drain until watch lands).
            await attach_viewer(viewer, ROBOT_ID, ["odom", "color_image"])

            frames = await collect_until(
                viewer,
                lambda fs: any(f.header.ch == "odom" for f in fs)
                and any(f.header.ch == "color_image" for f in fs),
                timeout=15.0,
            )
            odom = next(f for f in frames if f.header.ch == "odom")
            pose = json.loads(odom.payload)
            assert pose == {"x": 1.5, "y": -2.5, "z": 0.25, "yaw": 0.0, "ts": 42.5}
            assert odom.header.delivery == "reliable"

            image = next(f for f in frames if f.header.ch == "color_image")
            assert bytes(image.payload[:2]) == b"\xff\xd8"  # real TurboJPEG output
            assert image.header.meta == {"w": 64, "h": 48}
            assert image.header.delivery == "latest"

            # Unsub color_image: the bridge must stop encoding it entirely
            # while odom (still subscribed) keeps flowing.
            viewer.send_control(Unsub(ch="color_image"))
            deadline = time.monotonic() + 10
            while (
                bridge._session is not None
                and "color_image" in bridge._session.unsubs
                and time.monotonic() < deadline
            ):
                await asyncio.sleep(0.05)
            assert bridge._session is not None
            assert "color_image" not in bridge._session.unsubs, "bridge never heard the unsub"

            encoded_before = bridge.encoded["color_image"]
            await asyncio.sleep(0.5)  # publisher keeps publishing the whole time
            assert bridge.encoded["color_image"] == encoded_before
            assert "odom" in bridge._session.unsubs

    asyncio.run(flow())


def test_relay_child_death_respawns_and_recovers(
    respawn_bridge: RelayBridgeModule, publisher: _Publisher
) -> None:
    bridge = respawn_bridge
    assert bridge._relay is not None and bridge._relay._process is not None
    old_url = bridge._url
    bridge._relay._process.kill()  # SIGKILL: no CONNECTION_CLOSE reaches the bridge

    # The child watchdog notices, the supervisor respawns the relay (new QUIC
    # port + cert) and reconnects.
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        session = bridge._session
        if (
            bridge._url != old_url
            and bridge._relay.poll() is None
            and session is not None
            and not session.client.is_closed
        ):
            break
        time.sleep(0.1)
    assert bridge._url != old_url, "relay child was never respawned"

    async def flow() -> None:
        # A fresh viewer attaches to the new relay and frames flow again.
        assert bridge._url is not None
        async with await RelayClient.connect(bridge._url, "viewer") as viewer:
            await viewer.hello()
            await attach_viewer(viewer, ROBOT_ID, ["odom"])
            frames = await collect_until(
                viewer, lambda fs: any(f.header.ch == "odom" for f in fs), timeout=15.0
            )
            assert any(f.header.ch == "odom" for f in frames)

    asyncio.run(flow())
