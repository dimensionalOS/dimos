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
from datetime import datetime, timedelta, timezone
from ipaddress import IPv4Address
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any
import zlib

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID
import numpy as np
import pytest

from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.relay_bridge.e2e_support import (
    arm_teleop,
    attach_viewer,
    collect_until,
    stop_module,
)
from dimos.web.relay_bridge.module_test_support import wait_until
from dimos.web.relay_bridge.protocol import (
    Stop as WireStop,
    TeleopStart,
    Twist as WireTwist,
    Unsub,
)
from dimos.web.relay_bridge.relay_bridge_module import (
    RelayBridgeConfig,
    RelayBridgeModule,
    default_manifest,
)
from dimos.web.relay_bridge.relay_process import RelayProcess
from dimos.web.relay_bridge.wt_client import RelayClient, RelayRejectedError, fetch_relay_info

ROBOT_ID = "bridge-e2e"
POSE = PoseStamped(ts=42.5, position=[1.5, -2.5, 0.25], orientation=[0.0, 0.0, 0.0, 1.0])
COSTMAP_GRID = OccupancyGrid(
    grid=np.array([[-1, 0, 50], [100, 0, -1]], dtype=np.int8),
    resolution=0.05,
    origin=Pose(-1.25, 2.5, 0.0),
    ts=42.5,
)
COSTMAP_CELLS = bytes([255, 0, 50, 100, 0, 255])
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


def _start_bridge() -> tuple[RelayBridgeModule, tuple[pLCMTransport, ...]]:
    # web_build=False: tests must never trigger the npm-downloading build.
    module = RelayBridgeModule(local_port=0, open_browser=False, web_build=False, robot_id=ROBOT_ID)
    transports = (
        pLCMTransport("/rb_e2e/odom"),
        pLCMTransport("/rb_e2e/color_image"),
        pLCMTransport("/rb_e2e/global_costmap"),
    )
    for transport in transports:
        transport.start()
    module.odom.transport = transports[0]
    module.color_image.transport = transports[1]
    module.global_costmap.transport = transports[2]
    module.start()  # spawns the Deno relay, connects, registers
    return module, transports


async def _viewer(bridge: RelayBridgeModule) -> RelayClient:
    """A viewer on the bridge's relay, discovered the way the bridge does."""
    assert bridge._url is not None
    info = await fetch_relay_info(bridge._url)
    return await RelayClient.connect(info.wt_url, "viewer", insecure=info.cert_hash is not None)


def _session_live(bridge: RelayBridgeModule) -> bool:
    session = bridge._session
    return session is not None and not session.client.is_closed


@pytest.fixture(scope="module")
def bridge() -> Iterator[RelayBridgeModule]:
    module, _ = _start_bridge()
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
    module, transports = _start_bridge()
    try:
        yield module
    finally:
        stop_module(module)
        for transport in transports:
            transport.stop()


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
            module._spawn_relay(False, None)

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
        async with await _viewer(bridge) as viewer:
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


class _CostmapPublisher:
    """Stoppable costmap publisher: the resend tests must prove a frame
    arrives with no producer running, so it cannot ride the always-on
    _Publisher."""

    def __init__(self, grid: OccupancyGrid = COSTMAP_GRID) -> None:
        self.grid = grid
        self.transport = pLCMTransport("/rb_e2e/global_costmap")
        self.stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.transport.start()
        self._thread.start()

    def _run(self) -> None:
        while not self.stop.is_set():
            self.transport.publish(self.grid)
            time.sleep(0.05)

    def close(self) -> None:
        self.stop.set()
        self._thread.join(timeout=2)
        self.transport.stop()


async def _watch_costmap(bridge: RelayBridgeModule, expected_cells: bytes = COSTMAP_CELLS) -> Any:
    """Attach a fresh viewer, wait for one costmap frame, return that frame."""
    async with await _viewer(bridge) as viewer:
        await viewer.hello()
        await attach_viewer(viewer, ROBOT_ID, ["global_costmap"])
        frames = await collect_until(
            viewer,
            lambda fs: any(f.header.ch == "global_costmap" for f in fs),
            timeout=15.0,
        )
        frame = next(f for f in frames if f.header.ch == "global_costmap")
        assert frame.header.delivery == "latest"
        meta = frame.header.meta
        assert meta is not None
        assert (meta["w"], meta["h"]) == (3, 2)
        # The LCM leg narrows resolution to float32; exact 0.05 is not owed.
        assert meta["res"] == pytest.approx(0.05)
        assert meta["origin"][:2] == [-1.25, 2.5]
        assert meta["origin"][2] == pytest.approx(0.0, abs=1e-12)
        assert zlib.decompress(bytes(frame.payload)) == expected_cells
        return frame


def _wait_costmap_unsubscribed(bridge: RelayBridgeModule) -> None:
    """Wait until the relay reported the shrunken sub set and encoding stopped."""
    deadline = time.monotonic() + 10
    while (
        bridge._session is not None
        and "global_costmap" in bridge._session.unsubs
        and time.monotonic() < deadline
    ):
        time.sleep(0.05)
    assert bridge._session is not None
    assert "global_costmap" not in bridge._session.unsubs, "bridge never heard the unsub"


def test_costmap_full_grid_arrives_and_resends_on_subscribe(bridge: RelayBridgeModule) -> None:
    publisher = _CostmapPublisher()
    publisher.start()
    try:
        first = asyncio.run(_watch_costmap(bridge))
    finally:
        publisher.close()

    _wait_costmap_unsubscribed(bridge)

    # A fresh subscription with the producer stopped: the bridge must replay
    # the cached message instead of waiting for a publish that never comes.
    encoded_before = bridge.encoded["global_costmap"]
    second = asyncio.run(_watch_costmap(bridge))
    assert bytes(second.payload) == bytes(first.payload)
    # Re-encoded from the raw cache; the counter tracks live encodes only.
    assert bridge.encoded["global_costmap"] == encoded_before


def test_costmap_replay_reflects_publishes_while_unwatched(bridge: RelayBridgeModule) -> None:
    # A different grid published with zero viewers must land in the raw cache,
    # so the next subscriber gets it - stamped with its arrival time (honest
    # staleness on the wire), not the replay time.
    _wait_costmap_unsubscribed(bridge)
    grid_b = OccupancyGrid(
        grid=np.array([[100, 100, 100], [0, 0, -1]], dtype=np.int8),
        resolution=0.05,
        origin=Pose(-1.25, 2.5, 0.0),
        ts=43.0,
    )
    t0 = time.time()
    publisher = _CostmapPublisher(grid_b)
    publisher.start()
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            cached = bridge._last_msg.get("global_costmap")
            if cached is not None and cached[0].grid[0, 0] == 100:
                break
            time.sleep(0.05)
    finally:
        publisher.close()
    t1 = time.time()
    cached = bridge._last_msg.get("global_costmap")
    assert cached is not None and cached[0].grid[0, 0] == 100, "cache never saw grid B"

    frame = asyncio.run(_watch_costmap(bridge, expected_cells=bytes([100, 100, 100, 0, 0, 255])))
    assert t0 <= frame.header.ts <= t1


def test_relay_child_death_respawns_and_recovers(
    respawn_bridge: RelayBridgeModule, publisher: _Publisher
) -> None:
    bridge = respawn_bridge
    assert bridge._relay is not None and bridge._relay._process is not None
    assert bridge._relay_info is not None
    old_wt_url = bridge._relay_info.wt_url
    bridge._relay._process.kill()  # SIGKILL: no CONNECTION_CLOSE reaches the bridge

    # The child watchdog notices, the supervisor respawns the relay (new QUIC
    # port + cert), rediscovers it through /api/info and reconnects.
    def rediscovered() -> bool:
        info = bridge._relay_info
        return info is not None and info.wt_url != old_wt_url

    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if rediscovered() and bridge._relay.poll() is None and _session_live(bridge):
            break
        time.sleep(0.1)
    assert rediscovered(), "relay child was never respawned"

    async def flow() -> None:
        # A fresh viewer attaches to the new relay and frames flow again.
        async with await _viewer(bridge) as viewer:
            await viewer.hello()
            await attach_viewer(viewer, ROBOT_ID, ["odom"])
            frames = await collect_until(
                viewer, lambda fs: any(f.header.ch == "odom" for f in fs), timeout=15.0
            )
            assert any(f.header.ch == "odom" for f in frames)

    asyncio.run(flow())


# Teleop e2e: viewer datagrams -> relay lease gate -> bridge publishes.


def _start_teleop_bridge() -> tuple[RelayBridgeModule, tuple[pLCMTransport, ...]]:
    manifest = default_manifest(
        RelayBridgeConfig(), ("color_image", "odom", "global_costmap", "tele_cmd_vel")
    )
    module = RelayBridgeModule(
        local_port=0,
        open_browser=False,
        web_build=False,
        robot_id=ROBOT_ID,
        manifest=manifest,
    )
    transports = (
        pLCMTransport("/rb_e2e/odom"),
        pLCMTransport("/rb_e2e/color_image"),
        pLCMTransport("/rb_e2e/global_costmap"),
    )
    for transport in transports:
        transport.start()
    module.odom.transport = transports[0]
    module.color_image.transport = transports[1]
    module.global_costmap.transport = transports[2]
    module.start()
    return module, transports


@pytest.fixture
def teleop_bridge() -> Iterator[RelayBridgeModule]:
    """Function-scoped: teleop lease state must not leak between tests."""
    module, transports = _start_teleop_bridge()
    try:
        yield module
    finally:
        stop_module(module)
        for transport in transports:
            transport.stop()


async def _until(cond, what: str, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return
        await asyncio.sleep(0.02)
    raise TimeoutError(what)


async def _drive(viewer: RelayClient, seq: int, stop: asyncio.Event, vx: float = 0.5) -> int:
    """Send nonzero twists at ~20 Hz until stopped; returns the last seq used."""
    while not stop.is_set():
        seq += 1
        try:
            viewer.send_control(WireTwist(vx=vx, vy=0.0, wz=0.0, seq=seq, ts=time.time()))
        except Exception:
            break  # dead relay: sends are best-effort, the watchdog owns safety
        await asyncio.sleep(0.05)
    return seq


def test_teleop_drive_and_estop(teleop_bridge: RelayBridgeModule) -> None:
    bridge = teleop_bridge
    twists: list[Twist] = []
    bridge.tele_cmd_vel.subscribe(twists.append)

    async def flow() -> None:
        async with await _viewer(bridge) as viewer:
            await viewer.hello()
            await attach_viewer(viewer, ROBOT_ID, [])
            await arm_teleop(viewer)

            stop = asyncio.Event()
            driver = asyncio.create_task(_drive(viewer, 0, stop))
            await _until(lambda: any(not t.is_zero() for t in twists), "no twist published")

            # E-stop: the stop datagram publishes an unconditional zero.
            stop.set()
            seq = await driver
            marker = len(twists)
            viewer.send_control(WireStop(seq=seq + 1, ts=time.time()))
            await _until(
                lambda: len(twists) > marker and twists[-1].is_zero(), "no zero after stop"
            )

    asyncio.run(flow())


@pytest.fixture
def deadman_bridge() -> Iterator[tuple[RelayProcess, RelayBridgeModule]]:
    """A teleop bridge attached to an externally spawned relay. With no local
    child there is no 1 s child watchdog, and a SIGKILLed relay sends no
    CONNECTION_CLOSE so the session-teardown zero stays out of reach: the
    teleop deadman is the only path that can stop the robot."""
    relay = RelayProcess()
    ready = relay.start()
    manifest = default_manifest(RelayBridgeConfig(), ("tele_cmd_vel",))
    module = RelayBridgeModule(
        relay_url=ready.open_url,
        open_browser=False,
        web_build=False,
        robot_id=ROBOT_ID,
        manifest=manifest,
    )
    module.start()
    try:
        yield relay, module
    finally:
        stop_module(module)
        relay.stop()


def test_teleop_relay_kill_deadman_deadline(
    deadman_bridge: tuple[RelayProcess, RelayBridgeModule],
) -> None:
    relay, bridge = deadman_bridge
    twists: list[Twist] = []
    bridge.tele_cmd_vel.subscribe(twists.append)

    async def flow() -> None:
        async with await _viewer(bridge) as viewer:
            await viewer.hello()
            await attach_viewer(viewer, ROBOT_ID, [])
            await arm_teleop(viewer)

            stop = asyncio.Event()
            driver = asyncio.create_task(_drive(viewer, 0, stop))
            await _until(lambda: any(not t.is_zero() for t in twists), "no twist published")

            marker = len(twists)
            assert relay._process is not None
            killed_at = time.monotonic()
            relay._process.kill()
            await _until(
                lambda: len(twists) > marker and twists[-1].is_zero(),
                "no zero after relay kill (deadman broken?)",
                timeout=5.0,
            )
            # The deadline enforces the deadman itself: watchdogMs 300
            # (default) + _TELEOP_POLL_S 0.05 + CI scheduling margin.
            assert time.monotonic() - killed_at < 1.0
            stop.set()
            await driver

    asyncio.run(flow())


def test_teleop_lease_exclusive_and_handover(teleop_bridge: RelayBridgeModule) -> None:
    bridge = teleop_bridge
    twists: list[Twist] = []
    bridge.tele_cmd_vel.subscribe(twists.append)

    async def flow() -> None:
        async with await _viewer(bridge) as second:
            await second.hello()
            await attach_viewer(second, ROBOT_ID, [])
            async with await _viewer(bridge) as holder:
                await holder.hello()
                await attach_viewer(holder, ROBOT_ID, [])
                await arm_teleop(holder)

                # The second viewer is refused while the lease is held (the
                # error reply lands in relay_error; retried because replies
                # ride lossy datagrams).
                async def held() -> bool:
                    second.send_control(TeleopStart())
                    await asyncio.sleep(0.1)
                    error = second._session.relay_error
                    return error is not None and error.code == "teleop_held"

                deadline = time.monotonic() + 10
                while not await held():
                    assert time.monotonic() < deadline, "teleop_held never reported"

                # The holder drives; the bystander's twists are gated out.
                second.send_control(WireTwist(vx=9.0, vy=0.0, wz=0.0, seq=1, ts=time.time()))
                holder.send_control(WireTwist(vx=0.5, vy=0.0, wz=0.0, seq=1, ts=time.time()))
                await _until(lambda: any(not t.is_zero() for t in twists), "no twist published")
                assert all(t.linear.x != 9.0 for t in twists)

            # Holder disconnected mid-drive: the relay releases the lease and
            # sends teleop_stop, so the bridge zeroes without waiting for the
            # watchdog, and the second viewer can now arm and drive.
            await _until(lambda: twists[-1].is_zero(), "no zero after holder disconnect")
            await arm_teleop(second)
            marker = len(twists)
            second.send_control(WireTwist(vx=0.25, vy=0.0, wz=0.0, seq=1, ts=time.time()))
            await _until(
                lambda: len(twists) > marker and twists[-1].linear.x == 0.25,
                "handover drive never published",
            )

    asyncio.run(flow())


# --relay-url: a relay started by hand.


def _external_bridge(
    relay_url: str, relay_ca: str | None = None, relay_key: str | None = None
) -> RelayBridgeModule:
    """A bridge attached to a relay it did not spawn (the --relay-url path)."""
    return RelayBridgeModule(
        relay_url=relay_url,
        relay_ca=relay_ca,
        relay_key=relay_key,
        open_browser=False,
        web_build=False,
        robot_id=ROBOT_ID,
        manifest=default_manifest(RelayBridgeConfig(), ("odom",)),
    )


def test_external_relay_restart_reattaches() -> None:
    # The relay's HTTP port is its only stable coordinate: a restart brings a
    # new QUIC port and certificate, rediscovered through /api/info.
    relay = RelayProcess()
    ready = relay.start()
    bridge = _external_bridge(ready.open_url)
    restarted: RelayProcess | None = None
    try:
        bridge.start()
        assert bridge._relay_info is not None and bridge._relay_info.wt_url == ready.wt_url

        relay.stop()  # SIGTERM: the relay closes the session before exiting
        assert wait_until(lambda: bridge._session is None, timeout=10.0), "session loss unnoticed"
        restarted = RelayProcess(port=ready.http_port)
        ready2 = restarted.start()
        assert ready2.wt_url != ready.wt_url

        def reattached() -> bool:
            info = bridge._relay_info
            return info is not None and info.wt_url == ready2.wt_url and _session_live(bridge)

        assert wait_until(reattached, timeout=30.0), "bridge never reattached"

        async def flow() -> None:
            async with await _viewer(bridge) as viewer:
                await viewer.hello()
                await attach_viewer(viewer, ROBOT_ID, [])  # its manifest proves registration

        asyncio.run(flow())
    finally:
        stop_module(bridge)
        relay.stop()
        if restarted is not None:
            restarted.stop()


def _self_signed_cert(directory: Path) -> tuple[Path, Path]:
    """A one-day P-256 certificate for 127.0.0.1, its own trust anchor, as
    PEM files (certificate, PKCS#8 key)."""
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "dimos relay e2e")])
    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(hours=1))
        .not_valid_after(now + timedelta(days=1))
        .add_extension(
            x509.SubjectAlternativeName([x509.IPAddress(IPv4Address("127.0.0.1"))]),
            critical=False,
        )
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
        .sign(key, hashes.SHA256())
    )
    cert_path = directory / "relay.pem"
    key_path = directory / "relay-key.pem"
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    key_path.write_bytes(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        )
    )
    return cert_path, key_path


def test_external_relay_with_real_certificate(tmp_path: Path) -> None:
    # A relay given --cert/--key advertises no hash, so both client legs
    # verify the certificate: against relay_ca (mkcert, a private CA) or the
    # default trust stores. Never insecure=True.
    cert, key = _self_signed_cert(tmp_path)
    with RelayProcess(cert=cert, key=key) as ready:
        assert ready.cert_hash is None
        assert ready.open_url == f"https://127.0.0.1:{ready.http_port}/"
        wt_url = f"https://127.0.0.1:{ready.http_port}"

        async def unverified() -> None:
            # The default stores do not know this certificate.
            with pytest.raises(OSError, match="CERTIFICATE_VERIFY_FAILED"):
                await fetch_relay_info(ready.open_url)
            with pytest.raises(ConnectionError):
                await RelayClient.connect(wt_url, "robot", insecure=False)

        asyncio.run(unverified())

        bridge = _external_bridge(ready.open_url, relay_ca=str(cert))
        try:
            bridge.start()  # returns only after hello/welcome: registered
            info = bridge._relay_info
            assert info is not None and info.cert_hash is None and info.wt_url == wt_url
            assert _session_live(bridge)
        finally:
            stop_module(bridge)


def test_external_relay_with_auth(tmp_path: Path) -> None:
    # --auth-file: relay_key must be the key bound to the bridge's robot id. A
    # wrong one fails start with auth_failed (no retry loop); the right one
    # registers.
    auth_file = tmp_path / "auth.json"
    auth_file.write_text(
        json.dumps({"robots": {ROBOT_ID: "robot-key-e2e-0123456789abcdef"}, "viewers": {}})
    )
    with RelayProcess(auth_file=auth_file) as ready:
        wrong = _external_bridge(ready.open_url, relay_key="wrong-key-e2e-0123456789abcdef")
        try:
            with pytest.raises(RelayRejectedError, match="auth_failed: invalid robot key"):
                wrong.start()
        finally:
            stop_module(wrong)

        bridge = _external_bridge(ready.open_url, relay_key="robot-key-e2e-0123456789abcdef")
        try:
            bridge.start()  # returns only after hello/welcome: registered
            assert _session_live(bridge)
        finally:
            stop_module(bridge)


def test_start_waits_out_robot_id_conflict_on_relay(monkeypatch: pytest.MonkeyPatch) -> None:
    # The same robot id twice on one relay: the second bridge's start waits
    # until the first lets go (a killed predecessor expires at the relay's
    # idle timeout; a clean stop frees the id at once).
    relay = RelayProcess()
    ready = relay.start()
    first = _external_bridge(ready.open_url)
    second = _external_bridge(ready.open_url)
    starter = threading.Thread(target=second.start, daemon=True)
    conflict_seen = threading.Event()
    real_hello = RelayClient.hello

    async def observed_hello(self: RelayClient, *args: Any, **kwargs: Any) -> None:
        try:
            await real_hello(self, *args, **kwargs)
        except RelayRejectedError as error:
            if error.code == "robot_id_conflict":
                conflict_seen.set()
            raise

    monkeypatch.setattr(RelayClient, "hello", observed_hello)
    first_stopped = False
    try:
        first.start()
        starter.start()
        assert conflict_seen.wait(timeout=10.0), "no conflict seen"
        assert starter.is_alive() and not _session_live(second)
        stop_module(first)
        first_stopped = True
        starter.join(timeout=15.0)
        assert not starter.is_alive(), "second bridge never registered"
        assert _session_live(second)
    finally:
        stop_module(second)
        if not first_stopped:
            stop_module(first)
        relay.stop()
