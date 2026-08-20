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

"""End-to-end tests against a real relay child process (aioquic both legs).

One file on purpose: --dist=loadfile keeps the module-scoped relay on a
single xdist worker.
"""

import asyncio
from collections.abc import AsyncIterator, Iterator, Sequence
import hashlib
import json
import statistics
import time
import urllib.request

import pytest

from dimos.web.relay_bridge.e2e_support import attach_viewer, collect_until, wait_subs
from dimos.web.relay_bridge.protocol import (
    DataFrame,
    FrameHeader,
    Hello,
    RobotInfo,
    RobotManifest,
    Sub,
    Unsub,
    Watch,
    encode_datagram,
)
from dimos.web.relay_bridge.relay_process import RelayProcess, RelayReadyInfo
from dimos.web.relay_bridge.wt_client import RelayClient, RelayRejectedError

# One robot identity for the whole module; function-scoped clients close before
# the next test registers it. No manifest on purpose: undeclared channels take
# the frame header's delivery, which is what these transport tests steer with.
ROBOT = RobotInfo(id="e2e-bot", name="E2E Bot", model="test")


@pytest.fixture(scope="module")
def relay() -> Iterator[RelayReadyInfo]:
    process = RelayProcess()
    try:
        yield process.start()
    finally:
        process.stop()


@pytest.fixture
def own_relay() -> Iterator[RelayProcess]:
    """A started relay process the test may stop itself (stop() is idempotent)."""
    process = RelayProcess()
    try:
        process.start()
        yield process
    finally:
        process.stop()


@pytest.fixture
async def robot(relay: RelayReadyInfo) -> AsyncIterator[RelayClient]:
    """A connected robot client with the hello handshake done."""
    async with await RelayClient.connect(relay.wt_url, "robot") as client:
        await client.hello(robot=ROBOT)
        yield client


@pytest.fixture
async def viewer(relay: RelayReadyInfo) -> AsyncIterator[RelayClient]:
    """A connected viewer client with the hello handshake done."""
    async with await RelayClient.connect(relay.wt_url, "viewer") as client:
        await client.hello()
        yield client


async def attach(robot: RelayClient, viewer: RelayClient, chs: Sequence[str]) -> None:
    """Standard preamble on top of the fixtures' hellos: viewer watch+sub, subs barrier."""
    await attach_viewer(viewer, ROBOT.id, chs)
    await wait_subs(robot, set(chs))


async def fetch_stats(relay: RelayReadyInfo) -> dict:
    def _get() -> dict:
        with urllib.request.urlopen(f"http://127.0.0.1:{relay.http_port}/api/stats") as response:
            return json.load(response)

    return await asyncio.to_thread(_get)


def test_info_matches_ready_line(relay: RelayReadyInfo) -> None:
    with urllib.request.urlopen(f"http://127.0.0.1:{relay.http_port}/api/info") as response:
        info = json.load(response)
    assert info == {"wtUrl": f"{relay.wt_url}/viewer", "certHash": relay.cert_hash, "v": relay.v}
    assert relay.wt_url.startswith("https://127.0.0.1:")


async def test_robot_handshake_and_datagram_rtt(relay: RelayReadyInfo) -> None:
    # Connects manually: the hello handshake itself is under test here.
    async with await RelayClient.connect(relay.wt_url, "robot") as robot:
        await robot.hello(robot=ROBOT)
        rtts = [await robot.ping() for _ in range(20)]
    assert statistics.median(rtts) < 0.1


async def test_robot_hello_without_identity_is_rejected(relay: RelayReadyInfo) -> None:
    async with await RelayClient.connect(relay.wt_url, "robot") as robot:
        with pytest.raises(Exception, match="missing_robot_id"):
            await robot.hello()


async def test_datagram_hello_from_an_old_bridge_is_rejected(relay: RelayReadyInfo) -> None:
    # v5 moved the robot hello onto @control stream frames; a datagram hello
    # is how every v4-or-older bridge announces itself and must fail the
    # handshake loudly. Datagrams are lossy, so resend until the error lands.
    async with await RelayClient.connect(relay.wt_url, "robot") as old:
        deadline = time.monotonic() + 5.0
        while old._session.relay_error is None and time.monotonic() < deadline:
            old.send_control(Hello(v=1, role="robot", robot=ROBOT))
            await asyncio.sleep(0.05)
        error = old._session.relay_error
        assert error is not None and error.code == "version_mismatch"


async def test_fat_manifest_registers_beyond_the_datagram_budget(relay: RelayReadyInfo) -> None:
    # The whole point of the v5 stream hello: a manifest far beyond the old
    # ~1100 B datagram budget registers, gates subs, and forwards frames.
    manifest: RobotManifest = {
        "version": 1,
        "channels": [
            {
                "ch": f"ch_{i}",
                "encoding": "pose.json.v1",
                "delivery": "reliable",
                "maxHz": 10.5,
                "params": {"note": f"padding for channel {i} so the hello dwarfs a datagram"},
            }
            for i in range(40)
        ],
    }
    fat_robot = RobotInfo(id="fat-bot", name="Fat Bot", model="test")
    hello_size = len(encode_datagram(Hello(v=5, role="robot", robot=fat_robot, manifest=manifest)))
    assert hello_size > 1200, f"fat hello must exceed the datagram budget, got {hello_size} B"

    async with await RelayClient.connect(relay.wt_url, "robot") as robot:
        await robot.hello(robot=fat_robot, manifest=manifest)
        async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
            await viewer.hello()
            # This test viewer's control plane is datagrams, and the fat
            # manifest reply exceeds a datagram, so the usual manifest-ack
            # ordering barrier is unavailable (a browser viewer receives it
            # on its control stream - pinned by the Deno-side fat test).
            # Retry the idempotent watch+sub pair until the snapshot lands.
            deadline = time.monotonic() + 10.0
            while True:
                viewer.send_control(Watch(robotId=fat_robot.id))
                viewer.send_control(Sub(ch="ch_7"))
                try:
                    await wait_subs(robot, {"ch_7"}, timeout=1.0)
                    break
                except TimeoutError:
                    if time.monotonic() >= deadline:
                        raise
            # The sub was accepted only because the fat manifest declared
            # ch_7: an undeclared channel is refused, proving the relay
            # parsed and enforces the whole manifest.
            viewer.send_control(Sub(ch="undeclared"))
            while viewer._session.relay_error is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError("no unknown_channel error for the undeclared sub")
                await asyncio.sleep(0.05)
            assert viewer._session.relay_error.code == "unknown_channel"

            robot.send_frame("ch_7", b"fat-frame", delivery="reliable")
            frames = await collect_until(viewer, lambda fs: any(f.header.ch == "ch_7" for f in fs))
            assert [bytes(f.payload) for f in frames if f.header.ch == "ch_7"] == [b"fat-frame"]


async def test_invalid_manifest_hello_is_rejected(relay: RelayReadyInfo) -> None:
    # version: 1 on purpose, so this keeps pinning the duplicate-channel rule
    # rather than the version gate.
    duplicated: RobotManifest = {
        "version": 1,
        "channels": [
            {"ch": "odom", "encoding": "pose.json.v1", "delivery": "reliable", "maxHz": 20.0},
            {"ch": "odom", "encoding": "jpeg.v1", "delivery": "latest", "maxHz": 15.0},
        ],
    }
    async with await RelayClient.connect(relay.wt_url, "robot") as robot:
        with pytest.raises(RelayRejectedError) as exc_info:
            await robot.hello(robot=ROBOT, manifest=duplicated)
        assert exc_info.value.code == "invalid_manifest"


async def test_reliable_channel_is_complete_and_intact(
    robot: RelayClient, viewer: RelayClient
) -> None:
    await attach(robot, viewer, ["odom"])
    count = 100
    payloads = [seq.to_bytes(4, "little") * 256 for seq in range(count)]
    for seq, payload in enumerate(payloads):
        robot.send_frame("odom", payload, delivery="reliable", meta={"i": seq})

    frames = await collect_until(
        viewer,
        lambda fs: len({f.header.seq for f in fs if f.header.ch == "odom"}) >= count,
    )
    odom = {f.header.seq: f for f in frames if f.header.ch == "odom"}
    # Reliable = complete, no drops. One-stream-per-message may reorder;
    # completeness is the contract, headers carry the sequence.
    assert sorted(odom) == list(range(count))
    assert all(bytes(odom[seq].payload) == payloads[seq] for seq in range(count))
    assert odom[0].header.delivery == "reliable"
    assert odom[0].header.meta == {"i": 0}


async def test_latest_channel_newest_wins(robot: RelayClient, viewer: RelayClient) -> None:
    await attach(robot, viewer, ["cam"])
    writer = robot.latest_writer("cam")
    offered = 200
    for i in range(offered):
        writer.offer(i.to_bytes(4, "little") + b"\xab" * 2000)
        # Yield so the pump interleaves with the offers; without this all
        # 200 land in one loop turn and the mailbox collapses to sent=1,
        # never exercising the concurrent send-while-in-flight path.
        await asyncio.sleep(0)

    def newest_arrived(frames: list[DataFrame]) -> bool:
        return any(
            f.header.ch == "cam" and f.payload[:4] == (offered - 1).to_bytes(4, "little")
            for f in frames
        )

    frames = await collect_until(viewer, newest_arrived)
    cam = [f for f in frames if f.header.ch == "cam"]
    markers = [int.from_bytes(bytes(f.payload[:4]), "little") for f in cam]
    # The newest offered frame always lands; the mailbox shed the rest.
    assert newest_arrived(frames), f"newest frame missing; got markers {markers}"
    assert writer.dropped + writer.sent == offered
    assert 0 < len(cam) <= offered
    # The interleaving must actually exercise multiple sends (the old
    # single-turn version guaranteed sent==1).
    assert writer.sent >= 2, f"pump never interleaved; sent={writer.sent}"
    # Everything the writer actually sent arrived (loopback: no transport loss).
    assert len(cam) == writer.sent


async def test_large_frame_1mib(robot: RelayClient, viewer: RelayClient) -> None:
    await attach(robot, viewer, ["blob"])
    payload = bytes(range(256)) * 4096  # 1 MiB
    robot.send_frame("blob", payload, delivery="reliable")
    frames = await collect_until(viewer, lambda fs: any(f.header.ch == "blob" for f in fs))
    blob = next(f for f in frames if f.header.ch == "blob")
    assert len(blob.payload) == len(payload)
    assert hashlib.sha256(blob.payload).hexdigest() == hashlib.sha256(payload).hexdigest()


async def test_reset_stale_discards_partial_frame(robot: RelayClient, viewer: RelayClient) -> None:
    """A reset mid-frame must drop the partial on the relay and nothing else."""
    await attach(robot, viewer, ["cam"])
    # 8 MiB cannot be flushed + ACKed within the same event-loop turn, so
    # the reset below reliably lands mid-transfer.
    big = robot.send_frame("cam", b"\xcd" * (8 * 1024 * 1024), delivery="latest")
    assert robot._session.reset_if_in_flight(big)
    small = b"\x01\x02\x03\x04" * 8
    robot.send_frame("cam", small, delivery="latest")

    frames = await collect_until(viewer, lambda fs: any(f.header.ch == "cam" for f in fs))
    cam = [f for f in frames if f.header.ch == "cam"]
    assert [bytes(f.payload) for f in cam] == [small]

    # The relay survived the reset: control still answers.
    assert await robot.ping() < 5.0


async def test_reset_burst_does_not_wedge_robot_leg(
    robot: RelayClient, viewer: RelayClient
) -> None:
    """Resets racing stream acceptance must not kill the relay's robot data path.

    A stream reset before the relay has read its WebTransport preamble errors
    Deno's wt.incomingBidirectionalStreams permanently (rejected pull), which
    used to silently end the robot stream loop. Bursting resets in the same
    event-loop turn as the sends makes that race near-certain.
    """
    await attach(robot, viewer, ["cam"])
    for rnd in range(5):
        # The accept glue cannot have read all 50 preambles before the
        # resets land, so some streams are reset pre-acceptance.
        ids = [robot.send_frame("cam", b"\xcd" * (16 * 1024), delivery="latest") for _ in range(50)]
        for stream_id in ids:
            robot._session.reset_if_in_flight(stream_id)
        marker = f"alive-{rnd}".encode()
        robot.send_frame("cam", marker, delivery="latest")

        frames = await collect_until(
            viewer,
            lambda fs, marker=marker: any(bytes(f.payload) == marker for f in fs),
            timeout=5.0,
        )
        assert any(bytes(f.payload) == marker for f in frames), (
            f"robot data path wedged in round {rnd}"
        )


async def test_stats_reflect_traffic(
    relay: RelayReadyInfo, robot: RelayClient, viewer: RelayClient
) -> None:
    await attach(robot, viewer, ["odom"])
    robot.send_frame("odom", b"{}", delivery="reliable")
    await collect_until(viewer, lambda fs: len(fs) >= 1, timeout=5.0)

    stats = await fetch_stats(relay)
    assert {"id": ROBOT.id, "name": ROBOT.name, "model": ROBOT.model} in stats["robots"]
    assert stats["viewers"] >= 1
    assert stats["perRobot"][ROBOT.id]["subs"] == ["odom"]
    # This module's robot declares no manifest, so its traffic lands in the
    # one aggregate bucket: per-channel ingress stats exist only for declared
    # channels (arbitrary ch strings must not grow the map).
    assert stats["perRobot"][ROBOT.id]["channels"] == {}
    undeclared = stats["perRobot"][ROBOT.id]["undeclared"]
    assert undeclared["framesIn"] >= 1
    assert undeclared["bytesIn"] >= 2
    assert isinstance(undeclared["fps"], (int, float))
    viewer_stats = next(v for v in stats["perViewer"] if v["watched"] == ROBOT.id)
    odom = viewer_stats["channels"]["odom"]
    assert odom["sent"] >= 1
    assert odom["bytesOut"] >= 2
    # A healthy reliable channel never resets anything.
    assert (odom["aborted"], odom["expired"], odom["inflight"]) == (0, 0, 0)


async def test_duplicate_robot_id_is_terminal_until_first_disconnects(
    relay: RelayReadyInfo,
) -> None:
    async with await RelayClient.connect(relay.wt_url, "robot") as first:
        await first.hello(robot=ROBOT)
        async with await RelayClient.connect(relay.wt_url, "robot") as second:
            with pytest.raises(RelayRejectedError) as exc_info:
                await second.hello(robot=ROBOT)
            assert exc_info.value.code == "robot_id_conflict"
        assert not first.is_closed
        assert await first.ping() < 5.0

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        stats = await fetch_stats(relay)
        if not stats["robots"]:
            break
        await asyncio.sleep(0.05)
    assert (await fetch_stats(relay))["robots"] == []

    async with await RelayClient.connect(relay.wt_url, "robot") as replacement:
        await replacement.hello(robot=ROBOT)
        assert await replacement.ping() < 5.0


async def test_unsub_stops_forwarding_and_bridge_hears_it(
    robot: RelayClient, viewer: RelayClient
) -> None:
    """Unsub drops the viewer's channel and the robot gets the shrunk snapshot."""
    await attach(robot, viewer, ["cam", "odom"])
    viewer.send_control(Unsub(ch="cam"))
    # The snapshot shrinking to odom-only is both the assertion that the
    # bridge hears the transition and the barrier that unsub was processed.
    await wait_subs(robot, {"odom"}, exact=True)

    robot.send_frame("cam", b"not-forwarded", delivery="latest")
    robot.send_frame("odom", b"forwarded", delivery="reliable")
    frames = await collect_until(viewer, lambda fs: any(f.header.ch == "odom" for f in fs))
    assert [bytes(f.payload) for f in frames if f.header.ch == "odom"] == [b"forwarded"]
    assert [f for f in frames if f.header.ch == "cam"] == []


async def test_send_frame_paces_with_wait_delivered(robot: RelayClient) -> None:
    start = time.monotonic()
    stream_id = robot.send_frame("odom", b"x" * 1000, delivery="reliable")
    assert await robot.wait_delivered(stream_id, timeout=5.0)
    assert time.monotonic() - start < 5.0


async def test_malformed_robot_frame_is_dropped(
    relay: RelayReadyInfo, robot: RelayClient, viewer: RelayClient
) -> None:
    """A well-framed frame with an invalid header is dropped, not fatal."""
    await attach(robot, viewer, ["cam"])
    before = (await fetch_stats(relay)).get("framesDropped", 0)
    # The client validates delivery, so model_construct skips it to put a
    # bogus value on the wire; the relay's validator must reject it.
    bad_header = FrameHeader.model_construct(ch="cam", seq=0, ts=time.time(), delivery="bogus")
    bad_id = robot._session.send_frame(bad_header, b"junk")
    assert await robot.wait_delivered(bad_id, timeout=5.0)
    # A following valid frame proves the channel still forwards.
    robot.send_frame("cam", b"good", delivery="reliable")
    frames = await collect_until(viewer, lambda fs: any(f.header.ch == "cam" for f in fs))
    cam = [bytes(f.payload) for f in frames if f.header.ch == "cam"]
    assert cam == [b"good"], f"only the valid frame should forward, got {cam}"

    # The drop was counted (poll: onRobotFrame runs just after the ACK).
    after = before
    for _ in range(100):
        after = (await fetch_stats(relay)).get("framesDropped", 0)
        if after - before >= 1:
            break
        await asyncio.sleep(0.05)
    assert after - before == 1
    # The session survived the bad frame: control still answers.
    assert await robot.ping() < 5.0


async def test_latest_writer_resets_stale_stream(robot: RelayClient, viewer: RelayClient) -> None:
    """The writer auto-resets an in-flight stream when a newer frame is waiting."""
    await attach(robot, viewer, ["cam"])
    writer = robot.latest_writer("cam", stale_after=0.02)
    # 8 MiB can't flush + ACK within stale_after, so it stays in flight.
    writer.offer(b"\xcd" * (8 * 1024 * 1024))
    # Wait until the pump has begun sending the big frame.
    for _ in range(1000):
        if writer.sent >= 1:
            break
        await asyncio.sleep(0.005)
    assert writer.sent >= 1, "pump never sent the first frame"
    # A newer small frame makes the stalled big stream stale -> reset.
    writer.offer(b"\x01\x02\x03\x04")

    frames = await collect_until(
        viewer, lambda fs: any(f.header.ch == "cam" and len(f.payload) < 100 for f in fs)
    )
    cam = [f for f in frames if f.header.ch == "cam"]
    assert cam, "no cam frame reached the viewer"
    # Only the small frame arrives; the 8 MiB frame was reset mid-flight.
    assert all(len(f.payload) < 100 for f in cam)
    assert writer.resets >= 1, "the stale stream was never reset"
    # The relay survived the reset.
    assert await robot.ping() < 5.0


async def test_close_signal_stops_writer_and_wakes_waiter(own_relay: RelayProcess) -> None:
    """Relay death terminates the connection, wakes wait_closed, stops the pump."""
    async with await RelayClient.connect(own_relay.info.wt_url, "robot") as robot:
        await robot.hello(robot=ROBOT)
        writer = robot.latest_writer("cam")
        writer.offer(b"x" * 1000)
        await asyncio.sleep(0.1)  # let the pump start
        own_relay.stop()  # graceful shutdown sends CONNECTION_CLOSE

        await asyncio.wait_for(robot.wait_closed(), timeout=10.0)
        assert robot.is_closed
        await asyncio.sleep(0.1)  # let the pump observe the close
        assert writer._task.done()
        # A dead channel is visible at the producer.
        with pytest.raises(RuntimeError):
            writer.offer(b"y")
