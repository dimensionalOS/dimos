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

"""Topic wire tests against a real relay; no robot or Zenoh publisher needed."""

import asyncio
from collections.abc import AsyncIterator, Iterator
from contextlib import AsyncExitStack, asynccontextmanager
import json
from pathlib import Path
import sys
import threading
from typing import Any

from dimos_lcm.sensor_msgs import Image
import pytest
import zenoh

from dimos.protocol.service.zenoh_wire import make_zenoh_config
from dimos.utils.asyncio_timeout import timeout
from dimos.utils.deno import ensure_deno
from dimos.web.relay_bridge import topic_service, topic_transform
from dimos.web.relay_bridge.locate import relay_run_cmd
from dimos.web.relay_bridge.protocol import FrameHeader, ProtocolError, encode_data_frame
from dimos.web.relay_bridge.relay_process import RelayProcess, RelayReadyInfo
from dimos.web.relay_bridge.topic_metadata import METADATA_PREFIX, DescriptorSnapshot
from dimos.web.relay_bridge.topic_profiles import PROFILE_OPTIONS
from dimos.web.relay_bridge.topic_protocol import (
    TOPIC_GUARD_STREAM_MAGIC,
    TOPIC_PUBLISH_STREAM_MAGIC,
    TopicActivate,
    TopicApplied,
    TopicClose,
    TopicDescriptor,
    TopicError,
    TopicGuardClosed,
    TopicGuardHeartbeat,
    TopicGuardReady,
    TopicGuardResult,
    TopicGuardStop,
    TopicGuardTicket,
    TopicList,
    TopicOpen,
    TopicOpened,
    TopicOptions,
    TopicPage,
    TopicPubResult,
    TopicReady,
    TopicSettings,
    TopicSettingsRequest,
    TopicSources,
    TopicSub,
    TopicUnsub,
    TopicUpdate,
)
from dimos.web.relay_bridge.topic_service import TopicServiceConfig
from dimos.web.relay_bridge.topic_test_support import (
    next_message,
    provider_connection,
    route_variants,
)
from dimos.web.relay_bridge.topic_transport import OrderedTopicWriter, TopicByteBudget
from dimos.web.relay_bridge.wt_client import RelayClient

pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def relay() -> Iterator[RelayReadyInfo]:
    """A relay without an auth file: topics open to every viewer, providers keyless."""
    process = RelayProcess()
    try:
        yield process.start()
    finally:
        process.stop()


@pytest.fixture
def managed_child(tmp_path):
    @asynccontextmanager
    async def start(*command, cwd=None):
        with (tmp_path / "child-stderr.log").open("wb") as errors:
            process = await asyncio.create_subprocess_exec(
                *command,
                cwd=cwd,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=errors,
            )
            try:
                yield process
            finally:
                if process.stdin is not None:
                    process.stdin.close()
                if process.returncode is None:
                    process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), 3)
                    except asyncio.TimeoutError:
                        process.kill()
                        await process.wait()

    return start


async def test_managed_relay_exits_when_its_parent_pipe_closes(managed_child):
    web = Path(__file__).parents[3] / "web"
    # Use the production flags so startup cannot install the workspace's frontend dependencies.
    async with managed_child(
        *relay_run_cmd(ensure_deno(), web, "--port", "0", "--exit-on-stdin-eof"),
    ) as process:
        async with timeout(5):
            while True:
                line = await process.stdout.readline()
                assert line, "relay exited before readiness"
                if line.startswith(b'{"event":"ready"'):
                    break
        process.stdin.close()
        assert await asyncio.wait_for(process.wait(), 5) == 0


async def test_managed_provider_exits_and_unregisters_when_its_parent_pipe_closes(
    relay,
    tmp_path,
    managed_child,
):
    config = tmp_path / "provider.json"
    config.write_text(
        json.dumps(
            {
                "source": "workspace",
                "relay_url": f"http://127.0.0.1:{relay.http_port}",
                "zenoh": {
                    "mode": "peer",
                    "listen": ["tcp/127.0.0.1:0"],
                    "multicast": False,
                    "gossip": False,
                },
            }
        )
    )
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        async with managed_child(
            sys.executable,
            "-m",
            "dimos.web.relay_bridge.topic_service",
            "--config",
            str(config),
            "--exit-on-stdin-eof",
        ) as process:
            async with timeout(5):
                while not (await next_message(viewer, TopicSources)).sources:
                    pass
            process.stdin.close()
            assert await asyncio.wait_for(process.wait(), 5) == 0
            assert not (await next_message(viewer, TopicSources)).sources


@pytest.fixture(params=[True])
async def source(
    relay: RelayReadyInfo, request: pytest.FixtureRequest
) -> AsyncIterator[zenoh.Session]:
    async with running_source(relay, write=request.param) as session:
        yield session


def local_session() -> zenoh.Session:
    return zenoh.open(
        make_zenoh_config(
            {"mode": "peer", "listen": ["tcp/127.0.0.1:0"], "multicast": False, "gossip": False}
        )
    )


@asynccontextmanager
async def running_source(
    relay: RelayReadyInfo, *, write: bool = True
) -> AsyncIterator[zenoh.Session]:
    with local_session() as session:
        async with provider_connection(relay, session, write=write):
            yield session
    await asyncio.get_running_loop().shutdown_default_executor()


async def test_provider_reconnect_restores_more_than_64_desired_groups(relay: RelayReadyInfo):
    async with AsyncExitStack() as stack:
        viewers = [
            await stack.enter_async_context(await RelayClient.connect(relay.wt_url, "viewer"))
            for _ in range(2)
        ]
        for index, viewer in enumerate(viewers):
            await viewer.hello(topics=True)
            for key in range(40):
                viewer.send_control(
                    TopicSub(
                        id=f"s{key}",
                        source="workspace",
                        keyExpr=f"many/{index}/{key}",
                        delivery="reliable",
                        profile="raw",
                        maxHz=1000,
                    )
                )
                error = await next_message(viewer, TopicError)
                assert (error.id, error.code) == (f"s{key}", "source_offline")

        async def restored(viewer):
            ready = [await next_message(viewer, TopicReady) for _ in range(40)]
            assert {message.id for message in ready} == {f"s{key}" for key in range(40)}
            assert not viewer.is_closed

        async with running_source(relay):
            await asyncio.gather(*(restored(viewer) for viewer in viewers))


async def test_corrupt_camera_sample_is_dropped_and_later_media_and_events_arrive(
    relay: RelayReadyInfo,
    source: zenoh.Session,
    mocker,
):
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(v for v in vectors if v["name"] == "color_rgb")
    invalid_decoded = asyncio.Event()
    loop = asyncio.get_running_loop()
    original = topic_transform._encode

    def encode(profile, data, settings):
        try:
            return original(profile, data, settings)
        finally:
            if data == b"not an image at all":
                loop.call_soon_threadsafe(invalid_decoded.set)

    mocker.patch.object(topic_transform, "_encode", side_effect=encode)
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        for key, delivery, profile in (
            ("sensor/cam", "latest", vector["profile"]),
            ("events/test", "reliable", "raw"),
        ):
            viewer.send_control(
                TopicSub(
                    id=key,
                    source="workspace",
                    keyExpr=key,
                    delivery=delivery,
                    profile=profile,
                    maxHz=30,
                )
            )
            ready = await next_message(viewer, TopicReady)
            viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        frames = viewer.frames()
        source.put("sensor/cam", bytes.fromhex(vector["lcm"]))
        first = await asyncio.wait_for(anext(frames), 5)
        source.put("sensor/cam", b"not an image at all")
        await asyncio.wait_for(invalid_decoded.wait(), 5)
        source.put("sensor/cam", bytes.fromhex(vector["lcm"]))
        source.put("events/test", b"still connected")
        received = [await asyncio.wait_for(anext(frames), 5) for _ in range(2)]
        by_key = {frame.header.meta["key"]: frame for frame in received}
        assert by_key["sensor/cam"].header.ch == first.header.ch
        assert by_key["sensor/cam"].payload == first.payload
        assert by_key["sensor/cam"].header.meta["decodeFailures"] == 1
        assert by_key["events/test"].payload == b"still connected"
        assert not viewer.is_closed


async def test_standalone_service_retries_source_conflict_until_old_registration_closes(
    relay: RelayReadyInfo,
    mocker,
):
    conflict = asyncio.Event()
    warning = topic_service.logger.warning

    def observe_warning(message, **fields):
        warning(message, **fields)
        if message == "topic source still registered; retrying":
            conflict.set()

    mocker.patch.object(topic_service.logger, "warning", side_effect=observe_warning)
    config = TopicServiceConfig(
        source="workspace",
        relay_url=f"http://127.0.0.1:{relay.http_port}",
        zenoh={"mode": "peer", "listen": ["tcp/127.0.0.1:0"], "multicast": False, "gossip": False},
    )
    stop = asyncio.Event()
    async with (
        await RelayClient.connect(relay.wt_url, "robot") as previous,
        await RelayClient.connect(relay.wt_url, "viewer") as viewer,
    ):
        await previous.hello(topic_source="workspace")
        await viewer.hello(topics=True)
        initial = await next_message(viewer, TopicSources)
        task = asyncio.create_task(topic_service.serve(config, stop))
        try:
            await asyncio.wait_for(conflict.wait(), 5)
            await previous.close()
            async with timeout(5):
                while True:
                    sources = await next_message(viewer, TopicSources)
                    if sources.sources and sources.sources[0].gen != initial.sources[0].gen:
                        current = sources.sources[0]
                        break
            assert not task.done()
            viewer.send_control(
                TopicSub(
                    id="recovered",
                    source="workspace",
                    keyExpr="after/conflict",
                    delivery="reliable",
                    profile="raw",
                    maxHz=1000,
                )
            )
            ready = await next_message(viewer, TopicReady)
            assert ready.gen == current.gen
        finally:
            stop.set()
            await asyncio.wait_for(task, 5)


async def test_standalone_service_reconnects_after_a_bad_relay_message(
    relay: RelayReadyInfo,
    mocker,
):
    run = topic_service.TopicProvider.run
    reconnected = asyncio.Event()
    runs = 0

    async def bad_message_once(provider):
        nonlocal runs
        runs += 1
        if runs == 1:
            raise ProtocolError("invalid topic publication provenance")
        reconnected.set()
        await run(provider)

    mocker.patch.object(topic_service.TopicProvider, "run", bad_message_once)
    config = TopicServiceConfig(
        source="workspace",
        relay_url=f"http://127.0.0.1:{relay.http_port}",
        zenoh={"mode": "peer", "listen": ["tcp/127.0.0.1:0"], "multicast": False, "gossip": False},
    )
    stop = asyncio.Event()
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        task = asyncio.create_task(topic_service.serve(config, stop))
        try:
            await asyncio.wait_for(reconnected.wait(), 10)
            assert not task.done()
            viewer.send_control(
                TopicSub(
                    id="recovered",
                    source="workspace",
                    keyExpr="after/bad-message",
                    delivery="reliable",
                    profile="raw",
                    maxHz=1000,
                )
            )
            await next_message(viewer, TopicReady)
        finally:
            stop.set()
            await asyncio.wait_for(task, 5)


async def test_persistent_topic_stream_preserves_every_event_and_concrete_key(
    relay: RelayReadyInfo,
) -> None:
    async with (
        await RelayClient.connect(relay.wt_url, "robot") as provider,
        await RelayClient.connect(relay.wt_url, "viewer") as viewer,
    ):
        await provider.hello(topic_source="workspace")
        await viewer.hello(topics=True)
        viewer.send_control(
            TopicSub(
                id="events",
                source="workspace",
                keyExpr="demo/**",
                delivery="reliable",
                profile="raw",
                maxHz=1000,
            )
        )
        opened = await next_message(provider, TopicOpen)
        provider.send_control_frame(TopicOpened(route=opened.route, gen=opened.gen))
        ready = await next_message(viewer, TopicReady)
        viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        activated = await next_message(provider, TopicActivate)
        assert activated.route == opened.route

        errors: asyncio.Queue[str] = asyncio.Queue()
        budget = TopicByteBudget()
        writer = OrderedTopicWriter(provider._session, opened.route, budget, errors.put_nowait)
        writer.start()
        try:
            for seq in range(1, 201):
                writer.offer(
                    FrameHeader(
                        ch=opened.route,
                        seq=seq,
                        ts=3,
                        delivery="reliable",
                        meta={
                            "key": f"demo/key{seq % 3}",
                            "gen": opened.gen,
                            "encoding": "application/octet-stream",
                            "ingressTs": 2,
                            "sourceTs": 1,
                        },
                    ),
                    seq.to_bytes(4, "little") + b"\x00\xff",
                )
            frames = viewer.frames()
            async with timeout(5):
                received = [await anext(frames) for _ in range(200)]
            assert [int.from_bytes(frame.payload[:4], "little") for frame in received] == list(
                range(1, 201)
            )
            assert [frame.header.meta["key"] for frame in received] == [
                f"demo/key{seq % 3}" for seq in range(1, 201)
            ]
            assert all(
                frame.header.ch == ready.route and frame.payload[-2:] == b"\x00\xff"
                for frame in received
            )
            assert viewer.frames_dropped == 0
            assert errors.empty()
            viewer.send_control(TopicUnsub(id="events"))
            closed = await next_message(provider, TopicClose)
            assert closed.route == opened.route
        finally:
            await writer.aclose()
        assert budget.used == 0


async def test_route_interruption_reaches_viewer_without_closing_other_routes(relay):
    code = "topic_overflow"
    async with (
        await RelayClient.connect(relay.wt_url, "robot") as provider,
        await RelayClient.connect(relay.wt_url, "viewer") as viewer,
    ):
        await provider.hello(topic_source="workspace")
        await viewer.hello(topics=True)
        opened = []
        for key in ("broken", "healthy"):
            viewer.send_control(
                TopicSub(
                    id=key,
                    source="workspace",
                    keyExpr=key,
                    delivery="reliable",
                    profile="raw",
                    maxHz=1000,
                )
            )
            route = await next_message(provider, TopicOpen)
            opened.append(route)
            provider.send_control_frame(TopicOpened(route=route.route, gen=route.gen))
            ready = await next_message(viewer, TopicReady)
            viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
            await next_message(provider, TopicActivate)

        provider.send_control_frame(
            TopicError(
                id=opened[0].route,
                code=code,
                message=code,
                outcome="interrupted",
            )
        )
        error = await next_message(viewer, TopicError)
        assert (error.id, error.code, error.outcome) == ("broken", code, "interrupted")
        errors = []
        writer = provider.topic_writer(
            opened[1].route, "reliable", TopicByteBudget(), errors.append
        )
        writer.start()
        try:
            writer.offer(
                FrameHeader(
                    ch=opened[1].route,
                    seq=1,
                    ts=1,
                    delivery="reliable",
                    meta={
                        "key": "healthy",
                        "gen": opened[1].gen,
                        "encoding": "application/octet-stream",
                        "ingressTs": 1,
                    },
                ),
                b"still connected",
            )
            frame = await asyncio.wait_for(anext(viewer.frames()), 5)
            assert frame.payload == b"still connected"
            assert not viewer.is_closed
            assert errors == []
        finally:
            await writer.aclose()


async def test_zenoh_subscription_is_ready_before_a_publisher_exists(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        viewer.send_control(
            TopicList(id="before", source="workspace", prefix="", cursor="", limit=32)
        )
        before = await next_message(viewer, TopicPage)
        assert before.entries == []
        viewer.send_control(
            TopicSub(
                id="events",
                source="workspace",
                keyExpr="runtime/**",
                delivery="reliable",
                profile="raw",
                maxHz=1000,
            )
        )
        ready = await next_message(viewer, TopicReady)
        viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))

        # Local Zenoh publication starts only after installation acknowledged.
        # Activation may still be in flight; the provider must retain this burst.
        expected = [f'{{"n":{n}}}'.encode() for n in range(100)]
        for n, payload in enumerate(expected):
            source.put(f"runtime/key{n % 3}", payload, encoding=zenoh.Encoding.APPLICATION_JSON)
        frames = viewer.frames()
        async with timeout(5):
            received = [await anext(frames) for _ in expected]
        assert [frame.payload for frame in received] == expected
        assert [frame.header.meta["key"] for frame in received] == [
            f"runtime/key{n % 3}" for n in range(100)
        ]
        assert all(frame.header.meta["encoding"] == "application/json" for frame in received)
        viewer.send_control(
            TopicList(id="after", source="workspace", prefix="runtime/", cursor="", limit=32)
        )
        after = await next_message(viewer, TopicPage)
        assert [(entry.key, entry.availability) for entry in after.entries] == [
            (f"runtime/key{n}", "observed") for n in range(3)
        ]


async def test_descriptor_load_unload_and_external_liveliness_reach_topic_listing(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    descriptor = TopicDescriptor(
        key="domain/custom/temperature",
        encoding="application/cbor",
        type="custom.Reading",
        availability="known",
    )
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)

        async def listing_until(expected: list[TopicDescriptor]) -> None:
            async with timeout(5):
                while True:
                    viewer.send_control(
                        TopicList(
                            id="metadata", source="workspace", prefix="domain/", cursor="", limit=32
                        )
                    )
                    page = await next_message(viewer, TopicPage)
                    if page.entries == expected:
                        return
                    await asyncio.sleep(0.001)

        # No payload subscription exists: listing is descriptor metadata alone.
        for revision, entries in [(1, [descriptor]), (2, []), (3, [descriptor])]:
            source.put(
                METADATA_PREFIX + "runtime",
                DescriptorSnapshot(
                    owner="runtime", revision=revision, entries=entries
                ).model_dump_json(),
            )
            await listing_until(entries)
        token = source.liveliness().declare_token(descriptor.key)
        try:
            await listing_until([descriptor.model_copy(update={"availability": "advertised"})])
        finally:
            token.undeclare()
        await listing_until([descriptor])


@pytest.mark.parametrize(
    "name", ["depth16_big_padding", "color_bgr", "color_compressed", "cloud_big"]
)
async def test_fixed_media_profiles_cross_the_relay_with_original_acquisition_metadata(
    relay: RelayReadyInfo, source: zenoh.Session, name: str
) -> None:
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(v for v in vectors if v["name"] == name)
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        capabilities = await next_message(viewer, TopicSources)
        assert vector["profile"] in capabilities.sources[0].profiles
        viewer.send_control(
            TopicSub(
                id="media",
                source="workspace",
                keyExpr="sensor/test",
                delivery="latest",
                profile=vector["profile"],
                maxHz=30,
            )
        )
        ready = await next_message(viewer, TopicReady)
        viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        source.put("sensor/test", bytes.fromhex(vector["lcm"]))
        async with timeout(5):
            frame = await anext(viewer.frames())
        assert frame.payload == bytes.fromhex(vector["payload"])
        assert frame.header.meta is not None
        assert frame.header.meta["encoding"] == vector["encoding"]
        assert frame.header.meta["acquisitionTs"] == 123.456
        assert frame.header.meta["ingressTs"] > frame.header.meta["acquisitionTs"]
        assert frame.header.meta["profile"] == vector["profile"]
        assert frame.header.meta["encodeMs"] >= 0
        assert frame.header.meta["queueMs"] >= 0


async def test_camera_viewers_at_different_rates_share_one_encode(relay, source, mocker):
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(item for item in vectors if item["name"] == "color_bgr")
    encode = mocker.spy(topic_transform, "_encode")
    async with (
        await RelayClient.connect(relay.wt_url, "viewer") as slow,
        await RelayClient.connect(relay.wt_url, "viewer") as fast,
    ):
        for viewer, hz in ((slow, 2), (fast, 30)):
            await viewer.hello(topics=True)
            viewer.send_control(
                TopicSub(
                    id="camera",
                    source="workspace",
                    keyExpr="sensor/shared",
                    delivery="latest",
                    profile=vector["profile"],
                    maxHz=hz,
                )
            )
            ready = await next_message(viewer, TopicReady)
            viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        source.put("sensor/shared", bytes.fromhex(vector["lcm"]))
        frames = await asyncio.wait_for(
            asyncio.gather(anext(slow.frames()), anext(fast.frames())), 5
        )
        assert frames[0].payload == frames[1].payload == bytes.fromhex(vector["payload"])
        assert encode.call_count == 1


async def test_viewer_disconnect_closes_its_route_and_stops_encoding(
    relay: RelayReadyInfo, mocker
) -> None:
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(item for item in vectors if item["name"] == "color_rgb")
    encode = mocker.spy(topic_transform, "_encode")
    with local_session() as session:
        async with provider_connection(relay, session) as provider:
            async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
                await viewer.hello(topics=True)
                viewer.send_control(
                    TopicSub(
                        id="camera",
                        source="workspace",
                        keyExpr="sensor/gone",
                        delivery="latest",
                        profile=vector["profile"],
                        maxHz=30,
                    )
                )
                ready = await next_message(viewer, TopicReady)
                viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
                session.put("sensor/gone", bytes.fromhex(vector["lcm"]))
                frame = await asyncio.wait_for(anext(viewer.frames()), 5)
                assert frame.payload == bytes.fromhex(vector["payload"])
            # The viewer vanished without unsubscribing. The relay closes its
            # route and the provider drops the Zenoh subscription with it.
            async with timeout(5):
                while provider._routes:
                    await asyncio.sleep(0.01)
            for _ in range(20):
                session.put("sensor/gone", bytes.fromhex(vector["lcm"]))
            async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
                await viewer.hello(topics=True)
                viewer.send_control(
                    TopicSub(
                        id="events",
                        source="workspace",
                        keyExpr="events/after",
                        delivery="reliable",
                        profile="raw",
                        maxHz=1000,
                    )
                )
                ready = await next_message(viewer, TopicReady)
                viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
                session.put("events/after", b"no camera viewer")
                frame = await asyncio.wait_for(anext(viewer.frames()), 5)
                assert frame.payload == b"no camera viewer"
            assert encode.call_count == 1
    await asyncio.get_running_loop().shutdown_default_executor()


async def test_media_workers_select_latest_after_capacity_and_leave_events_responsive(
    relay: RelayReadyInfo, source: zenoh.Session, mocker
) -> None:
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    vector = next(v for v in vectors if v["name"] == "color_rgb")
    image = Image.lcm_decode(bytes.fromhex(vector["lcm"]))
    release = threading.Event()
    both_started = asyncio.Event()
    loop = asyncio.get_running_loop()
    original = topic_transform._encode
    started = 0
    lock = threading.Lock()

    def slow(profile, data, settings):
        nonlocal started
        with lock:
            started += 1
            if started == 2:
                loop.call_soon_threadsafe(both_started.set)
        if not release.wait(timeout=5):
            raise TimeoutError("test media worker was not released")
        return original(profile, data, settings)

    encode = mocker.patch.object(topic_transform, "_encode", side_effect=slow)
    try:
        async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
            await viewer.hello(topics=True)
            for key in ("a", "b", "c", "events"):
                viewer.send_control(
                    TopicSub(
                        id=key,
                        source="workspace",
                        keyExpr=key,
                        delivery="reliable" if key == "events" else "latest",
                        profile="raw" if key == "events" else vector["profile"],
                        maxHz=1000,
                    )
                )
                ready = await next_message(viewer, TopicReady)
                viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
            source.put("a", image.lcm_encode())
            source.put("b", image.lcm_encode())
            await asyncio.wait_for(both_started.wait(), 2)
            for sequence in range(100):
                image.header.seq = sequence
                source.put("c", image.lcm_encode())
            source.put("events", b"control remains responsive")
            frames = viewer.frames()
            async with timeout(2):
                event = await anext(frames)
            assert event.payload == b"control remains responsive"
            assert encode.call_count == 2
            release.set()
            async with timeout(5):
                received = [await anext(frames) for _ in range(3)]
            by_key = {frame.header.meta["key"]: frame for frame in received}
            assert by_key["c"].header.meta["headerSeq"] == 99
            assert encode.call_count == 3
            for key in ("a", "b", "c"):
                viewer.send_control(TopicUnsub(id=key))
            # Ordered provider control: listing returns after subscriber cleanup.
            viewer.send_control(
                TopicList(id="closed", source="workspace", prefix="", cursor="", limit=32)
            )
            await next_message(viewer, TopicPage)
            source.put("a", image.lcm_encode())
            source.put("events", b"no media viewers")
            async with timeout(2):
                event = await anext(frames)
            assert event.payload == b"no media viewers"
            assert encode.call_count == 3
    finally:
        release.set()


async def test_invalid_zenoh_expression_is_rejected_without_losing_the_source(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        viewer.send_control(
            TopicSub(
                id="bad",
                source="workspace",
                keyExpr="bad//expression",
                delivery="reliable",
                profile="raw",
                maxHz=1000,
            )
        )
        error = await next_message(viewer, TopicError)
        assert (error.id, error.code) == ("bad", "invalid_key_expression")
        viewer.send_control(
            TopicSub(
                id="good",
                source="workspace",
                keyExpr="runtime/new",
                delivery="latest",
                profile="raw",
                maxHz=30,
            )
        )
        ready = await next_message(viewer, TopicReady)
        viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        source.put("runtime/new", b"\x00\xffopaque")
        async with timeout(5):
            frame = await anext(viewer.frames())
        assert frame.payload == b"\x00\xffopaque"


def publish_stream(
    client: RelayClient, frames: list[tuple[str, str, bytes]], generation: int | float
) -> None:
    """Test viewer's binary wire surface; production browser uses SDK publish."""
    session = client._session
    assert session.session_id is not None
    stream = session.h3.create_webtransport_stream(session.session_id, is_unidirectional=False)
    session._quic.send_stream_data(stream, TOPIC_PUBLISH_STREAM_MAGIC, end_stream=False)
    for request_id, key, payload in frames:
        session._quic.send_stream_data(
            stream,
            encode_data_frame(
                FrameHeader(
                    ch="@put",
                    seq=1,
                    ts=0,
                    delivery="reliable",
                    meta={
                        "id": request_id,
                        "source": "workspace",
                        "gen": generation,
                        "key": key,
                        "encoding": "application/octet-stream",
                    },
                ),
                payload,
            ),
            end_stream=False,
        )
    session.transmit()


def guard_frame(
    client: RelayClient,
    meta: dict[str, Any],
    payload: bytes,
    *,
    sequence: int = 0,
    stream: int | None = None,
) -> int:
    session = client._session
    assert session.session_id is not None
    if stream is None:
        stream = session.h3.create_webtransport_stream(session.session_id, is_unidirectional=False)
        session._quic.send_stream_data(stream, TOPIC_GUARD_STREAM_MAGIC, end_stream=False)
    session._quic.send_stream_data(
        stream,
        encode_data_frame(
            FrameHeader(ch="@guard", seq=sequence, ts=0, delivery="reliable", meta=meta), payload
        ),
        end_stream=False,
    )
    session.transmit()
    return stream


async def test_guarded_commands_expire_without_browser_stop_and_reject_rearming(
    relay: RelayReadyInfo,
    source: zenoh.Session,
) -> None:
    received: asyncio.Queue[bytes] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    def sample(value: zenoh.Sample) -> None:
        loop.call_soon_threadsafe(received.put_nowait, value.payload.to_bytes())

    with source.declare_subscriber("robot/cmd", sample):
        async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
            await viewer.hello(topics=True)
            sources = await next_message(viewer, TopicSources)
            gen = sources.sources[0].gen
            assert sources.sources[0].guardedPublishers
            stream = guard_frame(
                viewer,
                {
                    "action": "open",
                    "id": "drive",
                    "source": "workspace",
                    "gen": gen,
                    "key": "robot/cmd",
                    "encoding": "application/octet-stream",
                    "group": "drive",
                    "timeoutMs": 400,
                    "takeover": False,
                },
                b"STOP",
            )
            ready = await next_message(viewer, TopicGuardReady)
            assert await asyncio.wait_for(received.get(), 2) == b"STOP"
            meta = {
                "action": "command",
                "source": "workspace",
                "gen": gen,
                "handle": ready.handle,
                "ticket": ready.ticket,
            }
            guard_frame(viewer, meta, b"MOVE", sequence=1, stream=stream)
            result = await next_message(viewer, TopicGuardResult)
            assert (result.seq, result.outcome) == (1, "accepted")
            assert await asyncio.wait_for(received.get(), 2) == b"MOVE"
            viewer.send_control(
                TopicGuardHeartbeat(handle=ready.handle, gen=gen, ticket=ready.ticket)
            )
            await next_message(viewer, TopicGuardTicket)
            expired = await next_message(viewer, TopicGuardClosed)
            assert expired.code == "guard_expired"
            assert await asyncio.wait_for(received.get(), 2) == b"STOP"
            guard_frame(viewer, meta, b"STALE", sequence=2, stream=stream)
            rejected = await next_message(viewer, TopicGuardResult)
            assert rejected.outcome == "rejected"
            viewer.send_control(TopicGuardStop(handle=ready.handle, gen=gen))
            await viewer.ping()
            assert received.empty()


async def test_page_disconnect_stops_guarded_commands(
    relay: RelayReadyInfo,
    source: zenoh.Session,
) -> None:
    received: asyncio.Queue[bytes] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    def sample(value: zenoh.Sample) -> None:
        loop.call_soon_threadsafe(received.put_nowait, value.payload.to_bytes())

    with source.declare_subscriber("robot/cmd", sample):
        async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
            await viewer.hello(topics=True)
            gen = (await next_message(viewer, TopicSources)).sources[0].gen
            stream = guard_frame(
                viewer,
                {
                    "action": "open",
                    "id": "drive",
                    "source": "workspace",
                    "gen": gen,
                    "key": "robot/cmd",
                    "encoding": "application/octet-stream",
                    "group": "drive",
                    "timeoutMs": 1000,
                    "takeover": False,
                },
                b"STOP",
            )
            ready = await next_message(viewer, TopicGuardReady)
            assert await asyncio.wait_for(received.get(), 2) == b"STOP"
            guard_frame(
                viewer,
                {
                    "action": "command",
                    "source": "workspace",
                    "gen": gen,
                    "handle": ready.handle,
                    "ticket": ready.ticket,
                },
                b"MOVE",
                sequence=1,
                stream=stream,
            )
            result = await next_message(viewer, TopicGuardResult)
            assert result.outcome == "accepted"
            assert await asyncio.wait_for(received.get(), 2) == b"MOVE"
        # The viewer has disappeared; no browser timer or explicit stop runs.
        assert await asyncio.wait_for(received.get(), 2) == b"STOP"


async def test_guard_watchdog_and_receipts_run_while_all_media_workers_are_busy(
    relay: RelayReadyInfo,
    source: zenoh.Session,
    mocker,
) -> None:
    vector = next(
        v
        for v in json.loads(
            (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
        )
        if v["name"] == "color_rgb"
    )
    release = threading.Event()
    both_started = asyncio.Event()
    received: asyncio.Queue[bytes] = asyncio.Queue()
    loop = asyncio.get_running_loop()
    original = topic_transform._encode
    count = 0
    lock = threading.Lock()

    def encode(profile, data, settings):
        nonlocal count
        with lock:
            count += 1
            if count == 2:
                loop.call_soon_threadsafe(both_started.set)
        if not release.wait(timeout=5):
            raise TimeoutError("media test did not finish")
        return original(profile, data, settings)

    def sample(value: zenoh.Sample) -> None:
        loop.call_soon_threadsafe(received.put_nowait, value.payload.to_bytes())

    mocker.patch.object(topic_transform, "_encode", side_effect=encode)
    try:
        with source.declare_subscriber("robot/cmd", sample):
            async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
                await viewer.hello(topics=True)
                gen = (await next_message(viewer, TopicSources)).sources[0].gen
                for key in ("camera/a", "camera/b"):
                    viewer.send_control(
                        TopicSub(
                            id=key,
                            source="workspace",
                            keyExpr=key,
                            delivery="latest",
                            profile=vector["profile"],
                            maxHz=30,
                        )
                    )
                    ready = await next_message(viewer, TopicReady)
                    viewer.send_control(TopicActivate(route=ready.route, gen=gen))
                    source.put(key, bytes.fromhex(vector["lcm"]))
                await asyncio.wait_for(both_started.wait(), 2)
                stream = guard_frame(
                    viewer,
                    {
                        "action": "open",
                        "id": "drive",
                        "source": "workspace",
                        "gen": gen,
                        "key": "robot/cmd",
                        "encoding": "application/octet-stream",
                        "group": "drive",
                        "timeoutMs": 400,
                        "takeover": False,
                    },
                    b"STOP",
                )
                ready = await next_message(viewer, TopicGuardReady)
                assert await asyncio.wait_for(received.get(), 1) == b"STOP"
                guard_frame(
                    viewer,
                    {
                        "action": "command",
                        "source": "workspace",
                        "gen": gen,
                        "handle": ready.handle,
                        "ticket": ready.ticket,
                    },
                    b"MOVE",
                    sequence=1,
                    stream=stream,
                )
                result = await next_message(viewer, TopicGuardResult)
                assert result.outcome == "accepted"
                assert await asyncio.wait_for(received.get(), 1) == b"MOVE"
                closed = await next_message(viewer, TopicGuardClosed)
                assert closed.code == "guard_expired"
                assert await asyncio.wait_for(received.get(), 1) == b"STOP"
                assert not release.is_set()
    finally:
        release.set()


async def test_binary_publish_preserves_large_payload_and_returns_checked_provider_receipt(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    received: asyncio.Queue[bytes] = asyncio.Queue()
    loop = asyncio.get_running_loop()

    def sample(value: zenoh.Sample) -> None:
        loop.call_soon_threadsafe(received.put_nowait, value.payload.to_bytes())

    payload = bytes(range(256)) * 8192  # 2 MiB, including zero/non-UTF8 bytes
    with source.declare_subscriber("runtime/new", sample):
        async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
            await viewer.hello(topics=True)
            sources = await next_message(viewer, TopicSources)
            publish_stream(viewer, [("put", "runtime/new", payload)], sources.sources[0].gen)
            result = await next_message(viewer, TopicPubResult)
            async with timeout(5):
                actual = await received.get()
            assert actual == payload
            assert (result.id, result.outcome, result.code) == ("put", "accepted", "")
            assert result.providerTs > 0
            await viewer.ping()  # bulk stream did not replace the control carrier


@pytest.mark.parametrize("source", [False], indirect=True)
async def test_provider_disabled_writes_reject_even_a_relay_authorized_request(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        sources = await next_message(viewer, TopicSources)
        publish_stream(viewer, [("put", "runtime/new", b"ignored")], sources.sources[0].gen)
        result = await next_message(viewer, TopicPubResult)
        assert (result.id, result.outcome, result.code) == ("put", "rejected", "topic_write_denied")


async def test_invalid_concrete_key_rejects_without_disabling_later_publications(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        sources = await next_message(viewer, TopicSources)
        publish_stream(
            viewer,
            [("bad", "runtime//bad", b"no"), ("ok", "runtime/new", b"yes")],
            sources.sources[0].gen,
        )
        results = [await next_message(viewer, TopicPubResult) for _ in range(2)]
        assert [(result.id, result.outcome) for result in results] == [
            ("bad", "rejected"),
            ("ok", "accepted"),
        ]


# Settings updates (PLAN_2a): variants per route, acknowledged moves.


async def expect(client: RelayClient, kind: type, *, forbid: type = TopicError) -> Any:
    """The next control message of `kind`; a `forbid` message before it fails."""
    async with timeout(5):
        while True:
            message = await client._session.control_msgs.get()
            if isinstance(message, kind):
                return message
            assert not isinstance(message, forbid), message


def boxed_vector() -> dict[str, Any]:
    vectors = json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    return next(v for v in vectors if v["name"] == "color_rgb_boxed")


async def subscribe_camera(
    viewer: RelayClient,
    key: str,
    *,
    sub_id: str = "camera",
    profile: str = "image.jpeg.balanced.v1",
    settings: TopicSettingsRequest | None = None,
    max_hz: float = 30,
) -> TopicReady:
    viewer.send_control(
        TopicSub(
            id=sub_id,
            source="workspace",
            keyExpr=key,
            delivery="latest",
            profile=profile,
            maxHz=max_hz,
            **({} if settings is None else {"settings": settings}),
        )
    )
    ready = await expect(viewer, TopicReady)
    viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
    return ready


async def test_profile_options_are_advertised_and_different_quality_shares_the_subscription_not_the_encode(
    relay: RelayReadyInfo, mocker
) -> None:
    vector = boxed_vector()
    encode = mocker.spy(topic_transform, "_encode")
    with local_session() as session:
        async with (
            provider_connection(relay, session) as provider,
            await RelayClient.connect(relay.wt_url, "viewer") as crisp,
            await RelayClient.connect(relay.wt_url, "viewer") as soft,
        ):
            for viewer, settings in ((crisp, None), (soft, TopicSettingsRequest(quality=40))):
                await viewer.hello(topics=True)
                info = (await next_message(viewer, TopicSources)).sources[0]
                assert info.profileOptions is not None
                advertised = {
                    profile: {
                        name: option.model_dump(exclude_none=True)
                        for name, option in ranges.items()
                    }
                    for profile, ranges in info.profileOptions.items()
                }
                assert advertised == PROFILE_OPTIONS
                ready = await subscribe_camera(viewer, "sensor/quality", settings=settings)
                assert ready.settings == TopicSettings(
                    maxHz=30, maxResolution=[1280, 720], quality=75 if settings is None else 40
                )
            session.put("sensor/quality", bytes.fromhex(vector["lcm"]))
            frames = await asyncio.wait_for(
                asyncio.gather(anext(crisp.frames()), anext(soft.frames())), 5
            )
            assert [frame.header.meta["quality"] for frame in frames] == [75, 40]
            assert frames[0].payload != frames[1].payload
            assert "variant" not in frames[0].header.meta
            assert encode.call_count == 2
            assert len(provider._routes) == 1
            (variants,) = route_variants(provider).values()
            assert sorted(v["quality"] for v in variants.values()) == [40, 75]
    await asyncio.get_running_loop().shutdown_default_executor()


async def test_resolution_update_switches_without_a_gap_then_resets_and_retires_the_old_variant(
    relay: RelayReadyInfo,
) -> None:
    vector = boxed_vector()
    with local_session() as session:
        async with (
            provider_connection(relay, session) as provider,
            await RelayClient.connect(relay.wt_url, "viewer") as viewer,
        ):
            await viewer.hello(topics=True)
            await subscribe_camera(viewer, "sensor/box")
            frames = viewer.frames()
            session.put("sensor/box", bytes.fromhex(vector["lcm"]))
            first = await asyncio.wait_for(anext(frames), 5)
            assert (first.header.meta["w"], first.header.meta["h"]) == (32, 16)
            viewer.send_control(
                TopicUpdate(id="camera", revision=1, options=TopicOptions(maxResolution=[16, 16]))
            )
            # No interruption and no reopen: the acknowledgement names the
            # settings in force, and frames keep flowing on the same route.
            applied = await expect(viewer, TopicApplied)
            assert applied.revision == 1
            assert applied.settings == TopicSettings(maxHz=30, maxResolution=[16, 16], quality=75)
            session.put("sensor/box", bytes.fromhex(vector["lcm"]))
            boxed = await asyncio.wait_for(anext(frames), 5)
            assert boxed.header.ch == first.header.ch
            assert (boxed.header.meta["w"], boxed.header.meta["h"]) == (16, 8)
            (route,) = provider._routes
            async with timeout(5):
                while len(route_variants(provider)[route]) != 1:
                    await asyncio.sleep(0.05)
            (remaining,) = route_variants(provider)[route].values()
            assert remaining["maxResolution"] == [16, 16]
            viewer.send_control(
                TopicUpdate(id="camera", revision=2, options=TopicOptions(maxResolution=None))
            )
            applied = await expect(viewer, TopicApplied)
            assert applied.settings == TopicSettings(
                maxHz=30, maxResolution=[1280, 720], quality=75
            )
            session.put("sensor/box", bytes.fromhex(vector["lcm"]))
            restored = await asyncio.wait_for(anext(frames), 5)
            assert (restored.header.meta["w"], restored.header.meta["h"]) == (32, 16)
            assert not viewer.is_closed
    await asyncio.get_running_loop().shutdown_default_executor()


async def test_a_fifth_settings_set_rejects_with_variant_limit_and_rejections_keep_the_settings(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    vector = boxed_vector()
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        for quality in (10, 20, 30, 40):
            await subscribe_camera(
                viewer,
                "sensor/limit",
                sub_id=f"q{quality}",
                settings=TopicSettingsRequest(quality=quality),
            )
        viewer.send_control(
            TopicSub(
                id="q50",
                source="workspace",
                keyExpr="sensor/limit",
                delivery="latest",
                profile="image.jpeg.balanced.v1",
                maxHz=30,
                settings=TopicSettingsRequest(quality=50),
            )
        )
        error = await next_message(viewer, TopicError)
        assert (error.id, error.code, error.outcome) == ("q50", "variant_limit", "rejected")
        for revision, options, code in (
            (1, TopicOptions(quality=50), "variant_limit"),
            (2, TopicOptions(quality=0), "option_out_of_range"),
            (3, TopicOptions(keepEvery=2), "unsupported_option"),
        ):
            viewer.send_control(TopicUpdate(id="q10", revision=revision, options=options))
            error = await next_message(viewer, TopicError)
            assert (error.id, error.revision, error.code) == ("q10", revision, code)
            assert error.outcome == "rejected"
        source.put("sensor/limit", bytes.fromhex(vector["lcm"]))
        frames = viewer.frames()
        received = [await asyncio.wait_for(anext(frames), 5) for _ in range(4)]
        assert sorted(frame.header.meta["quality"] for frame in received) == [10, 20, 30, 40]
        assert not viewer.is_closed


async def test_a_raw_rate_update_is_acknowledged_by_the_relay_and_reliable_events_all_arrive(
    relay: RelayReadyInfo, source: zenoh.Session
) -> None:
    async with await RelayClient.connect(relay.wt_url, "viewer") as viewer:
        await viewer.hello(topics=True)
        viewer.send_control(
            TopicSub(
                id="events",
                source="workspace",
                keyExpr="events/rate",
                delivery="reliable",
                profile="raw",
                maxHz=1000,
            )
        )
        ready = await expect(viewer, TopicReady)
        assert ready.settings == TopicSettings(maxHz=1000)
        viewer.send_control(TopicActivate(route=ready.route, gen=ready.gen))
        viewer.send_control(TopicUpdate(id="events", revision=1, options=TopicOptions(maxHz=1)))
        applied = await expect(viewer, TopicApplied)
        assert (applied.revision, applied.settings) == (1, TopicSettings(maxHz=1))
        for n in range(5):
            source.put("events/rate", bytes([n]))
        frames = viewer.frames()
        received = [await asyncio.wait_for(anext(frames), 5) for _ in range(5)]
        assert [frame.payload for frame in received] == [bytes([n]) for n in range(5)]
        viewer.send_control(TopicUpdate(id="events", revision=2, options=TopicOptions(quality=1)))
        error = await next_message(viewer, TopicError)
        assert (error.revision, error.code) == (2, "unsupported_option")
