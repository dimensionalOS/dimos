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

"""Two HostDaemons as linked routers: a cross-Host stream arrives, a same-Host one never leaves.

The edge Host runs Source -> Relay over ``local_events`` (32 KB per message); Relay sends a
short counter over ``cross_events`` to Sink on the base Host. The proof that ``local_events``
stays on the edge router is twofold: the edge router's subscriber table names only itself
for it (no other router declared interest, so zenoh never forwards it), and the bytes the
base router receives on its TCP link from the edge router stay far below what
``local_events`` carried meanwhile.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Iterator
from contextlib import ExitStack, suppress
import json
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import time
from typing import Any

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hosted.config import HostConfig
from dimos.hosted.daemon import HostDaemon
from dimos.hosted.deploy import deployed, wait_for_hosts
from dimos.hosted.machine import code_revision
from dimos.msgs.std_msgs.String import String
from dimos.protocol.rpc.zenohrpc import ZenohRPC
from dimos.protocol.service.zenohservice import ZenohSessionPool

PAYLOAD_BYTES = 32_000


class SourceConfig(ModuleConfig):
    interval_seconds: float = 0.02


class Source(Module):
    config: SourceConfig
    local_events: Out[String]

    async def main(self) -> AsyncIterator[None]:
        task = asyncio.create_task(self._publish())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task

    async def _publish(self) -> None:
        while True:
            self.local_events.publish(String("x" * PAYLOAD_BYTES))
            await asyncio.sleep(self.config.interval_seconds)


class Relay(Module):
    local_events: In[String]
    cross_events: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._bytes = 0

    async def handle_local_events(self, message: String) -> None:
        self._bytes += len(message.data)
        self.cross_events.publish(String(str(self._bytes)))


class SinkConfig(ModuleConfig):
    result_path: str


class Sink(Module):
    config: SinkConfig
    cross_events: In[String]

    async def handle_cross_events(self, message: String) -> None:
        path = Path(self.config.result_path)
        path.with_suffix(".tmp").write_text(json.dumps({"local_bytes": int(message.data)}))
        path.with_suffix(".tmp").replace(path)


class Provider(Module):
    @rpc
    def add(self, a: int, b: int) -> int:
        return a + b


class Consumer(Module):
    """Calls its provider's RPC once started and leaves the answer on disk."""

    config: SinkConfig
    provider: Provider

    async def main(self) -> AsyncIterator[None]:
        loop = asyncio.get_running_loop()
        answer = await loop.run_in_executor(None, self.provider.add, 20, 22)
        Path(self.config.result_path).write_text(str(answer))
        yield


def _free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return int(probe.getsockname()[1])


def _local_bytes(path: Path, timeout: float) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with suppress(FileNotFoundError, json.JSONDecodeError):
            return int(json.loads(path.read_text())["local_bytes"])
        time.sleep(0.05)
    raise TimeoutError("cross_events never reached the base Host")


def _received_on(src: str, dst: str) -> int:
    """Kernel byte count the socket ``src`` received from ``dst``, both host:port."""
    out = subprocess.run(
        ["ss", "-tinH", "state", "established", "src", src, "dst", dst],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    match = re.search(r"bytes_received:(\d+)", out)
    return int(match.group(1)) if match else 0


def _subscribers(rpc: ZenohRPC, zid: str) -> dict[str, dict[str, list[str]]]:
    replies = rpc.session.get(f"@/{zid}/router/subscriber/**", timeout=2.0)
    prefix = f"@/{zid}/router/subscriber/"
    return {
        str(r.ok.key_expr).removeprefix(prefix): json.loads(r.ok.payload.to_string())
        for r in replies
        if r.ok is not None
    }


@pytest.fixture
def hosts(tmp_path: Path) -> Iterator[tuple[ZenohRPC, ZenohRPC, ZenohRPC]]:
    edge_port, base_port = _free_port(), _free_port()
    revision = code_revision()
    edge = HostDaemon(
        HostConfig(name="edge", listen=f"tcp/127.0.0.1:{edge_port}", autodiscovery=False),
        host_id="edge-id",
        revision=revision,
        log_root=tmp_path / "edge",
    )
    base = HostDaemon(
        HostConfig(
            name="base",
            listen=f"tcp/127.0.0.1:{base_port}",
            connect=f"tcp/127.0.0.1:{edge_port}",
            autodiscovery=False,
        ),
        host_id="base-id",
        revision=revision,
        log_root=tmp_path / "base",
    )
    with ExitStack() as stack:
        edge_rpc = stack.enter_context(edge.serve())
        base_rpc = stack.enter_context(base.serve())
        pool = ZenohSessionPool()
        stack.callback(pool.close_all)
        controller = ZenohRPC(
            session_pool=pool, mode="client", connect=[base.client_endpoint], multicast=False
        )
        controller.start()
        stack.callback(controller.stop)
        yield edge_rpc, base_rpc, controller


@pytest.mark.skipif(shutil.which("ss") is None, reason="needs iproute2 ss for link byte counts")
def test_cross_host_stream_arrives_and_same_host_stream_stays_local(
    hosts: tuple[ZenohRPC, ZenohRPC, ZenohRPC], tmp_path: Path
) -> None:
    edge_rpc, base_rpc, controller = hosts
    result = tmp_path / "result.json"
    app = autoconnect(
        autoconnect(Source.blueprint(), Relay.blueprint()).hosted(host="edge"),
        Sink.blueprint(result_path=str(result)),
    ).global_config(n_workers=1)

    descriptors = wait_for_hosts(controller, {"edge", "base"}, timeout=20.0)
    logs: list[tuple[str, str]] = []
    with deployed(
        app,
        controller,
        run_id="routing-1",
        local_host_id="base-id",
        descriptors=descriptors,
        on_log=lambda host, line: logs.append((host, line)),
    ) as placement:
        assert placement == {"source": "edge", "relay": "edge", "sink": "base"}

        # Cross-Host: Relay's counter reaches Sink through both routers.
        _local_bytes(result, timeout=30.0)

        # Same-Host, by routing table: only the edge router itself wants local_events.
        edge_zid, base_zid = str(edge_rpc.session.zid()), str(base_rpc.session.zid())
        local = {k: v for k, v in _subscribers(edge_rpc, edge_zid).items() if "local_events" in k}
        cross = {k: v for k, v in _subscribers(edge_rpc, edge_zid).items() if "cross_events" in k}
        assert local and all(v["routers"] == [edge_zid] for v in local.values())
        assert cross and all(base_zid in v["routers"] for v in cross.values())

        # Same-Host, by bytes on the wire: the base router's link from the edge router.
        (link,) = [
            link
            for link in base_rpc.session.info.links()
            if str(link.dst).endswith(edge_rpc.config.listen[0].rpartition("/")[2])
        ]
        src, dst = str(link.src).rpartition("/")[2], str(link.dst).rpartition("/")[2]
        bytes_before, local_before = _received_on(src, dst), _local_bytes(result, 1.0)
        time.sleep(2.0)
        bytes_after, local_after = _received_on(src, dst), _local_bytes(result, 1.0)

    # The edge fragment's worker output reached the controller, tagged with its Host.
    assert any(host == "edge" and "Deployed module" in line for host, line in logs), logs[:20]
    carried_locally = local_after - local_before
    crossed = bytes_after - bytes_before
    assert carried_locally > 20 * PAYLOAD_BYTES
    assert crossed < carried_locally / 20, (crossed, carried_locally)


def test_a_module_ref_calls_its_provider_on_another_host(
    hosts: tuple[ZenohRPC, ZenohRPC, ZenohRPC], tmp_path: Path
) -> None:
    _edge_rpc, _base_rpc, controller = hosts
    result = tmp_path / "answer.txt"
    # Pinned apart on purpose: a chatty pair is kept together by pinning both sides alike.
    app = autoconnect(
        Provider.blueprint().hosted(host="edge"),
        Consumer.blueprint(result_path=str(result)).hosted(host="base"),
    ).global_config(n_workers=1)

    descriptors = wait_for_hosts(controller, {"edge", "base"}, timeout=20.0)
    with deployed(
        app, controller, run_id="refs-1", local_host_id="base-id", descriptors=descriptors
    ) as placement:
        assert placement == {"provider": "edge", "consumer": "base"}
        deadline = time.monotonic() + 30.0
        while not result.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert result.read_text() == "42"


@pytest.mark.skipif(sys.platform == "darwin", reason="scouts multicast on Linux loopback")
def test_hosts_scout_each_other_and_answer_probes(tmp_path: Path) -> None:
    from dimos.hosted.discovery import merge, probe_all, scouted_endpoints

    group = f"224.0.0.224:{_free_port()}"
    config = HostConfig(autodiscovery=True, scout_addr=group, scout_interface="lo")
    daemons = [
        HostDaemon(
            config.model_copy(update={"name": name, "listen": f"tcp/127.0.0.1:{_free_port()}"}),
            host_id=f"{name}-id",
            revision="r",
            log_root=tmp_path / name,
        )
        for name in ("one", "two")
    ]
    with ExitStack() as stack:
        one, two = (stack.enter_context(d.serve()) for d in daemons)
        deadline = time.monotonic() + 10.0
        while not list(one.session.info.routers_zid()) and time.monotonic() < deadline:
            time.sleep(0.1)
        assert [str(z) for z in one.session.info.routers_zid()] == [daemons[1].router_zid]
        assert [str(z) for z in two.session.info.routers_zid()] == [daemons[0].router_zid]

        # What `dimos host ls` does without a local Host: scout the group, probe what answers.
        scouted = scouted_endpoints(config, timeout=1.0)
        assert set(scouted) >= {d.listen[0] for d in daemons}
        found = merge(probe_all(scouted, timeout=2.0))
        assert [(f.descriptor.name, f.endpoints) for f in found] == [
            ("one", (daemons[0].listen[0],)),
            ("two", (daemons[1].listen[0],)),
        ]


def test_a_connect_seed_is_linked_when_it_comes_up_and_when_it_comes_back(tmp_path: Path) -> None:
    from dimos.hosted.client import discover_host_ids

    two_port = _free_port()
    one = HostDaemon(
        HostConfig(
            name="one",
            listen=f"tcp/127.0.0.1:{_free_port()}",
            connect=f"tcp/127.0.0.1:{two_port}",
            autodiscovery=False,
        ),
        host_id="one-id",
        revision="r",
        log_root=tmp_path / "one",
    )
    two = HostDaemon(
        HostConfig(name="two", listen=f"tcp/127.0.0.1:{two_port}", autodiscovery=False),
        host_id="two-id",
        revision="r",
        log_root=tmp_path / "two",
    )

    def hosts_seen_by_one(rpc: ZenohRPC, expected: tuple[str, ...], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while discover_host_ids(rpc, timeout=1.0) != expected and time.monotonic() < deadline:
            time.sleep(0.1)
        return discover_host_ids(rpc, timeout=1.0) == expected

    # The seed is not up yet: zenoh keeps dialing it, nothing of ours does.
    with one.serve() as one_rpc:
        assert hosts_seen_by_one(one_rpc, ("one-id",), 5.0)
        with two.serve():
            assert hosts_seen_by_one(one_rpc, ("one-id", "two-id"), 15.0)
        assert hosts_seen_by_one(one_rpc, ("one-id",), 15.0)
        # The seed comes back on the same port and is relinked.
        with two.serve():
            assert hosts_seen_by_one(one_rpc, ("one-id", "two-id"), 30.0)
