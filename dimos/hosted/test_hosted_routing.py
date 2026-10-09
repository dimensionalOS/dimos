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
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hosted.daemon import HostDaemon, code_revision
from dimos.hosted.deploy import deployed, wait_for_hosts
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
    versions: dict[str, str | int] = {"application_revision": code_revision()}
    edge = HostDaemon(
        "edge-id",
        name="edge",
        versions=versions,
        log_root=tmp_path / "edge",
        listen=[f"tcp/127.0.0.1:{edge_port}"],
    )
    base = HostDaemon(
        "base-id",
        name="base",
        versions=versions,
        log_root=tmp_path / "base",
        listen=[f"tcp/127.0.0.1:{base_port}"],
        connect=[f"tcp/127.0.0.1:{edge_port}"],
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
        local_host="base",
        application_name="routing",
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


@pytest.mark.skipif(sys.platform == "darwin", reason="scouts multicast on Linux loopback")
def test_hosts_scout_each_other_and_answer_probes(tmp_path: Path) -> None:
    from dimos.hosted.discovery import merge, probe_all, scouted_endpoints

    group = f"224.0.0.224:{_free_port()}"
    daemons = [
        HostDaemon(
            f"{name}-id",
            name=name,
            log_root=tmp_path / name,
            listen=[f"tcp/127.0.0.1:{_free_port()}"],
            autodiscovery=True,
            scout_addr=group,
            scout_interface="lo",
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

        scouted = scouted_endpoints(group, "lo", timeout=1.0)
        assert {e for group in scouted for e in group} >= {d.listen[0] for d in daemons}
        found = merge(probe_all([daemons[0].listen[0], daemons[1].listen[0]], timeout=2.0))
        assert [(h.name, e) for h, e in found] == [
            ("one", (daemons[0].listen[0],)),
            ("two", (daemons[1].listen[0],)),
        ]
