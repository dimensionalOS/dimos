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

"""Tests for hosted-data node recommendation."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import json
from pathlib import Path
import threading

import pytest

from dimos.hosted_data import nodes as hosted_nodes
from dimos.hosted_data.nodes import (
    NodeProbe,
    ReplayNode,
    choose_server_url,
    load_replay_nodes,
    probe_node,
    recommend_node,
)
from dimos.hosted_data.repository import (
    ReplayRepository,
    ReplayRepositoryServer,
)


@contextmanager
def _health_server(root: Path) -> Iterator[str]:
    server = ReplayRepositoryServer(
        ("127.0.0.1", 0),
        ReplayRepository(root),
        token="secret",
        node_name="cn-beijing",
        region="china",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        host_text = host.decode() if isinstance(host, bytes) else host
        yield f"http://{host_text}:{port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5.0)


def test_health_endpoint_is_public_and_advertises_region(tmp_path: Path) -> None:
    with _health_server(tmp_path / "objects") as url:
        result = probe_node(ReplayNode(name="candidate", url=url, region="other"))

    assert result.healthy
    assert result.latency_ms is not None
    assert result.advertised_name == "cn-beijing"
    assert result.advertised_region == "china"


def test_recommendation_selects_fastest_healthy_node(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    china = ReplayNode(name="cn", url="https://cn.example", region="china")
    us = ReplayNode(name="us", url="https://us.example", region="us")

    def fake_probe(node: ReplayNode, **_: object) -> NodeProbe:
        latency = 25.0 if node.region == "china" else 180.0
        return NodeProbe(
            node=node,
            healthy=True,
            latency_ms=latency,
            advertised_region=node.region,
        )

    monkeypatch.setattr(hosted_nodes, "probe_node", fake_probe)
    result = recommend_node(
        (china, us),
        cache_path=tmp_path / "recommendation.json",
    )

    assert result.selected == china
    assert result.detected_region == "china"
    assert [probe.latency_ms for probe in result.probes] == [25.0, 180.0]


def test_recommendation_ignores_unreachable_node(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    china = ReplayNode(name="cn", url="https://cn.example", region="china")
    us = ReplayNode(name="us", url="https://us.example", region="us")

    def fake_probe(node: ReplayNode, **_: object) -> NodeProbe:
        if node == china:
            return NodeProbe(node=node, healthy=False, latency_ms=None, error="timeout")
        return NodeProbe(node=node, healthy=True, latency_ms=120.0, advertised_region="us")

    monkeypatch.setattr(hosted_nodes, "probe_node", fake_probe)

    assert (
        recommend_node(
            (china, us),
            cache_path=tmp_path / "recommendation.json",
        ).selected
        == us
    )


def test_hourly_recommendation_cache_avoids_reprobe(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    node = ReplayNode(name="cn", url="https://cn.example", region="china")
    calls = 0

    def fake_probe(candidate: ReplayNode, **_: object) -> NodeProbe:
        nonlocal calls
        calls += 1
        return NodeProbe(
            node=candidate,
            healthy=True,
            latency_ms=20.0,
            advertised_region="china",
        )

    monkeypatch.setattr(hosted_nodes, "probe_node", fake_probe)
    cache = tmp_path / "recommendation.json"

    first = recommend_node((node,), cache_path=cache)
    second = recommend_node((node,), cache_path=cache)
    forced = recommend_node((node,), cache_path=cache, force=True)

    assert first.selected == second.selected == forced.selected
    assert calls == 2
    assert second.probes == ()


def test_load_nodes_and_choose_server_from_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    raw = json.dumps(
        [
            {"name": "cn", "url": "https://cn.example", "region": "china"},
            {"name": "us", "url": "https://us.example", "region": "us"},
        ]
    )
    nodes = load_replay_nodes(raw)
    assert [node.region for node in nodes] == ["china", "us"]

    monkeypatch.setenv("DIMOS_REPLAY_NODES", raw)
    monkeypatch.delenv("DIMOS_REPLAY_SERVER_URL", raising=False)
    monkeypatch.setattr(
        hosted_nodes,
        "recommend_node",
        lambda _: hosted_nodes.NodeRecommendation(
            selected=nodes[1],
            detected_region="us",
            measured_at=0.0,
            probes=(),
        ),
    )
    assert choose_server_url(None) == "https://us.example"
    assert choose_server_url("https://explicit.example/") == "https://explicit.example"
    assert not tuple(tmp_path.iterdir())


def test_invalid_node_environment_is_rejected() -> None:
    with pytest.raises(ValueError, match="JSON array"):
        load_replay_nodes('{"name": "not-an-array"}')
