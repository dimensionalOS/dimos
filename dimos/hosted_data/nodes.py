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

"""Probe hosted-data nodes and cache the fastest healthy route."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from typing import Any, Literal
from urllib.parse import urlsplit

import requests

from dimos.constants import CACHE_DIR

ReplayRegion = Literal["china", "us", "other"]
_REGIONS = {"china", "us", "other"}
_DEFAULT_REFRESH_SECONDS = 3600.0


def _server_url(value: str) -> str:
    parsed = urlsplit(value)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise ValueError("replay node URL must be an absolute HTTP(S) URL")
    if parsed.username is not None or parsed.password is not None:
        raise ValueError("replay node URL must not contain credentials")
    if parsed.query or parsed.fragment:
        raise ValueError("replay node URL must not contain a query or fragment")
    return value.rstrip("/")


@dataclass(frozen=True)
class ReplayNode:
    name: str
    url: str
    region: ReplayRegion = "other"

    def __post_init__(self) -> None:
        if not self.name or len(self.name) > 64:
            raise ValueError("replay node name must contain 1-64 characters")
        object.__setattr__(self, "url", _server_url(self.url))
        if self.region not in _REGIONS:
            raise ValueError(f"unsupported replay node region {self.region!r}")


@dataclass(frozen=True)
class NodeProbe:
    node: ReplayNode
    healthy: bool
    latency_ms: float | None
    advertised_name: str | None = None
    advertised_region: ReplayRegion | None = None
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "node": asdict(self.node),
            "healthy": self.healthy,
            "latency_ms": self.latency_ms,
            "advertised_name": self.advertised_name,
            "advertised_region": self.advertised_region,
            "error": self.error,
        }


@dataclass(frozen=True)
class NodeRecommendation:
    selected: ReplayNode
    detected_region: ReplayRegion
    measured_at: float
    probes: tuple[NodeProbe, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "version": 1,
            "selected": asdict(self.selected),
            "detected_region": self.detected_region,
            "measured_at": self.measured_at,
            "probes": [probe.to_dict() for probe in self.probes],
        }


def probe_node(node: ReplayNode, *, timeout_seconds: float = 3.0) -> NodeProbe:
    started = time.perf_counter()
    try:
        response = requests.get(
            f"{node.url}/healthz",
            timeout=(timeout_seconds, timeout_seconds),
        )
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict) or payload.get("status") != "ok":
            raise RuntimeError("invalid replay node health response")
        region = str(payload.get("region", node.region))
        if region not in _REGIONS:
            raise RuntimeError(f"node advertised unsupported region {region!r}")
        return NodeProbe(
            node=node,
            healthy=True,
            latency_ms=(time.perf_counter() - started) * 1000.0,
            advertised_name=str(payload.get("node", node.name)),
            advertised_region=region,  # type: ignore[arg-type]
        )
    except (requests.RequestException, RuntimeError, ValueError) as exc:
        return NodeProbe(node=node, healthy=False, latency_ms=None, error=str(exc))


def _node_fingerprint(nodes: tuple[ReplayNode, ...]) -> str:
    payload = json.dumps([asdict(node) for node in nodes], sort_keys=True).encode()
    return hashlib.sha256(payload).hexdigest()


def _read_cached_recommendation(
    path: Path,
    *,
    nodes: tuple[ReplayNode, ...],
    now: float,
    max_age_seconds: float,
) -> NodeRecommendation | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("node_fingerprint") != _node_fingerprint(nodes):
            return None
        measured_at = float(data["measured_at"])
        if now - measured_at >= max_age_seconds:
            return None
        selected_data = data["selected"]
        selected = ReplayNode(**selected_data)
        configured = {node.url: node for node in nodes}
        if selected.url not in configured:
            return None
        detected_region = data["detected_region"]
        if detected_region not in _REGIONS:
            return None
        return NodeRecommendation(
            selected=configured[selected.url],
            detected_region=detected_region,
            measured_at=measured_at,
            probes=(),
        )
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


def _write_cached_recommendation(
    path: Path,
    recommendation: NodeRecommendation,
    nodes: tuple[ReplayNode, ...],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        **recommendation.to_dict(),
        "node_fingerprint": _node_fingerprint(nodes),
    }
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f".{path.name}.",
            suffix=".part",
            dir=path.parent,
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(payload, temporary, sort_keys=True)
            temporary.write("\n")
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)


def recommend_node(
    nodes: tuple[ReplayNode, ...],
    *,
    cache_path: str | Path | None = None,
    max_age_seconds: float = _DEFAULT_REFRESH_SECONDS,
    force: bool = False,
    timeout_seconds: float = 3.0,
) -> NodeRecommendation:
    """Return the fastest healthy node, refreshing the result at most hourly."""
    if not nodes:
        raise ValueError("at least one replay node is required")
    if max_age_seconds < 0:
        raise ValueError("max_age_seconds cannot be negative")
    path = (
        Path(cache_path)
        if cache_path is not None
        else CACHE_DIR / "replay-node-recommendation.json"
    )
    now = time.time()
    if not force:
        cached = _read_cached_recommendation(
            path,
            nodes=nodes,
            now=now,
            max_age_seconds=max_age_seconds,
        )
        if cached is not None:
            return cached

    probes: list[NodeProbe] = []
    with ThreadPoolExecutor(max_workers=min(8, len(nodes))) as executor:
        futures = {
            executor.submit(probe_node, node, timeout_seconds=timeout_seconds): node
            for node in nodes
        }
        for future in as_completed(futures):
            probes.append(future.result())
    probes.sort(key=lambda probe: nodes.index(probe.node))
    healthy = [probe for probe in probes if probe.healthy and probe.latency_ms is not None]
    if not healthy:
        details = "; ".join(f"{probe.node.name}: {probe.error}" for probe in probes)
        raise RuntimeError(f"no healthy replay nodes: {details}")
    fastest = min(
        healthy,
        key=lambda probe: probe.latency_ms if probe.latency_ms is not None else float("inf"),
    )
    detected_region = fastest.advertised_region or fastest.node.region
    recommendation = NodeRecommendation(
        selected=fastest.node,
        detected_region=detected_region,
        measured_at=now,
        probes=tuple(probes),
    )
    _write_cached_recommendation(path, recommendation, nodes)
    return recommendation


def load_replay_nodes(value: str | None = None) -> tuple[ReplayNode, ...]:
    """Load node definitions from DIMOS_REPLAY_NODES JSON."""
    raw = value if value is not None else os.environ.get("DIMOS_REPLAY_NODES")
    if not raw:
        return ()
    data = json.loads(raw)
    if not isinstance(data, list) or any(not isinstance(item, dict) for item in data):
        raise ValueError("DIMOS_REPLAY_NODES must be a JSON array of node objects")
    return tuple(ReplayNode(**item) for item in data)


def choose_server_url(explicit: str | None) -> str:
    """Use an explicit URL, the hourly node recommendation, or localhost."""
    if explicit:
        return _server_url(explicit)
    configured_url = os.environ.get("DIMOS_REPLAY_SERVER_URL")
    if configured_url:
        return _server_url(configured_url)
    nodes = load_replay_nodes()
    if nodes:
        return recommend_node(nodes).selected.url
    return "http://127.0.0.1:8765"
