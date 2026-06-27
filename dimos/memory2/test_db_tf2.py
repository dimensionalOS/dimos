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

"""Tests for DbTf2 (graph-stream tf lookups): the in-RAM graph cache vs query
fallback, correctness vs a full-load buffer, latched-static handling, and disjoint
(multi-robot) graphs. Data is written one-transform-per-row + child_frame tag +
topology change-log, exactly as the live recorder does."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.memory2.db_tf2 import DbTf2, TfGraphWriter
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

_T0 = 1000.0
_DYN_RATE = 30.0
_DURATION = 10.0


def _yaw(theta: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


def _static(parent: str, child: str, xyz: tuple[float, float, float], ts: float) -> Transform:
    return Transform(
        translation=Vector3(*xyz),
        rotation=Quaternion(0, 0, 0, 1),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def _append(
    store: SqliteStore, graph: TfGraphWriter, transform: Transform, is_static: bool
) -> None:
    """Record one transform exactly as the live recorder does: one row, tagged
    child_frame, plus a topology-change row when the structure changes."""
    store.stream("tf", TFMessage).append(
        TFMessage(transform),
        ts=transform.ts,
        pose=None,
        tags={"child_frame": transform.child_frame_id},
    )
    graph.record(transform.child_frame_id, transform.frame_id, is_static, transform.ts)


def _record_single_robot(path: Path, *, static_repeat: bool) -> list[Transform]:
    """world->map->odom->base_link->sensor. Statics emitted once (latched) unless
    static_repeat, in which case they're re-emitted each second."""
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")
    written: list[Transform] = []

    statics = [
        ("world", "map", (0.0, 0.0, 0.0)),
        ("map", "odom", (0.0, 0.0, 0.0)),
        ("base_link", "sensor", (0.0, 0.0, 0.3)),
    ]
    static_times = [_T0 + j for j in range(int(_DURATION))] if static_repeat else [_T0]
    for ts in static_times:
        for parent, child, xyz in statics:
            t = _static(parent, child, xyz, ts)
            _append(store, graph, t, is_static=True)
            written.append(t)

    for i in range(int(_DURATION * _DYN_RATE)):
        ts = _T0 + i / _DYN_RATE
        t = Transform(
            translation=Vector3(0.5 * i / _DYN_RATE, 0.1 * i / _DYN_RATE, 0.0),
            rotation=_yaw(0.02 * i),
            frame_id="odom",
            child_frame_id="base_link",
            ts=ts,
        )
        _append(store, graph, t, is_static=False)
        written.append(t)

    graph.close()
    store.stop()
    return written


def _reference(transforms: list[Transform]) -> MultiTBuffer:
    buffer = MultiTBuffer(buffer_size=1.0e15)
    buffer.receive_transform(*transforms)
    return buffer


def _diff(a: Transform, b: Transform) -> float:
    return (
        abs(a.translation.x - b.translation.x)
        + abs(a.translation.y - b.translation.y)
        + abs(a.translation.z - b.translation.z)
        + abs(a.rotation.x - b.rotation.x)
        + abs(a.rotation.y - b.rotation.y)
        + abs(a.rotation.z - b.rotation.z)
        + abs(a.rotation.w - b.rotation.w)
    )


def test_correctness_vs_full_load(tmp_path: Path) -> None:
    """With statics re-emitted (so a naive full-load buffer also resolves), DbTf2
    matches it within tolerance at off-sample (interpolated) query times."""
    transforms = _record_single_robot(tmp_path / "r.db", static_repeat=True)
    reference = _reference(transforms)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store)
    compared = 0
    for k in range(25):
        q = _T0 + 0.013 + k * 0.317
        want = reference.lookup("world", "sensor", q, 0.5)
        got = db.get("world", "sensor", q, 0.5)
        assert (want is None) == (got is None), f"None mismatch at {q}"
        if want is not None and got is not None:
            assert _diff(want, got) < 1e-6, f"diff at {q}: {_diff(want, got)}"
            compared += 1
    assert compared >= 20
    store.stop()


def test_graph_loaded_into_ram_when_few_changes(tmp_path: Path) -> None:
    """A stable single-robot tree has few topology changes → graph is held in RAM →
    lookups issue zero graph queries."""
    _record_single_robot(tmp_path / "r.db", static_repeat=True)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store, max_graph_changes_in_ram=20)
    for k in range(30):
        db.get("world", "sensor", _T0 + 0.05 + k * 0.3, 0.5)
    assert db.graph_queries == 0  # never queried the graph table


def test_fallback_to_query_when_many_changes(tmp_path: Path) -> None:
    """If there are at least the threshold many topology changes, DbTf2 does NOT
    load them into RAM and instead queries the graph per lookup (multi-robot churn)."""
    _record_single_robot(tmp_path / "r.db", static_repeat=True)  # 4 topology changes
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store, max_graph_changes_in_ram=2)  # 4 >= 2 → fallback
    n = 10
    for k in range(n):
        db.get("world", "sensor", _T0 + 0.05 + k * 0.3, 0.5)
    assert db.graph_queries == n  # one graph query per lookup


def test_latched_static_resolves(tmp_path: Path) -> None:
    """A static recorded once at the very start still resolves at a much later time
    (no bracket, no tolerance) — the case a plain time-bracket would drop."""
    transforms = _record_single_robot(tmp_path / "r.db", static_repeat=False)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf2(store)
    q = _T0 + 9.5  # ~9.5 s after the statics were recorded
    got = db.get("world", "sensor", q, 0.5)
    assert got is not None  # latched static did not get dropped
    # base_link→sensor is a fixed +0.3 z; the chain should reflect it (sanity)
    reference_dyn = _reference([t for t in transforms if t.child_frame_id == "base_link"])
    # world/map/odom are identity here, so world->base_link == odom->base_link
    odom_to_base = reference_dyn.lookup("odom", "base_link", q, 0.5)
    assert odom_to_base is not None
    store.stop()


def test_disjoint_graph_returns_none(tmp_path: Path) -> None:
    """Two unconnected components (two robots) → a cross-component query is None."""
    path = tmp_path / "two.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(str(path), "tf")
    # robot A: worldA -> baseA ; robot B: worldB -> baseB   (no shared frame)
    for i in range(20):
        ts = _T0 + i / _DYN_RATE
        _append(
            store,
            graph,
            Transform(
                translation=Vector3(i * 0.1, 0, 0),
                rotation=_yaw(0),
                frame_id="worldA",
                child_frame_id="baseA",
                ts=ts,
            ),
            is_static=False,
        )
        _append(
            store,
            graph,
            Transform(
                translation=Vector3(0, i * 0.1, 0),
                rotation=_yaw(0),
                frame_id="worldB",
                child_frame_id="baseB",
                ts=ts,
            ),
            is_static=False,
        )
    graph.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf2(store)
    q = _T0 + 0.3
    assert db.get("baseA", "worldA", q, 0.5) is not None  # same component: ok
    assert db.get("baseB", "baseA", q, 0.5) is None  # different components: no transform
    store.stop()


def test_threshold_is_configurable(tmp_path: Path) -> None:
    _record_single_robot(tmp_path / "r.db", static_repeat=True)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    assert DbTf2(store)._max_in_ram == 20  # default
    assert DbTf2(store, max_graph_changes_in_ram=5)._max_in_ram == 5
    store.stop()
