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

"""Integration tests for DbTf (graph-stream transform lookups). Data is written
one-transform-per-row + child_frame tag + topology change-log, exactly as the live
recorder does. These cover the behaviours that matter end-to-end: interpolation
against a full-load buffer, a latched static, time-varying topology (relocalization
re-parents a frame), and a disjoint multi-robot graph."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.memory2.db_tf import DbTf, TfGraphWriter
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

_T0 = 1000.0
_DYN_RATE = 30.0
_DURATION = 10.0
_NO_PRUNE = 1.0e15


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


def _ref(transforms: list[Transform]) -> MultiTBuffer:
    buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
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


def _record_single_robot(path: Path, *, static_repeat: bool) -> list[Transform]:
    """world->map->odom->base_link->sensor. Statics emitted once (latched) unless
    static_repeat, in which case they're re-emitted each second."""
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(store)
    written: list[Transform] = []

    statics = [
        ("world", "map", (0.0, 0.0, 0.0)),
        ("map", "odom", (0.0, 0.0, 0.0)),
        ("base_link", "sensor", (0.0, 0.0, 0.3)),
    ]
    static_times = [_T0 + j for j in range(int(_DURATION))] if static_repeat else [_T0]
    for ts in static_times:
        for parent, child, xyz in statics:
            transform = _static(parent, child, xyz, ts)
            _append(store, graph, transform, is_static=True)
            written.append(transform)

    for i in range(int(_DURATION * _DYN_RATE)):
        ts = _T0 + i / _DYN_RATE
        transform = Transform(
            translation=Vector3(0.5 * i / _DYN_RATE, 0.1 * i / _DYN_RATE, 0.0),
            rotation=_yaw(0.02 * i),
            frame_id="odom",
            child_frame_id="base_link",
            ts=ts,
        )
        _append(store, graph, transform, is_static=False)
        written.append(transform)

    graph.close()
    store.stop()
    return written


def test_interpolates_and_matches_full_load(tmp_path: Path) -> None:
    """At off-sample (interpolated) query times, DbTf matches a naive full-load
    buffer within tolerance — proving the chain compose + per-edge interpolation."""
    transforms = _record_single_robot(tmp_path / "r.db", static_repeat=True)
    reference = _ref(transforms)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf(store)
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


def test_latched_static_resolves(tmp_path: Path) -> None:
    """A static recorded once at the very start still resolves at a much later time
    (no bracket, no tolerance) — the case a plain time-bracket would drop."""
    _record_single_robot(tmp_path / "r.db", static_repeat=False)
    store = SqliteStore(path=str(tmp_path / "r.db"), must_exist=True)
    db = DbTf(store)
    assert db.get("world", "sensor", _T0 + 9.5, 0.5) is not None  # ~9.5s after statics
    store.stop()


def test_reparent_midrun_uses_graph_as_of_query_time(tmp_path: Path) -> None:
    """world->map(+10) and map->odom(+100) are non-identity statics; at t0+5 a
    relocalization re-parents base_link from odom to map. An early lookup composes
    the odom branch (+100), a late one does not (+10) — wrong topology is off by
    ~100. Exercises time-varying / multi-robot topology."""
    path = tmp_path / "reparent.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(store)
    _append(store, graph, _static("world", "map", (10.0, 0, 0), _T0), is_static=True)
    _append(store, graph, _static("map", "odom", (100.0, 0, 0), _T0), is_static=True)

    era1: list[Transform] = []
    for i in range(150):  # [t0, t0+5)
        ts = _T0 + i / _DYN_RATE
        transform = Transform(
            translation=Vector3(0.5 * i / _DYN_RATE, 0, 0),
            rotation=_yaw(0.01 * i),
            frame_id="odom",
            child_frame_id="base_link",
            ts=ts,
        )
        _append(store, graph, transform, is_static=False)
        era1.append(transform)
    switch = _T0 + 5.0
    era2: list[Transform] = []
    for i in range(150):  # [t0+5, t0+10): base_link now hangs off map directly
        ts = switch + i / _DYN_RATE
        transform = Transform(
            translation=Vector3(7.0 + 0.3 * i / _DYN_RATE, 0, 0),
            rotation=_yaw(0.01 * i),
            frame_id="map",
            child_frame_id="base_link",
            ts=ts,
        )
        _append(store, graph, transform, is_static=False)
        era2.append(transform)
    graph.close()
    store.stop()

    store = SqliteStore(path=str(path), must_exist=True)
    db = DbTf(store)

    q1 = _T0 + 2.013
    want1 = _ref(
        [
            _static("world", "map", (10.0, 0, 0), q1),
            _static("map", "odom", (100.0, 0, 0), q1),
            *era1,
        ]
    ).lookup("world", "base_link", q1, 0.5)
    got1 = db.get("world", "base_link", q1, 0.5)
    assert want1 is not None and got1 is not None
    assert _diff(want1, got1) < 1e-6
    assert got1.translation.x > 100.0  # odom branch present

    q2 = switch + 2.013
    want2 = _ref([_static("world", "map", (10.0, 0, 0), q2), *era2]).lookup(
        "world", "base_link", q2, 0.5
    )
    got2 = db.get("world", "base_link", q2, 0.5)
    assert want2 is not None and got2 is not None
    assert _diff(want2, got2) < 1e-6
    assert got2.translation.x < 20.0  # odom branch gone
    store.stop()


def test_disjoint_multirobot_returns_none(tmp_path: Path) -> None:
    """Two unconnected components (two robots, no shared frame) → a cross-component
    query is None, an in-component query resolves."""
    path = tmp_path / "two.db"
    store = SqliteStore(path=str(path))
    graph = TfGraphWriter(store)
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
    db = DbTf(store)
    q = _T0 + 0.3
    assert db.get("baseA", "worldA", q, 0.5) is not None  # same component
    assert db.get("baseB", "baseA", q, 0.5) is None  # different components
    store.stop()
