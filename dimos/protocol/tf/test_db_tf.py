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

from __future__ import annotations

from dimos.memory2.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.db_tf import DbTf


def _shift(parent: str, child: str, dx: float, ts: float) -> Transform:
    """Identity-rotation transform translated by ``dx`` along x."""
    return Transform(
        translation=Vector3(dx, 0.0, 0.0),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def test_from_store_returns_none_without_tf_stream() -> None:
    store = MemoryStore()
    with store:
        store.stream("lidar", str).append("frame", ts=1.0)
        assert DbTf.from_store(store) is None


def test_get_composes_chain_across_frames() -> None:
    store = MemoryStore()
    with store:
        tf_stream = store.stream("tf", TFMessage)
        tf_stream.append(TFMessage(_shift("world", "base_link", 1.0, 5.0)), ts=5.0)
        tf_stream.append(TFMessage(_shift("base_link", "lidar", 0.25, 5.0)), ts=5.0)

        db_tf = DbTf.from_store(store)
        assert db_tf is not None

        composed = db_tf.get("world", "lidar", time_point=5.0)
        assert composed is not None
        # world<-base_link (+1.0) composed with base_link<-lidar (+0.25).
        assert composed.translation.x == 1.25


def test_get_returns_none_for_unknown_frame() -> None:
    store = MemoryStore()
    with store:
        tf_stream = store.stream("tf", TFMessage)
        tf_stream.append(TFMessage(_shift("world", "base_link", 1.0, 5.0)), ts=5.0)

        db_tf = DbTf.from_store(store)
        assert db_tf is not None
        assert db_tf.get("world", "camera_optical", time_point=5.0) is None


def test_get_respects_time_tolerance() -> None:
    store = MemoryStore()
    with store:
        tf_stream = store.stream("tf", TFMessage)
        tf_stream.append(TFMessage(_shift("world", "base_link", 1.0, 5.0)), ts=5.0)

        db_tf = DbTf.from_store(store)
        assert db_tf is not None
        # A lookup far outside an explicit tolerance window finds nothing.
        assert db_tf.get("world", "base_link", time_point=100.0, time_tolerance=0.5) is None


def test_static_transform_latches_across_recording() -> None:
    """A once-published static transform resolves at any later timestamp."""
    store = MemoryStore()
    with store:
        tf_stream = store.stream("tf", TFMessage)
        # A single static mount published only at the start of the recording.
        tf_stream.append(TFMessage(_shift("world", "base_link", 1.0, 5.0)), ts=5.0)

        db_tf = DbTf.from_store(store)
        assert db_tf is not None
        # Default (unbounded) tolerance latches the static transform long after.
        far = db_tf.get("world", "base_link", time_point=1000.0)
        assert far is not None
        assert far.translation.x == 1.0
