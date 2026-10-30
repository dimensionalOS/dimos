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

"""Tests for localize rig TF windowing."""

from __future__ import annotations

from dimos.memory.store.memory import MemoryStore
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.perception.localize.rig import TF_TOLERANCE, window_tf


def _tf(parent: str, child: str, ts: float, x: float = 0.0) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=child,
        ts=ts,
    )


def test_window_tf_latches_static_head_edges_for_late_lookups() -> None:
    """Static world/extrinsic edges published once at t=0 must resolve mid-window."""
    with MemoryStore() as store:
        stream = store.stream("tf", TFMessage)
        # Static chain once at the start (china_whale / Go2 bag pattern).
        stream.append(
            TFMessage(
                _tf("world", "map", 0.0),
                _tf("map", "odom", 0.0),
                _tf("base_link", "camera_optical", 0.0),
            ),
            ts=0.0,
        )
        # Only the odom chain streams during the later localize window.
        for k in range(20):
            ts = 100.0 + 0.5 * k
            stream.append(
                TFMessage(_tf("odom", "base_link", ts, x=float(k))),
                ts=ts,
            )

        tf = window_tf(store, 100.0, 110.0)
        mid = 105.0
        pose = tf.get("camera_optical", "world", mid, TF_TOLERANCE, warn=False)
        assert pose is not None
        # get(camera, world) walks the inverse of odom→base (x≈10 at mid).
        assert pose.translation.x == -10.0


def test_window_tf_keeps_dynamic_odom_without_static_restamp_blowup() -> None:
    """Continuous odom edges stay time-varying; mid lookup follows the moving sample."""
    with MemoryStore() as store:
        stream = store.stream("tf", TFMessage)
        stream.append(TFMessage(_tf("world", "odom", 0.0)), ts=0.0)
        for k in range(10):
            ts = 50.0 + k
            stream.append(TFMessage(_tf("odom", "base_link", ts, x=float(k))), ts=ts)

        tf = window_tf(store, 50.0, 60.0)
        early = tf.get("base_link", "world", 51.0, TF_TOLERANCE, warn=False)
        late = tf.get("base_link", "world", 58.0, TF_TOLERANCE, warn=False)
        assert early is not None and late is not None
        assert early.translation.x == -1.0
        assert late.translation.x == -8.0
