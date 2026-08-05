# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import dataclass
import time
from typing import Any
from unittest.mock import patch

import rerun as rr

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.rerun.bridge import RerunBridgeModule


@dataclass
class Topic:
    name: str


def _bridge() -> RerunBridgeModule:
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    return bridge


def _logged_paths(mock_log: Any, archetype: type = rr.Transform3D) -> list[str]:
    """Paths logged with the given archetype, transforms by default.

    Each newly committed path also gets a TransformAxes3D, so filtering by archetype
    keeps these assertions about tree shape rather than about what is drawn.
    """
    return [call.args[0] for call in mock_log.call_args_list if isinstance(call.args[1], archetype)]


def test_tfmessage_logs_tree_shaped_entity_paths() -> None:
    """Entity paths mirror the tf tree, not the flat child frame ids."""
    bridge = _bridge()

    try:
        with patch("rerun.log") as mock_log:
            bridge._on_message(
                TFMessage(
                    Transform(frame_id="odom", child_frame_id="base_link", ts=1.0),
                    Transform(frame_id="base_link", child_frame_id="camera", ts=1.0),
                ),
                Topic("/tf"),
            )
            paths = _logged_paths(mock_log)
            axes_paths = _logged_paths(mock_log, rr.TransformAxes3D)
            archetypes = [
                call.args[1]
                for call in mock_log.call_args_list
                if isinstance(call.args[1], rr.Transform3D)
            ]
    finally:
        bridge.stop()

    assert paths == ["world/tf/odom/base_link", "world/tf/odom/base_link/camera"]
    # a Transform3D renders nothing by itself, so every frame gets axes to make it visible
    assert axes_paths == paths
    # named frames are kept, so entities anchored on tf#/<frame> still resolve
    assert archetypes[1].parent_frame.as_arrow_array().to_pylist() == ["tf#/base_link"]
    assert archetypes[1].child_frame.as_arrow_array().to_pylist() == ["tf#/camera"]


def test_tfmessage_reanchors_when_parent_arrives_late() -> None:
    """A frame whose ancestry grows is cleared from its stale path and re-logged."""
    bridge = _bridge()
    camera = Transform(frame_id="base_link", child_frame_id="camera", ts=1.0)

    try:
        with patch("rerun.log") as mock_log:
            # camera first: base_link looks like a root
            bridge._on_message(TFMessage(camera), Topic("/tf"))
            assert _logged_paths(mock_log) == ["world/tf/base_link/camera"]

            # the parent edge shows up afterwards
            bridge._on_message(
                TFMessage(Transform(frame_id="odom", child_frame_id="base_link", ts=2.0)),
                Topic("/tf"),
            )
            mock_log.reset_mock()

            bridge._on_message(TFMessage(camera), Topic("/tf"))
            calls = list(mock_log.call_args_list)
    finally:
        bridge.stop()

    assert [(call.args[0], type(call.args[1])) for call in calls] == [
        # the stale branch goes first, then the frame re-appears deeper with fresh axes
        ("world/tf/base_link/camera", rr.Clear),
        ("world/tf/odom/base_link/camera", rr.TransformAxes3D),
        ("world/tf/odom/base_link/camera", rr.Transform3D),
    ]


def test_tf_chain_stops_on_cycle() -> None:
    """A malformed tree truncates the chain instead of hanging the bridge."""
    bridge = _bridge()
    bridge._tf_parents = {"a": "b", "b": "a"}

    try:
        assert bridge._tf_chain("b") == ["a", "b"]
    finally:
        bridge.stop()


def test_tfmessage_throttling_keeps_feeding_the_tree() -> None:
    """A throttled tf topic still updates the tree, so paths stay correct."""
    bridge = _bridge()

    try:
        bridge._on_message(
            TFMessage(Transform(frame_id="odom", child_frame_id="base_link", ts=1.0)),
            Topic("/tf"),
        )

        # rate limit the topic, having just logged, so the next message is dropped
        bridge._min_intervals = {"world/tf": 3600.0}
        bridge._last_log = {"world/tf": time.monotonic()}

        with patch("rerun.log") as mock_log:
            bridge._on_message(
                TFMessage(Transform(frame_id="base_link", child_frame_id="camera", ts=2.0)),
                Topic("/tf"),
            )
            logged = mock_log.call_args_list

        chain = bridge._tf_chain("camera")
    finally:
        bridge.stop()

    assert logged == []
    assert chain == ["odom", "base_link", "camera"]
