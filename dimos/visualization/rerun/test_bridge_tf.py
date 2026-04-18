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

import sys
import time
import types

from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.tf import MultiTBuffer
from dimos.visualization.rerun.bridge import RerunBridgeModule


class _DummyRPC:
    def serve_module_rpc(self, _module) -> None:  # type: ignore[no-untyped-def]
        return None

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None


class FakeTransform3D:
    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.kwargs = kwargs


class FakeQuaternion:
    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.kwargs = kwargs


def test_bridge_reconstructs_tf_paths(monkeypatch) -> None:
    logged: list[tuple[str, FakeTransform3D, bool]] = []
    fake_rerun = types.SimpleNamespace(
        Quaternion=FakeQuaternion,
        Transform3D=FakeTransform3D,
        log=lambda path, archetype, static=False: logged.append((path, archetype, static)),
    )
    monkeypatch.setitem(sys.modules, "rerun", fake_rerun)

    bridge = RerunBridgeModule(viewer_mode="none", pubsubs=[], rpc_transport=_DummyRPC)
    bridge._tf_buffer = MultiTBuffer()
    unsubscribe = bridge._tf_buffer.subscribe(bridge._log_tf_transform)

    try:
        bridge._on_message(
            TFMessage(
                Transform(frame_id="world", child_frame_id="robot", ts=time.time()),
                Transform(frame_id="robot", child_frame_id="camera", ts=time.time()),
            ),
            Topic("/tf", TFMessage),
        )
    finally:
        unsubscribe()
        bridge.stop()

    assert [path for path, _, _ in logged] == [
        "world/tf/robot",
        "world/tf/robot/camera",
    ]
    assert logged[1][1].kwargs["parent_frame"] == "tf#/robot"
    assert logged[1][1].kwargs["child_frame"] == "tf#/camera"
