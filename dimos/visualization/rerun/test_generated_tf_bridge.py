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

from types import SimpleNamespace
from unittest.mock import patch

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest
import rerun as rr

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.tf_tree import TfFrameTree


@pytest.mark.parametrize("axes", [0.0, 0.5])
def test_generated_tf_reaches_bridge_with_or_without_axes(axes: float) -> None:
    bridge = RerunBridgeModule(tf_axes=axes)
    bridge._min_intervals = {}
    edge = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="base_link",
        transform=Transform(translation=Vector3(x=3), rotation=Quaternion(w=1)),
    )
    message = TFMessage.decode(TFMessage(transforms=[edge]).encode())
    try:
        with patch("rerun.log") as log:
            bridge._on_message(message, SimpleNamespace(name="/tf"))
            transforms = [
                call for call in log.call_args_list if isinstance(call.args[1], rr.Transform3D)
            ]
            assert len(transforms) == 1
            assert transforms[0].args[0] == "tf_links/base_link"
            assert transforms[0].args[1].translation.as_arrow_array().to_pylist() == [[3, 0, 0]]
            if axes:
                assert bridge._tf_tree.placements()["base_link"] == "world/tf/world/base_link"
    finally:
        bridge.stop()


def test_generated_tree_reparents_and_clears_old_axes() -> None:
    tree = TfFrameTree(root="frames")
    edge = TransformStamped(header=Header(frame_id="first"), child_frame_id="child")
    with patch("rerun.log") as log:
        tree.update([edge])
        edge.header.frame_id = "second"
        tree.update([edge])
        assert tree.placements()["child"] == "frames/second/child"
        clears = [
            call
            for call in log.call_args_list
            if call.args[0] == "frames/first/child" and isinstance(call.args[1], rr.Arrows3D)
        ]
        assert len(clears) == 1
        assert clears[0].args[1].vectors.as_arrow_array().to_pylist() == []
