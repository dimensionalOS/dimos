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

"""Recorded generated TF messages feed the offline planner viewer."""

from unittest.mock import patch

from dimos_generated.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.memory.type.observation import Observation
from dimos.navigation.nav_3d.mls_planner.utils.plan_rrd import _log_odometry, _TfSync
from dimos.visualization.rerun.message_helpers import register_colormap_annotation


def test_tf_sync_generated_edges_and_odometry() -> None:
    edge = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="base_link",
        transform=Transform(translation=Vector3(x=2), rotation=Quaternion(w=1)),
    )
    message = TFMessage.decode(TFMessage(transforms=[edge]).encode())
    sync = _TfSync([Observation(id=0, ts=1.0, _data=message)])
    with patch("rerun.log") as log:
        sync.up_to(0.5)
        log.assert_not_called()
        sync.up_to(1.0)
        entity, archetype = log.call_args.args
        assert entity == "tf_links/base_link"
        assert archetype.parent_frame.as_arrow_array().to_pylist() == ["tf#/world"]
        assert archetype.child_frame.as_arrow_array().to_pylist() == ["tf#/base_link"]
        assert archetype.translation.as_arrow_array().to_pylist() == [[2.0, 0.0, 0.0]]
        sync.up_to(2.0)
        assert log.call_count == 1
        _log_odometry((2, 0, 0, 0, 0, 0, 1), 1.0, [], edge)
        assert log.call_args.args[0] == "world/robot_body"
        assert log.call_args.args[1].translation.as_arrow_array().to_pylist() == [[2.0, 0.0, 0.0]]


def test_colormap_annotation_has_all_indices() -> None:
    with patch("rerun.log") as log:
        register_colormap_annotation()
        assert log.call_args.args[0] == "/"
        assert log.call_args.kwargs == {"static": True}
        context = log.call_args.args[1].context.as_arrow_array().to_pylist()
        assert len(context[0]) == 256
