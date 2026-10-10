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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos_generated.geometry_msgs.msg import Point
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import encode as cdr_encode, schema as cdr_schema
import numpy as np
import rerun as rr
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from dimos.msgs.helpers import resolve_msg_type
from dimos.visualization.rerun.message_helpers import entity_points


def test_generated_entity_schema_independently_decodes_labels_positions_and_stamp() -> None:
    message = EntityMarkers(
        header=Header(stamp=Time(sec=1700000000, nanosec=999999999), frame_id="world"),
        markers=[
            EntityMarker(
                entity_id="E1",
                label="机器人",
                entity_type="person",
                position=Point(x=1.0, y=2.0, z=0.3),
            )
        ],
    )
    independent = get_typestore(Stores.ROS2_HUMBLE)
    independent.register(
        get_types_from_msg(cdr_schema(EntityMarkers.__msgtype__), EntityMarkers.__msgtype__)
    )
    result = independent.deserialize_cdr(cdr_encode(message), EntityMarkers.__msgtype__)
    assert result.header.frame_id == "world"
    assert (result.header.stamp.sec, result.header.stamp.nanosec) == (1700000000, 999999999)
    (marker,) = result.markers
    assert (marker.entity_id, marker.label, marker.entity_type) == ("E1", "机器人", "person")
    assert (marker.position.x, marker.position.y, marker.position.z) == (1, 2, 0.3)
    assert resolve_msg_type(EntityMarkers.__msgtype__) is EntityMarkers


def test_generated_entity_viewer_helper_preserves_points_labels_and_radius() -> None:
    message = EntityMarkers(
        markers=[
            EntityMarker(
                entity_id="E1",
                label="person walking",
                entity_type="person",
                position=Point(x=1, y=2, z=0.3),
            ),
            EntityMarker(
                entity_id="E2", label="table", entity_type="object", position=Point(x=3, y=4, z=0.3)
            ),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    archetype = entity_points(message)
    assert isinstance(archetype, rr.Points3D)
    assert np.allclose(archetype.positions.as_arrow_array().to_pylist(), [[1, 2, 0.3], [3, 4, 0.3]])
    assert archetype.labels.as_arrow_array().to_pylist() == ["E1: person walking", "E2: table"]
    assert np.allclose(archetype.radii.as_arrow_array().to_pylist(), [0.15, 0.15])
    assert not hasattr(message, "to_rerun")
