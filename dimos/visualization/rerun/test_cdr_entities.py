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

"""Tests for refactored temporal memory components."""

from dimos_cdrgen.registry import decode as cdr_decode, encode as cdr_encode
import rerun as rr

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import EntityMarker, EntityMarkers
from dimos.messages.geometry_msgs.msg import Point
from dimos.messages.std_msgs.msg import Header
from dimos.visualization.rerun.cdr import entity_points


def test_markers_to_rerun() -> None:
    markers = EntityMarkers(
        markers=[
            EntityMarker(
                entity_id="E1",
                label="person walking",
                entity_type="person",
                position=Point(x=1.0, y=2.0, z=0.3),
            ),
            EntityMarker(
                entity_id="E2",
                label="wooden table",
                entity_type="object",
                position=Point(x=3.0, y=4.0, z=0.3),
            ),
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    archetype = entity_points(cdr_decode(cdr_encode(markers), EntityMarkers))
    # Should return rr.Points3D

    assert isinstance(archetype, rr.Points3D)
