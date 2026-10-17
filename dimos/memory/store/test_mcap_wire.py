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

"""The self-describing dimos wire channel: naming and decode, no file needed."""

import pytest

from dimos.memory.store.mcap import McapStore, _dimos_wire, _DimosCodec, _slug
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_a_dimos_topic_names_its_own_type():
    assert _dimos_wire("dimos/planner_path/nav_msgs.Path") == ("planner_path", Path)
    assert _dimos_wire("dimos/local_map/sensor_msgs.PointCloud2") == (
        "local_map",
        PointCloud2,
    )


def test_everything_else_stays_the_injected_codecs_business():
    # a DDS topic, an app topic, a dimos topic whose type does not resolve, and
    # a bare dimos prefix with no port
    assert _dimos_wire("rt/utlidar/cloud") is None
    assert _dimos_wire("control_log") is None
    assert _dimos_wire("dimos/mystery/not_a_package.Nope") is None
    assert _dimos_wire("dimos/nav_msgs.Path") is None


def test_the_stream_name_is_the_port_not_the_wire_name():
    # the type segment is dropped: "dimos_planner_path_nav_msgs.Path" carries a
    # dot, so it is not addressable as store.streams.<name>
    assert _slug("dimos/planner_path/nav_msgs.Path") == "planner_path"
    assert "." not in _slug("dimos/tf/tf2_msgs.TFMessage")
    # and the rt/ and app cases are untouched
    assert _slug("rt/utlidar/cloud") == "utlidar_cloud"
    assert _slug("control_log") == "control_log"


def test_the_codec_decodes_the_wire_bytes():
    path = Path(
        ts=1.0,
        frame_id="odom",
        poses=[PoseStamped(ts=1.0, frame_id="odom", position=Vector3(0.5, 0.0, 0.0))],
    )
    _, msg_type = _dimos_wire("dimos/path/nav_msgs.Path")
    got = _DimosCodec(msg_type).decode(path.lcm_encode())
    assert isinstance(got, Path)
    assert got.frame_id == "odom"
    assert abs(got.poses[0].position.x - 0.5) < 1e-9


def _write(path, topics):
    """Empty channels are enough: names are decided at registration."""
    mcap_writer = pytest.importorskip("mcap.writer")
    with path.open("wb") as output:
        writer = mcap_writer.Writer(output)
        writer.start(profile="dimos", library="test")
        for topic in topics:
            writer.register_channel(topic=topic, message_encoding="lcm", schema_id=0)
        writer.finish()


def test_two_types_on_one_port_stay_two_streams(tmp_path):
    path = tmp_path / "shared.mcap"
    _write(path, ["dimos/shared/sensor_msgs.Imu", "dimos/shared/geometry_msgs.Vector3"])
    with McapStore(path=str(path)) as store:
        assert store.list_streams() == ["shared", "shared_Vector3"]


def test_a_real_collision_is_refused_not_overwritten(tmp_path):
    path = tmp_path / "slash.mcap"
    _write(path, ["dimos/slash/name/nav_msgs.Path", "dimos/slash_name/nav_msgs.Path"])
    with pytest.raises(ValueError, match="slash_name"):
        McapStore(path=str(path))
    # and the way out is to name them yourself
    with McapStore(path=str(path), streams={"a": "dimos/slash/name/nav_msgs.Path"}) as store:
        assert "a" in store.list_streams()
