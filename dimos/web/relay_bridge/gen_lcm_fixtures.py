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

"""Golden *.lcm.v1 vectors: the Python side is the reference.

Writes web/shared/fixtures/lcm_frames.json: per message the encoding id, the
schema cockpit() puts in params["lcm"], the exact lcm_encode() bytes and the
decoded field values. vitest (web/sdk/src/decoders/lcm.test.ts) compiles the
schema, decodes payload_b64 and compares with value; pytest
(dimos/web/test_lcm_codec.py) re-exports the schema and re-encodes, so drift
on either side fails a suite.

Regenerate with:  uv run python -m dimos.web.relay_bridge.gen_lcm_fixtures

gen.ts does not write this file: the payloads must be the Python encoders'.
"""

from __future__ import annotations

import base64
import json
from typing import Any

from dimos_lcm import sensor_msgs as lcm_sensor_msgs, std_msgs as lcm_std_msgs
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.web.lcm_codec import (
    LCM_PRIMITIVES,
    default_encoding,
    encode_lcm_v1,
    export_schema,
    schema_class_for,
)
from dimos.web.relay_bridge.locate import find_web_dir

TS = 1757400000.5  # exact in float64: stamp (1757400000, 500000000)
Q = Quaternion(0.0, 0.0, 0.6, 0.8)  # unit, exact doubles


def _header(frame_id: str, seq: int = 0) -> Any:
    # Fresh instances: the generated constructors share one default header.
    stamp = lcm_std_msgs.Time(sec=int(TS), nsec=500_000_000)
    return lcm_std_msgs.Header(seq=seq, stamp=stamp, frame_id=frame_id)


def build_messages() -> list[tuple[str, Any]]:
    """One message per wire feature. Generated classes stand in where dimos
    has no overlay (LaserScan) or the overlay needs open3d (PointCloud2)."""
    laser_scan = lcm_sensor_msgs.LaserScan(
        ranges_length=5,
        intensities_length=0,
        header=_header("laser", seq=7),
        angle_min=-1.5,
        angle_max=1.5,
        angle_increment=0.75,
        time_increment=0.0,
        scan_time=0.1,
        range_min=0.25,
        range_max=30.0,
        ranges=[0.5, 1.25, 2.75, 30.0, 12.5],
        intensities=[],
    )
    xyz = np.array([[1.0, 2.0, 3.0], [-4.0, 5.5, 6.25]], dtype=np.float32)
    point_cloud = lcm_sensor_msgs.PointCloud2(
        fields_length=3,
        data_length=xyz.nbytes,
        header=_header("lidar"),
        height=1,
        width=2,
        fields=[
            lcm_sensor_msgs.PointField(name=axis, offset=4 * i, datatype=7, count=1)
            for i, axis in enumerate("xyz")
        ],
        is_bigendian=False,
        point_step=12,
        row_step=xyz.nbytes,
        data=xyz.tobytes(),
        is_dense=True,
    )
    return [
        # doubles, nested structs, string, int32
        (
            "pose_stamped",
            PoseStamped(ts=TS, frame_id="map", position=[1.5, -2.5, 0.25], orientation=Q),
        ),
        # variable float arrays, one of them empty
        ("laser_scan", laser_scan),
        # fixed double[9] arrays
        (
            "imu",
            Imu(
                angular_velocity=Vector3(0.1, -0.2, 0.3),
                linear_acceleration=Vector3(0.0, 0.0, 9.5),
                orientation=Q,
                orientation_covariance=[0.01] * 9,
                angular_velocity_covariance=[float(i) for i in range(9)],
                linear_acceleration_covariance=[0.03] * 9,
                frame_id="imu",
                ts=TS,
            ),
        ),
        # int8 cells with negatives, MapMetaData (float32 resolution)
        (
            "occupancy_grid",
            OccupancyGrid(
                grid=np.array([[0, 100, -1], [50, 0, 7]], dtype=np.int8),
                resolution=0.05,
                origin=Pose(-5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                frame_id="map",
                ts=TS,
            ),
        ),
        # string arrays, non-ASCII
        (
            "joint_state",
            JointState(
                ts=TS,
                frame_id="arm",
                name=["shoulder", "elbow", "wrist_é"],
                position=[0.1, -0.2, 0.3],
                velocity=[0.01, 0.02, 0.03],
                effort=[1.0, 2.0, 3.0],
            ),
        ),
        # byte[], booleans, a struct array with a byte scalar
        ("point_cloud", point_cloud),
        # struct array with strings
        (
            "tf_message",
            TFMessage(
                Transform(
                    translation=Vector3(0.1, 0.0, 0.5),
                    rotation=Q,
                    frame_id="map",
                    child_frame_id="base_link",
                    ts=TS,
                ),
                Transform(
                    translation=Vector3(-1.0, 2.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="base_link",
                    child_frame_id="camera",
                    ts=TS,
                ),
            ),
        ),
        # int64 beyond 2^53: a lossy decoder cannot reproduce it
        ("int64", lcm_std_msgs.Int64(data=-9007199254740993)),
    ]


def walk_value(obj: Any, cls: type[Any]) -> Any:
    """Decoded generated object -> the JSON value the browser decoder must
    produce: byte arrays as base64, int64 as decimal strings, the rest plain."""
    out: dict[str, Any] = {}
    for slot, type_name, dims in zip(
        cls.__slots__, cls.__typenames__, cls.__dimensions__, strict=True
    ):
        value = getattr(obj, slot)
        if type_name not in LCM_PRIMITIVES:
            sub = cls._get_field_type(slot)
            out[slot] = (
                walk_value(value, sub) if dims is None else [walk_value(v, sub) for v in value]
            )
        elif dims is None:
            out[slot] = str(value) if type_name == "int64_t" else value
        elif type_name == "byte":
            out[slot] = base64.b64encode(bytes(value)).decode()
        elif type_name == "int64_t":
            out[slot] = [str(v) for v in value]
        else:
            out[slot] = list(value)
    return out


def build_vectors() -> list[dict[str, Any]]:
    vectors: list[dict[str, Any]] = []
    for name, msg in build_messages():
        generated = schema_class_for(type(msg))
        schema = export_schema(generated)
        payload = encode_lcm_v1(msg, {"lcm": schema})
        vectors.append(
            {
                "name": name,
                "encoding": default_encoding(type(msg), "rx"),
                "schema": schema,
                "payload_b64": base64.b64encode(payload).decode(),
                "value": walk_value(generated.lcm_decode(payload), generated),
            }
        )
    return vectors


def main() -> None:
    path = find_web_dir() / "shared" / "fixtures" / "lcm_frames.json"
    path.write_text(json.dumps({"vectors": build_vectors()}, indent=2) + "\n")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
