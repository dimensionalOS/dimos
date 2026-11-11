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

from dimos_lcm import (
    geometry_msgs as lcm_geometry_msgs,
    nav_msgs as lcm_nav_msgs,
    sensor_msgs as lcm_sensor_msgs,
    std_msgs as lcm_std_msgs,
    tf2_msgs as lcm_tf2_msgs,
)
import numpy as np

from dimos.web.lcm_codec import (
    LCM_PRIMITIVES,
    LCM_V1_SUFFIX,
    canonical_name,
    encode_lcm_v1,
    export_schema,
    schema_class_for,
)
from dimos.web.relay_bridge.locate import find_web_dir

TS = 1757400000.5  # exact in float64: stamp (1757400000, 500000000)
Q = lcm_geometry_msgs.Quaternion(0.0, 0.0, 0.6, 0.8)  # unit, exact doubles


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
    pose = lcm_geometry_msgs.Pose(position=lcm_geometry_msgs.Point(1.5, -2.5, 0.25), orientation=Q)
    origin = lcm_geometry_msgs.Pose(
        position=lcm_geometry_msgs.Point(-5.0, -5.0, 0.0),
        orientation=lcm_geometry_msgs.Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    transforms = [
        lcm_geometry_msgs.TransformStamped(
            header=_header("map", seq=1),
            child_frame_id="base_link",
            transform=lcm_geometry_msgs.Transform(
                translation=lcm_geometry_msgs.Vector3(0.1, 0, 0.5), rotation=Q
            ),
        ),
        lcm_geometry_msgs.TransformStamped(
            header=_header("base_link", seq=1),
            child_frame_id="camera",
            transform=lcm_geometry_msgs.Transform(
                translation=lcm_geometry_msgs.Vector3(-1, 2, 0),
                rotation=lcm_geometry_msgs.Quaternion(0, 0, 0, 1),
            ),
        ),
    ]
    return [
        ("pose_stamped", lcm_geometry_msgs.PoseStamped(header=_header("map"), pose=pose)),
        ("laser_scan", laser_scan),
        (
            "imu",
            lcm_sensor_msgs.Imu(
                header=_header("imu"),
                orientation=Q,
                angular_velocity=lcm_geometry_msgs.Vector3(0.1, -0.2, 0.3),
                linear_acceleration=lcm_geometry_msgs.Vector3(0, 0, 9.5),
                orientation_covariance=[0.01] * 9,
                angular_velocity_covariance=[float(i) for i in range(9)],
                linear_acceleration_covariance=[0.03] * 9,
            ),
        ),
        (
            "occupancy_grid",
            lcm_nav_msgs.OccupancyGrid(
                header=_header("map"),
                info=lcm_nav_msgs.MapMetaData(
                    map_load_time=lcm_std_msgs.Time(int(TS), 500000000),
                    resolution=0.05,
                    width=3,
                    height=2,
                    origin=origin,
                ),
                data_length=6,
                data=[0, 100, -1, 50, 0, 7],
            ),
        ),
        (
            "joint_state",
            lcm_sensor_msgs.JointState(
                header=_header("arm"),
                name_length=3,
                name=["shoulder", "elbow", "wrist_é"],
                position_length=3,
                position=[0.1, -0.2, 0.3],
                velocity_length=3,
                velocity=[0.01, 0.02, 0.03],
                effort_length=3,
                effort=[1.0, 2.0, 3.0],
            ),
        ),
        ("point_cloud", point_cloud),
        ("tf_message", lcm_tf2_msgs.TFMessage(transforms_length=2, transforms=transforms)),
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
                "encoding": canonical_name(type(msg)) + LCM_V1_SUFFIX,
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
