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

import argparse
from dataclasses import dataclass
from enum import Enum
import hashlib
import json
from pathlib import Path
import pickle
import pickletools
from typing import Any

from dimos_generated.geometry_msgs.msg import PoseStamped
from dimos_generated.sensor_msgs.msg import Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.core.multiarray import _reconstruct

from dimos.msgs.image import image_from_array, image_view
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds
from dimos.robot.unitree.type.odometry import pose_from_webrtc_odometry


# One-time export of the hash-verified project archive. These inert state records
# are not installed in DimOS and are never used by runtime or pytest readers.
@dataclass
class ArchivedState:
    def __setstate__(self, state: Any) -> None:
        if isinstance(state, tuple):
            state = state[0] | state[1]
        self.__dict__.update(state)


class ArchivedFormat(Enum):
    BGR = "BGR"


def points_array(points: Any) -> Any:
    return points


class ArchiveReader(pickle.Unpickler):
    def find_class(self, module: str, name: str) -> Any:
        allowed = {
            ("dimos.msgs.sensor_msgs.Image", "Image"): ArchivedState,
            ("dimos.msgs.sensor_msgs.Image", "ImageFormat"): ArchivedFormat,
            ("dimos.robot.unitree_webrtc.type.lidar", "LidarMessage"): ArchivedState,
            ("dimos.msgs.geometry_msgs.Vector3", "Vector3"): ArchivedState,
            ("dimos.core.o3dpickle", "reconstruct_pointcloud"): points_array,
            ("numpy.core.multiarray", "_reconstruct"): _reconstruct,
            ("numpy", "ndarray"): np.ndarray,
            ("numpy", "dtype"): np.dtype,
        }
        if (module, name) not in allowed:
            raise ValueError(f"Unexpected archived global: {module}.{name}")
        return allowed[module, name]


parser = argparse.ArgumentParser(
    description="Export only the five pinned Go2 regression moments; not a runtime legacy reader."
)
parser.add_argument(
    "--source",
    type=Path,
    required=True,
    help="Safely extracted unitree_go2_lidar_corrected directory",
)
parser.add_argument("--archive", type=Path, required=True, help="Original project LFS archive")
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
with args.archive.open("rb") as archive:
    digest = hashlib.sha256()
    for chunk in iter(lambda: archive.read(1024 * 1024), b""):
        digest.update(chunk)
    assert digest.hexdigest() == "51a817f2b5664c9e2f2856293db242e030f0edce276e21da0edc2821d947aad2"
root = args.source
out = args.output
out.mkdir(exist_ok=False)


def stamp(path: Path) -> float:
    with path.open("rb") as f:
        for op, arg, _pos in pickletools.genops(f):
            if op.name == "BINFLOAT":
                assert isinstance(arg, float)
                return arg
    raise ValueError(path)


indexes = {
    k: [(stamp(p), p) for p in sorted((root / k).glob("*.pickle"))]
    for k in ["video", "lidar", "odom"]
}


def nearest(kind: str, target: float) -> tuple[float, Path, Any]:
    ts, p = min(indexes[kind], key=lambda item: abs(item[0] - target))
    with p.open("rb") as f:
        actual, value = ArchiveReader(f).load()
    assert actual == ts
    return ts, p, value


manifest: dict[str, Any] = {
    "source_lfs_sha256": "51a817f2b5664c9e2f2856293db242e030f0edce276e21da0edc2821d947aad2",
    "moments": [],
}
for seek in [10, 12, 14, 16, 18]:
    lidar = nearest("lidar", indexes["lidar"][0][0] + seek)
    selected = {
        "lidar": lidar,
        "video": nearest("video", lidar[2].ts),
        "odom": nearest("odom", lidar[2].ts),
    }
    entry: dict[str, Any] = {"seek": seek, "streams": {}}
    msg: Image | PointCloud2 | PoseStamped
    for kind, (ts, p, value) in selected.items():
        if kind == "lidar":
            msg = pointcloud_from_xyz(
                value.pointcloud,
                header=Header(stamp=time_from_seconds(value.ts), frame_id=value.frame_id),
            )
            np.testing.assert_allclose(pointcloud_xyz(msg), value.pointcloud, rtol=1e-6, atol=1e-6)
        elif kind == "video":
            assert value.format is ArchivedFormat.BGR
            msg = image_from_array(
                value.data,
                encoding="bgr8",
                header=Header(stamp=time_from_seconds(value.ts), frame_id=value.frame_id),
            )
            np.testing.assert_array_equal(image_view(msg), value.data)
        else:
            msg = pose_from_webrtc_odometry(value)
        data = msg.encode()
        assert type(msg).decode(data).encode() == data
        name = f"{seek}-{kind}.cdr"
        (out / name).write_bytes(data)
        entry["streams"][kind] = {
            "file": name,
            "msg_name": msg.msg_name,
            "recorded_ts": ts,
            "source": str(p.relative_to(root)),
            "source_sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            "cdr_sha256": hashlib.sha256(data).hexdigest(),
            "source_sec": msg.header.stamp.sec,
            "source_nanosec": msg.header.stamp.nanosec,
        }
    manifest["moments"].append(entry)
message_types: tuple[type[Image] | type[PointCloud2] | type[PoseStamped], ...] = (
    Image,
    PointCloud2,
    PoseStamped,
)
(out / "schemas.json").write_text(
    json.dumps({t.msg_name: t.schema for t in message_types}, indent=2) + "\n"
)
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(
    json.dumps(
        {
            "output": str(out),
            "moments": len(manifest["moments"]),
            "bytes": sum(p.stat().st_size for p in out.iterdir()),
        }
    )
)
