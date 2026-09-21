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

"""Full LFS transcription through Rust, checked against independent ROS readers."""

from __future__ import annotations

from collections import Counter
from contextlib import closing
from fractions import Fraction
import hashlib
import itertools
import json
from pathlib import Path
import re
import sqlite3
import subprocess
import sys
from tempfile import TemporaryDirectory
from typing import Any

from dimos_lcm.geometry_msgs import PoseStamped, Transform as LCMTransform, TransformStamped
from dimos_lcm.sensor_msgs import Image, PointCloud2
from dimos_lcm.tf2_msgs import TFMessage as LCMTFMessage
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
import numpy as np
import pytest
import requests

from dimos.memory.cli.dataset import open_store
from dimos.memory.cli.render import render_store
from dimos.memory.cli.summary import main as summarize
from dimos.memory.codecs.jpeg import JpegCodec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as DimosPoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as DimosPointCloud2
from dimos.robot.unitree.go2.dds.extrinsics import CAM_Q, CAM_T
from dimos.utils.data import get_data

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.timeout(1200)]

COUNTS = {"color_image": 855, "lidar": 461, "odom": 1122}
TYPES = {"color_image": Image, "lidar": PointCloud2, "odom": PoseStamped}
SCHEMAS = {
    "color_image": "sensor_msgs/msg/Image",
    "lidar": "sensor_msgs/msg/PointCloud2",
    "odom": "geometry_msgs/msg/PoseStamped",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
SNAPSHOTS = {
    "geometry2": "f702874b1c8535d6a038230ab2cda0ba5d521ebd",
    "common_interfaces": "a941f14bb318d8d904505ed935ccbb97f24a70a4",
    "rcl_interfaces": "7aa3caf43377ea6ad615bc1040832e2c7566bfbe",
}
LFS_SHA256 = "8a19846a0adf5755815fd039492c0255e0bc282e9df75a06648d7585cae8d2d2"


def _rows(db: sqlite3.Connection, name: str) -> sqlite3.Cursor:
    # Only the fixed stream names above are interpolated; source metadata is not imported.
    assert name in TYPES
    return db.execute(
        f'SELECT o.ts, b.data FROM "{name}" o JOIN "{name}_blob" b ON o.id=b.id ORDER BY o.ts'
    )


def _source_stamp(name: str, blob: bytes) -> int:
    stamp = TYPES[name].lcm_decode(blob).header.stamp
    return stamp.sec * 10**9 + stamp.nsec


def _go2_transforms(db: sqlite3.Connection) -> dict[str, list[tuple[float, TransformStamped]]]:
    """Derive this legacy fixture's missing TF using its Go2 base-pose convention."""
    dynamic = []
    for ts, blob in _rows(db, "odom"):
        pose = PoseStamped.lcm_decode(blob)
        assert pose.header.frame_id == "world"
        dynamic.append(
            (
                ts,
                TransformStamped(
                    header=pose.header,
                    child_frame_id="base_link",
                    transform=LCMTransform(
                        translation=pose.pose.position, rotation=pose.pose.orientation
                    ),
                ),
            )
        )
    first_ts, first = dynamic[0]
    static = [
        Transform(
            translation=Vector3(CAM_T),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=first_ts,
        ),
        Transform(
            rotation=Quaternion(CAM_Q),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=first_ts,
        ),
    ]
    mounts = []
    for transform in static:
        edge = transform.lcm_transform()
        edge.header.stamp = first.header.stamp
        mounts.append((first_ts, edge))
    return {"/tf": dynamic, "/tf_static": mounts}


def _verify_transform(actual: Any, expected: TransformStamped) -> None:
    assert actual.header.frame_id == expected.header.frame_id
    assert actual.header.stamp.sec == expected.header.stamp.sec
    assert actual.header.stamp.nanosec == expected.header.stamp.nsec
    assert actual.child_frame_id == expected.child_frame_id
    for field, axes in (("translation", "xyz"), ("rotation", "xyzw")):
        assert [getattr(getattr(actual.transform, field), axis) for axis in axes] == [
            getattr(getattr(expected.transform, field), axis) for axis in axes
        ]


def _transcribe(db: sqlite3.Connection, out: Path, jpeg: bool, writer: Any) -> None:
    streams = []
    with TemporaryDirectory(dir=out.parent) as temporary:
        directory = Path(temporary)
        records = []
        for name in ["color_image"] if jpeg else COUNTS:
            streams.append(
                {
                    "name": name,
                    "port": name,
                    "payload_type": f"dimos.msgs.{SCHEMAS[name].replace('/msg/', '.')}.{TYPES[name].__name__}",
                    "codec": "ros-jpeg" if jpeg else "cdr",
                }
            )
            for index, (ts, blob) in enumerate(_rows(db, name)):
                if name == "color_image" and not jpeg:
                    message = Image.lcm_decode(blob)
                    pixels = JpegCodec().decode(blob).data
                    # Keep the original integer header stamp, avoiding a float round trip.
                    message.encoding = "rgb8"
                    message.is_bigendian = 0
                    message.step = message.width * 3
                    message.data = pixels.tobytes()
                    message.data_length = len(message.data)
                    blob = message.lcm_encode()
                filename = f"{name}-{index}.lcm"
                (directory / filename).write_bytes(blob)
                records.append({"stream": name, "reception_ts": repr(ts), "payload_path": filename})
        for name, transforms in _go2_transforms(db).items():
            streams.append(
                {
                    "name": name,
                    "port": name,
                    "payload_type": "dimos.msgs.tf2_msgs.TFMessage.TFMessage",
                    "codec": "cdr",
                }
            )
            for index, (ts, transform) in enumerate(transforms):
                filename = f"{name.lstrip('/')}-{index}.lcm"
                (directory / filename).write_bytes(
                    LCMTFMessage(transforms_length=1, transforms=[transform]).lcm_encode()
                )
                records.append({"stream": name, "reception_ts": repr(ts), "payload_path": filename})
        records.sort(
            key=lambda item: (
                float(item["reception_ts"]),
                {"/tf_static": 0, "/tf": 1}.get(item["stream"], 2),
            )
        )
        (directory / "messages.jsonl").write_text(
            "".join(json.dumps(record) + "\n" for record in records)
        )
        writer(directory, out, streams)


def _normalized(definition: str) -> list[str]:
    return [
        " ".join(line.split())
        for raw in definition.splitlines()
        if (line := raw.partition("#")[0].strip())
    ]


def _verify_schema(name: str, data: bytes, upstream: dict[str, str]) -> None:
    name = name.replace("/msg/", "/")
    sections = re.split(r"={80}\nMSG: ([^\n]+)\n", data.decode())
    definitions = {name: sections[0], **dict(zip(sections[1::2], sections[2::2], strict=True))}
    dependencies = {name}
    for type_name, definition in definitions.items():
        package, short_name = type_name.split("/")
        canonical_name = f"{package}/msg/{short_name}"
        repository = {"builtin_interfaces": "rcl_interfaces", "tf2_msgs": "geometry2"}.get(
            package, "common_interfaces"
        )
        if canonical_name not in upstream:
            url = f"https://raw.githubusercontent.com/ros2/{repository}/{SNAPSHOTS[repository]}/{package}/msg/{short_name}.msg"
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            upstream[canonical_name] = response.text
        assert _normalized(definition) == _normalized(upstream[canonical_name]), type_name
        for line in _normalized(definition):
            field_type = line.split()[0].split("[")[0]
            if "/" in field_type:
                dep_package, dep_type = field_type.split("/")
                dependencies.add(f"{dep_package}/{dep_type}")
            elif field_type[0].isupper():
                dependencies.add(f"{package}/{field_type}")
    assert set(definitions) == dependencies


def _verify_payload(name: str, actual: Any, blob: bytes, jpeg: bool) -> Any:
    expected = TYPES[name].lcm_decode(blob)
    assert actual.header.frame_id == expected.header.frame_id
    assert actual.header.stamp.sec == expected.header.stamp.sec
    assert actual.header.stamp.nanosec == expected.header.stamp.nsec
    if name == "color_image":
        if jpeg:
            assert actual.format == "jpeg"
            assert bytes(actual.data) == bytes(expected.data)
        else:
            assert (
                actual.width,
                actual.height,
                actual.encoding,
                actual.step,
                actual.is_bigendian,
            ) == (
                expected.width,
                expected.height,
                "rgb8",
                expected.width * 3,
                0,
            )
            assert bytes(actual.data) == JpegCodec().decode(blob).data.tobytes()
    elif name == "lidar":
        for attr in ("height", "width", "is_bigendian", "point_step", "row_step", "is_dense"):
            assert getattr(actual, attr) == getattr(expected, attr), attr
        assert [(f.name, f.offset, f.datatype, f.count) for f in actual.fields] == [
            (f.name, f.offset, f.datatype, f.count) for f in expected.fields
        ]
        assert bytes(actual.data) == bytes(expected.data)
    else:
        for attr, axes in (("position", "xyz"), ("orientation", "xyzw")):
            assert [getattr(getattr(actual.pose, attr), axis) for axis in axes] == [
                getattr(getattr(expected.pose, attr), axis) for axis in axes
            ]
    return expected


def _verify_recording(
    db: sqlite3.Connection, out: Path, jpeg: bool, upstream: dict[str, str]
) -> dict[str, Any]:
    payload_counts = {"color_image": 855} if jpeg else COUNTS
    transforms = _go2_transforms(db)
    counts = {**payload_counts, **{name: len(edges) for name, edges in transforms.items()}}
    schemas = {
        name: "sensor_msgs/msg/CompressedImage" if jpeg and name == "color_image" else SCHEMAS[name]
        for name in counts
    }
    cursors = {name: iter(_rows(db, name)) for name in payload_counts}
    tf_cursors = {name: iter(edges) for name, edges in transforms.items()}
    parents: dict[str, str] = {}
    seen: Counter[str] = Counter()
    ranges: dict[str, list[float]] = {name: [] for name in counts}
    with out.open("rb") as source:
        reader = make_reader(source, validate_crcs=True, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        assert summary is not None and summary.statistics is not None
        assert summary.statistics.message_count == sum(counts.values())
        assert summary.chunk_indexes and all(c.message_index_offsets for c in summary.chunk_indexes)
        assert len(summary.channels) == len(counts)
        for channel in summary.channels.values():
            assert channel.schema_id != 0 and channel.message_encoding == "cdr"
            schema = summary.schemas[channel.schema_id]
            expected_name = schemas[channel.topic]
            assert schema.name == expected_name and schema.encoding == "ros2msg"
            assert summary.statistics.channel_message_counts[channel.id] == counts[channel.topic]
            _verify_schema(schema.name, schema.data, upstream)
        for _, channel, message, decoded in reader.iter_decoded_messages():
            name = channel.topic
            if name in tf_cursors:
                ts, expected_transform = next(tf_cursors[name])
                assert len(decoded.transforms) == 1
                edge = decoded.transforms[0]
                _verify_transform(edge, expected_transform)
                parents[edge.child_frame_id] = edge.header.frame_id
                stamp = expected_transform.header.stamp
            else:
                ts, blob = next(cursors[name])
                expected = _verify_payload(name, decoded, blob, jpeg)
                stamp = expected.header.stamp
            assert message.log_time == round(Fraction(ts) * 10**9)
            assert abs(message.publish_time - (stamp.sec * 10**9 + stamp.nsec)) <= 256
            assert message.sequence == seen[name]
            seen[name] += 1
            ranges[name].append(message.publish_time / 1e9)
    assert dict(seen) == counts
    assert all(next(cursor, None) is None for cursor in cursors.values())
    assert all(next(cursor, None) is None for cursor in tf_cursors.values())
    assert parents == {
        "base_link": "world",
        "camera_link": "base_link",
        "camera_optical": "camera_link",
    }
    # Every camera exposure is bracketed by actual recorded robot poses.
    assert min(ranges["/tf"]) <= min(ranges["color_image"])
    assert max(ranges["/tf"]) >= max(ranges["color_image"])
    with open_store(out) as store:
        assert store.list_streams() == sorted(counts)
        for name, count in counts.items():
            stream = store.stream(name)
            assert stream.count() == count
            assert stream.get_time_range() == (min(ranges[name]), max(ranges[name]))
            assert stream.first().ts == ranges[name][0]
            assert stream.last().ts == max(ranges[name])
            if name in tf_cursors:
                for actual, (_, expected_edge) in zip(stream, transforms[name], strict=True):
                    edge = actual.data.transforms[0]
                    assert (edge.frame_id, edge.child_frame_id) == (
                        expected_edge.header.frame_id,
                        expected_edge.child_frame_id,
                    )
                continue
            source_rows = sorted(
                _rows(db, name),
                key=lambda row: _source_stamp(name, row[1]),
            )
            for index in (0, count // 2, count - 1):
                actual = stream.order_by("ts").offset(index).first().data
                blob = source_rows[index][1]
                header = TYPES[name].lcm_decode(blob).header
                assert actual.frame_id == header.frame_id
                assert actual.ts == header.stamp.sec + header.stamp.nsec / 1e9
                if name == "color_image":
                    pixels = actual.data[:, :, ::-1] if jpeg else actual.data
                    np.testing.assert_array_equal(pixels, JpegCodec().decode(blob).data)
                elif name == "lidar":
                    np.testing.assert_array_equal(
                        actual.points(), DimosPointCloud2.lcm_decode(blob).points()
                    )
                else:
                    expected = DimosPoseStamped.lcm_decode(blob)
                    assert actual.position == expected.position
                    assert actual.orientation == expected.orientation
        render_store(store, out=str(out.with_suffix(".memory.rrd")), no_gui=True)
    result = subprocess.run(
        [
            str(Path(sys.executable).parent / "rerun"),
            "mcap",
            "convert",
            str(out),
            "-o",
            str(out.with_suffix(".direct.rrd")),
            "--decoder",
            "ros2msg",
            "--disable-raw-fallback",
        ],
        capture_output=True,
        text=True,
        timeout=600,
    )
    out.with_suffix(".rerun.log").write_text(result.stdout + result.stderr)
    assert result.returncode == 0, result.stderr
    for name in counts:
        assert f"/{name.lstrip('/')}" in result.stdout + result.stderr
    assert out.with_suffix(".memory.rrd").stat().st_size > 0
    assert out.with_suffix(".direct.rrd").stat().st_size > 0
    with out.open("rb") as source:
        digest = hashlib.file_digest(source, "sha256").hexdigest()
    return {
        "path": str(out),
        "bytes": out.stat().st_size,
        "sha256": digest,
        "counts": counts,
        "schemas": schemas,
        "derived_transforms": {
            "dynamic": "recorded odom pose: world -> base_link",
            "static": "Go2 CAM_T/CAM_Q: base_link -> camera_link -> camera_optical",
            "calibration_source": "dimos/robot/unitree/go2/dds/extrinsics.py",
            "camera_mount_measured_in_recording": False,
        },
        "message_encoding": "cdr",
        "schema_encoding": "ros2msg",
        "publish_time_ranges": {k: [min(v), max(v)] for k, v in ranges.items()},
        "source_time_regressions": {
            k: sum(b < a for a, b in itertools.pairwise(v)) for k, v in ranges.items()
        },
        "checks": [
            "pinned_upstream_schemas",
            "dependency_closure",
            "chunk_crcs",
            "indexes",
            "all_payloads",
            "timestamps",
            "sequences",
            "memory_reader",
            "memory_rerun",
            "direct_rerun",
            "tf_values_and_frame_chain",
            "camera_tf_time_coverage",
        ],
    }


def test_lfs_recording_transcription(
    tmp_path: Path,
    native_mcap_writer: Any,
    foxglove_validator: Any,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Preserve every primary observation, schema, and source JPEG bitstream."""
    source = get_data("go2_short.db")
    report: dict[str, Any] = {
        "source": str(source),
        "source_archive_sha256": LFS_SHA256,
        "excluded_derived_streams": ["color_image_embedded"],
        "artifacts": [],
    }
    upstream: dict[str, str] = {}
    with closing(sqlite3.connect(f"file:{source}?mode=ro&immutable=1", uri=True)) as db:
        for jpeg in (False, True):
            out = tmp_path / ("go2_short_jpeg.mcap" if jpeg else "go2_short.mcap")
            _transcribe(db, out, jpeg, native_mcap_writer)
            report["artifacts"].append(_verify_recording(db, out, jpeg, upstream))
            foxglove_validator(out)
            report["artifacts"][-1]["checks"].append("foxglove_cdr_sample_roundtrip")
            summarize(str(out))
            output = capsys.readouterr().out
            for name, count in report["artifacts"][-1]["counts"].items():
                assert f'Stream("{name}"): {count} items' in output
    report["verified_schema_definitions"] = sorted(upstream)
    (tmp_path / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
