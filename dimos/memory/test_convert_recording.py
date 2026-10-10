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

"""Offline migration preserves data and fails closed before publishing outputs."""

import copy
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

from dimos_generated.std_msgs.msg import String
from dimos_message_build.registry import decode as cdr_decode
import lz4.frame
from mcap.reader import make_reader
from mcap.writer import Writer
import numpy as np
import pytest
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
import sqlite_vec

from dimos.memory import convert_recording
from dimos.memory.convert_recording import convert
from dimos.memory.recording_migration import inspect_recording
from dimos.memory.store.sqlite import SqliteStore
from dimos.models.embedding.base import Embedding


@pytest.fixture(autouse=True)
def restore_reference_types_module():
    previous = sys.modules.get("rosbags.usertypes")
    try:
        yield
    finally:
        if previous is None:
            sys.modules.pop("rosbags.usertypes", None)
        else:
            sys.modules["rosbags.usertypes"] = previous


def write_mcap(path, streams):
    with path.open("wb") as f:
        writer = Writer(f)
        writer.start()
        for name, payload_type, codec, messages in streams:
            channel = writer.register_channel(
                name,
                codec,
                0,
                metadata={
                    "dimos.payload_type": payload_type,
                    "dimos.observation_time": "publish_time",
                },
            )
            for i, data in enumerate(messages):
                writer.add_message(channel, 2_000_000_000 + i, data, 1_000_000_000 + i, 17 + i)
        writer.finish()


def read_mcap(path):
    with path.open("rb") as f:
        return list(make_reader(f, validate_crcs=True).iter_messages())


def legacy_pose():
    cls = pytest.importorskip("dimos_lcm.geometry_msgs.PoseStamped").PoseStamped
    value = cls()
    value.header.seq = 42
    value.header.stamp.sec = 1
    value.header.stamp.nsec = 123456789
    value.header.frame_id = "world"
    value.pose.position.x = 3.25
    value.pose.orientation.w = 1.0
    return value


@pytest.mark.parametrize("suffix", [".mcap", ".db"])
@pytest.mark.parametrize("codec", ["lcm", "lz4+lcm"])
def test_pose_preserves_payload_header_and_both_envelope_times(tmp_path, suffix, codec):
    old = legacy_pose()
    data = old.lcm_encode()
    if codec.startswith("lz4"):
        data = lz4.frame.compress(data)
    source, output = tmp_path / "old.mcap", tmp_path / ("new" + suffix)
    write_mcap(
        source, [("pose", "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped", codec, [data])]
    )
    digest = hashlib.sha256(source.read_bytes()).digest()
    assert convert(source, output)["counts"] == {"pose": 1}
    assert hashlib.sha256(source.read_bytes()).digest() == digest
    if suffix == ".mcap":
        [(schema, channel, row)] = read_mcap(output)
        assert (row.log_time, row.publish_time, row.sequence) == (2_000_000_000, 1_000_000_000, 17)
        assert channel.message_encoding == "cdr" and schema.encoding == "ros2msg"
        assert (
            b"MSG: std_msgs/Header" in schema.data
            and b"MSG: builtin_interfaces/Time" in schema.data
        )
        store = get_typestore(Stores.EMPTY)
        store.register(get_types_from_msg(schema.data.decode(), schema.name))
        value = store.deserialize_cdr(row.data, schema.name)
    else:
        with SqliteStore(path=str(output), must_exist=True) as store:
            [obs] = list(store.stream("pose"))
            assert obs.ts == 1.0
            assert obs.tags["cdr_conversion"]["log_time_ns"] == 2_000_000_000
            assert obs.tags["cdr_conversion"]["sequence"] == 17
            value = obs.data
        with sqlite3.connect(output) as db:
            assert (
                json.loads(db.execute("SELECT config FROM _streams").fetchone()[0])["codec_id"]
                == "cdr"
            )
    assert value.header.stamp.sec == 1 and value.header.stamp.nanosec == 123456789
    assert value.header.frame_id == "world" and value.pose.position.x == 3.25
    report = [
        json.loads(line)
        for line in Path(str(output) + ".conversion.jsonl").read_text().splitlines()
    ]
    assert report[1]["legacy_header_sequences"] == {"header.seq": 42}


def test_jpeg_is_standard_compressed_image_without_recompression(tmp_path):
    cls = pytest.importorskip("dimos_lcm.sensor_msgs.Image").Image
    old = cls()
    old.header.stamp.sec = 8
    old.header.stamp.nsec = 7
    old.header.frame_id = "camera"
    old.encoding = "jpeg"
    # Conversion must preserve encoded bytes, not involve a JPEG decode/encode.
    old.data = b"\xff\xd8opaque-compressed-image\xff\xd9"
    old.data_length = len(old.data)
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(
        source, [("image", "dimos.msgs.sensor_msgs.Image.Image", "jpeg", [old.lcm_encode()])]
    )
    convert(source, output)
    [(schema, _, row)] = read_mcap(output)
    assert schema.name == "sensor_msgs/msg/CompressedImage"
    store = get_typestore(Stores.EMPTY)
    store.register(get_types_from_msg(schema.data.decode(), schema.name))
    msg = store.deserialize_cdr(row.data, schema.name)
    assert msg.format == "jpeg" and bytes(msg.data) == old.data
    assert msg.header.frame_id == "camera" and msg.header.stamp.nanosec == 7


@pytest.mark.parametrize("codec", ["json", "lz4+json"])
def test_json_string_preserves_utf8_and_stream_name(tmp_path, codec):
    data = '{"note": "世界", "value": 1}'.encode()
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    wire = lz4.frame.compress(data) if codec.startswith("lz4") else data
    write_mcap(source, [("status/raw", "dimos.msgs.std_msgs.String.String", codec, [wire])])
    convert(source, output)
    [(schema, channel, row)] = read_mcap(output)
    assert channel.topic == "status/raw" and schema.name == "std_msgs/msg/String"
    assert cdr_decode(row.data, String).data.encode() == data


def test_all_unsupported_streams_reported_before_output(tmp_path):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(
        source,
        [
            ("one", "os.system", "pickle", [b"not executed"]),
            ("two", "unknown.Type", "lcm", []),
            ("safe", "dimos.msgs.std_msgs.String.String", "json", [b"{}"]),
        ],
    )
    with pytest.raises(ValueError, match="one:.*\n.*two:"):
        convert(source, output)
    assert not output.exists() and not Path(str(output) + ".conversion.jsonl").exists()


@pytest.mark.parametrize("bad", [b"not-json", b'{"bad": NaN}'])
def test_bad_late_payload_publishes_nothing(tmp_path, bad):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(source, [("status", "dimos.msgs.std_msgs.String.String", "json", [b"{}", bad])])
    with pytest.raises(ValueError):
        convert(source, output)
    assert not output.exists() and not list(tmp_path.glob(".cdr-convert-*"))


def test_bad_lcm_fingerprint_rejected(tmp_path):
    old = legacy_pose().lcm_encode()
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(
        source,
        [
            (
                "pose",
                "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
                "lcm",
                [b"XXXXXXXX" + old[8:]],
            )
        ],
    )
    with pytest.raises(ValueError, match="Decode error"):
        convert(source, output)
    assert not output.exists()


def test_no_overwrite_or_inplace_conversion(tmp_path):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    source.write_bytes(b"original")
    output.write_bytes(b"keep")
    for destination in [source, output]:
        with pytest.raises(FileExistsError):
            convert(source, destination)
    assert source.read_bytes() == b"original" and output.read_bytes() == b"keep"


def test_empty_stream_is_preserved(tmp_path):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(source, [("status", "dimos.msgs.std_msgs.String.String", "json", [])])
    assert convert(source, output)["counts"] == {"status": 0}
    with output.open("rb") as f:
        summary = make_reader(f).get_summary()
        assert [c.topic for c in summary.channels.values()] == ["status"]


def legacy_sqlite(path, *, external=False):
    old = legacy_pose()
    with sqlite3.connect(path) as db:
        db.executescript(
            "CREATE TABLE _streams(name TEXT,config TEXT); CREATE TABLE pose(id INTEGER,ts REAL,value NUMERIC,pose_x REAL,pose_y REAL,pose_z REAL,pose_qx REAL,pose_qy REAL,pose_qz REAL,pose_qw REAL,tags TEXT); CREATE TABLE pose_blob(id INTEGER,data BLOB);"
        )
        config = {
            "payload_module": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
            "codec_id": "lcm",
            "blob_store": {
                "class": "dimos.memory2.blobstore.sqlite.SqliteBlobStore",
                "config": {"path": "/do-not-open" if external else None},
            },
        }
        db.execute("INSERT INTO _streams VALUES (?,?)", ("pose", json.dumps(config)))
        db.execute("INSERT INTO pose VALUES (9,2.5,7,1,2,3,0,0,0,1,?)", ('{"note":"preserve"}',))
        db.execute("INSERT INTO pose_blob VALUES (?,?)", (9, old.lcm_encode()))


@pytest.mark.parametrize("suffix", [".mcap", ".db"])
def test_sqlite_registry_without_dynamic_component_imports(tmp_path, suffix):
    source, output = tmp_path / "old.db", tmp_path / ("new" + suffix)
    legacy_sqlite(source)
    original = source.read_bytes()
    convert(source, output)
    assert source.read_bytes() == original
    report = [
        json.loads(line)
        for line in Path(str(output) + ".conversion.jsonl").read_text().splitlines()
    ]
    row = report[1]
    assert row["log_time_ns"] == 2_500_000_000 and row["publish_time_ns"] == 1_123_456_789
    assert row["sequence"] == 42
    assert row["original"]["sqlite_id"] == 9 and row["original"]["tags"] == {"note": "preserve"}
    if suffix == ".db":
        with SqliteStore(path=str(output), must_exist=True) as store:
            [obs] = list(store.stream("pose"))
            assert obs.ts == 2.5 and obs.pose_tuple == (1, 2, 3, 0, 0, 0, 1)
            assert obs.data.header.stamp.nanosec == 123456789


def test_external_sqlite_components_and_missing_blobs_rejected(tmp_path):
    source, output = tmp_path / "old.db", tmp_path / "new.mcap"
    legacy_sqlite(source, external=True)
    with pytest.raises(ValueError, match="external/custom blob"):
        convert(source, output)
    source.unlink()
    legacy_sqlite(source)
    with sqlite3.connect(source) as db:
        db.execute("DELETE FROM pose_blob")
    with pytest.raises(ValueError, match="no payload blob"):
        convert(source, output)
    assert not output.exists()


@pytest.mark.parametrize(
    "typename",
    ["sensor_msgs.Imu", "sensor_msgs.PointCloud2", "nav_msgs.Odometry", "tf2_msgs.TFMessage"],
)
def test_standard_nested_and_array_mappings(tmp_path, typename):
    module = pytest.importorskip("dimos_lcm." + typename)
    cls = getattr(module, typename.rsplit(".", 1)[1])
    old = cls()
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    payload_type = "dimos.msgs." + typename + "." + cls.__name__
    write_mcap(source, [("data", payload_type, "lcm", [old.lcm_encode()])])
    convert(source, output)
    [(schema, _, row)] = read_mcap(output)
    store = get_typestore(Stores.EMPTY)
    store.register(get_types_from_msg(schema.data.decode(), schema.name))
    result = store.deserialize_cdr(row.data, schema.name)
    assert schema.name == typename.replace(".", "/msg/")
    if cls.__name__ == "Imu":
        assert len(result.orientation_covariance) == 9
    elif cls.__name__ == "PointCloud2":
        assert len(result.fields) == 0 and len(result.data) == 0
    elif cls.__name__ == "Odometry":
        assert len(result.pose.covariance) == 36 and len(result.twist.covariance) == 36
    else:
        assert len(result.transforms) == 0


def test_existing_report_and_racing_destination_are_not_overwritten(tmp_path, monkeypatch):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(source, [("status", "dimos.msgs.std_msgs.String.String", "json", [b"{}"])])
    report = Path(str(output) + ".conversion.jsonl")
    report.write_text("keep")
    with pytest.raises(FileExistsError):
        convert(source, output)
    assert report.read_text() == "keep"
    report.unlink()
    original_link = convert_recording.os.link

    def competing_link(src, dst):
        if dst == output:
            output.write_bytes(b"other writer")
        return original_link(src, dst)

    monkeypatch.setattr(convert_recording.os, "link", competing_link)
    with pytest.raises(FileExistsError):
        convert(source, output)
    assert output.read_bytes() == b"other writer" and not report.exists()


def test_mcap_container_metadata_is_kept_and_attachments_rejected(tmp_path):
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    with source.open("wb") as f:
        writer = Writer(f)
        writer.start()
        writer.register_channel(
            "status",
            "json",
            0,
            metadata={"dimos.payload_type": "dimos.msgs.std_msgs.String.String"},
        )
        writer.add_metadata("source-info", {"robot": "offline-fixture"})
        writer.finish()
    convert(source, output)
    manifest = json.loads(Path(str(output) + ".conversion.jsonl").read_text().splitlines()[0])
    assert manifest["streams"][0]["metadata"]["source_mcap_metadata"] == [
        {"name": "source-info", "metadata": {"robot": "offline-fixture"}}
    ]
    attached = tmp_path / "attached.mcap"
    with attached.open("wb") as f:
        writer = Writer(f)
        writer.start()
        writer.add_attachment(1, 1, "robot.urdf", "text/xml", b"<robot/>")
        writer.finish()
    with pytest.raises(ValueError, match="attachments"):
        convert(attached, tmp_path / "no.mcap")
    assert not (tmp_path / "no.mcap").exists()


def test_sqlite_vectors_preserved_by_new_ids_and_rejected_for_mcap(tmp_path):
    source, output = tmp_path / "source.db", tmp_path / "converted.db"
    with SqliteStore(path=str(source)) as store:
        stream = store.stream("images", String, codec="cdr")
        stream.append(
            String(data="first"), ts=1, embedding=Embedding(np.array([1.0, 0.0], dtype=np.float32))
        )
        stream.append(
            String(data="second"), ts=2, embedding=Embedding(np.array([0.0, 1.0], dtype=np.float32))
        )
    with sqlite3.connect(source) as conn:
        conn.enable_load_extension(True)
        sqlite_vec.load(conn)
        conn.execute("UPDATE images SET id=42 WHERE id=2")
        conn.execute("UPDATE images_blob SET id=42 WHERE id=2")
        vector = conn.execute("SELECT embedding FROM images_vec WHERE rowid=2").fetchone()[0]
        conn.execute("DELETE FROM images_vec WHERE rowid=2")
        conn.execute("INSERT INTO images_vec(rowid,embedding) VALUES(42,?)", (vector,))
    assert inspect_recording(source, "db")["total"] == 2
    with pytest.raises(ValueError, match="SQLite output"):
        convert(source, tmp_path / "rejected.mcap")
    convert(source, output)
    with SqliteStore(path=str(output)) as store:
        hits = (
            store.stream("images")
            .search(Embedding(np.array([0.0, 1.0], dtype=np.float32)), k=1)
            .to_list()
        )
        assert hits[0].data.data == "second"
        assert hits[0].similarity == pytest.approx(1)
    with sqlite3.connect(output) as conn:
        conn.enable_load_extension(True)
        sqlite_vec.load(conn)
        assert (
            conn.execute("SELECT embedding FROM images_vec WHERE rowid=2").fetchone()[0] == vector
        )
    audit = [
        json.loads(line)
        for line in Path(str(output) + ".conversion.jsonl").read_text().splitlines()
    ]
    assert audit[2]["original"]["sqlite_id"] == 42
    assert audit[2]["output_sqlite_id"] == 2
    with sqlite3.connect(source) as conn:
        conn.enable_load_extension(True)
        sqlite_vec.load(conn)
        conn.execute("INSERT INTO images_vec(rowid,embedding) VALUES(999,?)", (vector,))
    with pytest.raises(ValueError, match="orphan vectors"):
        convert(source, tmp_path / "orphan.db")
    assert not (tmp_path / "orphan.db").exists()


@pytest.mark.parametrize(
    "typename", ["sensor_msgs.Image", "sensor_msgs.JointState", "sensor_msgs.Imu"]
)
def test_nonempty_numeric_and_byte_arrays_preserve_values(tmp_path, typename):
    module = pytest.importorskip("dimos_lcm." + typename)
    old = getattr(module, typename.rsplit(".", 1)[1])()
    if typename.endswith("Image"):
        old.encoding = "rgb8"
        old.height, old.width, old.step = 1, 2, 6
        old.data = bytes([0, 1, 127, 128, 254, 255])
        old.data_length = len(old.data)
        expected = {"data": np.frombuffer(old.data, dtype=np.uint8)}
    elif typename.endswith("JointState"):
        old.name = ["left", "right"]
        old.name_length = 2
        expected = {}
        for field, values in {
            "position": [-1.5, 2.25],
            "velocity": [0.125, -0.25],
            "effort": [3.0, 4.0],
        }.items():
            setattr(old, field, values)
            setattr(old, field + "_length", len(values))
            expected[field] = np.array(values, dtype=np.float64)
    else:
        old.orientation_covariance = [i / 8 for i in range(9)]
        expected = {
            "orientation_covariance": np.array(old.orientation_covariance, dtype=np.float64)
        }
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(source, [("data", typename.replace(".", "/msg/"), "lcm", [old.lcm_encode()])])

    convert(source, output)

    [(schema, _, row)] = read_mcap(output)
    oracle = get_typestore(Stores.EMPTY)
    oracle.register(get_types_from_msg(schema.data.decode(), schema.name))
    value = oracle.deserialize_cdr(row.data, schema.name)
    for field, expected_array in expected.items():
        actual = getattr(value, field)
        assert actual.dtype == expected_array.dtype
        np.testing.assert_array_equal(actual, expected_array)
    if typename.endswith("JointState"):
        assert value.name == ["left", "right"]


@pytest.mark.parametrize("suffix", [".mcap", ".db"])
@pytest.mark.parametrize(
    "stream_name,package,name",
    [
        ("map_regions", "sensor_msgs", "PointCloud2"),
        ("seed_bounds", "geometry_msgs", "PoseStamped"),
        ("node_edges", "nav_msgs", "Path"),
    ],
)
def test_builtin_region_conversion_retains_signed_identity_and_geometry(
    tmp_path, suffix, stream_name, package, name
):
    old = getattr(pytest.importorskip(f"dimos_lcm.{package}.{name}"), name)()
    old.header.seq = -196603
    old.header.stamp.sec = 1
    old.header.stamp.nsec = 34
    old.header.frame_id = "map"
    if name == "PointCloud2":
        old.height = 1
        old.point_step = 12
    elif name == "PoseStamped":
        old.pose.position.x, old.pose.position.y = -3.0, 5.0
        old.pose.orientation.x, old.pose.orientation.y, old.pose.orientation.z = 2.5, -1.0, 4.0
    else:
        endpoint = pytest.importorskip("dimos_lcm.geometry_msgs.PoseStamped").PoseStamped
        old.poses = []
        for x, weight in [(1.0, 0.5), (4.0, 0.5), (9.0, 100.0), (6.0, 100.0)]:
            pose = copy.deepcopy(endpoint())
            pose.header.frame_id = "map"
            pose.header.stamp.sec, pose.header.stamp.nsec = 1, 34
            pose.pose.position.x = x
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z = (
                0.0,
                0.0,
                0.0,
            )
            pose.pose.orientation.w = weight
            old.poses.append(pose)
        old.poses_length = len(old.poses)
    source, output = tmp_path / "old.mcap", tmp_path / ("new" + suffix)
    write_mcap(
        source, [(stream_name, f"dimos.msgs.{package}.{name}.{name}", "lcm", [old.lcm_encode()])]
    )
    assert convert(source, output)["counts"] == {stream_name: 1}
    if suffix == ".mcap":
        embedded, channel, record = read_mcap(output)[0]
        value = cdr_decode(record.data, convert_recording.resolve_msg_type(embedded.name))
        assert b"MSG: std_msgs/Header" in embedded.data
        assert record.sequence == 17
    else:
        with SqliteStore(path=str(output), must_exist=True) as store:
            value = store.stream(stream_name).first().data
    assert value.region_id == -196603
    if name == "PointCloud2":
        assert (
            value.cloud.width,
            value.cloud.header.frame_id,
            value.cloud.header.stamp.nanosec,
        ) == (0, "map", 34)
    elif name == "PoseStamped":
        assert (value.center.x, value.center.y, value.radius, value.z_min, value.z_max) == (
            -3.0,
            5.0,
            2.5,
            -1.0,
            4.0,
        )
    else:
        assert [segment.weight for segment in value.lines.segments] == [0.5, 100.0]
        assert [(segment.start.x, segment.end.x) for segment in value.lines.segments] == [
            (1.0, 4.0),
            (9.0, 6.0),
        ]


def test_region_contract_is_not_inferred_for_arbitrary_cloud_streams():
    ordinary = convert_recording.Stream("custom/map_regions", "sensor_msgs/msg/PointCloud2", "lcm")
    assert ordinary.region_type is None
    standard = convert_recording.Stream("map_regions", "sensor_msgs/msg/PointCloud2", "cdr")
    assert standard.region_type is None


@pytest.mark.parametrize("invalid", ["count", "frame", "timestamp", "orientation", "weight"])
def test_region_edges_reject_unrepresentable_endpoint_fields(tmp_path, invalid):
    path_type = pytest.importorskip("dimos_lcm.nav_msgs.Path").Path
    pose_type = pytest.importorskip("dimos_lcm.geometry_msgs.PoseStamped").PoseStamped
    old = copy.deepcopy(path_type())
    old.header.frame_id = "map"
    old.header.stamp.sec, old.header.stamp.nsec = 1, 34
    old.poses = [copy.deepcopy(pose_type()), copy.deepcopy(pose_type())]
    for pose in old.poses:
        pose.header = copy.deepcopy(old.header)
        pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.5
    if invalid == "count":
        old.poses.pop()
    elif invalid == "frame":
        old.poses[0].header.frame_id = "odom"
    elif invalid == "timestamp":
        old.poses[0].header.stamp.nsec += 1
    elif invalid == "orientation":
        old.poses[0].pose.orientation.x = 1.0
    else:
        old.poses[0].pose.orientation.w = 2.0
    old.poses_length = len(old.poses)
    source, output = tmp_path / "old.mcap", tmp_path / "new.mcap"
    write_mcap(source, [("node_edges", "dimos.msgs.nav_msgs.Path.Path", "lcm", [old.lcm_encode()])])
    with pytest.raises(ValueError, match="Legacy region edge"):
        convert(source, output)
    assert not output.exists()
    assert not output.with_name(output.name + ".conversion.jsonl").exists()
