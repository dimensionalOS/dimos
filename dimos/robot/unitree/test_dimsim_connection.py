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

"""Canonical DimSim browser/Deno/Python sensor and control interoperability."""

from contextlib import ExitStack
from copy import deepcopy
from functools import partial
import json
import os
from pathlib import Path
import subprocess

from dimos_cdrgen.registry import decode, encode, schema
import numpy as np
import pytest

from dimos.core import cdr_transport
from dimos.core.global_config import GlobalConfig
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos.messages.sensor_msgs.msg import Image, PointCloud2, PointField
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.image import image_view
from dimos.protocol.service.lcmservice import LCMConfig
from dimos.robot.unitree import dimsim_connection
from dimos.robot.unitree.dimsim_connection import DimSimConnection, _odom_to_tf
from dimos.utils.testing.waiting import wait_until

_ROOT = Path(__file__).resolve().parents[3]
_DIMSIM = _ROOT / "misc/DimSim"


def _samples():
    stamp = Header(Time(1_700_000_000, 1), "world")
    return [
        PoseStamped(deepcopy(stamp), Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0))),
        Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)),
        Image(deepcopy(stamp), 2, 2, "rgb8", 0, 8, np.arange(16, dtype=np.uint8)),
        PointCloud2(
            deepcopy(stamp),
            1,
            1,
            [
                PointField(name, index * 4, 7, 1)
                for index, name in enumerate(("x", "y", "z", "intensity"))
            ],
            False,
            16,
            16,
            np.asarray([1.0, 2.0, 3.0, 0.5], dtype="<f4").view(np.uint8),
            True,
        ),
    ]


def test_dimsim_schema_artifact_is_exactly_the_canonical_package_output():
    artifact = json.loads((_DIMSIM / "cli/bridge/cdr_schemas.json").read_text())
    assert artifact == {value.__msgtype__: schema(value.__msgtype__) for value in _samples()}


@pytest.fixture(scope="module")
def typescript_results(tmp_path_factory):
    deno = os.environ.get("DIMOS_DIMSIM_DENO")
    if not deno:
        pytest.skip(
            "Set DIMOS_DIMSIM_DENO after preparing DimSim dependencies; CI requires this acceptance gate"
        )
    assert Path(deno).is_file(), "configured Deno must exist"
    folder = tmp_path_factory.mktemp("dimsim-cdr")
    fixtures = folder / "input.json"
    output = folder / "result.json"
    samples = _samples()
    fixtures.write_text(
        json.dumps(
            {
                "values": [
                    {
                        "type": value.__msgtype__,
                        "hex": encode(value, little_endian=endian).hex(),
                        "canonicalHex": encode(value).hex(),
                    }
                    for value in samples
                    for endian in (False, True)
                ],
                "command": encode(samples[1]).hex(),
            }
        )
    )
    result = subprocess.run(
        [
            deno,
            "test",
            "--no-prompt",
            "--cached-only",
            "--node-modules-dir=manual",
            "--config",
            str(_DIMSIM / "cli/deno.json"),
            "--allow-read",
            "--allow-write=" + str(output),
            "--allow-env=DIMOS_DIMSIM_CDR_FIXTURES,DIMOS_DIMSIM_CDR_RESULT,DIMSIM_PROFILE_PHYSICS,DIMSIM_PROFILE_LIDAR",
            str(_DIMSIM / "cli/bridge/cdr_test.ts"),
        ],
        env={
            **os.environ,
            "DIMOS_DIMSIM_CDR_FIXTURES": str(fixtures),
            "DIMOS_DIMSIM_CDR_RESULT": str(output),
            "DIMSIM_PROFILE_PHYSICS": "0",
            "DIMSIM_PROFILE_LIDAR": "0",
        },
        capture_output=True,
        text=True,
        timeout=45,
        check=False,
    )
    assert result.returncode == 0, result.stdout[-6000:] + result.stderr[-6000:]
    return json.loads(output.read_text())


def test_actual_typescript_browser_image_outputs_decode_with_exact_pixels_and_clock(
    typescript_results,
):
    color = decode(bytes.fromhex(typescript_results["color"]), Image)
    depth = decode(bytes.fromhex(typescript_results["depth"]), Image)
    assert (color.encoding, color.step) == ("rgba8", 8)
    np.testing.assert_array_equal(image_view(color), [[[1, 2, 3, 255], [4, 5, 6, 128]]])
    assert (depth.encoding, depth.step, depth.is_bigendian) == ("16UC1", 6, 0)
    np.testing.assert_array_equal(image_view(depth), [[0, 0, 500], [1250, 65535, 0]])
    for value in (color, depth):
        assert (value.header.stamp.sec, value.header.stamp.nanosec, value.header.frame_id) == (
            1_700_000_000,
            1_000_000,
            "camera_optical",
        )


def test_actual_rapier_pose_deadman_and_lidar_keep_ros_coordinates(typescript_results):
    pose = decode(bytes.fromhex(typescript_results["pose"]), PoseStamped)
    stopped = decode(bytes.fromhex(typescript_results["stoppedPose"]), PoseStamped)
    cloud = decode(bytes.fromhex(typescript_results["cloud"]), PointCloud2)
    assert pose.pose.position.x > 0.0 and pose.pose.orientation.z > 0.0
    assert (stopped.pose.position.x, stopped.pose.position.y) == (
        pose.pose.position.x,
        pose.pose.position.y,
    )
    assert (pose.header.stamp.nanosec, stopped.header.stamp.nanosec) == (1_000_000, 502_000_000)
    assert cloud.header.frame_id == pose.header.frame_id == "world"
    assert 0 < cloud.width <= 15000 and cloud.row_step == cloud.width * 16
    assert [(field.name, field.offset, field.datatype, field.count) for field in cloud.fields] == [
        (name, index * 4, 7, 1) for index, name in enumerate(("x", "y", "z", "intensity"))
    ]
    points = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 4)
    assert np.isfinite(points).all()
    np.testing.assert_allclose(points[:, 2], 0.0, atol=1e-5)


@pytest.mark.parametrize("headless", [False, True])
def test_python_connection_uses_canonical_factory_and_closes_owned_tf_after_pose(
    headless, mocker, monkeypatch
):
    monkeypatch.setattr(cdr_transport, "LCMConfig", partial(LCMConfig, url="memq://"))
    process = mocker.patch.object(dimsim_connection, "DimSimProcess").return_value
    config = GlobalConfig(transport="lcm", dimsim_headless=headless)
    sample = _samples()[0]
    with ExitStack() as cleanup:
        connection = DimSimConnection(config)
        cleanup.callback(connection.stop)
        observed = []
        cleanup.callback(connection._tf_transport.subscribe(observed.append))
        process.start.side_effect = lambda: connection._odom_transport.publish(sample)
        connection.start()
        wait_until(lambda: bool(observed), timeout=5)
        assert connection._first_odom.is_set()
        transforms = observed[0].transforms
        assert [(t.header.frame_id, t.child_frame_id) for t in transforms] == [
            ("world", "base_link"),
            ("base_link", "camera_link"),
            ("camera_link", "camera_optical"),
            ("base_link", "lidar_link"),
        ]
        assert all(t.header.stamp.nanosec == 1 for t in transforms)
        assert transforms[0].transform.translation.x == 1.0
        assert transforms[1].transform.translation.x == 0.3
        q = transforms[2].transform.rotation
        assert (q.x, q.y, q.z, q.w) == (-0.5, 0.5, -0.5, 0.5)
    assert connection._odom_transport._pubsub is None
    assert connection._tf_transport._pubsub is None
    process.stop.assert_called_once_with()


def test_transform_headers_are_independent_of_the_driver_source():
    source = _samples()[0]
    transforms = _odom_to_tf(source)
    source.header.stamp.nanosec = 2
    source.pose.position.x = 99.0
    assert all(value.header.stamp.nanosec == 1 for value in transforms)
    assert transforms[0].transform.translation.x == 1.0
