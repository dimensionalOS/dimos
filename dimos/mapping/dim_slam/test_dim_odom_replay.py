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

"""Replays a stereo recording through the dim_odom PyPI wheel, fully in-process.

No modules, no transports, no sockets: the recording is read with SqliteStore
and handed straight to dim_odom's CuvslamOdometry, so the output only depends
on the recorded data and must therefore be identical run to run.
"""

import math

import pytest

# The recording is a Git LFS fixture, which the regular CI job caps at 1 MiB, and the
# linux dim_odom wheel is CUDA-only, so this needs the GPU runner rather than the
# containerized one.
pytestmark = pytest.mark.self_hosted_large

dim_odom = pytest.importorskip("dim_odom")

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.utils.data import get_data

# 12 s of alfred (D455) driving ~2.5 m: stereo IR pairs at 15 fps, camera infos, tf.
SNIPPET = "alfred_stereo_short.db"
SNIPPET_STEREO_PAIRS = 179
CAMERA_STREAMS = ("infrared_left", "infrared_right")
CAMERA_FRAMES = ("camera_infra1_optical_frame", "camera_infra2_optical_frame")

# 15 s of the same drive, no images: 3 s stationary (IMU bias init) then ~2.3 m of
# driving. IMU at 400 Hz, wheel odometry at 48 Hz, tf.
FUSION_SNIPPET = "alfred_fusion_short.db"
IMU_FRAME = "camera_accel_optical_frame"
# The D455's datasheet figures, as recorded in the drive's imu_info stream.
FUSION_CONFIG = {
    "imus": [
        {
            "frame_id": IMU_FRAME,
            "gyro_noise_density": 2e-4,
            "gyro_random_walk": 1e-5,
            "accel_noise_density": 1.8e-3,
            "accel_random_walk": 1e-4,
        }
    ],
    # Wheel twist only: vx and wz are what a diff drive measures.
    "odom_sources": [
        {
            "parent_frame_id": "wheel_odom",
            "child_frame_id": "base_link",
            "twist_variances": [0.01, 0.0, 0.0, 0.0, 0.0, 0.02],
        }
    ],
    # Ground robot: pin vz, roll and pitch rates.
    "per_dimension_error_variance": [0.0, 0.0, 1e-6, 1e-6, 1e-6, 0.0],
}


def _tf_lookup(store):
    """The recording's tf tree as the (parent, child) callable dim_odom expects."""
    tf = StreamTF.from_store(store)

    def lookup(parent, child):
        transform = tf.get(parent, child, warn=False)
        if transform is None:
            return None
        translation, rotation = transform.translation, transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    return lookup


def _image_frame(image):
    return dim_odom.ImageFrame(
        timestamp_ns=round(image.ts * 1e9),
        frame_id=image.frame_id,
        width=image.width,
        height=image.height,
        encoding="mono8",
        step=image.width,
        data=bytes(image.data),
    )


def _replay_trajectory(db_path):
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        tracker = dim_odom.CuvslamOdometry(
            {
                "camera_mode": "stereo",
                "use_gpu": False,
                "cameras": [{"frame_id": frame} for frame in CAMERA_FRAMES],
            },
            tf=_tf_lookup(store),
        )
        for stream in CAMERA_STREAMS:
            info = store.stream(f"{stream}_camera_info").first().data
            tracker.handle_camera_info(
                dim_odom.CameraModel(
                    timestamp_ns=round(info.ts * 1e9),
                    frame_id=info.frame_id,
                    width=info.width,
                    height=info.height,
                    distortion=list(info.D),
                    intrinsics=list(info.K),
                )
            )

        left, right = (store.stream(stream).order_by("ts") for stream in CAMERA_STREAMS)
        estimates = []
        pairs = 0
        # Half a frame period at 15 fps: pairs match, neighboring frames never do.
        for pair in left.align(right, tolerance=1 / 30):
            pairs += 1
            for image in (pair.data.infrared_left.data, pair.data.infrared_right.data):
                estimate = tracker.handle_image(_image_frame(image))
                if estimate is not None:
                    estimates.append(
                        (estimate.timestamp_ns, estimate.translation, estimate.rotation_xyzw)
                    )
        assert pairs == SNIPPET_STEREO_PAIRS
        return estimates
    finally:
        store.stop()


def _replay_fusion(db_path):
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        base_from_imu = _tf_lookup(store)("base_link", IMU_FRAME)
        assert base_from_imu is not None
        fusion = dim_odom.OdometryFusion(FUSION_CONFIG)

        events = sorted(
            [("imu", record.ts, record.data) for record in store.stream("imu")]
            + [("wheel", record.ts, record.data) for record in store.stream("wheel_odometry")],
            key=lambda event: event[1],
        )
        estimates = []
        for kind, ts, message in events:
            if kind == "imu":
                fusion.handle_imu(
                    dim_odom.ImuSample(
                        timestamp_ns=round(ts * 1e9),
                        frame_id=message.frame_id,
                        angular_velocity=tuple(message.angular_velocity),
                        linear_acceleration=tuple(message.linear_acceleration),
                    ),
                    base_from_imu,
                )
            else:
                fusion.handle_source(
                    dim_odom.OdometryEstimate(
                        timestamp_ns=round(ts * 1e9),
                        frame_id=message.frame_id,
                        child_frame_id=message.child_frame_id,
                        translation=tuple(message.position),
                        rotation_xyzw=(
                            message.orientation.x,
                            message.orientation.y,
                            message.orientation.z,
                            message.orientation.w,
                        ),
                        twist_linear=(message.vx, message.vy, message.vz),
                        twist_angular=(message.wx, message.wy, message.wz),
                    )
                )
            estimate = fusion.maybe_publish()
            if estimate is not None:
                estimates.append(
                    (estimate.timestamp_ns, estimate.translation, estimate.rotation_xyzw)
                )
        return estimates
    finally:
        store.stop()


def test_dim_odom_stereo_replay():
    """The same recording twice through dim_odom gives the same trajectory."""
    db_path = str(get_data(SNIPPET))
    first = _replay_trajectory(db_path)
    second = _replay_trajectory(db_path)

    assert len(first) >= SNIPPET_STEREO_PAIRS // 2, (
        f"expected odometry for most stereo pairs, got {len(first)}"
    )
    displacement = math.dist(first[0][1], first[-1][1])
    assert 1.0 < displacement < 5.0, (
        f"net displacement {displacement:.2f} m outside the ~2.5 m recorded drive"
    )
    assert first == second, "replaying the same data twice diverged"


def test_dim_odom_fusion_replay():
    """IMU + wheel odometry through OdometryFusion, twice, identically."""
    db_path = str(get_data(FUSION_SNIPPET))
    first = _replay_fusion(db_path)
    second = _replay_fusion(db_path)

    assert len(first) >= 500, f"expected ~100 Hz output over ~15 s, got {len(first)}"
    displacement = math.dist(first[0][1], first[-1][1])
    assert 1.0 < displacement < 4.0, (
        f"net displacement {displacement:.2f} m outside the ~2.3 m recorded drive"
    )
    assert first == second, "replaying the same data twice diverged"
