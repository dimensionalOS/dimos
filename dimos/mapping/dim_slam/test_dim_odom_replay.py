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

import heapq
import math

import pytest

# The recording is a Git LFS fixture, which the regular CI job caps at 1 MiB, and the
# linux dim_odom wheel is CUDA-only, so this needs the GPU runner rather than the
# containerized one.
pytestmark = pytest.mark.self_hosted_large

dim_odom = pytest.importorskip("dim_odom")

from dimos.memory.store.sqlite import SqliteStore
from dimos.utils.data import get_data

# 12 s of alfred (D455) driving ~2.5 m: stereo IR pairs at 15 fps, camera infos, tf.
SNIPPET = "alfred_stereo_short.db"
SNIPPET_STEREO_PAIRS = 179
CAMERA_STREAMS = ("infrared_left", "infrared_right")
CAMERA_FRAMES = ("camera_infra1_optical_frame", "camera_infra2_optical_frame")
RIG_FRAME = "base_link"


def _static_tf_lookup(replay):
    """Collects the recording's static tf chain into a dim_odom tf callable."""
    parent_of = {}
    for _ts, message in replay.stream("tf").iterate_ts():
        for transform in message.transforms:
            translation = transform.translation
            rotation = transform.rotation
            parent_of.setdefault(
                transform.child_frame_id,
                (
                    transform.frame_id,
                    (
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                    ),
                ),
            )
        if all(_chain(parent_of, RIG_FRAME, frame) for frame in CAMERA_FRAMES):
            break

    def lookup(parent, child):
        chain = _chain(parent_of, parent, child)
        if chain is None:
            return None
        parent_from_child = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        for step in chain:
            parent_from_child = dim_odom.compose(parent_from_child, step)
        return parent_from_child

    return lookup


def _chain(parent_of, parent, child):
    """The transforms composing parent_from_child, topmost first, or None."""
    chain = []
    frame = child
    while frame != parent:
        if frame not in parent_of:
            return None
        frame, transform = parent_of[frame]
        chain.append(transform)
    chain.reverse()
    return chain


def _replay_trajectory(db_path):
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()
    try:
        replay = store.replay()
        tracker = dim_odom.CuvslamOdometry(
            {
                "camera_mode": "stereo",
                "use_gpu": False,
                "cameras": [{"frame_id": frame} for frame in CAMERA_FRAMES],
            },
            tf=_static_tf_lookup(replay),
        )
        for stream in CAMERA_STREAMS:
            info = replay.stream(f"{stream}_camera_info").first()
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

        def tagged(priority, stream):
            for ts, message in replay.stream(stream).iterate_ts():
                yield ts, priority, message

        estimates = []
        pairs = 0
        merged = heapq.merge(
            *(tagged(priority, stream) for priority, stream in enumerate(CAMERA_STREAMS)),
            key=lambda item: item[:2],
        )
        for _ts, priority, image in merged:
            pairs += priority
            estimate = tracker.handle_image(
                dim_odom.ImageFrame(
                    timestamp_ns=round(image.ts * 1e9),
                    frame_id=image.frame_id,
                    width=image.width,
                    height=image.height,
                    encoding="mono8",
                    step=image.width,
                    data=bytes(image.data),
                )
            )
            if estimate is not None:
                estimates.append(
                    (estimate.timestamp_ns, estimate.translation, estimate.rotation_xyzw)
                )
        assert pairs == SNIPPET_STEREO_PAIRS
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
