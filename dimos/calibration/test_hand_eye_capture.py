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

import math

import numpy as np
import pytest

from dimos.calibration import hand_eye_capture
from dimos.calibration.hand_eye_capture import TopicCapture, check_color_frame
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


@pytest.fixture
def topic_source(mocker):
    callbacks = {}
    cleanup = []
    now = [1000.0]
    events = []

    def subscribe(topic, callback, *, msg_type):
        callbacks[topic] = callback
        transport = mocker.Mock()
        transport.stop.side_effect = lambda: cleanup.append((topic, "stop"))
        return transport, lambda: cleanup.append((topic, "unsubscribe"))

    subscription = mocker.patch.object(
        hand_eye_capture, "subscribe_pubsub_uri", side_effect=subscribe
    )
    mocker.patch.object(hand_eye_capture, "time", side_effect=lambda: now[0])
    mocker.patch.object(hand_eye_capture, "monotonic", side_effect=lambda: now[0])
    source = TopicCapture("camera", "tf", "base", "ee")

    def wait(timeout):
        if events:
            delay, event = events.pop(0)
            now[0] += delay
            event()
        else:
            now[0] += timeout

    mocker.patch.object(source._condition, "wait", side_effect=wait)

    def pair(*, offset=0.0, angle=0.0, tf_age=0.0, image_age=0.0, with_tf=True):
        image = Image(ts=now[0] - image_age, frame_id="camera_optical")
        if with_tf:
            matrix = np.eye(4)
            matrix[0, 3] = offset
            cosine, sine = math.cos(angle), math.sin(angle)
            matrix[:2, :2] = [[cosine, -sine], [sine, cosine]]
            pose = Transform.from_matrix(
                matrix, ts=now[0] - tf_age, frame_id="base", child_frame_id="ee"
            )
            callbacks["tf"](TFMessage(pose))
        callbacks["camera"](image)

    yield source, pair, events, callbacks, cleanup, subscription


def test_returns_fresh_stationary_image_and_matching_pose(topic_source):
    source, pair, events, _, _, _ = topic_source
    with source:
        pair(offset=0.2)  # A frame already present when capture starts is not sufficient.
        events.extend([(0.25, lambda: pair(offset=0.3))] * 3)

        image, pose = source.capture()

    assert image.ts == pose.ts == 1000.75
    assert pose.frame_id == "base" and pose.child_frame_id == "ee"
    assert pose.translation.x == 0.3


def test_rejects_depth_image_during_color_stream_check(marker_image):
    image, info, _ = marker_image
    depth = Image(
        data=image.data.astype(np.uint16), format=ImageFormat.DEPTH16, frame_id=image.frame_id
    )

    with pytest.raises(ValueError, match="8-bit color or grayscale"):
        check_color_frame(depth, info)


@pytest.mark.parametrize("mode", ["missing_tf", "stale_tf", "stale_image"])
def test_rejects_missing_or_stale_sources(topic_source, mode):
    source, pair, events, _, _, _ = topic_source
    options = {
        "missing_tf": {"with_tf": False},
        "stale_tf": {"tf_age": 2.0},
        "stale_image": {"image_age": 2.0},
    }[mode]
    events.extend([(0.25, lambda: pair(**options))] * 3)

    with source, pytest.raises(RuntimeError, match="Capture timed out:.*(TF|timestamps)"):
        source.capture(timeout_sec=1.0)


@pytest.mark.parametrize("motion", [{"offset": 0.002}, {"angle": math.radians(1)}])
def test_motion_restarts_stationary_window(topic_source, motion):
    source, pair, events, _, _, _ = topic_source
    events.extend([(0.25, pair), (0.25, lambda: pair(**motion)), (0.25, pair)])

    with source, pytest.raises(RuntimeError, match="Hold the arm still"):
        source.capture(timeout_sec=1.0)


def test_waits_for_tf_arriving_after_image(topic_source):
    source, pair, events, callbacks, _, _ = topic_source

    def tf_only():
        callbacks["tf"](TFMessage(Transform(ts=1000.75, frame_id="base", child_frame_id="ee")))

    events.extend([(0.25, pair), (0.5, lambda: pair(with_tf=False)), (0.05, tf_only)])
    with source:
        image, pose = source.capture()

    assert image.ts == pose.ts == 1000.75


@pytest.mark.parametrize("drop_image", [False, True])
def test_restarts_stationary_window_after_a_gap(topic_source, drop_image):
    source, pair, events, _, _, _ = topic_source
    events.append((0.25, pair))
    if drop_image:
        events.append((0.75, pair))
    else:
        events.extend([(0.25, lambda: pair(with_tf=False)), (0.25, pair)])
    events.extend([(0.25, pair)] * 2)

    with source:
        image, _ = source.capture()

    assert image.ts == (1001.5 if drop_image else 1001.25)


def test_rejects_stale_edge_in_transform_chain(topic_source):
    source, _, events, callbacks, _, _ = topic_source

    def pair_with_stale_edge():
        callbacks["tf"](
            TFMessage(
                Transform(ts=1000.25, frame_id="base", child_frame_id="joint"),
                Transform(ts=998.0, frame_id="joint", child_frame_id="ee"),
            )
        )
        callbacks["camera"](Image(ts=1000.25))

    events.append((0.25, pair_with_stale_edge))
    with source, pytest.raises(RuntimeError, match="No fresh TF"):
        source.capture(timeout_sec=1.0)


def test_cleans_up_after_timeout(topic_source):
    source, _, _, _, cleanup, _ = topic_source
    with pytest.raises(RuntimeError, match="No fresh images"), source:
        source.capture(timeout_sec=1.0)

    assert cleanup == [
        ("camera", "unsubscribe"),
        ("camera", "stop"),
        ("tf", "unsubscribe"),
        ("tf", "stop"),
    ]


def test_cleans_up_tf_if_camera_subscription_fails(topic_source):
    source, _, _, _, cleanup, subscription = topic_source
    subscribe = subscription.side_effect

    def fail_camera(topic, callback, *, msg_type):
        if topic == "camera":
            raise RuntimeError("camera unavailable")
        return subscribe(topic, callback, msg_type=msg_type)

    subscription.side_effect = fail_camera
    with pytest.raises(RuntimeError, match="camera unavailable"), source:
        pass

    assert cleanup == [("tf", "unsubscribe"), ("tf", "stop")]
