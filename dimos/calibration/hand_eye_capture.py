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

from __future__ import annotations

from contextlib import ExitStack
import math
from threading import Condition
from time import monotonic, time
from types import TracebackType

from dimos.calibration.hand_eye import check_image_pixels
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.registry import make_pubsub_transport, subscribe_pubsub_uri
from dimos.protocol.tf.tf import MultiTBuffer

TF_TOLERANCE_SEC = 0.15
STABLE_DURATION_SEC = 0.5
MAX_FRAME_AGE_SEC = 1.0


def check_color_frame(image: Image, info: CameraInfo) -> None:
    if not image.frame_id:
        raise ValueError("Image is missing its optical frame")
    if info.frame_id and image.frame_id != info.frame_id:
        raise ValueError(
            f"CameraInfo frame {info.frame_id!r} differs from image frame {image.frame_id!r}. "
            "Select topics from the same color stream."
        )
    check_image_pixels(image, info)


def read_camera_info(topic: str, timeout_sec: float) -> CameraInfo:
    """Read one color CameraInfo message and release the subscription."""
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        raise ValueError("CameraInfo timeout must be positive and finite.")
    with make_pubsub_transport(topic, msg_type=CameraInfo) as transport:
        try:
            info = transport.get_next(timeout=timeout_sec)
        except Exception as exc:
            raise RuntimeError(f"Could not read CameraInfo from {topic!r}. {exc}") from exc
    if not isinstance(info, CameraInfo):
        raise ValueError(f"Expected CameraInfo on {topic!r}, received {type(info).__name__}.")
    return info


class TopicCapture:
    """Read stationary image/pose pairs without commanding the robot.

    Each TF edge is the nearest sample within 150 ms of the image timestamp;
    this is not hardware synchronization. Hold the arm still during capture.
    All observed poses must stay within 1 mm / 0.5 degrees of the first pose
    for at least 0.5 seconds. The returned transform maps end effector to base.
    """

    def __init__(self, image_topic: str, tf_topic: str, base_frame: str, ee_frame: str) -> None:
        if not base_frame or not ee_frame or base_frame == ee_frame:
            raise ValueError("Specify distinct, nonempty base and end-effector frames.")
        self.image_topic, self.tf_topic = image_topic, tf_topic
        self.base_frame, self.ee_frame = base_frame, ee_frame
        self._condition = Condition()
        self._buffer = MultiTBuffer()
        self._image: Image | None = None
        self._sequence = 0
        self._stack: ExitStack | None = None

    def __enter__(self) -> TopicCapture:
        if self._stack is not None:
            raise RuntimeError("TopicCapture is already open.")
        with ExitStack() as stack:
            transport, unsubscribe = subscribe_pubsub_uri(
                self.tf_topic, self._receive_tf, msg_type=TFMessage
            )
            stack.callback(transport.stop)
            stack.callback(unsubscribe)
            transport, unsubscribe = subscribe_pubsub_uri(
                self.image_topic, self._receive_image, msg_type=Image
            )
            stack.callback(transport.stop)
            stack.callback(unsubscribe)
            self._stack = stack.pop_all()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        with self._condition:
            stack, self._stack = self._stack, None
            self._condition.notify_all()
        if stack is not None:
            stack.close()

    def _receive_image(self, image: Image) -> None:
        with self._condition:
            self._image = image
            self._sequence += 1
            self._condition.notify_all()

    def _receive_tf(self, message: TFMessage) -> None:
        with self._condition:
            self._buffer.receive_tfmessage(message)
            self._condition.notify_all()

    def capture(self, timeout_sec: float = 10.0) -> tuple[Image, Transform]:
        """Wait for fresh, stationary pairs; raise RuntimeError on timeout."""
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError("Capture timeout must be positive and finite.")
        deadline, started = monotonic() + timeout_sec, time()
        first_pose = None
        first_ts = last_ts = -math.inf
        unmatched_ts = None
        reason = f"No fresh images on {self.image_topic}; check the camera publisher."
        with self._condition:
            sequence = self._sequence
            while True:
                if self._stack is None:
                    raise RuntimeError("Use TopicCapture inside a with block.")
                remaining = deadline - monotonic()
                if remaining <= 0:
                    raise RuntimeError(f"Capture timed out: {reason}")
                image = self._image
                if image is not None and self._sequence > sequence and image.ts > last_ts:
                    if unmatched_ts is not None and image.ts != unmatched_ts:
                        # The previous image was replaced before its TF arrived.
                        first_pose = None
                    age = time() - image.ts
                    if image.ts < started or not -TF_TOLERANCE_SEC <= age <= MAX_FRAME_AGE_SEC:
                        first_pose = None
                        reason = (
                            "Image timestamps are stale; check the live stream and system clocks."
                        )
                    else:
                        pose = self._buffer.get(
                            self.base_frame,
                            self.ee_frame,
                            time_point=image.ts,
                            time_tolerance=TF_TOLERANCE_SEC,
                            warn=False,
                        )
                        if pose is None:
                            unmatched_ts = image.ts
                            reason = (
                                f"No fresh TF from {self.base_frame} to {self.ee_frame}; "
                                f"check frame names and the publisher on {self.tf_topic}."
                            )
                        else:
                            unmatched_ts = None
                            if (
                                first_pose is None
                                or image.ts - last_ts > STABLE_DURATION_SEC
                                or first_pose.translation.distance(pose.translation) > 0.001
                                or first_pose.rotation.angle_to(pose.rotation) > math.radians(0.5)
                            ):
                                first_pose, first_ts = pose, image.ts
                            elif image.ts - first_ts >= STABLE_DURATION_SEC:
                                return image, pose
                            last_ts = image.ts
                            reason = "Hold the arm still within 1 mm / 0.5 degrees for 0.5 seconds."
                self._condition.wait(timeout=remaining)
