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

"""SiyiA8Gimbal: the A8 mini's frame chain, camera intrinsics, and aim requests.

Consumes ``gimbal_attitude`` (JointState, radians, MAVLink signs) and publishes the tf
chain ``base_link -> gimbal_base -> gimbal_link -> a8_optical`` and the CameraInfo of the
1280x720 stream. Aim requests go out on ``gimbal_target`` for whichever module owns the
MAVLink link; this one opens no MAVLink socket. The A8 reports yaw body-relative and pitch
earth-stabilised; the pitch is placed in the body frame as reported, exact when the
airframe is level.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.gimbal.siyi.frame import (
    PITCH_MAX_DEG,
    PITCH_MIN_DEG,
    YAW_MAX_DEG,
    YAW_MIN_DEG,
    decode_flags,
)
from dimos.hardware.gimbal.siyi.sdk import SiyiSdk
from dimos.hardware.sensors.camera.spec import OPTICAL_ROTATION
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

A8_HFOV_DEG = 81.0  # A8 mini main stream at 1x
_TARGET_JOINTS = ["gimbal_pitch", "gimbal_yaw"]


def a8_camera_info(
    width: int = 1280, height: int = 720, frame_id: str = "a8_optical"
) -> CameraInfo:
    """Pinhole intrinsics of the A8 mini at 1x for frames of this size (fx ~ 749.3 px at 1280)."""
    f = (width / 2) / math.tan(math.radians(A8_HFOV_DEG) / 2)
    return CameraInfo.from_intrinsics(f, f, width / 2, height / 2, width, height, frame_id)


@dataclass
class _Attitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    flags: int
    failure_flags: int
    ts: float
    rx_mono: float


class SiyiA8GimbalConfig(ModuleConfig):
    # base_link -> gimbal_base, metres, FLU: set per airframe.
    mount_xyz: tuple[float, float, float] = Field(default=(0.0, 0.0, 0.0))
    # gimbal_link -> a8_optical: the lens sits ahead of and below the pivot.
    optical_xyz: tuple[float, float, float] = Field(default=(0.03, 0.0, -0.02))
    base_frame_id: str = Field(default="base_link")
    gimbal_base_frame_id: str = Field(default="gimbal_base")
    gimbal_link_frame_id: str = Field(default="gimbal_link")
    optical_frame_id: str = Field(default="a8_optical")
    tf_hz: float = Field(default=10.0, gt=0)
    camera_info_hz: float = Field(default=1.0)
    # No transform from an attitude older than this: a stale gimbal angle mislocates the target.
    attitude_max_age_s: float = Field(default=1.0)
    # Turn target_los into gimbal_target aim requests.
    aim_enabled: bool = Field(default=False)
    aim_hz: float = Field(default=10.0, gt=0)
    # Camera address for the SIYI SDK (zoom poll). None = SDK off. No default: it is the
    # site's network config, set in the blueprint.
    ip: str | None = Field(default=None)
    # camera_info is withheld while the polled zoom is unknown or not 1x.
    zoom_poll_s: float = Field(default=1.0, gt=0)


class SiyiA8Gimbal(Module):
    """A8 mini frame chain, intrinsics and aim requests. No MAVLink; one UDP socket to the
    camera when ``ip`` is set."""

    config: SiyiA8GimbalConfig

    gimbal_attitude: In[JointState]
    target_los: In[PoseStamped]

    tf: Out[TFMessage]
    camera_info: Out[CameraInfo]
    gimbal_target: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._attitude: _Attitude | None = None
        self._zoom: float | None = None
        self._last_aim_mono = 0.0
        self._aim_sent = 0
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._sdk: SiyiSdk | None = None
        self._camera_info = a8_camera_info(frame_id=self.config.optical_frame_id)

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.ip is not None:
            self._sdk = SiyiSdk(self.config.ip)
            self._sdk.open()
        self.register_disposable(Disposable(self.gimbal_attitude.subscribe(self._on_attitude)))
        self.register_disposable(Disposable(self.target_los.subscribe(self._on_target_los)))
        self._stop_event.clear()
        self._threads = [threading.Thread(target=self._tf_loop, name="siyi-tf", daemon=True)]
        if self._sdk is not None:
            self._threads.append(
                threading.Thread(target=self._zoom_loop, name="siyi-zoom", daemon=True)
            )
        for t in self._threads:
            t.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._threads.clear()
        if self._sdk is not None:
            self._sdk.close()
            self._sdk = None
        super().stop()

    # Inputs

    def _on_attitude(self, msg: JointState) -> None:
        angles = dict(zip(msg.name, msg.position, strict=False))
        try:
            roll, pitch, yaw = (angles[k] for k in ("gimbal_roll", "gimbal_pitch", "gimbal_yaw"))
        except KeyError:
            logger.warning("gimbal_attitude without the three gimbal joints", names=msg.name)
            return
        flags = int(msg.effort[0]) if msg.effort else 0
        failure = int(msg.effort[1]) if len(msg.effort) > 1 else 0
        with self._lock:
            self._attitude = _Attitude(
                math.degrees(roll),
                math.degrees(pitch),
                math.degrees(yaw),
                flags,
                failure,
                msg.ts,
                time.monotonic(),
            )

    def _on_target_los(self, msg: PoseStamped) -> None:
        if not self.config.aim_enabled:
            return
        try:
            pitch, yaw = msg.pitch, msg.yaw
        except ValueError:  # zero or non-finite quaternion
            return
        # Line of sight in base_link (FLU): yaw counter-clockwise, pitch nose-down positive.
        # The A8 wants body yaw clockwise positive and pitch up positive.
        self._request_aim(-math.degrees(pitch), -math.degrees(yaw), msg.ts)

    def _request_aim(self, pitch_deg: float, yaw_deg: float, ts: float) -> bool:
        if not (math.isfinite(pitch_deg) and math.isfinite(yaw_deg)):
            return False  # np.clip passes NaN through
        now = time.monotonic()
        if now - self._last_aim_mono < 1.0 / self.config.aim_hz:
            return False
        self._last_aim_mono = now
        self.gimbal_target.publish(
            JointState(
                ts=ts,
                frame_id=self.config.gimbal_base_frame_id,
                name=list(_TARGET_JOINTS),
                position=[
                    math.radians(float(np.clip(pitch_deg, PITCH_MIN_DEG, PITCH_MAX_DEG))),
                    math.radians(float(np.clip(yaw_deg, YAW_MIN_DEG, YAW_MAX_DEG))),
                ],
                velocity=[],
                effort=[],
            )
        )
        self._aim_sent += 1
        return True

    # Threads

    def _tf_loop(self) -> None:
        cfg = self.config
        period = 1.0 / cfg.tf_hz
        info_every = max(1, round(cfg.tf_hz / cfg.camera_info_hz)) if cfg.camera_info_hz > 0 else 0
        tick = 0
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            self.publish_transforms()
            if info_every and tick % info_every == 0:
                self.publish_camera_info()
            tick += 1
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _zoom_loop(self) -> None:
        assert self._sdk is not None
        while True:
            zoom = self._sdk.query_zoom()
            if zoom is not None:  # a lost reply keeps the last known zoom
                with self._lock:
                    self._zoom = zoom
            if self._stop_event.wait(self.config.zoom_poll_s):
                return

    def publish_transforms(self) -> bool:
        """The chain at the latest attitude. Returns False (nothing published) when stale."""
        cfg = self.config
        with self._lock:
            att = self._attitude
        if att is None or time.monotonic() - att.rx_mono > cfg.attitude_max_age_s:
            return False
        ts = att.ts
        # MAVLink gimbal angles: yaw clockwise positive, pitch up positive; FLU is the reverse.
        gimbal_rot = Quaternion.from_euler(
            Vector3(
                math.radians(att.roll_deg), -math.radians(att.pitch_deg), -math.radians(att.yaw_deg)
            )
        )
        self.tf.publish(
            TFMessage(
                Transform(
                    translation=Vector3(*cfg.mount_xyz),
                    frame_id=cfg.base_frame_id,
                    child_frame_id=cfg.gimbal_base_frame_id,
                    ts=ts,
                ),
                Transform(
                    rotation=gimbal_rot,
                    frame_id=cfg.gimbal_base_frame_id,
                    child_frame_id=cfg.gimbal_link_frame_id,
                    ts=ts,
                ),
                Transform(
                    translation=Vector3(*cfg.optical_xyz),
                    rotation=OPTICAL_ROTATION,
                    frame_id=cfg.gimbal_link_frame_id,
                    child_frame_id=cfg.optical_frame_id,
                    ts=ts,
                ),
            )
        )
        return True

    def publish_camera_info(self) -> bool:
        """Intrinsics of the main stream at 1x; withheld while the polled zoom is unknown
        or not 1x."""
        with self._lock:
            zoom = self._zoom
        if zoom is None:
            if self.config.ip is not None:
                return False
        elif abs(zoom - 1.0) > 1e-3:
            return False
        self.camera_info.publish(self._camera_info.with_ts(time.time()))
        return True

    # RPCs

    @rpc
    def aim(self, pitch_deg: float, yaw_deg: float) -> bool:
        """Request an absolute pitch/yaw (degrees, body frame). Returns whether a request
        was published."""
        return self._request_aim(pitch_deg, yaw_deg, time.time())

    @rpc
    def state(self) -> dict[str, Any]:
        """Latest reported attitude (degrees), its age, decoded flags, zoom, aim count."""
        with self._lock:
            att, zoom = self._attitude, self._zoom
        if att is None:
            return {"attitude": None, "zoom": zoom, "aim_sent": self._aim_sent}
        return {
            "attitude": {"roll": att.roll_deg, "pitch": att.pitch_deg, "yaw": att.yaw_deg},
            "age_s": time.monotonic() - att.rx_mono,
            "flags": decode_flags(att.flags),
            "failure_flags": att.failure_flags,
            "zoom": zoom,
            "aim_sent": self._aim_sent,
        }
