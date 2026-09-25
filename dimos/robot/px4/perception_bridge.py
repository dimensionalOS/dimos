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

"""PerceptionBridge: a selected detection -> line of sight -> ground target, for FOLLOW and YAW_TRACK.

Consumes ``detections`` (Detection2DModule: pixel boxes with the detector's track ids) and
the connection's odometry, gimbal attitude and home-relative pose, and publishes the three
streams the connection reads: ``target_state``, ``target_valid``, ``target_los``. The
operator picks the track with the ``select_track`` RPC. The geometry and the target filter
are ``dimos/perception/geolocation``.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.geolocation.estimators import (
    EstimatorConfig,
    GimbalGeo,
    LosEstimator,
    LosResult,
    Observation,
    TargetEstimator,
    TargetState,
    VehicleGeo,
)
from dimos.perception.geolocation.geometry import CameraModel, GimbalFrameConfig, LOSSolver
from dimos.robot.px4.mavlink import flu_to_ned, ned_to_flu
from dimos.robot.px4.timebase import TimedBuffer
from dimos.types.timestamped import to_timestamp
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# COCO: person, bicycle, car, motorcycle, bus, truck, dog.
FOLLOWABLE_CLASS_IDS = frozenset({0, 1, 2, 3, 5, 7, 16})


def followable(det: Any) -> bool:
    """The ``filter`` for Detection2DModule: drop every other class."""
    return det.class_id in FOLLOWABLE_CLASS_IDS


def _gap_s(buffer: TimedBuffer, t: float) -> float:
    """How far ``t`` lies outside the buffered stamps; 0 inside. ``at`` clamps out there."""
    return max(0.0, t - buffer.t[-1], buffer.t[0] - t)


class PerceptionBridgeConfig(ModuleConfig):
    # Intrinsics of the frames the detector ran on; the same object Detection2DModule takes.
    camera_info: CameraInfo
    gimbal_frame: GimbalFrameConfig = Field(default_factory=GimbalFrameConfig)
    estimator: EstimatorConfig = Field(default_factory=EstimatorConfig)
    odom_frame_id: str = Field(default="odom")
    base_frame_id: str = Field(default="base_link")
    # Detection2DArray encodes results_length=0, so the class does not cross the transport:
    # every target gets this class and its aim height.
    default_class: str = Field(default="person")


class PerceptionBridge(Module):
    """Track selection, line of sight and target estimator on Detection2DModule's output."""

    config: PerceptionBridgeConfig

    detections: In[Detection2DArray]
    odometry: In[Odometry]
    gimbal_attitude: In[JointState]
    global_pose: In[PoseStamped]

    target_state: Out[Odometry]
    target_valid: Out[Bool]
    target_los: Out[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        cfg = self.config
        self._lock = threading.Lock()
        self._los = LosEstimator(
            LOSSolver(CameraModel(cfg.camera_info), cfg.gimbal_frame), cfg.estimator
        )
        self._target = TargetEstimator(cfg.estimator)
        self._heading = TimedBuffer(2.0, angular=("yaw",))
        self._gimbal = TimedBuffer(2.0, angular=("pitch", "yaw"))
        self._gimbal_failure = 0
        self._position: tuple[float, float, float] | None = None
        self._rel_alt: float | None = None
        self._selected: int | None = None
        self._observations: list[Observation] = []
        self._last_los: LosResult | None = None
        self._last_state: TargetState | None = None
        self._frames = 0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.detections.subscribe(self._on_detections)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.gimbal_attitude.subscribe(self._on_gimbal)))
        self.register_disposable(Disposable(self.global_pose.subscribe(self._on_global_pose)))

    # Inputs

    def _on_detections(self, msg: Detection2DArray) -> None:
        try:
            self.process(msg)
        except Exception:
            logger.exception("perception frame failed")

    def _on_odometry(self, msg: Odometry) -> None:
        # dimOS FLU yaw -> the NED heading the geolocation maths uses.
        with self._lock:
            self._heading.push(msg.ts, {"yaw": -math.degrees(msg.orientation.to_euler().z)})
            self._position = flu_to_ned(msg.x, msg.y, msg.z)

    def _on_gimbal(self, msg: JointState) -> None:
        angles = dict(zip(msg.name, msg.position, strict=False))
        if not {"gimbal_pitch", "gimbal_yaw"} <= set(angles):
            return
        with self._lock:
            self._gimbal.push(
                msg.ts,
                {
                    "pitch": math.degrees(angles["gimbal_pitch"]),
                    "yaw": math.degrees(angles["gimbal_yaw"]),
                },
            )
            # effort is (device flags, failure flags), the connection's layout.
            self._gimbal_failure = int(msg.effort[1]) if len(msg.effort) > 1 else 0

    def _on_global_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._rel_alt = float(msg.z)

    # Processing

    def process(self, msg: Detection2DArray, now: float | None = None) -> TargetState:
        """Run line of sight and the target filter on one detection frame and publish."""
        wall = time.time() if now is None else now
        # Published in-process the message is the bare LCM type, which has no ``ts``.
        ts = to_timestamp(msg.header.stamp)
        observations = self._parse(msg)
        with self._lock:
            self._observations = observations
            selected = self._selected
            veh, gimbal = self._geo(ts)
        los = self._los.process(observations, selected, ts, veh, gimbal)
        state = self._target.process(los, veh, wall)
        with self._lock:
            self._last_los, self._last_state = los, state
            self._frames += 1
        self._publish(los, state)
        return state

    def _parse(self, msg: Detection2DArray) -> list[Observation]:
        out: list[Observation] = []
        for d in msg.detections[: msg.detections_length]:
            # No track id (a detector without a tracker says -1): nothing to select by.
            if not d.id.isdigit():
                continue
            w, h = d.bbox.size_x, d.bbox.size_y
            x, y = d.bbox.center.position.x - w / 2.0, d.bbox.center.position.y - h / 2.0
            out.append(Observation(int(d.id), self.config.default_class, (x, y, w, h)))
        return out

    def _geo(self, capture_time: float) -> tuple[VehicleGeo, GimbalGeo | None]:
        """The vehicle and the gimbal as they were when the frame was captured.

        Their ages are the frame's distance from the buffered stamps, so a dead stream and a
        frame clock that disagrees with the vehicle's both read as stale.
        """
        heading = self._heading.at(capture_time)
        pos = self._position
        veh = VehicleGeo(
            yaw_deg=heading["yaw"] if heading else None,
            attitude_age_s=_gap_s(self._heading, capture_time) if heading else math.inf,
            n=pos[0] if pos else None,
            e=pos[1] if pos else None,
            d=pos[2] if pos else None,
            rel_alt=self._rel_alt,
        )
        gim = self._gimbal.at(capture_time)
        if gim is None:
            return veh, None
        return veh, GimbalGeo(
            gim["pitch"], gim["yaw"], _gap_s(self._gimbal, capture_time), self._gimbal_failure
        )

    # Outputs

    def _publish(self, los: LosResult, state: TargetState) -> None:
        self.target_valid.publish(Bool(data=state.valid))
        if state.n is not None and state.e is not None:
            x, y, _ = ned_to_flu(state.n, state.e, 0.0)
            vx, vy, _ = ned_to_flu(state.vn or 0.0, state.ve or 0.0, 0.0)
            self.target_state.publish(
                Odometry(
                    ts=state.t,
                    frame_id=self.config.odom_frame_id,
                    child_frame_id="target",
                    pose=Pose(Vector3(x, y, 0.0), Quaternion()),
                    twist=Twist(Vector3(vx, vy, 0.0), Vector3()),
                )
            )
        if los.valid and los.gimbal_yaw_body_deg is not None and los.elevation_deg is not None:
            # base_link FLU: yaw counter-clockwise, pitch nose-down positive.
            self.target_los.publish(
                PoseStamped(
                    ts=los.capture_time,
                    frame_id=self.config.base_frame_id,
                    position=Vector3(),
                    orientation=Quaternion.from_euler(
                        Vector3(
                            0.0,
                            -math.radians(los.elevation_deg),
                            -math.radians(los.gimbal_yaw_body_deg),
                        )
                    ),
                )
            )

    # RPCs

    @rpc
    def select_track(self, track_id: int) -> dict[str, Any]:
        """Follow this track id. The selection persists while the track is lost."""
        with self._lock:
            self._selected = int(track_id)
        logger.info("target selected", track_id=track_id)
        return {"selected_track_id": int(track_id)}

    @rpc
    def clear_selection(self) -> dict[str, Any]:
        """Drop the selection; target_valid goes false."""
        with self._lock:
            self._selected = None
        return {"selected_track_id": None}

    @rpc
    def status(self) -> dict[str, Any]:
        """Selection, visible tracks, the last line-of-sight verdict and target state."""
        with self._lock:
            los, state, observations = self._last_los, self._last_state, list(self._observations)
            return {
                "selected_track_id": self._selected,
                "frames": self._frames,
                "tracks": [
                    {"track_id": o.track_id, "class_name": o.class_name} for o in observations
                ],
                "los": None
                if los is None
                else {
                    "valid": los.valid,
                    "reason": los.reason,
                    "azimuth_deg": los.azimuth_deg,
                    "elevation_deg": los.elevation_deg,
                },
                "target": None
                if state is None
                else {
                    "valid": state.valid,
                    "reason": state.reason,
                    "n": state.n,
                    "e": state.e,
                    "range_m": state.range_m,
                    "bearing_deg": state.bearing_deg,
                },
            }
