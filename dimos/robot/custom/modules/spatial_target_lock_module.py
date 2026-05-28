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

import json
import math
import threading
import time
from typing import Any, Literal

from dimos_lcm.sensor_msgs import CameraInfo as DimosLcmCameraInfo  # type: ignore[import-untyped]
from dimos_lcm.std_msgs import Bool, String  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.robot.custom.modules.bbox_selection_module import BBoxSelectionModule
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_FRAME_ID = "camera_optical"
_WORLD_FRAME_ID = "world"

LockState = Literal["unselected", "waiting_for_pose", "locked", "using_memory"]
PoseSource = Literal["none", "selected_bbox", "spatial_reacquire", "memory"]


def _safe_track_id(raw: Any) -> int:
    try:
        return int(raw)
    except (TypeError, ValueError):
        return 0


class SpatialTargetLockConfig(ModuleConfig):
    tf_time_tolerance: float = 0.5
    reacquire_distance_m: float = 0.75
    reacquire_max_z_delta_m: float = 0.35
    reacquire_by_class: bool = True
    preserve_identity_on_spatial_reacquire: bool = True


class SpatialTargetLockModule(Module):
    """Keep a selected bbox locked as a world-frame target pose."""

    config: SpatialTargetLockConfig

    detections: In[Detection2DArray]
    selected_bbox: In[Detection2DArray]
    lidar: In[PointCloud2]
    camera_info: In[CameraInfo]
    stop_movement: In[Bool]
    clear_selection_request: In[Bool]

    locked_bbox: Out[Detection2DArray]
    target_pose: Out[PoseStamped]
    lock_status: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._state: LockState = "unselected"
        self._target_id: str | None = None
        self._stable_target_id: str | None = None
        self._target_class_id: str | None = None
        self._last_pose: PoseStamped | None = None
        self._pose_source: PoseSource = "none"
        self._last_seen_at: float | None = None
        self._latest_lidar: PointCloud2 | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._last_selected_detection: Any | None = None
        self._last_header: Header | None = None
        self._last_projection_block_reason: str | None = None

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(
            "SpatialTargetLockModule: started "
            f"tf_time_tolerance={self.config.tf_time_tolerance:.3f}s "
            f"reacquire_distance_m={self.config.reacquire_distance_m:.3f} "
            f"reacquire_max_z_delta_m={self.config.reacquire_max_z_delta_m:.3f} "
            f"reacquire_by_class={self.config.reacquire_by_class} "
            "preserve_identity_on_spatial_reacquire="
            f"{self.config.preserve_identity_on_spatial_reacquire}"
        )
        self.register_disposable(Disposable(self.selected_bbox.subscribe(self._on_selected_bbox)))
        self.register_disposable(Disposable(self.detections.subscribe(self._on_detections)))
        self.register_disposable(Disposable(self.lidar.subscribe(self._on_lidar)))
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))
        self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop_movement)))
        self.register_disposable(
            Disposable(self.clear_selection_request.subscribe(self._on_clear_selection_request))
        )
        self._publish_status(force=True)

    @rpc
    def clear_lock(self) -> str:
        self._clear_lock()
        return "spatial target lock cleared"

    @rpc
    def get_lock_state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "target_id": self._target_id,
                "target_class_id": self._target_class_id,
                "last_pose": self._pose_to_dict(self._last_pose),
                "pose_source": self._pose_source,
                "last_seen_at": self._last_seen_at,
            }

    def _on_lidar(self, lidar: PointCloud2) -> None:
        with self._lock:
            self._latest_lidar = lidar
            should_log = self._state == "waiting_for_pose"
        self._try_project_waiting_selection()
        if should_log:
            logger.debug(
                "SpatialTargetLockModule: received lidar while waiting "
                f"frame_id={lidar.frame_id!r} ts={float(lidar.ts or 0.0):.3f}"
            )

    def _on_camera_info(self, camera_info: CameraInfo) -> None:
        with self._lock:
            self._latest_camera_info = camera_info
            should_log = self._state == "waiting_for_pose"
        self._try_project_waiting_selection()
        if should_log:
            logger.debug(
                "SpatialTargetLockModule: received camera_info while waiting "
                f"width={camera_info.width} height={camera_info.height}"
            )

    def _on_selected_bbox(self, selected_bbox: Detection2DArray) -> None:
        with self._lock:
            self._last_header = selected_bbox.header

        detection = self._extract_single_detection(selected_bbox)
        if detection is None:
            with self._lock:
                has_active_lock = self._target_id is not None or self._last_pose is not None
            if not has_active_lock:
                logger.debug(
                    "SpatialTargetLockModule: received empty selected_bbox with no active lock; "
                    "publishing empty locked_bbox"
                )
                self.locked_bbox.publish(self._empty_detection_array(selected_bbox.header))
                self._set_state("unselected", pose_source="none")
            else:
                logger.debug(
                    "SpatialTargetLockModule: received empty selected_bbox but keeping spatial "
                    "memory target active"
                )
            return

        with self._lock:
            has_frozen_pose = self._last_pose is not None
        if has_frozen_pose:
            logger.debug(
                "SpatialTargetLockModule: ignoring selected_bbox update because target pose "
                "is already frozen"
            )
            return

        self._set_selected_target(detection, selected_bbox.header)
        logger.info(
            "SpatialTargetLockModule: selected bbox "
            f"{self._detection_summary(detection)} "
            f"header_frame={selected_bbox.header.frame_id!r} "
            f"header_ts={self._header_timestamp(selected_bbox.header):.3f}"
        )
        pose = self._project_detection_pose(detection)
        if pose is None:
            with self._lock:
                last_pose = self._last_pose
            self.locked_bbox.publish(self._single_detection_array(detection, selected_bbox.header))
            if last_pose is not None:
                logger.info(
                    "SpatialTargetLockModule: selected bbox projection failed; "
                    f"using memory pose {self._pose_summary(last_pose)}"
                )
                self._publish_memory(selected_bbox.header)
            else:
                logger.info(
                    "SpatialTargetLockModule: selected bbox projection failed; "
                    "waiting for lidar/camera_info/tf"
                )
                self._set_state("waiting_for_pose", pose_source="none")
            return

        self._accept_detection_pose(
            detection,
            selected_bbox.header,
            pose,
            pose_source="selected_bbox",
        )

    def _on_detections(self, detections: Detection2DArray) -> None:
        with self._lock:
            self._last_header = detections.header
            target_id = self._target_id
            last_pose = self._last_pose

        if target_id is None and last_pose is None:
            self.locked_bbox.publish(self._empty_detection_array(detections.header))
            self._set_state("unselected", pose_source="none")
            return

        # First pass: YOLO is only used to provide the initial selected bbox.  Once a
        # 3D target pose exists, keep publishing that frozen memory pose and do not
        # spatially re-match detections.
        self._publish_memory(detections.header)

    def _on_stop_movement(self, msg: Bool) -> None:
        if bool(getattr(msg, "data", False)):
            logger.info("SpatialTargetLockModule: stop_movement requested; clearing lock")
            self._clear_lock()

    def _on_clear_selection_request(self, msg: Bool) -> None:
        if bool(getattr(msg, "data", False)):
            logger.info("SpatialTargetLockModule: clear_selection_request received; clearing lock")
            self._clear_lock()

    def _try_project_waiting_selection(self) -> None:
        with self._lock:
            detection = self._last_selected_detection
            header = self._last_header
            should_try = detection is not None and self._last_pose is None

        if not should_try:
            return

        pose = self._project_detection_pose(detection)
        if pose is None:
            logger.debug("SpatialTargetLockModule: waiting selection projection retry failed")
            return

        logger.info(
            "SpatialTargetLockModule: waiting selection projection recovered "
            f"pose={self._pose_summary(pose)}"
        )
        self._accept_detection_pose(detection, header, pose, pose_source="selected_bbox")

    def _set_selected_target(self, detection: Any, header: Header | None) -> None:
        stable_id = self._stable_detection_id(detection)
        with self._lock:
            self._target_id = stable_id or self._detection_id(detection, fallback_index=0)
            self._stable_target_id = stable_id
            self._target_class_id = self._detection_class_id(detection)
            self._last_selected_detection = detection
            self._last_header = header
        logger.debug(
            "SpatialTargetLockModule: target identity set "
            f"target_id={self._target_id!r} stable_id={stable_id!r} "
            f"class_id={self._target_class_id!r}"
        )

    def _accept_detection_pose(
        self,
        detection: Any,
        header: Header | None,
        pose: PoseStamped,
        pose_source: PoseSource,
        *,
        preserve_identity: bool = False,
    ) -> None:
        now = time.monotonic()
        stable_id = self._stable_detection_id(detection)
        with self._lock:
            previous_target_id = self._target_id
            previous_stable_target_id = self._stable_target_id
            previous_target_class_id = self._target_class_id
            if not preserve_identity:
                self._target_id = stable_id or self._detection_id(detection, fallback_index=0)
                self._stable_target_id = stable_id
                self._target_class_id = self._detection_class_id(detection)
            self._last_pose = pose
            self._pose_source = pose_source
            self._last_seen_at = now
            self._last_selected_detection = detection
            self._last_header = header
            target_id = self._target_id
            target_class_id = self._target_class_id

        self.locked_bbox.publish(self._single_detection_array(detection, header))
        self.target_pose.publish(pose)
        if preserve_identity:
            logger.info(
                "SpatialTargetLockModule: spatial reacquire preserving selected identity "
                f"target_id={previous_target_id!r} stable_id={previous_stable_target_id!r} "
                f"class_id={previous_target_class_id!r} "
                f"matched_detection_id={stable_id!r}"
            )
        logger.info(
            "SpatialTargetLockModule: accepted target pose "
            f"source={pose_source} target_id={target_id!r} "
            f"class_id={target_class_id!r} pose={self._pose_summary(pose)}"
        )
        self._set_state("locked", pose_source=pose_source)

    def _publish_memory(self, header: Header | None) -> None:
        with self._lock:
            last_pose = self._last_pose
        if last_pose is None:
            logger.info(
                "SpatialTargetLockModule: no memory pose available; waiting for pose"
            )
            self.locked_bbox.publish(self._empty_detection_array(header))
            self._set_state("waiting_for_pose", pose_source="none")
            return

        self.locked_bbox.publish(self._empty_detection_array(header))
        self.target_pose.publish(last_pose)
        logger.debug(
            "SpatialTargetLockModule: publishing memory target pose "
            f"{self._pose_summary(last_pose)}"
        )
        self._set_state("using_memory", pose_source="memory")

    def _find_spatial_reacquire(
        self,
        detections: Detection2DArray,
        target_class_id: str | None,
        last_pose: PoseStamped | None,
    ) -> tuple[Any, PoseStamped] | None:
        if last_pose is None or not detections.detections:
            if last_pose is not None:
                logger.debug(
                    "SpatialTargetLockModule: cannot spatially reacquire; "
                    "detections are empty"
                )
            return None

        candidates = list(detections.detections)
        original_count = len(candidates)
        if self.config.reacquire_by_class and target_class_id is not None:
            class_matches = [
                detection
                for detection in candidates
                if self._detection_class_id(detection) == target_class_id
            ]
            if class_matches:
                candidates = class_matches
        logger.debug(
            "SpatialTargetLockModule: spatial reacquire candidates "
            f"original_count={original_count} filtered_count={len(candidates)} "
            f"target_class_id={target_class_id!r}"
        )

        best: tuple[float, Any, PoseStamped] | None = None
        projected_count = 0
        rejected_by_distance = 0
        rejected_by_z = 0
        for detection in candidates:
            pose = self._project_detection_pose(detection)
            if pose is None:
                continue
            projected_count += 1
            z_delta = abs(float(pose.position.z) - float(last_pose.position.z))
            if z_delta > self.config.reacquire_max_z_delta_m:
                rejected_by_z += 1
                logger.debug(
                    "SpatialTargetLockModule: spatial reacquire rejected by z delta "
                    f"z_delta={z_delta:.3f} gate_m={self.config.reacquire_max_z_delta_m:.3f} "
                    f"{self._detection_summary(detection)} pose={self._pose_summary(pose)} "
                    f"memory_pose={self._pose_summary(last_pose)}"
                )
                continue
            distance = self._xyz_distance(pose.position, last_pose.position)
            if distance > self.config.reacquire_distance_m:
                rejected_by_distance += 1
                continue
            if best is None or distance < best[0]:
                best = (distance, detection, pose)

        if best is None:
            logger.debug(
                "SpatialTargetLockModule: spatial reacquire failed "
                f"candidates={len(candidates)} projected={projected_count} "
                f"rejected_by_distance={rejected_by_distance} "
                f"rejected_by_z={rejected_by_z} "
                f"distance_gate_m={self.config.reacquire_distance_m:.3f} "
                f"z_gate_m={self.config.reacquire_max_z_delta_m:.3f}",
            )
            return None
        logger.debug(
            "SpatialTargetLockModule: spatial reacquire selected "
            f"distance_to_memory={best[0]:.3f} "
            f"{self._detection_summary(best[1])} pose={self._pose_summary(best[2])}"
        )
        return best[1], best[2]

    def _project_detection_pose(self, detection: Any) -> PoseStamped | None:
        with self._lock:
            lidar = self._latest_lidar
            camera_info = self._latest_camera_info

        if lidar is None or camera_info is None:
            missing = []
            if lidar is None:
                missing.append("lidar")
            if camera_info is None:
                missing.append("camera_info")
            self._set_projection_block_reason("missing_" + "_and_".join(missing))
            return None

        x1, y1, x2, y2 = BBoxSelectionModule._bbox_corners(detection)
        class_id = self._detection_class_id(detection)
        det2d = Detection2DBBox(
            bbox=(x1, y1, x2, y2),
            track_id=_safe_track_id(self._stable_detection_id(detection)),
            class_id=_safe_track_id(class_id),
            confidence=self._detection_confidence(detection),
            name=class_id or "",
            ts=float(lidar.ts or 0.0),
            image=None,  # type: ignore[arg-type]
        )

        ts = float(lidar.ts or 0.0)
        try:
            world_to_optical = self.tf.get(
                "camera_optical",
                lidar.frame_id,
                ts,
                time_tolerance=self.config.tf_time_tolerance,
            )
        except RuntimeError as exc:
            self._set_projection_block_reason(
                f"missing_tf camera_optical->{lidar.frame_id!r}: {exc}"
            )
            return None
        if world_to_optical is None:
            self._set_projection_block_reason(
                f"missing_tf camera_optical->{lidar.frame_id!r}"
            )
            return None

        lcm_camera_info = DimosLcmCameraInfo()
        lcm_camera_info.K = camera_info.K
        lcm_camera_info.width = camera_info.width
        lcm_camera_info.height = camera_info.height

        detection_3d = Detection3DPC.from_2d(
            det=det2d,
            world_pointcloud=lidar,
            camera_info=lcm_camera_info,
            world_to_optical_transform=world_to_optical,
            filters=[],
        )
        if detection_3d is None:
            self._set_projection_block_reason(
                "no_3d_points_in_bbox "
                f"bbox=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}) "
                f"lidar_frame={lidar.frame_id!r}"
            )
            return None

        self._set_projection_block_reason(None)
        center = detection_3d.center
        return PoseStamped(
            ts=float(lidar.ts or time.time()),
            frame_id=lidar.frame_id or _WORLD_FRAME_ID,
            position=Vector3(center.x, center.y, center.z),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def _clear_lock(self) -> None:
        with self._lock:
            header = self._last_header
            target_id = self._target_id
            self._target_id = None
            self._stable_target_id = None
            self._target_class_id = None
            self._last_pose = None
            self._pose_source = "none"
            self._last_seen_at = None
            self._last_selected_detection = None
            self._last_projection_block_reason = None
        logger.info(f"SpatialTargetLockModule: lock cleared previous_target_id={target_id!r}")
        self.locked_bbox.publish(self._empty_detection_array(header))
        self._set_state("unselected", pose_source="none")

    def _set_state(self, state: LockState, pose_source: PoseSource) -> None:
        with self._lock:
            old_state = self._state
            old_source = self._pose_source
            self._state = state
            self._pose_source = pose_source
            target_id = self._target_id
            last_pose = self._last_pose
        status_changed = old_state != state or old_source != pose_source
        is_memory_refresh = state == "using_memory" and old_state in {
            "locked",
            "using_memory",
        }
        if status_changed and not is_memory_refresh:
            logger.info(
                "SpatialTargetLockModule: state changed "
                f"{old_state}/{old_source} -> {state}/{pose_source} "
                f"target_id={target_id!r} pose={self._pose_summary(last_pose)}"
            )
        self._publish_status(force=status_changed)

    def _set_projection_block_reason(self, reason: str | None) -> None:
        with self._lock:
            previous = self._last_projection_block_reason
            if previous == reason:
                return
            self._last_projection_block_reason = reason
        if reason is None:
            if previous is not None:
                logger.info(
                    "SpatialTargetLockModule: projection recovered "
                    f"previous_reason={previous}"
                )
            return
        logger.info(f"SpatialTargetLockModule: projection blocked reason={reason}")

    def _publish_status(self, force: bool = False) -> None:
        with self._lock:
            payload = {
                "state": self._state,
                "pose_source": self._pose_source,
                "target_id": self._target_id,
                "target_class_id": self._target_class_id,
                "last_pose": self._pose_to_dict(self._last_pose),
                "last_seen_at": self._last_seen_at,
            }
        if force or self._state in ("unselected", "using_memory", "waiting_for_pose"):
            self.lock_status.publish(String(json.dumps(payload, ensure_ascii=True)))

    @staticmethod
    def _extract_single_detection(detections: Detection2DArray | None) -> Any | None:
        if detections is None or not detections.detections:
            return None
        return detections.detections[0]

    @staticmethod
    def _single_detection_array(detection: Any, header: Header | None) -> Detection2DArray:
        safe_header = header if header is not None else Header(time.time(), _DEFAULT_FRAME_ID)
        return Detection2DArray(detections_length=1, header=safe_header, detections=[detection])

    @staticmethod
    def _empty_detection_array(header: Header | None) -> Detection2DArray:
        safe_header = header if header is not None else Header(time.time(), _DEFAULT_FRAME_ID)
        return Detection2DArray(detections_length=0, header=safe_header, detections=[])

    @classmethod
    def _find_by_stable_id(
        cls, detections: Detection2DArray, stable_target_id: str
    ) -> Any | None:
        for detection in detections.detections:
            if cls._stable_detection_id(detection) == stable_target_id:
                return detection
        return None

    @staticmethod
    def _stable_detection_id(detection: Any) -> str | None:
        detection_id = str(getattr(detection, "id", "")).strip()
        if not detection_id or detection_id == "-1":
            return None
        return detection_id

    @classmethod
    def _detection_id(cls, detection: Any, fallback_index: int) -> str:
        return cls._stable_detection_id(detection) or str(fallback_index)

    @staticmethod
    def _detection_class_id(detection: Any) -> str | None:
        results = getattr(detection, "results", [])
        if not results:
            return None
        hypothesis = results[0].hypothesis
        class_id = getattr(hypothesis, "class_id", None)
        return str(class_id) if class_id is not None else None

    @staticmethod
    def _detection_confidence(detection: Any) -> float:
        results = getattr(detection, "results", [])
        if not results:
            return 1.0
        hypothesis = results[0].hypothesis
        return float(getattr(hypothesis, "score", 1.0) or 1.0)

    @classmethod
    def _detection_summary(cls, detection: Any) -> str:
        x1, y1, x2, y2 = BBoxSelectionModule._bbox_corners(detection)
        return (
            f"id={cls._stable_detection_id(detection)!r} "
            f"class_id={cls._detection_class_id(detection)!r} "
            f"score={cls._detection_confidence(detection):.3f} "
            f"bbox=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f})"
        )

    @staticmethod
    def _xy_distance(a: Vector3, b: Vector3) -> float:
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def _xyz_distance(a: Vector3, b: Vector3) -> float:
        dx = float(a.x) - float(b.x)
        dy = float(a.y) - float(b.y)
        dz = float(a.z) - float(b.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _pose_to_dict(pose: PoseStamped | None) -> dict[str, Any] | None:
        if pose is None:
            return None
        return {
            "frame_id": pose.frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "ts": pose.ts,
        }

    @classmethod
    def _pose_summary(cls, pose: PoseStamped | None) -> str:
        payload = cls._pose_to_dict(pose)
        if payload is None:
            return "None"
        return (
            f"frame={payload['frame_id']!r} "
            f"x={float(payload['x']):.3f} "
            f"y={float(payload['y']):.3f} "
            f"z={float(payload['z']):.3f} "
            f"ts={float(payload['ts'] or 0.0):.3f}"
        )

    @staticmethod
    def _header_timestamp(header: Header | None) -> float:
        if header is None:
            return 0.0
        try:
            return float(header.timestamp)
        except (AttributeError, TypeError, ValueError):
            pass
        try:
            return float(header.ts)
        except (AttributeError, TypeError, ValueError):
            pass
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return 0.0
        sec = float(getattr(stamp, "sec", 0.0) or 0.0)
        nsec = float(getattr(stamp, "nsec", 0.0) or 0.0)
        return sec + nsec / 1_000_000_000.0


__all__ = [
    "SpatialTargetLockConfig",
    "SpatialTargetLockModule",
]
