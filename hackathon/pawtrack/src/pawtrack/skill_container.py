"""DimOS skill container for PawTrack: find and track a described subject.

One container exposes the perception stage as DimOS skills: the user (or the
agent) describes a subject -- a person sitting on a chair, a person in a red
shirt, any object -- a VLM localizes it, EdgeTAM tracks the selected subject
frame-to-frame, and JSON status (bbox, centering, size) is published. Decision
logic lives in the pure ``MonitorState``; this module runs the effects
(tracker/VLM, publish snapshots, debug frames). The monitor loop fails loudly --
any exception is logged and surfaced as an ``error`` status rather than letting
the worker thread die silently. When EdgeTAM drops a fast-moving subject, an
optional frame-motion fallback re-seeds the tracker before retrying.

Alongside the status stream, the tracked subject's ground position is published
for an upstream planner: the subject's ground-contact pixel (bottom-center of
the bbox) is back-projected onto the floor plane (approach "B") in the live
``world`` frame, and -- when relocalization is running -- in the prebuilt
``map`` frame too.

This container is perception only: it never drives the robot. Motion (wander,
approach, greet) belongs to a separate container.
"""

from __future__ import annotations

import base64
import threading
import time
from typing import Any

import cv2
import numpy as np
from reactivex.disposable import Disposable
from turbojpeg import TurboJPEG

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.models.qwen.bbox import BBox
from dimos.models.segmentation.edge_tam import EdgeTAMProcessor
from dimos.models.vl.base import VlModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.robot.unitree.dimsim_connection import DimSimConnection
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.utils.logging_config import setup_logger

from pawtrack.ground_raycast import CameraIntrinsics, pixel_to_ground_point
from pawtrack.motion_fallback import detect_motion_bbox
from pawtrack.qwen_china import QwenChinaVlModel
from pawtrack.track_state import (
    MonitorParams,
    MonitorState,
    TrackSnapshot,
    VisualObservation,
    bbox_in_image,
    clamp_bbox,
    ground_contact_pixel,
    is_bbox_shape,
    valid_bbox,
    visual_metrics,
)

logger = setup_logger()


class Config(ModuleConfig):
    """Configuration for the PawTrack subject-tracking skills."""

    # Subject tracking.
    monitor_loop_hz: float = 15.0
    max_lost_frames: int = 15  # frames without a mask before reacquiring
    reacquire_interval_frames: int = 15  # VLM retry cadence while lost
    max_reacquire_attempts: int = 5  # give up after this many failed retries
    # Re-seed the tracker from frame motion on a miss. OFF by default: it
    # re-seeds on the largest MOVING blob, which is wrong whenever the camera
    # itself moves (a wandering/approaching dog makes the whole frame move, so
    # it would lock onto the background). Reacquire via the VLM instead.
    motion_fallback: bool = False
    max_center_jump_frac: float = 2.5  # reject jumps beyond this x subject width
    max_area_factor: float = 4.0  # reject area changes beyond this factor
    stale_timeout_s: float = 2.0  # no fix longer than this -> "stale"
    # Subject position (ground-plane raycast, approach "B"). The tracked
    # subject's ground-contact pixel -- the bottom-center of the bbox, where it
    # meets the floor -- is back-projected to the floor in the live world frame
    # for the planner. If relocalization is live, the same point is also
    # published in the stable map frame.
    camera_info: CameraInfo | None = None  # intrinsics; auto-resolved if None
    # Floor height (z) in each frame. The raycast plane is exactly this height:
    # the bottom of the bbox (feet / seat / chair base) rests on the floor, so
    # no object-size offset is added.
    world_floor_z_m: float = 0.0  # floor z in the live odometry frame
    map_floor_z_m: float = 0.0  # floor z in the prebuilt-map frame
    world_frame_id: str = "world"  # always-on live odometry frame
    map_frame_id: str = "map"  # optional prebuilt-map frame
    camera_optical_frame_id: str = "camera_optical"
    tf_tolerance_s: float = 1.0  # max age for the live world/odom TF lookup
    # The map<-world TF is republished only every ~2 s by relocalization, so the
    # map lookup needs a looser tolerance than the live odom chain or it would
    # pause for the back half of each interval even when relocalization is
    # healthy. Must exceed the relocalization publish interval (2.0 s).
    map_tf_tolerance_s: float = 3.0
    # While a pose frame's TF is absent, retry its lookup at most this often
    # rather than every tracked frame -- ``tf.get`` warns on each miss, which
    # would otherwise flood logs at the monitor loop rate.
    pose_probe_interval_s: float = 2.0


class PawTrackSkillContainer(Module):
    """Find and track a user-described subject, publishing status and position."""

    config: Config

    color_image: In[Image]
    subject_status: Out[str]  # JSON status (LCM diagnostic stream)
    debug_image: Out[Image]  # annotated camera frame (shows in Rerun)
    subject_world_pose: Out[PoseStamped]  # position in live odometry frame
    subject_map_pose: Out[PoseStamped]  # optional position in premap frame

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._monitor_stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest_image: Image | None = None
        self._prev_image: Image | None = None
        self._state = MonitorState(MonitorParams(
            max_lost_frames=self.config.max_lost_frames,
            reacquire_interval_frames=self.config.reacquire_interval_frames,
            max_reacquire_attempts=self.config.max_reacquire_attempts,
            max_center_jump_frac=self.config.max_center_jump_frac,
            max_area_factor=self.config.max_area_factor,
            stale_timeout_s=self.config.stale_timeout_s,
        ))
        self._snapshot = self._state.snapshot
        self._vl_model: VlModel | None = None
        self._tracker: EdgeTAMProcessor | None = None
        self._intrinsics = self._resolve_intrinsics()
        self._warned_no_world_tf = False
        self._warned_no_map_tf = False
        # Monotonic time before which a pose frame's TF lookup is skipped, per
        # frame_id, to throttle retries while that frame is absent.
        self._next_pose_probe_s: dict[str, float] = {}

    def _resolve_intrinsics(self) -> CameraIntrinsics | None:
        """Pinhole intrinsics for the ground-plane raycast.

        Prefer an explicit ``config.camera_info``; otherwise use the static
        intrinsics of whichever connection backs this run (sim or the real
        Go2), so the raycast works without extra blueprint wiring. Returns
        ``None`` if no usable intrinsics are found (pose publishing then
        no-ops).
        """
        camera_info = self.config.camera_info
        if camera_info is None:
            simulation = self.config.g.simulation
            if simulation == "mujoco":
                camera_info = MujocoConnection.camera_info_static
            elif simulation == "dimsim":
                camera_info = DimSimConnection.camera_info_static
            else:
                camera_info = GO2Connection.camera_info_static
        if (
            camera_info is None
            or len(camera_info.K) != 9
            or camera_info.K[0] <= 0.0
        ):
            return None
        return CameraIntrinsics.from_k(list(camera_info.K))

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.color_image.subscribe(self._on_color_image))
        )

    @rpc
    def stop(self) -> None:
        self._stop_monitoring()
        with self._lock:
            tracker = self._tracker
            self._tracker = None
            vl_model = self._vl_model
            self._vl_model = None
        if tracker is not None:
            tracker.stop()
        if vl_model is not None:
            vl_model.stop()
        super().stop()

    # -- Perception skills --

    @skill
    def track_subject(
        self,
        description: str = "a person sitting on a chair",
        initial_bbox: list[float] | None = None,
        initial_image: str | None = None,
    ) -> str:
        """Start tracking the subject matching a visual description.

        Use this to find, track, watch, or monitor anything the user describes
        -- a person sitting on a chair, a person in a red shirt, a bottle, a
        backpack -- so never refuse based on what the subject is. The
        description should identify it visually, for example "a person sitting
        on a chair" or "the person in the blue jacket". With no description, it
        looks for a person sitting on a chair (the greeter's default target).

        Args:
            description: Visual description of the subject to track. Defaults to
                "a person sitting on a chair".
            initial_bbox: Optional bbox ``[x1, y1, x2, y2]`` to skip VLM
                acquisition when another tool or UI already selected the subject.
            initial_image: Optional base64 JPEG matching ``initial_bbox``.

        Returns:
            Status text describing whether tracking started.
        """
        if initial_bbox is not None and not is_bbox_shape(initial_bbox):
            return "initial_bbox must be four numbers [x1, y1, x2, y2]."

        self._stop_monitoring()

        with self._lock:
            latest_image = self._latest_image
        if latest_image is None:
            return "No image available to identify the subject."

        # Reset the state machine for the new target before EITHER acquisition
        # path, so a previous track's last fix cannot carry forward under the
        # new description (via the drift gate or a carried-forward bbox).
        self._set_snapshot(self._state.begin(description))

        detection_image = None
        if initial_bbox is not None:
            bbox: BBox = (
                initial_bbox[0],
                initial_bbox[1],
                initial_bbox[2],
                initial_bbox[3],
            )
            if initial_image is not None:
                detection_image = _decode_base64_image(initial_image)
        else:
            detected = get_object_bbox_from_image(
                self._get_vl_model(),
                latest_image,
                description,
            )
            if detected is None:
                self._set_snapshot(TrackSnapshot(
                    status="lost",
                    description=description,
                    message=f"Could not find {description} in view.",
                ))
                return f"Could not find '{description}' in the current view."
            bbox = detected

        return self._start_tracking(description, bbox, detection_image)

    @skill
    def stop_tracking(self) -> str:
        """Stop tracking the current subject."""
        self._stop_monitoring()
        self._set_snapshot(self._state.stopped())
        return "Stopped tracking."

    @skill
    def tracking_status(self) -> str:
        """Return the latest tracked-subject bbox, size, and centering metrics."""
        with self._lock:
            snapshot = self._snapshot
        return snapshot.to_json()

    # -- Internals: perception --

    def _on_color_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image

    def _get_vl_model(self) -> VlModel:
        with self._lock:
            if self._vl_model is None:
                self._vl_model = QwenChinaVlModel()
            return self._vl_model

    def _get_tracker(self) -> EdgeTAMProcessor:
        with self._lock:
            if self._tracker is None:
                self._tracker = EdgeTAMProcessor()
            return self._tracker

    def _start_tracking(
        self,
        description: str,
        bbox: BBox,
        detection_image: Image | None,
    ) -> str:
        with self._lock:
            latest_image = self._latest_image
        if latest_image is None:
            return "No image available to start tracking."
        init_image = (
            detection_image if detection_image is not None else latest_image
        )
        if not valid_bbox(bbox, init_image.width, init_image.height):
            logger.error(
                "track_subject: invalid bbox %s for a %dx%d image",
                bbox, init_image.width, init_image.height,
            )
            self._set_snapshot(self._state.errored(
                description, f"Invalid bounding box for {description}."
            ))
            return (
                f"Invalid bounding box for '{description}'; cannot start "
                "tracking."
            )
        bbox = clamp_bbox(bbox, init_image.width, init_image.height)

        tracker = self._get_tracker()
        initial_detections = tracker.init_track(
            image=init_image, box=np.array(bbox, dtype=np.float32), obj_id=1
        )
        if len(initial_detections) == 0:
            self._set_snapshot(TrackSnapshot(
                status="lost",
                description=description,
                message=f"Tracker could not initialize on {description}.",
            ))
            return f"Could not initialize tracking for '{description}'."

        self.start_tool("track_subject")
        with self._lock:
            self._prev_image = None
            self._monitor_stop.clear()
            self._thread = threading.Thread(
                target=self._monitor_loop,
                args=(tracker, description),
                name="subject-monitor",
                daemon=True,
            )
            self._thread.start()

        return (
            f"Started tracking '{description}'. Call tracking_status for"
            " the latest size and position metrics."
        )

    def _monitor_loop(
        self,
        tracker: EdgeTAMProcessor,
        description: str,
    ) -> None:
        period = 1.0 / self.config.monitor_loop_hz
        next_time = time.monotonic()

        while not self._monitor_stop.is_set():
            next_time += period
            try:
                with self._lock:
                    image = self._latest_image
                detection = self._track_frame(tracker, image)
                metrics = (
                    self._metrics(image, detection)
                    if detection is not None else None
                )
                snapshot, action = self._state.observe(
                    description, metrics, time.monotonic())
                self._set_snapshot(snapshot)
                if action == "give_up":
                    break
                if action == "reacquire" and image is not None:
                    if self._reacquire(tracker, description, image):
                        self._state.reacquired()
                if image is not None:
                    drawn = detection if snapshot.status == "tracking" else None
                    self._publish_debug(description, image, drawn)
                    if drawn is not None and snapshot.bbox is not None:
                        self._publish_ground_poses(image, snapshot.bbox)
            except Exception:  # noqa: BLE001 - thread-boundary guard
                # Deliberately broad: this is the worker-thread top level. Fail
                # LOUDLY (full traceback + an `error` status) instead of letting
                # the thread die silently and freeze the status.
                logger.exception(
                    "subject monitor loop crashed for %r", description)
                self._set_snapshot(self._state.errored(
                    description, "Monitor loop error; see logs."))
                break

            now = time.monotonic()
            sleep_duration = next_time - now
            if sleep_duration > 0:
                self._monitor_stop.wait(sleep_duration)
            else:
                # Fell behind (e.g. a slow VLM reacquire); drop the banked lag
                # instead of spinning flat-out forever.
                next_time = now

        self.stop_tool("track_subject")

    def _track_frame(
        self, tracker: EdgeTAMProcessor, image: Image | None
    ) -> Detection2DBBox | None:
        """Track one frame, with a frame-motion re-seed fallback on a miss."""
        if image is None:
            return None
        detection = self._tracked_detection(tracker, image)
        prev_image = self._prev_image
        self._prev_image = image
        if detection is not None:
            return detection
        # EdgeTAM lost it: try to recover from frame motion (cheap, no VLM).
        # NOTE: this re-seeds on the largest moving blob over the whole frame,
        # which assumes a roughly static camera. Under robot ego-motion the
        # background can dominate and re-seed onto the wrong thing -- disable
        # via the `motion_fallback` config (off by default), or gate it on
        # robot motion state once a motion layer drives the camera.
        if not self.config.motion_fallback or prev_image is None:
            return None
        bbox = detect_motion_bbox(prev_image.to_opencv(), image.to_opencv())
        if bbox is None or not bbox_in_image(bbox, image.width, image.height):
            return None
        tracker.init_track(
            image=image, box=np.array(bbox, dtype=np.float32), obj_id=1)
        recovered = self._tracked_detection(tracker, image)
        if recovered is not None:
            # Motion recovery is a reacquire: re-baseline so the drift gate
            # does not reject the (possibly far) recovered position.
            self._state.reacquired()
        return recovered

    def _tracked_detection(
        self, tracker: EdgeTAMProcessor, image: Image
    ) -> Detection2DBBox | None:
        detections = tracker.process_image(image)
        if len(detections) == 0:
            return None
        best = max(detections.detections, key=lambda d: d.bbox_2d_volume())
        if not bbox_in_image(best.bbox, image.width, image.height):
            return None
        return best

    def _reacquire(
        self, tracker: EdgeTAMProcessor, description: str, image: Image
    ) -> bool:
        """Re-run the VLM for the description and re-init the tracker on it."""
        bbox = get_object_bbox_from_image(
            self._get_vl_model(), image, description
        )
        if bbox is None:
            return False
        if not valid_bbox(bbox, image.width, image.height):
            logger.error(
                "reacquire: VLM returned an invalid bbox %s for %r",
                bbox, description,
            )
            return False
        bbox = clamp_bbox(bbox, image.width, image.height)
        detections = tracker.init_track(
            image=image, box=np.array(bbox, dtype=np.float32), obj_id=1
        )
        return len(detections) > 0

    def _metrics(self, image: Image, detection: Detection2DBBox):
        mask = getattr(detection, "mask", None)
        mask_area = None if mask is None else float(np.count_nonzero(mask))
        return visual_metrics(VisualObservation(
            bbox=detection.bbox,
            image_width=image.width,
            image_height=image.height,
            confidence=detection.confidence,
            mask_area_px=mask_area,
        ))

    def _publish_debug(
        self,
        description: str,
        image: Image,
        detection: Detection2DBBox | None,
    ) -> None:
        """Publish an annotated frame for Rerun: green box when tracking, red
        label while searching."""
        if detection is not None:
            frame = _draw_overlay(
                image, detection.bbox, description, (0, 255, 0)
            )
        else:
            frame = _draw_overlay(
                image, None, f"searching: {description}", (0, 0, 255)
            )
        self.debug_image.publish(frame)

    def _publish_ground_poses(
        self, image: Image, bbox: tuple[float, float, float, float]
    ) -> None:
        """Publish the tracked subject's ground position for planning.

        Back-projects the subject's ground-contact pixel -- the bottom-center
        of its bbox, where it meets the floor -- onto the floor plane (approach
        "B") and publishes it as ``PoseStamped`` in the live ``world`` frame.
        If relocalization is publishing a prebuilt ``map`` frame, also publishes
        the same observation there. Tracking is never blocked on missing TF.

        Args:
            image: The frame the detection came from (supplies the timestamp).
            bbox: ``(x1, y1, x2, y2)`` of the tracked subject in pixels.
        """
        contact_px = ground_contact_pixel(bbox)
        self._publish_ground_pose(
            image,
            contact_px,
            self.config.world_frame_id,
            self.config.world_floor_z_m,
            self.subject_world_pose,
            self.config.tf_tolerance_s,
            required=True,
        )
        self._publish_ground_pose(
            image,
            contact_px,
            self.config.map_frame_id,
            self.config.map_floor_z_m,
            self.subject_map_pose,
            self.config.map_tf_tolerance_s,
            required=False,
        )

    def _publish_ground_pose(
        self,
        image: Image,
        contact_px: tuple[float, float],
        frame_id: str,
        floor_z: float,
        stream: Out[PoseStamped],
        tf_tolerance: float,
        required: bool,
    ) -> None:
        """Publish one ground-raycasted subject pose in ``frame_id``.

        Args:
            contact_px: ``(u, v)`` pixel where the subject meets the floor.
            floor_z: Floor height in ``frame_id``; the raycast plane is exactly
                ``floor_z`` because the contact pixel rests on the floor.
            tf_tolerance: Max age for the ``frame_id <- camera_optical`` lookup.
                The map frame's TF is republished slowly, so it needs a looser
                tolerance than the live odom chain.
            required: Whether this frame should always be present (live world)
                vs optional (prebuilt map); only changes the warning wording.
        """
        if self._intrinsics is None:
            return
        # Throttle retries while this frame's TF is absent: tf.get logs on every
        # miss, so probing each tracked frame would flood logs at loop rate.
        now = time.monotonic()
        if now < self._next_pose_probe_s.get(frame_id, 0.0):
            return
        transform = self.tf.get(
            frame_id,
            self.config.camera_optical_frame_id,
            time_point=image.ts,
            time_tolerance=tf_tolerance,
        )
        if transform is None:
            self._next_pose_probe_s[frame_id] = (
                now + self.config.pose_probe_interval_s)
            self._warn_missing_tf_once(frame_id, required)
            return
        self._next_pose_probe_s[frame_id] = 0.0  # present -> probe every frame
        point = pixel_to_ground_point(
            contact_px,
            self._intrinsics,
            transform.to_matrix(),
            ground_z=floor_z,
        )
        if point is None:
            return
        stream.publish(PoseStamped(
            ts=image.ts,
            frame_id=frame_id,
            position=list(point),
            orientation=[0.0, 0.0, 0.0, 1.0],
        ))

    def _warn_missing_tf_once(self, frame_id: str, required: bool) -> None:
        """Log a single warning the first time a pose frame's TF is missing."""
        if required and not self._warned_no_world_tf:
            logger.warning(
                "No %r<-%r transform yet; subject_world_pose paused.",
                frame_id,
                self.config.camera_optical_frame_id,
            )
            self._warned_no_world_tf = True
        elif not required and not self._warned_no_map_tf:
            logger.warning(
                "No %r<-%r transform yet; optional subject_map_pose paused "
                "until relocalization publishes the map frame.",
                frame_id,
                self.config.camera_optical_frame_id,
            )
            self._warned_no_map_tf = True

    def _set_snapshot(self, snapshot: TrackSnapshot) -> None:
        with self._lock:
            previous = self._snapshot
            self._snapshot = snapshot
        # Publish live tracking every frame; otherwise only on a real change,
        # so a long "lost" stretch does not spam identical status messages.
        if (
            snapshot.status == "tracking"
            or snapshot.status != previous.status
            or snapshot.message != previous.message
        ):
            self.subject_status.publish(snapshot.to_json())

    def _stop_monitoring(self) -> None:
        self._monitor_stop.set()
        with self._lock:
            thread = self._thread
            self._thread = None
        # Join outside the lock; the loop thread takes the lock each iteration.
        if thread is not None:
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


def _draw_overlay(
    image: Image,
    bbox: tuple[float, float, float, float] | None,
    label: str,
    color: tuple[int, int, int],
) -> Image:
    """Draw the tracked bbox and a label on a copy of the frame (BGR)."""
    frame = image.to_opencv().copy()
    if bbox is not None:
        x1, y1, x2, y2 = (int(round(v)) for v in bbox)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        text_y = y1 - 8 if y1 > 16 else y1 + 16
        cv2.putText(
            frame,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )
    else:
        cv2.putText(
            frame,
            label,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
        )
    return Image.from_opencv(frame, ts=image.ts)


def _decode_base64_image(b64: str) -> Image:
    bgr_array = TurboJPEG().decode(base64.b64decode(b64))
    return Image(data=bgr_array, format=ImageFormat.BGR)
