"""Pure subject-tracking logic: visual metrics + monitor state machine.

Two cohesive, dependency-free concerns for following a tracked subject frame to
frame:

- Visual metrics: summarize a tracked bbox/mask into stable image-space metrics
  (size and centering) for the status stream (:class:`VisualObservation`,
  :class:`TrackMetrics`, :func:`visual_metrics`), plus the ground-contact pixel
  for back-projection (:func:`ground_contact_pixel`).
- Monitor state machine: fold each frame's metrics (or a miss) into a status
  snapshot, applying a motion-aware drift gate and deciding when to reacquire
  (:class:`TrackSnapshot`, :class:`MonitorParams`, :class:`MonitorState`).

No DimOS imports -- unit tested. The DimOS ``PawTrackSkillContainer``
performs the effects (run the tracker/VLM, publish snapshots) and feeds this
machine each frame's result via :meth:`MonitorState.observe`; the machine
decides the status and what to do next.
"""

from __future__ import annotations

import dataclasses
import json
import math
import time
from typing import Literal


@dataclasses.dataclass(frozen=True)
class VisualObservation:
    """One tracked-subject observation in image coordinates.

    Attributes:
        bbox: Detection box ``(x1, y1, x2, y2)`` in pixels.
        image_width: Source image width in pixels.
        image_height: Source image height in pixels.
        confidence: Tracker/detector confidence.
        mask_area_px: Foreground mask area in pixels, when available.
    """

    bbox: tuple[float, float, float, float]
    image_width: int
    image_height: int
    confidence: float = 1.0
    mask_area_px: float | None = None


@dataclasses.dataclass(frozen=True)
class TrackMetrics:
    """Size and centering metrics for a tracked subject."""

    bbox: tuple[float, float, float, float]
    center_px: tuple[float, float]
    width_px: float
    height_px: float
    area_px: float
    area_ratio: float
    image_error_x: float
    image_error_y: float
    confidence: float
    mask_area_px: float | None
    mask_area_ratio: float | None


def visual_metrics(observation: VisualObservation) -> TrackMetrics:
    """Compute image-space monitoring metrics for one tracked subject.

    Args:
        observation: Tracked subject bbox/mask metadata for one frame.

    Returns:
        Normalized centering and size metrics suitable for status streams.

    Raises:
        ValueError: If image dimensions or bbox dimensions are invalid.
    """
    if observation.image_width <= 0 or observation.image_height <= 0:
        raise ValueError("image dimensions must be positive")

    x1, y1, x2, y2 = observation.bbox
    width = x2 - x1
    height = y2 - y1
    if width <= 0 or height <= 0:
        raise ValueError("bbox dimensions must be positive")

    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    image_area = float(observation.image_width * observation.image_height)
    area = width * height
    mask_ratio = (
        observation.mask_area_px / image_area
        if observation.mask_area_px is not None
        else None
    )
    return TrackMetrics(
        bbox=observation.bbox,
        center_px=(center_x, center_y),
        width_px=width,
        height_px=height,
        area_px=area,
        area_ratio=area / image_area,
        image_error_x=(center_x - observation.image_width / 2.0)
        / (observation.image_width / 2.0),
        image_error_y=(center_y - observation.image_height / 2.0)
        / (observation.image_height / 2.0),
        confidence=observation.confidence,
        mask_area_px=observation.mask_area_px,
        mask_area_ratio=mask_ratio,
    )


Bbox = tuple[float, float, float, float]


def ground_contact_pixel(bbox: Bbox) -> tuple[float, float]:
    """Pixel where the tracked subject meets the floor: bbox bottom-center.

    A standing or seated subject (or the chair it sits on) touches the floor at
    the bottom edge of its bounding box, so the bottom-center pixel back-projects
    to the subject's position on the ground. The bbox *center* would instead sit
    at torso/seat height and project to a biased ground point.

    Args:
        bbox: Detection box ``(x1, y1, x2, y2)`` in pixels.

    Returns:
        ``(u, v)`` pixel at the horizontal center of the bbox's bottom edge.
    """
    x1, _y1, x2, y2 = bbox
    return ((x1 + x2) / 2.0, y2)


def is_bbox_shape(values: list[float] | None) -> bool:
    """Whether ``values`` is a length-4 sequence of finite numbers.

    A structural guard for a caller-supplied box (e.g. an LLM tool argument)
    before it is indexed as ``[x1, y1, x2, y2]``, so a malformed value returns
    a clean error instead of raising.
    """
    if values is None:
        return False
    try:
        length_ok = len(values) == 4
    except TypeError:
        return False
    if not length_ok:
        return False
    return all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in values
    )


def bbox_in_image(bbox: Bbox, width: int, height: int) -> bool:
    """Whether a bbox is non-degenerate and overlaps the image."""
    x1, y1, x2, y2 = bbox
    if x2 <= x1 or y2 <= y1:
        return False
    return x2 > 0 and y2 > 0 and x1 < width and y1 < height


def valid_bbox(bbox: list[float] | None, width: int, height: int) -> bool:
    """Whether a bbox is finite, well-formed, and overlaps the image."""
    if not is_bbox_shape(bbox):
        return False
    return bbox_in_image((bbox[0], bbox[1], bbox[2], bbox[3]), width, height)


def clamp_bbox(bbox: Bbox, width: int, height: int) -> Bbox:
    """Clamp a (valid, overlapping) bbox to image bounds.

    EdgeTAM is initialized from this box, so a box that spills past the frame
    edge -- common for a VLM box on a subject against an edge -- would otherwise
    distort initialization. Clamping keeps the visible portion rather than
    rejecting the subject. A box that passed :func:`valid_bbox` stays
    non-degenerate after clamping.
    """
    x1, y1, x2, y2 = bbox
    fw, fh = float(width), float(height)
    return (
        min(max(x1, 0.0), fw),
        min(max(y1, 0.0), fh),
        min(max(x2, 0.0), fw),
        min(max(y2, 0.0), fh),
    )


Status = Literal[
    "idle", "acquiring", "tracking", "lost", "stale", "stopped", "error"
]
# What the control loop should do after a frame.
Action = Literal["track", "coast", "reacquire", "give_up"]


@dataclasses.dataclass(frozen=True)
class TrackSnapshot:
    """Latest monitor state, suitable for JSON status output."""

    status: Status
    description: str | None
    message: str
    seen_at: float | None = None
    bbox: tuple[float, float, float, float] | None = None
    center_px: tuple[float, float] | None = None
    width_px: float | None = None
    height_px: float | None = None
    area_px: float | None = None
    area_ratio: float | None = None
    image_error_x: float | None = None
    image_error_y: float | None = None
    confidence: float | None = None
    mask_area_px: float | None = None
    mask_area_ratio: float | None = None

    def to_json(self, now: float | None = None) -> str:
        """Serialize with a derived age field; drop the raw monotonic stamp.

        Args:
            now: Current monotonic timestamp. Defaults to ``time.monotonic()``.

        Returns:
            Compact JSON for the ``subject_status`` stream.
        """
        now = time.monotonic() if now is None else now
        data = dataclasses.asdict(self)
        data["last_seen_age_s"] = (
            None if self.seen_at is None else max(0.0, now - self.seen_at)
        )
        # seen_at is a raw monotonic clock value, meaningless to consumers.
        data.pop("seen_at", None)
        return json.dumps(data, separators=(",", ":"))


@dataclasses.dataclass(frozen=True)
class MonitorParams:
    """Thresholds for the monitor state machine."""

    max_lost_frames: int = 15  # coast this long before reacquiring
    reacquire_interval_frames: int = 15  # reacquire cadence while lost
    max_reacquire_attempts: int = 5  # give up after this many failed retries
    max_center_jump_frac: float = 2.5  # reject jumps beyond this x subject width
    max_area_factor: float = 4.0  # reject area changes beyond this factor
    stale_timeout_s: float = 2.0  # no fix longer than this -> "stale"


def _from_metrics(
    status: Status,
    description: str,
    message: str,
    metrics: TrackMetrics,
    seen_at: float,
) -> TrackSnapshot:
    return TrackSnapshot(
        status=status,
        description=description,
        message=message,
        seen_at=seen_at,
        bbox=metrics.bbox,
        center_px=metrics.center_px,
        width_px=metrics.width_px,
        height_px=metrics.height_px,
        area_px=metrics.area_px,
        area_ratio=metrics.area_ratio,
        image_error_x=metrics.image_error_x,
        image_error_y=metrics.image_error_y,
        confidence=metrics.confidence,
        mask_area_px=metrics.mask_area_px,
        mask_area_ratio=metrics.mask_area_ratio,
    )


class MonitorState:
    """Tracks monitor status across frames and decides when to reacquire.

    Feed each frame to :meth:`observe` with its tracked metrics (or None). A
    detection that jumps too far or changes area too abruptly versus the last
    good frame is rejected as drift (treated as a miss). A ``lost``/``stale``/
    ``error`` snapshot carries the *last good* fix forward (bbox + metrics +
    timestamp), only changing the status and message.
    """

    def __init__(self, params: MonitorParams | None = None):
        self._params = params if params is not None else MonitorParams()
        self._lost = 0
        self._reacquire_attempts = 0
        self._skip_gate = False
        self._last_tracked: TrackSnapshot | None = None
        self._snapshot = TrackSnapshot(
            status="idle",
            description=None,
            message="Nothing is being tracked.",
        )

    @property
    def snapshot(self) -> TrackSnapshot:
        """The current authoritative snapshot."""
        return self._snapshot

    def begin(self, description: str) -> TrackSnapshot:
        """Enter ``acquiring`` for a new target; reset counters/history."""
        self._lost = 0
        self._reacquire_attempts = 0
        self._skip_gate = False
        self._last_tracked = None
        self._snapshot = TrackSnapshot(
            status="acquiring",
            description=description,
            message=f"Looking for {description}.",
        )
        return self._snapshot

    def observe(
        self,
        description: str,
        metrics: TrackMetrics | None,
        now: float,
    ) -> tuple[TrackSnapshot, Action]:
        """Fold one frame's result in and decide the next action.

        Args:
            description: The tracked subject's description.
            metrics: This frame's tracked metrics, or None if the tracker had
                no usable detection.
            now: Current monotonic timestamp.

        Returns:
            ``(snapshot, action)``; action is one of ``track``/``coast``/
            ``reacquire``/``give_up``.
        """
        if metrics is not None and self._is_plausible(metrics):
            return self._accept(description, metrics, now), "track"
        return self._miss(description, now)

    def reacquired(self) -> None:
        """A reacquire (VLM or motion fallback) succeeded; reset + re-baseline.

        Clears the lost/reacquire counters and skips the drift gate on the next
        frame, since the new fix may legitimately be far from the old one.
        """
        self._lost = 0
        self._reacquire_attempts = 0
        self._skip_gate = True

    def errored(self, description: str, message: str) -> TrackSnapshot:
        """Surface a loud error status, keeping the last known fix."""
        self._snapshot = self._carry_forward("error", description, message)
        return self._snapshot

    def stopped(self) -> TrackSnapshot:
        """Reset to a clean stopped state."""
        self._lost = 0
        self._reacquire_attempts = 0
        self._skip_gate = False
        self._last_tracked = None
        self._snapshot = TrackSnapshot(
            status="stopped",
            description=None,
            message="Stopped tracking.",
        )
        return self._snapshot

    def _is_plausible(self, metrics: TrackMetrics) -> bool:
        last = self._last_tracked
        if self._skip_gate or last is None or last.center_px is None:
            return True
        ref = max(metrics.width_px, last.width_px or 0.0, 1.0)
        jump = math.dist(last.center_px, metrics.center_px)
        if jump > self._params.max_center_jump_frac * ref:
            return False
        if last.area_px and metrics.area_px:
            factor = max(
                metrics.area_px / last.area_px,
                last.area_px / metrics.area_px,
            )
            if factor > self._params.max_area_factor:
                return False
        return True

    def _accept(
        self, description: str, metrics: TrackMetrics, now: float
    ) -> TrackSnapshot:
        self._lost = 0
        self._reacquire_attempts = 0
        self._skip_gate = False
        snap = _from_metrics(
            "tracking", description, f"Tracking {description}.", metrics, now
        )
        self._last_tracked = snap
        self._snapshot = snap
        return snap

    def _miss(
        self, description: str, now: float
    ) -> tuple[TrackSnapshot, Action]:
        self._lost += 1
        action: Action = "coast"
        message = "Lost sight of the target."
        due = (
            self._lost >= self._params.max_lost_frames
            and (self._lost - self._params.max_lost_frames)
            % self._params.reacquire_interval_frames == 0
        )
        if due:
            self._reacquire_attempts += 1
            if self._reacquire_attempts > self._params.max_reacquire_attempts:
                action = "give_up"
                message = "Gave up reacquiring the target."
            else:
                action = "reacquire"
        status: Status = "lost"
        last = self._last_tracked
        if (
            last is not None
            and last.seen_at is not None
            and now - last.seen_at > self._params.stale_timeout_s
        ):
            status = "stale"
            if action == "coast":
                message = "No fix for a while; the target is stale."
        snap = self._carry_forward(status, description, message)
        self._snapshot = snap
        return snap, action

    def _carry_forward(
        self, status: Status, description: str, message: str
    ) -> TrackSnapshot:
        if self._last_tracked is None:
            return TrackSnapshot(status, description, message)
        return dataclasses.replace(
            self._last_tracked,
            status=status,
            description=description,
            message=message,
        )
