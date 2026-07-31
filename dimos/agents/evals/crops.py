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

"""The review images ``teacher.py --crops`` writes, and how their frames are chosen.

Review is the step that decides whether a detector label is a thing that can
honestly be asked about, and whether the position behind it is really on that
thing. It is done by a human looking at images, so which frames those images are
taken from decides what review can catch -- and the first version of this got it
wrong in a way worth keeping written down: it drew everything on the *brightest
frame a detection fired in*, which is a frame chosen for exposure while the robot
was moving. Reviewers passed a reference twice whose marker sat on a blank side
wall (see ``reference/go2_bigoffice/review.json``, ``elevator door``), because no
single blurred crop could show it.

So each reference gets two images, answering the two different questions review
asks (:func:`write_crops`):

* ``<question_id>.jpg`` -- *is this the thing the label says it is?* The sharpest
  nearby frame in which the reference still has LiDAR support at it.
* ``<question_id>_measurement.jpg`` -- *did the geometry land on it?* The original
  detection frame, carrying the whole chain that produced the position.

This module is deliberately separate from the pipeline that produces the table:
nothing here can move a reference position. ``refs.jsonl`` and ``manifest.json``
are byte-identical whether ``--crops`` ran or not, and a change to the picker is a
change to *presentation only* -- which is also why the picker can be measured and
replaced without regenerating the table.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
import math
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray

from dimos.agents.evals.teacher import (
    Camera,
    GateParams,
    Measurement,
    Projector,
    Reference,
    load_base_to_optical,
    load_camera,
)

if TYPE_CHECKING:
    from dimos.memory2.stream import Stream


#: pyproject.toml [tool.largefiles] max_size_kb -- a committed crop has to fit.
MAX_CROP_BYTES = 75 * 1024

#: Longest edge, then JPEG quality, tried in order until a crop fits
#: :data:`MAX_CROP_BYTES`. The first entry is what every crop the recording has
#: produced so far encodes to; the rest exist so an unusually detailed frame
#: degrades instead of failing the commit.
_CROP_ENCODINGS: tuple[tuple[int, int], ...] = ((512, 85), (448, 80), (384, 75), (320, 70))

# BGR overlay colors, in drawing order.
_MASK_COLOR = (255, 200, 0)  # cyan-ish: the detector's pixel-level claim
_BBOX_COLOR = (0, 255, 0)  # green: the detector's box
_INLIER_COLOR = (0, 220, 255)  # amber: the LiDAR points that voted
_REFERENCE_COLOR = (255, 0, 255)  # magenta: where the vote landed


def _encode_crop(crop: NDArray[np.uint8]) -> bytes:
    """JPEG-encode *crop*, shrinking it until it fits :data:`MAX_CROP_BYTES`."""
    # Function-local, here and below: cv2 loads a large native extension into
    # every process that transitively imports this module, and nothing outside
    # `--crops` needs it (dimos/codebase_checks/test_inline_heavy_imports.py).
    import cv2

    height, width = crop.shape[:2]
    encoded = b""
    for longest_edge, quality in _CROP_ENCODINGS:
        scale = min(1.0, longest_edge / max(height, width))
        resized = (
            crop
            if scale == 1.0
            else cv2.resize(
                crop,
                (max(1, int(width * scale)), max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        )
        ok, buffer = cv2.imencode(".jpg", resized, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed on a review crop")
        encoded = buffer.tobytes()
        if len(encoded) <= MAX_CROP_BYTES:
            return encoded
    raise ValueError(
        f"a review crop is {len(encoded)} bytes at the smallest encoding "
        f"{_CROP_ENCODINGS[-1]}, over the {MAX_CROP_BYTES}-byte large-file limit"
    )


def _reference_marker(
    projector: Projector, reference: Reference, pose_tuple: tuple[float, ...]
) -> tuple[tuple[int, int], float] | None:
    """Project a reference into one frame: its pixel *and* its depth, or ``None``.

    ``None`` covers both ways a frame cannot show the reference: it is behind the
    camera (:attr:`GateParams.front_z_m`) or off the sensor. The depth comes back
    with the pixel because every gate below compares against it, and re-deriving
    it from the pose would be a second chance to disagree.
    """
    uv, depth, _, _ = projector.project(
        np.array([[reference.x, reference.y, reference.z]], dtype=np.float64), pose_tuple
    )
    if len(uv) == 0:
        return None
    return (round(float(uv[0][0])), round(float(uv[0][1]))), float(depth[0])


#: Half-width of the candidate window for the identification frame: an
#: observation this close to a contributing detection still has LiDAR returns at
#: the reference, so its marker can be *checked* instead of taken on trust.
#: Measured on this recording, past about a second the robot has moved far
#: enough that the sweep no longer covers the reference at all and the marker
#: becomes a pure extrapolation.
CROP_WINDOW_S = 1.0

#: Brightness floor for an identification frame, far above the pipeline's own
#: :attr:`GateParams.min_gray`. Laplacian variance is gameable in the dark: on
#: this replay the highest-variance frames of a near-black stretch are specular
#: streaks rather than views of anything, so a frame is only ranked on sharpness
#: once it is bright enough for the ranking to mean something.
CROP_MIN_GRAY = 60.0
#: The pipeline floor, used only when nothing reaches :data:`CROP_MIN_GRAY`. The
#: fallback is reported in :class:`CropReport` rather than passed off as a
#: normal pick.
CROP_MIN_GRAY_FALLBACK = 30.0

#: LiDAR support at the marker: how many of the sweep's points must land within
#: :data:`CROP_SUPPORT_RADIUS_M` of the reference **in 3-D** *and* inside the
#: window the sharpness is measured in, so the geometry a reviewer is asked to
#: trust is inside the image they are given. What this rules out is an
#: *extrapolated* marker: on this recording the sharpest frames of a candidate
#: window are often taken from 2-4 m back, where the reference projects above the
#: Go2 LiDAR's vertical field of view and the sweep says nothing at all about it.
#:
#: Two things it deliberately does not do, both of which a reader will otherwise
#: assume from the word "supported":
#:
#: * It constrains the marker **only on the near side**. Together with the
#:   occlusion guard below, what a passing frame asserts is that nothing stands
#:   *in front of* the marker -- a surface some distance *behind* it passes
#:   unremarked, so a marker can float in front of the thing it names. That is
#:   the ``go2_short`` meeting table: the sweep reads +0.71 m at the marker
#:   pixel, through the see-through gap under the tabletop, while the support
#:   points within 0.4 m are the tabletop itself
#:   (``reference/go2_short/README.md``).
#: * It does **not** certify the object -- points near a reference sitting on a
#:   blank wall are the wall's, and the dropped ``elevator door`` reference
#:   passed this gate on every candidate frame. Only cross-frame context caught
#:   that one. Identity is a question for the human, and always was.
CROP_SUPPORT_RADIUS_M = 0.40
CROP_MIN_SUPPORT_POINTS = 20

#: Occlusion guard. The depth the sweep reports *at the marker pixel* -- the
#: median over the points within :data:`CROP_MARKER_RADIUS_PX` of it, and only
#: when at least :data:`CROP_MIN_MARKER_POINTS` are there to take a median of --
#: may not be nearer than the reference's own depth by more than
#: :data:`CROP_OCCLUSION_SLACK_M`. If it is, something stands between the camera
#: and the object, and the crop would show that something. The slack absorbs the
#: thickness of a sweep on a surface.
#:
#: **One-sided by construction**: only a *nearer* surface is a rejection. A
#: median depth behind the marker means the marker is in front of whatever the
#: sweep hit there, which is what a see-through gap looks like and is a
#: cosmetic complaint about the drawing, not evidence that the crop shows the
#: wrong thing -- so it passes. See the meeting-table note above.
#:
#: A frame with fewer than :data:`CROP_MIN_MARKER_POINTS` at the marker records
#: ``occluded=None``, i.e. *unknown*, and :attr:`CropCandidate.confirmed` treats
#: unknown as not-occluded (``is not True``). Deliberate: at this radius most
#: sweeps have only a handful of returns at any one pixel, so failing on
#: unknown would reject frames for the LiDAR's angular resolution rather than
#: for anything in the scene. The support count above is what still has to
#: pass, and it is counted in 3-D over the whole window rather than at the
#: single pixel.
CROP_MARKER_RADIUS_PX = 14
CROP_MIN_MARKER_POINTS = 3
CROP_OCCLUSION_SLACK_M = 0.30

#: Window side used when the object's apparent extent cannot be scaled into a
#: candidate frame (a detection or a marker without a usable depth). It is the
#: value the crop experiment measured with, and it is a *fallback*: the scaled
#: extent below is what normally sizes the window.
CROP_DEFAULT_WINDOW_PX = 300
#: Clamps on the scaled extent: large enough that a distant object still comes
#: with the context a human recognises it from, small enough that a close-up
#: does not end up ranking the sharpness of the whole frame.
CROP_MIN_WINDOW_PX = 160
CROP_MAX_WINDOW_PX = 640

#: The shipped crop is the sharpness window scaled by this. The window is sized
#: *to the object*, which is what the ranking and the support test want and what
#: a reviewer wants least: naming a thing needs its surroundings. Support is
#: still required inside the tight window, so the margin only ever adds context,
#: never evidence.
CROP_CONTEXT_SCALE = 2.0

#: Yaw rate is differenced against the nearest earlier observation at least
#: ``CROP_POSE_MIN_DT_S`` and at most ``CROP_POSE_MAX_DT_S`` away. Colour frames
#: arrive ~0.07 s apart while the recorded pose updates more slowly, so a
#: strictly adjacent difference reads 0 rad/s on a stale-pose pair -- which would
#: rank pose staleness rather than a steady camera -- and the upper bound stops a
#: difference from spanning the gap between two candidate windows.
CROP_POSE_MIN_DT_S = 0.10
CROP_POSE_MAX_DT_S = 0.60


def _yaw_of(pose_tuple: tuple[float, ...]) -> float:
    """Map-frame yaw of a ``(x, y, z, qx, qy, qz, qw)`` pose."""
    _, _, _, qx, qy, qz, qw = pose_tuple
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _wrap_pi(angle: float) -> float:
    """Fold *angle* into ``[-pi, pi)`` so a difference across the branch cut is small."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_rates(samples: Sequence[tuple[float, tuple[float, ...]]]) -> list[float | None]:
    """Absolute yaw rate (rad/s) at each ``(ts, pose)`` of *samples*, in ts order.

    Derived from consecutive recorded pose quaternions -- there is no angular
    velocity in the replay -- and ``None`` for a sample with no usable earlier
    neighbour (see :data:`CROP_POSE_MIN_DT_S`). Only a tiebreak: sharpness is
    measured directly rather than predicted from the kinematics.
    """
    rates: list[float | None] = []
    for i, (ts, pose) in enumerate(samples):
        rate: float | None = None
        for j in range(i - 1, -1, -1):
            dt = ts - samples[j][0]
            if dt < CROP_POSE_MIN_DT_S:
                continue
            if dt > CROP_POSE_MAX_DT_S:
                break
            rate = abs(_wrap_pi(_yaw_of(pose) - _yaw_of(samples[j][1]))) / dt
            break
        rates.append(rate)
    return rates


def merge_windows(stamps: Sequence[float], half_s: float) -> list[tuple[float, float]]:
    """Merge the ``+/- half_s`` windows around *stamps* into disjoint, sorted spans.

    Two contributing detections less than ``2 * half_s`` apart share candidate
    frames, and querying their windows separately would evaluate -- and could
    rank -- the same frame twice.
    """
    spans: list[list[float]] = []
    for ts in sorted(stamps):
        lo, hi = ts - half_s, ts + half_s
        if spans and lo <= spans[-1][1]:
            spans[-1][1] = max(spans[-1][1], hi)
        else:
            spans.append([lo, hi])
    return [(lo, hi) for lo, hi in spans]


def sharpness_window_px(
    bbox: tuple[float, float, float, float], detection_depth_m: float, marker_depth_m: float
) -> int:
    """Side of the square the sharpness is measured in, and the crop taken from.

    The object's *apparent extent* in the candidate frame: its pixel extent in
    the frame it was detected in, scaled by the depth ratio, because an object
    twice as far away covers half the pixels. Sizing the window this way is what
    keeps the ranking on the object rather than on whatever else a fixed box
    happened to contain.

    Falls back to :data:`CROP_DEFAULT_WINDOW_PX` when either depth is missing or
    non-positive, and is clamped to ``[CROP_MIN_WINDOW_PX, CROP_MAX_WINDOW_PX]``.
    """
    x1, y1, x2, y2 = bbox
    extent = max(x2 - x1, y2 - y1)
    if extent <= 0.0 or detection_depth_m <= 0.0 or marker_depth_m <= 0.0:
        return CROP_DEFAULT_WINDOW_PX
    scaled = extent * detection_depth_m / marker_depth_m
    return round(min(max(scaled, CROP_MIN_WINDOW_PX), CROP_MAX_WINDOW_PX))


def window_region(
    center: tuple[int, int], window_px: int, width: int, height: int
) -> tuple[int, int, int, int]:
    """A ``window_px``-square region around *center*, shifted to fit the image.

    Shifted rather than clipped: a marker near the rim would otherwise be ranked
    on a sliver of pixels, and the crop a reviewer sees would be a sliver too.
    """
    side = min(window_px, width, height)
    x1 = max(0, min(center[0] - side // 2, width - side))
    y1 = max(0, min(center[1] - side // 2, height - side))
    return x1, y1, x1 + side, y1 + side


def laplacian_sharpness(gray: NDArray[np.uint8], region: tuple[int, int, int, int]) -> float:
    """Variance of the Laplacian inside *region* -- higher is sharper.

    Measured locally, around the projected reference, because a frame can be
    sharp on the far wall and smeared on the object the crop is about.
    """
    import cv2

    x1, y1, x2, y2 = region
    patch = gray[y1:y2, x1:x2]
    if patch.size == 0:
        return 0.0
    return float(cv2.Laplacian(patch, cv2.CV_64F).var())


@dataclass(frozen=True)
class MarkerSupport:
    """What one frame's LiDAR sweep says about the reference marker drawn on it."""

    #: Points within :data:`CROP_SUPPORT_RADIUS_M` of the reference in 3-D whose
    #: pixels land inside the crop region.
    n_support: int
    #: Points whose pixels land within :data:`CROP_MARKER_RADIUS_PX` of the marker.
    n_at_marker: int
    #: Median depth of those, i.e. the depth of whatever surface the marker sits
    #: on; ``None`` when too few points are there to take a median of.
    seen_depth_m: float | None
    #: ``True``/``False`` from :data:`CROP_OCCLUSION_SLACK_M`, ``None`` when
    #: ``seen_depth_m`` is -- i.e. *unknown*, which
    #: :attr:`CropCandidate.confirmed` counts as not-occluded by design. Only a
    #: surface nearer than the reference is ever ``True``; one behind it is not
    #: occlusion.
    occluded: bool | None
    #: Pixels of the supporting points, drawn on the crop as the evidence.
    support_uv: tuple[tuple[float, float], ...]


def marker_support(
    uv: NDArray[np.float64],
    depth: NDArray[np.float64],
    world: NDArray[np.float64],
    reference_xyz: tuple[float, float, float],
    marker: tuple[int, int],
    region: tuple[int, int, int, int],
    marker_depth_m: float,
) -> MarkerSupport:
    """Count the sweep's vote for *marker*, and check nothing stands in front of it.

    *uv*, *depth* and *world* are one :meth:`Projector.project` return, i.e.
    index-aligned pixels, optical depths and map-frame points.
    """
    x1, y1, x2, y2 = region
    near = (
        np.linalg.norm(world - np.asarray(reference_xyz, dtype=np.float64), axis=1)
        <= CROP_SUPPORT_RADIUS_M
    )
    inside = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
    support = near & inside

    at_marker = (
        np.linalg.norm(uv - np.asarray(marker, dtype=np.float64), axis=1) <= CROP_MARKER_RADIUS_PX
    )
    n_at_marker = int(at_marker.sum())
    seen: float | None = None
    if n_at_marker >= CROP_MIN_MARKER_POINTS:
        seen = float(np.median(depth[at_marker]))
    return MarkerSupport(
        n_support=int(support.sum()),
        n_at_marker=n_at_marker,
        seen_depth_m=seen,
        occluded=None if seen is None else bool(seen < marker_depth_m - CROP_OCCLUSION_SLACK_M),
        support_uv=tuple((float(u), float(v)) for u, v in uv[support]),
    )


@dataclass(frozen=True)
class CropCandidate:
    """One frame the identification crop could be taken from, fully measured."""

    ts: float
    marker: tuple[int, int]
    marker_depth_m: float
    mean_gray: float
    sharpness: float
    window_px: int
    region: tuple[int, int, int, int]
    support: MarkerSupport
    #: Distance in time to the nearest detection this reference was built from;
    #: ``0.0`` on a contributing frame itself.
    dt_to_detection_s: float
    yaw_rate: float | None = None

    @property
    def confirmed(self) -> bool:
        """Does this frame's sweep vouch for the marker, and is nothing in front of it?

        ``is not True`` rather than ``is False``: unknown occlusion passes, for
        the reason given at :data:`CROP_MIN_MARKER_POINTS`. "Confirmed" is a
        statement about the *geometry being measured rather than extrapolated*,
        and only on the near side -- it says nothing about what the object is.
        """
        return (
            self.support.n_support >= CROP_MIN_SUPPORT_POINTS and self.support.occluded is not True
        )


def pick_identification_frame(
    candidates: Sequence[CropCandidate],
) -> tuple[CropCandidate | None, str]:
    """The sharpest LiDAR-confirmed candidate, plus a note when that took a fallback.

    Order of business, and why it is this order: confirmation first (an
    unconfirmed frame's marker is a guess however pretty the frame), brightness
    second (Laplacian variance ranks noise above content in the dark), sharpness
    last -- as a measurement, not a prediction from the kinematics.

    Ties are broken by the lower yaw rate and then the earlier timestamp, so two
    frames of equal measured sharpness cannot be separated by iteration order.
    """
    confirmed = [candidate for candidate in candidates if candidate.confirmed]
    if not confirmed:
        return None, (
            f"no candidate frame has LiDAR support at the reference "
            f"({len(candidates)} candidates examined): nothing in the recording shows "
            "the marker with geometry behind it, so no identification crop is written"
        )

    def rank(candidate: CropCandidate) -> tuple[float, float, float]:
        return (
            -candidate.sharpness,
            math.inf if candidate.yaw_rate is None else candidate.yaw_rate,
            candidate.ts,
        )

    for floor in (CROP_MIN_GRAY, CROP_MIN_GRAY_FALLBACK):
        bright = [candidate for candidate in confirmed if candidate.mean_gray >= floor]
        if not bright:
            continue
        best = min(bright, key=rank)
        note = ""
        if floor != CROP_MIN_GRAY:
            note = (
                f"no LiDAR-confirmed candidate reaches mean gray {CROP_MIN_GRAY}; "
                f"picked at the {CROP_MIN_GRAY_FALLBACK} pipeline floor "
                f"(mean gray {best.mean_gray:.1f}), so read the crop as dim"
            )
        return best, note
    return None, (
        f"{len(confirmed)} LiDAR-confirmed candidate(s), all below the "
        f"{CROP_MIN_GRAY_FALLBACK} pipeline brightness floor: no identification crop"
    )


@dataclass
class CropReport:
    """What ``--crops`` wrote, and everything it had to compromise on."""

    written: int = 0
    notes: list[str] = field(default_factory=list)


def _nearest_observation(images: Stream[Any], ts: float, tolerance: float) -> Any | None:
    """The observation nearest *ts* within *tolerance*, or ``None``.

    ``at(ts, tolerance)`` is a pure predicate over a stream that iterates in time
    order, so ``.first()`` on a window returns its *earliest* member -- a
    different frame, which for a crop means drawing one frame's boxes onto
    another. The nearest member is selected explicitly instead, exactly as
    ``scan`` does when pairing LiDAR.
    """
    window = list(images.at(ts, tolerance=tolerance))
    if not window:
        return None
    return min(window, key=lambda o: abs(o.ts - ts))


def _measurement_crop(
    images: Stream[Any],
    projector: Projector,
    reference: Reference,
    detection: Measurement,
) -> NDArray[np.uint8] | None:
    """The detection frame with the whole chain that produced the position on it.

    The mask outline (when the mask chose the points), the detector's box, the
    projected LiDAR inliers whose median *became* the position, and that position
    projected back in as a separate marker. This is the image that answers "did
    the geometry land on the object", and it can only be the frame the detection
    was made on -- the box and the inlier pixels mean nothing on another frame.
    """
    import cv2

    obs = _nearest_observation(images, detection.ts, tolerance=0.2)
    if obs is None:
        return None
    frame: NDArray[np.uint8] = cv2.cvtColor(obs.data.data, cv2.COLOR_RGB2BGR)

    for contour in detection.mask_outline:
        cv2.polylines(
            frame, [np.array(contour, dtype=np.int32).reshape(-1, 1, 2)], True, _MASK_COLOR, 2
        )
    x1, y1, x2, y2 = detection.bbox
    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), _BBOX_COLOR, 2)
    for u, v in detection.inlier_uv:
        cv2.circle(frame, (round(u), round(v)), 2, _INLIER_COLOR, -1)

    margin_x, margin_y = 0.12 * (x2 - x1), 0.12 * (y2 - y1)
    left, top = x1 - margin_x, y1 - margin_y
    right, bottom = x2 + margin_x, y2 + margin_y

    pose_tuple = obs.pose_tuple
    projected = (
        _reference_marker(projector, reference, pose_tuple) if pose_tuple is not None else None
    )
    if projected is not None:
        marker = projected[0]
        _draw_reference_marker(frame, marker)
        # The reference is a viewing-distance away from the box it was measured
        # in, so it often projects outside it. Widen the crop rather than
        # cropping out the one marker a reviewer has to check.
        left, top = min(left, marker[0] - 16.0), min(top, marker[1] - 16.0)
        right, bottom = max(right, marker[0] + 16.0), max(bottom, marker[1] + 16.0)

    cx1, cy1 = max(int(left), 0), max(int(top), 0)
    cx2, cy2 = min(int(right), frame.shape[1]), min(int(bottom), frame.shape[0])
    crop: NDArray[np.uint8] = frame[cy1:cy2, cx1:cx2]
    return crop if crop.size else None


def _draw_reference_marker(frame: NDArray[np.uint8], marker: tuple[int, int]) -> None:
    """Draw the "the vote landed here" marker, identically on both crops."""
    import cv2

    cv2.drawMarker(frame, marker, _REFERENCE_COLOR, cv2.MARKER_CROSS, 44, 3)
    cv2.circle(frame, marker, 15, _REFERENCE_COLOR, 3)


def crop_candidates(
    images: Stream[Any],
    lidar: Stream[Any],
    projector: Projector,
    params: GateParams,
    reference: Reference,
    detections: Sequence[Measurement],
    *,
    reference_bbox: tuple[float, float, float, float] | None = None,
    reference_depth_m: float | None = None,
) -> list[CropCandidate]:
    """Measure every frame near *detections* that could carry the identification crop.

    One pass over the merged candidate windows. A frame is measured only if it
    can be measured -- it has a pose, the reference projects into it in front of
    the camera, and it is not below the pipeline's own brightness floor (nothing
    darker can ever be picked, so paying for its LiDAR projection would be waste)
    -- and the window it is measured in is sized from the apparent extent of the
    object in *that* frame.

    *reference_bbox* / *reference_depth_m* override where the apparent extent
    comes from, for a caller that has a reference row but not the measurements
    behind it; by default it is the contributing detection nearest in time, whose
    box and depth are the closest thing to a measurement of this object's size.
    """
    import cv2

    reference_xyz = (reference.x, reference.y, reference.z)
    stamps = sorted({detection.ts for detection in detections})
    by_ts = {detection.ts: detection for detection in detections}

    rows: list[CropCandidate] = []
    posed: list[tuple[float, tuple[float, ...]]] = []
    for lo, hi in merge_windows(stamps, CROP_WINDOW_S):
        for obs in images.time_range(lo, hi):
            pose_tuple = obs.pose_tuple
            if pose_tuple is None:
                continue
            gray: NDArray[np.uint8] = cv2.cvtColor(obs.data.data, cv2.COLOR_RGB2GRAY)
            mean_gray = float(gray.mean())
            if mean_gray < CROP_MIN_GRAY_FALLBACK:
                continue
            projected = _reference_marker(projector, reference, pose_tuple)
            if projected is None:
                continue
            marker, marker_depth = projected

            nearest_ts = min(stamps, key=lambda ts: abs(obs.ts - ts))
            anchor = by_ts[nearest_ts]
            bbox = reference_bbox if reference_bbox is not None else anchor.bbox
            depth = reference_depth_m if reference_depth_m is not None else anchor.depth_m
            window_px = sharpness_window_px(bbox, depth, marker_depth)
            region = window_region(
                marker, window_px, projector.camera.width, projector.camera.height
            )

            nearby = list(lidar.at(obs.ts, tolerance=params.lidar_tolerance_s))
            if not nearby:
                continue
            cloud = min(nearby, key=lambda o: abs(o.ts - obs.ts))
            uv, cloud_depth, world, _ = projector.project(
                cloud.data.as_numpy()[0].astype(np.float64), pose_tuple
            )
            support = marker_support(
                uv, cloud_depth, world, reference_xyz, marker, region, marker_depth
            )

            posed.append((float(obs.ts), tuple(float(v) for v in pose_tuple)))
            rows.append(
                CropCandidate(
                    ts=float(obs.ts),
                    marker=marker,
                    marker_depth_m=marker_depth,
                    mean_gray=mean_gray,
                    sharpness=laplacian_sharpness(gray, region),
                    window_px=window_px,
                    region=region,
                    support=support,
                    dt_to_detection_s=abs(float(obs.ts) - nearest_ts),
                )
            )

    return [replace(row, yaw_rate=rate) for row, rate in zip(rows, yaw_rates(posed), strict=True)]


def _identification_crop(
    images: Stream[Any], camera: Camera, candidate: CropCandidate
) -> NDArray[np.uint8] | None:
    """The winning frame around the marker, with the points that vouch for it.

    Cropped to the sharpness window scaled by :data:`CROP_CONTEXT_SCALE`, and
    then, if that still leaves it small enough to magnify by a whole factor
    within the largest encoding :data:`_CROP_ENCODINGS` offers, magnified by that
    factor with nearest-neighbour sampling -- so a reference the robot only saw
    from 3 m does not ship as a thumbnail nobody can name. Nearest-neighbour, and
    a whole factor, because the magnification must not invent detail the
    recording does not have: an interpolated crop of a distant object reads as
    *sharper* than the frame it came from, which is the one thing an image used
    to judge sharpness must never do.
    """
    import cv2

    obs = _nearest_observation(images, candidate.ts, tolerance=0.05)
    if obs is None:
        return None
    frame: NDArray[np.uint8] = cv2.cvtColor(obs.data.data, cv2.COLOR_RGB2BGR)
    for u, v in candidate.support.support_uv:
        cv2.circle(frame, (round(u), round(v)), 2, _INLIER_COLOR, -1)
    _draw_reference_marker(frame, candidate.marker)

    x1, y1, x2, y2 = window_region(
        candidate.marker,
        round(candidate.window_px * CROP_CONTEXT_SCALE),
        camera.width,
        camera.height,
    )
    crop: NDArray[np.uint8] = frame[y1:y2, x1:x2]
    if not crop.size:
        return None
    magnification = _CROP_ENCODINGS[0][0] // max(crop.shape[:2])
    if magnification <= 1:
        return crop
    resized = cv2.resize(
        crop,
        (crop.shape[1] * magnification, crop.shape[0] * magnification),
        interpolation=cv2.INTER_NEAREST,
    )
    return resized.astype(np.uint8, copy=False)


def write_crops(
    dataset: str,
    members: dict[str, list[Measurement]],
    references: list[Reference],
    out_dir: Path,
) -> CropReport:
    """Write the two review images per reference, and report what they cost.

    A reviewer is being asked two separate questions and one frame cannot answer
    both, which is why there are two files per reference:

    ``<question_id>.jpg`` -- *is this the thing the label says it is?* The
    sharpest frame near the detections in which the reference still has LiDAR
    support, cropped around the object's apparent extent, with the reference
    marker and the supporting points drawn on.

    ``<question_id>_measurement.jpg`` -- *did the geometry land on it?* The
    original detection frame with the full chain: mask outline, detector box,
    the inliers whose median became the position, and that position projected
    back in (see :func:`_measurement_crop`).

    The identification frame is picked, not assumed:

    1. **candidates** are ``color_image`` observations within
       :data:`CROP_WINDOW_S` of a contributing detection, with a pose, in which
       the reference projects in-image at positive depth;
    2. **gates**: mean gray at least :data:`CROP_MIN_GRAY` (falling back to the
       pipeline floor with a note), at least :data:`CROP_MIN_SUPPORT_POINTS`
       sweep points within :data:`CROP_SUPPORT_RADIUS_M` of the reference and
       inside that window, and nothing standing in front of the marker
       (:data:`CROP_OCCLUSION_SLACK_M`);
    3. **ranked** on the variance of the Laplacian in a window sized from the
       object's apparent extent in that frame (:func:`sharpness_window_px`),
       tiebroken on the lower yaw rate.

    This replaced picking the brightest *contributing* frame, which is a frame
    chosen for exposure while the robot was mid-detection. On the three committed
    questions the picked frame measures 2.9x, 4.0x and 22.5x the local Laplacian
    variance of the frame that picker chose.

    Not speed-first, though that was the first hypothesis: blur here tracks *yaw*
    rate rather than linear speed (the Go2 turns in place at 1.5-1.9 rad/s, and
    at the exposures this dim scene forces that is what smears a frame), and the
    detections a reference is built from already select for low yaw -- so ranking
    on speed made the crops *worse* on three of four questions. Sharpness is
    therefore measured directly, and yaw rate kept only as the tiebreak.

    Deterministic: every metric is a pure function of recorded data, both streams
    iterate in timestamp order, and the ranking's tiebreaks leave no pair for
    iteration order to separate. Crops are named after the question id the label
    would produce, so a crop and a shard line up without a lookup table.
    """
    from dimos.agents.evals.questions import slugify
    from dimos.memory2.cli.dataset import open_dataset

    out_dir.mkdir(parents=True, exist_ok=True)
    params = GateParams()
    camera = load_camera()
    projector = Projector(camera, load_base_to_optical(), params.front_z_m)
    by_label = {reference.raw_label: reference for reference in references}
    report = CropReport()
    store = open_dataset(dataset)
    with store:
        images: Stream[Any] = store.stream("color_image")
        lidar: Stream[Any] = store.stream("lidar")
        for label in sorted(members):
            reference = by_label.get(label)
            if reference is None:
                report.notes.append(f"{label!r}: no reference row, so no marker to draw")
                continue
            question_id = f"{slugify(dataset)}-{slugify(label)}"

            # `max` keeps the first of equal-brightness detections, i.e. the
            # earliest in the cluster's timestamp order.
            detection = max(members[label], key=lambda m: m.mean_gray)
            measurement = _measurement_crop(images, projector, reference, detection)
            if measurement is None:
                report.notes.append(f"{question_id}: detection frame {detection.ts} is unreadable")
            else:
                (out_dir / f"{question_id}_measurement.jpg").write_bytes(_encode_crop(measurement))
                report.written += 1

            candidate, note = pick_identification_frame(
                crop_candidates(images, lidar, projector, params, reference, members[label])
            )
            if note:
                report.notes.append(f"{question_id}: {note}")
            if candidate is None:
                continue
            crop = _identification_crop(images, camera, candidate)
            if crop is None:
                report.notes.append(f"{question_id}: picked frame {candidate.ts} is unreadable")
                continue
            (out_dir / f"{question_id}.jpg").write_bytes(_encode_crop(crop))
            report.written += 1
    return report
