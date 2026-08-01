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

"""Offline generator of the replay-derived reference table the evals score against.

Every "where is X" question needs a position for X. This tool derives those
positions from a recording instead of having someone measure them by hand:
RGB frames carry the semantics (an open-vocabulary detector), the time-aligned
LiDAR sweep carries the geometry, and the recorded pose puts both in the map
frame. The teacher's privilege over the agent under test is *access*, not a
better model — it reads the whole replay offline, with per-frame geometry and
cross-frame aggregation, while the agent gets one online query against what it
memorized in a single pass.

Pipeline. Every threshold is a flag whose default is the frozen value, and
every value actually used is echoed into ``manifest.json``:

1. sample ``color_image`` at ``--sample-hz``, measured on the *recorded*
   timestamp (wall-clock sampling would depend on machine speed)
2. skip frames whose mean gray is below ``--min-gray``: the replay has long
   near-black stretches where an open-vocabulary detector only hallucinates
3. run YOLOe in LRPC (prompt-free) mode, keeping detections above ``--conf``
4. take the nearest ``lidar`` message within ``--lidar-tolerance``; its points
   are already in the map frame
5. project them with ``pose ∘ BASE_TO_OPTICAL`` and ``cv2.fisheye.projectPoints``
   — the Go2 front camera is equidistant, and the points are projected directly
   rather than rectified first
6. gather the points landing inside the bbox shrunk by ``--bbox-shrink`` toward
   its center, because bbox corners are mostly background. The weights are a
   ``-seg`` variant and every detection does carry a pixel-level mask, which
   would be the tighter claim — ``--point-selection mask`` uses it — but as
   shipped those masks are not aligned with the frame; see
   :data:`PointSelection` for the measurement
7. drop points below ``--ground-z`` in the map frame *before* the point-count
   and spread gates: the floor is the largest single source of in-detection
   points and otherwise drags every reference down onto z≈0
8. require ``--min-points`` of them and a depth IQR within ``--max-iqr``; the
   candidate position is the map-frame median of the [p25,p75] depth inliers
9. keep only candidates inside the envelope (``--max-range``, ``--max-height``),
   i.e. objects the robot actually got close to, at a height a ground robot can
   sensibly be asked to drive to
10. cluster a label's candidates greedily within ``--cluster-radius``, and
    qualify a cluster only if it has at least ``--min-independent-views``
    pairwise-independent views (the robot moved ``--min-baseline`` between them)
    and an XY spread within ``--max-spread``. Independence is what stops a fast
    sampling rate from manufacturing agreement out of near-duplicate frames.
11. emit a label only if *exactly one* of its clusters qualifies: two qualified
    clusters mean the scene holds two instances a question could not
    distinguish, so the label is dropped rather than guessed at
12. finally, link surviving labels that sit within ``--link-radius`` of each
    other **in XY** into a shared ``location_group``. Steps 1-11 all run per
    label, so nothing before this can notice that one physical object collected
    several names — which would otherwise ask about the same place several
    times and mark an agent wrong for the names it did not happen to match

The output is a *replay-derived reference*, not ground truth — geometric
concentration is not semantic correctness. Labels therefore pass through a
human review overlay before they become questions (see ``questions.py``).

Determinism: on one machine, the same replay and the same flags produce a
byte-identical ``refs.jsonl`` — every gate is a pure function of the recorded
data, and nothing samples on wall-clock. It is *not* a claim across machines:
the positions are medians of detector output, and the detector's own output
depends on the device it ran on (``cpu`` vs. ``cuda``) and on the torch build,
which the manifest records for exactly that reason. Only the timing and
timestamp fields of ``manifest.json`` vary between runs on one machine, which
is why the two artifacts are separate files.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field, replace
import datetime
import json
import math
import os
from pathlib import Path
import statistics
import sys
import time
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.memory2.stream import Stream

    # Deferred: importing the detector pulls in torch and ultralytics, which the
    # gate helpers below (and `--help`) have no use for.
    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector

#: Bumped whenever a change to this file can move a reference position. It is
#: recorded in the manifest, so a table can always be traced back to the code
#: that produced it.
PIPELINE_VERSION = "spatial-goal-eval/teacher/2"

DEFAULT_DATASET = "go2_bigoffice"
DEFAULT_YOLOE_MODEL = "yoloe-11s-seg-pf.pt"

_CAMERA_YAML_PACKAGE = "dimos.robot.unitree.go2"
_CAMERA_YAML_NAME = "front_camera_720.yaml"

# Used only when the packaged calibration above cannot be located at runtime.
# Copied verbatim from dimos/robot/unitree/go2/front_camera_720.yaml.
_FALLBACK_K = (
    797.4756164864929,
    0.0,
    643.5352167821186,
    0.0,
    796.4872112769983,
    349.2783605343087,
    0.0,
    0.0,
    1.0,
)
_FALLBACK_D = (
    -0.07309428880537933,
    -0.02341140740909078,
    -0.0069305931780026956,
    0.009238684474464793,
)
_FALLBACK_SIZE = (1280, 720)


#: How the points belonging to one detection are chosen. ``"bbox"`` takes the
#: bbox shrunk toward its center; ``"mask"`` takes the detector's own
#: segmentation mask, falling back to the shrunk bbox for a detection that
#: carries no usable one.
#:
#: ``"bbox"`` is the default even though the mask is the tighter claim, and the
#: reason is measured rather than assumed. The prompt-free weights *are* a
#: ``-seg`` variant: every detection arrives as a ``Detection2DSeg`` with an
#: image-sized 0/255 mask. But those masks are built by resizing ultralytics'
#: ``masks.data`` — which lives in the **letterboxed** model-input space —
#: straight to the image size (``Detection2DSeg.from_ultralytics_result``), so
#: the padding is stretched along with the content. On this 1280x720 recording
#: the mask sits 12-19 px *below* the object: over 45 detections the mask's top
#: edge is a median +12.0 px from its own bbox's top edge (up to +19 px, and
#: detections whose bbox starts at y=0 have masks starting at y=19), while the
#: left and right edges agree to within a few pixels. A vertical offset is
#: exactly what a LiDAR point test cannot absorb, and switching this default to
#: ``"mask"`` on this replay stops four labels qualifying at all (``bookstore``,
#: ``elevator door``, ``organization``, ``server room``; ``min_points`` drops rise
#: 379 -> 454) and moves ``window sill`` 1.9 m -- which costs two of the four
#: committed questions outright and moves a third. The flag stays so the mask path
#: is one flag away once the alignment is fixed in ``dimos/perception``, and so
#: that measurement can be reproduced.
PointSelection = Literal["mask", "bbox"]
POINT_SELECTIONS: tuple[PointSelection, ...] = ("mask", "bbox")


@dataclass(frozen=True)
class GateParams:
    """Every threshold in the pipeline, in pipeline order.

    The defaults are the frozen values the shipped reference table was
    generated with. The whole object is serialized into the manifest, so a
    table always travels with the gates that produced it.

    ``point_selection`` is the one member that is not a threshold: it names
    *which pixels of a detection* may contribute points, and the manifest funnel
    additionally reports how many measurements each path actually produced (a
    mask-mode run still falls back per detection, so the configured mode alone
    does not say what happened).
    """

    sample_hz: float = 4.0
    min_gray: float = 30.0
    conf: float = 0.25
    lidar_tolerance_s: float = 0.5
    point_selection: PointSelection = "bbox"
    bbox_shrink: float = 0.15
    front_z_m: float = 0.05
    ground_z_m: float = 0.10
    min_points: int = 20
    max_depth_iqr_m: float = 0.6
    max_range_m: float = 3.2
    max_height_m: float = 1.2
    cluster_radius_m: float = 0.75
    min_baseline_m: float = 0.3
    min_independent_views: int = 2
    max_spread_m: float = 0.5
    link_radius_m: float = 0.75

    @property
    def min_frame_gap_s(self) -> float:
        """Smallest recorded-timestamp gap between two sampled frames."""
        return 1.0 / self.sample_hz


@dataclass(frozen=True)
class Camera:
    """Fisheye intrinsics and the image size they were calibrated at."""

    matrix: NDArray[np.float64]
    distortion: NDArray[np.float64]
    width: int
    height: int
    source: str


@dataclass(frozen=True)
class Measurement:
    """One detection that survived projection: a label with a map-frame position.

    Constructing one implies at least one non-ground LiDAR point landed on the
    detection, so every gate downstream can assume a position exists.

    The last three fields are the audit trail ``--crops`` draws, not inputs to
    any gate: ``point_source`` says whether the mask or the bbox fallback chose
    the points, ``mask_outline`` is the mask's external contours in pixel
    coordinates, and ``inlier_uv`` holds the pixels of exactly the points whose
    map-frame median became ``world_xyz``. A human can therefore check the
    *geometry* of a reference on the crop, not only the plausibility of a label.
    """

    ts: float
    label: str
    conf: float
    bbox: tuple[float, float, float, float]
    mean_gray: float
    robot_xyz: tuple[float, float, float]
    world_xyz: tuple[float, float, float]
    n_points: int
    depth_m: float
    depth_iqr_m: float
    range_m: float
    point_source: PointSelection
    mask_outline: tuple[tuple[tuple[int, int], ...], ...]
    inlier_uv: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class Reference:
    """One qualified object — the row that can become a question.

    ``n_views_raw`` counts distinct contributing frames; ``n_independent_views``
    counts the subset that passed the independence test, and is therefore the
    number the qualification decision actually rests on (and the one a
    ``QuestionSpec`` carries).

    ``location_group`` links references that landed on the same spot: the
    per-label rules cannot notice that two *different* labels describe one
    physical object, and that ambiguity has to be resolved in review before
    either becomes a question.
    """

    dataset: str
    raw_label: str
    location_group: str
    x: float
    y: float
    z: float
    n_views_raw: int
    n_independent_views: int
    n_detections: int
    spread_m: float
    min_baseline_m: float
    max_baseline_m: float
    depth_median_m: float
    mean_conf: float
    frame_ts: list[float]
    robot_poses: list[list[float]]

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


@dataclass
class ScanStats:
    """Sweep-level funnel counters plus the timing samples the manifest reports."""

    frames_visited: int = 0
    frames_too_dark: int = 0
    frames_detected: int = 0
    frames_without_pose: int = 0
    frames_without_lidar: int = 0
    raw_detections: int = 0
    below_conf: int = 0
    #: Detections on a frame that had no LiDAR message within the tolerance.
    #: They were counted into ``raw_detections`` and can never become
    #: measurements, so the funnel has to name them or its arithmetic silently
    #: stops adding up on any recording with a LiDAR gap.
    no_lidar_match: int = 0
    no_points_in_detection: int = 0
    measured_from_mask: int = 0
    measured_from_bbox: int = 0
    sweep_s: float = 0.0
    inference_s: list[float] = field(default_factory=list)
    lidar_dt_ms: list[float] = field(default_factory=list)


def load_camera() -> Camera:
    """Load the Go2 front-camera calibration shipped inside the robot package.

    Falls back to the constants above — copied from that same yaml — when the
    package data is unreachable, so the tool still runs against an install that
    dropped non-python files.
    """
    from importlib import resources

    try:
        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

        ref = resources.files(_CAMERA_YAML_PACKAGE).joinpath(_CAMERA_YAML_NAME)
        with resources.as_file(ref) as path:
            info = CameraInfo.from_yaml(str(path))
    except (ImportError, OSError):
        return Camera(
            matrix=np.array(_FALLBACK_K, dtype=np.float64).reshape(3, 3),
            distortion=np.array(_FALLBACK_D, dtype=np.float64).reshape(-1, 1),
            width=_FALLBACK_SIZE[0],
            height=_FALLBACK_SIZE[1],
            source=f"{_CAMERA_YAML_NAME} (inlined fallback constants)",
        )

    if info.distortion_model != "equidistant":
        raise ValueError(
            f"{_CAMERA_YAML_NAME} declares distortion model {info.distortion_model!r}, "
            "but the projection below is cv2.fisheye, i.e. the equidistant model"
        )
    return Camera(
        matrix=np.array(info.K, dtype=np.float64).reshape(3, 3),
        distortion=np.array(info.D, dtype=np.float64).reshape(-1, 1),
        width=int(info.width),
        height=int(info.height),
        source=f"{_CAMERA_YAML_PACKAGE.replace('.', '/')}/{_CAMERA_YAML_NAME}",
    )


def load_base_to_optical() -> Transform:
    """Return the shipped ``base_link -> camera_optical`` mount transform.

    Imported rather than re-derived: if the mount chain is ever recalibrated,
    the reference table follows it instead of keeping stale extrinsics.
    """
    from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL

    return BASE_TO_OPTICAL


class Projector:
    """Projects map-frame LiDAR points into the camera image of a single frame."""

    def __init__(self, camera: Camera, base_to_optical: Transform, front_z_m: float) -> None:
        self.camera = camera
        self.base_to_optical = base_to_optical
        self.front_z_m = front_z_m

    def project(
        self, points_w: NDArray[np.float64], pose_tuple: tuple[float, ...]
    ) -> tuple[
        NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], tuple[float, float, float]
    ]:
        """Project *points_w*, seen from *pose_tuple* ``(x, y, z, qx, qy, qz, qw)``.

        Returns ``(uv, depth, world_points, robot_xyz)`` for the points that are
        both in front of the camera and inside the image. ``uv``, ``depth`` and
        ``world_points`` stay index-aligned, so a pixel test also selects the
        corresponding map-frame point.
        """
        # Function-local, here and below: cv2 loads a large native extension
        # into every process that transitively imports this module
        # (dimos/codebase_checks/test_inline_heavy_imports.py).
        import cv2

        x, y, z, qx, qy, qz, qw = pose_tuple
        world_to_base = Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw))
        world_to_optical = (world_to_base + self.base_to_optical).to_matrix()
        rotation, translation = world_to_optical[:3, :3], world_to_optical[:3, 3]
        robot_xyz = (float(x), float(y), float(z))

        # Rowwise rotation.T @ (p - translation): map frame -> optical frame.
        optical = (points_w - translation) @ rotation

        in_front = optical[:, 2] > self.front_z_m
        optical, world = optical[in_front], points_w[in_front]
        if len(optical) == 0:
            return np.zeros((0, 2)), np.zeros(0), np.zeros((0, 3)), robot_xyz

        pixels, _ = cv2.fisheye.projectPoints(
            optical.reshape(-1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            self.camera.matrix,
            self.camera.distortion,
        )
        uv = pixels.reshape(-1, 2).astype(np.float64)
        in_image = (
            (uv[:, 0] >= 0)
            & (uv[:, 0] < self.camera.width)
            & (uv[:, 1] >= 0)
            & (uv[:, 1] < self.camera.height)
        )
        return uv[in_image], optical[in_image][:, 2], world[in_image], robot_xyz


def shrink_bbox(
    bbox: tuple[float, float, float, float], fraction: float
) -> tuple[float, float, float, float]:
    """Shrink *bbox* toward its center by *fraction* of each half-extent."""
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    half_w = (x2 - x1) / 2.0 * (1.0 - fraction)
    half_h = (y2 - y1) / 2.0 * (1.0 - fraction)
    return cx - half_w, cy - half_h, cx + half_w, cy + half_h


@dataclass(frozen=True)
class PointMeasurement:
    """The points one detection claimed, and the position they voted for."""

    n_points: int
    depth_median_m: float
    depth_iqr_m: float
    world_xyz: tuple[float, float, float]
    #: Which rule picked the points -- ``"mask"`` or the ``"bbox"`` fallback.
    source: PointSelection
    #: Pixels of the [p25, p75] depth inliers, i.e. of exactly the points whose
    #: median is ``world_xyz``. Carried for the review crops.
    inlier_uv: tuple[tuple[float, float], ...]


def points_on_mask(uv: NDArray[np.float64], mask: NDArray[np.uint8]) -> NDArray[np.bool_]:
    """Which projected pixels land on *mask*.

    The pixel is the nearest integer sample of the mask, clipped to its bounds:
    ``Projector.project`` already dropped everything outside ``[0, width)`` x
    ``[0, height)``, so the clip only catches a coordinate that rounds up onto
    the far edge.
    """
    height, width = mask.shape[:2]
    cols = np.clip(np.rint(uv[:, 0]).astype(np.intp), 0, width - 1)
    rows = np.clip(np.rint(uv[:, 1]).astype(np.intp), 0, height - 1)
    return np.asarray(mask[rows, cols] > 0)


def measure_detection(
    uv: NDArray[np.float64],
    depth: NDArray[np.float64],
    world: NDArray[np.float64],
    bbox: tuple[float, float, float, float],
    mask: NDArray[np.uint8] | None,
    params: GateParams,
) -> PointMeasurement | None:
    """Estimate the map-frame position of whatever one detection covers.

    Returns ``None`` when no non-ground point landed on the detection. The
    position is the map-frame median of the points whose optical depth falls in
    [p25, p75]: trimming in depth stops a foreground railing or a background
    wall from pulling the estimate, and needs no fisheye unprojection.

    Which pixels count as "on the detection" is the mask when there is one and
    ``params.point_selection`` asks for it, and the shrunk bbox otherwise. The
    mask *would* be much the tighter claim -- a bbox around a shelf bay is mostly
    the aisle in front of it -- which is why the path exists at all; see
    :data:`PointSelection` for why it is not the default on this recording.
    Nothing here intersects the two: a mask is either trusted as the detector's
    own pixel-level answer or not used, and mixing them would hide which one an
    odd position came from.
    """
    usable_mask = mask if params.point_selection == "mask" else None
    source: PointSelection
    if usable_mask is not None:
        selected = points_on_mask(uv, usable_mask)
        source = "mask"
    else:
        x1, y1, x2, y2 = shrink_bbox(bbox, params.bbox_shrink)
        selected = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
        source = "bbox"

    inside = selected & (world[:, 2] >= params.ground_z_m)
    n_points = int(inside.sum())
    if n_points == 0:
        return None

    pixels, depths, points = uv[inside], depth[inside], world[inside]
    p25, p75 = float(np.percentile(depths, 25)), float(np.percentile(depths, 75))
    keep = (depths >= p25) & (depths <= p75)
    if not keep.any():  # degenerate: every point sits at one depth
        keep = np.ones(len(depths), dtype=bool)
    position = np.median(points[keep], axis=0)
    return PointMeasurement(
        n_points=n_points,
        depth_median_m=float(np.median(depths)),
        depth_iqr_m=p75 - p25,
        world_xyz=(float(position[0]), float(position[1]), float(position[2])),
        source=source,
        inlier_uv=tuple((float(u), float(v)) for u, v in pixels[keep]),
    )


def _load_detector(conf: float, model_name: str) -> tuple[Yoloe2DDetector, dict[str, Any]]:
    """Build the prompt-free YOLOe detector and describe it for the manifest.

    LRPC and PROMPT modes produce different label vocabularies and must never
    silently substitute for each other, so the mode is pinned here rather than
    exposed as a flag.
    """
    # Ultralytics pip-installs missing extras the first time a model runs. A
    # reference generator must never mutate its own environment mid-sweep, so
    # this is set here -- before the deferred YOLOe import below, and only in a
    # process that is actually about to run the model, so merely importing this
    # module (for `--help`, or for the gate helpers the tests exercise) does not
    # reach into the environment.
    os.environ["YOLO_AUTOINSTALL"] = "false"

    from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode

    detector = Yoloe2DDetector(model_name=model_name, prompt_mode=YoloePromptMode.LRPC, conf=conf)
    info = {
        "model": model_name,
        "prompt_mode": YoloePromptMode.LRPC.value,
        "conf": conf,
        "max_area_ratio": detector.max_area_ratio,
        "device": detector.device,
    }
    return detector, info


def detection_mask(detection: Any, camera: Camera) -> NDArray[np.uint8] | None:
    """The detection's segmentation mask, or ``None`` when there is none to use.

    ``Detection2DSeg`` carries an image-sized ``[H, W]`` uint8 mask (0/255) and
    the other ``Detection2D`` subclasses carry none at all, so the attribute is
    read defensively. A mask whose shape does not match the calibration the
    points were projected with is refused rather than indexed: the pixel
    coordinates would silently address the wrong sample.
    """
    mask = getattr(detection, "mask", None)
    if mask is None:
        return None
    array = np.asarray(mask)
    if array.ndim != 2 or array.shape != (camera.height, camera.width):
        return None
    return array.astype(np.uint8, copy=False)


def mask_outline(mask: NDArray[np.uint8] | None) -> tuple[tuple[tuple[int, int], ...], ...]:
    """External contours of *mask*, as plain pixel tuples for the review crops.

    Contours rather than the mask itself: a 1280x720 mask per surviving
    detection is hundreds of megabytes over a sweep, while its outline is what a
    crop can actually show.
    """
    import cv2

    if mask is None:
        return ()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return tuple(
        tuple((int(point[0][0]), int(point[0][1])) for point in contour)
        for contour in contours
        if len(contour) >= 2
    )


def scan(
    dataset: str, params: GateParams, yoloe_model: str, progress: bool = True
) -> tuple[list[Measurement], ScanStats, dict[str, Any]]:
    """Sweep *dataset* once and return every detection that got a position."""
    import cv2

    from dimos.memory2.cli.dataset import open_dataset

    camera = load_camera()
    projector = Projector(camera, load_base_to_optical(), params.front_z_m)
    detector, detector_info = _load_detector(params.conf, yoloe_model)

    measurements: list[Measurement] = []
    stats = ScanStats()
    started = time.monotonic()

    store = open_dataset(dataset)
    with store:
        images: Stream[Any] = store.stream("color_image")
        lidar: Stream[Any] = store.stream("lidar")
        last_ts: float | None = None

        for obs in images:
            if last_ts is not None and (obs.ts - last_ts) < params.min_frame_gap_s:
                continue
            last_ts = obs.ts
            stats.frames_visited += 1

            mean_gray = float(cv2.cvtColor(obs.data.data, cv2.COLOR_RGB2GRAY).mean())
            if mean_gray < params.min_gray:
                stats.frames_too_dark += 1
                continue
            pose_tuple = obs.pose_tuple
            if pose_tuple is None:
                # Nothing in the frame can be placed in the map frame, so there
                # is no point paying for detection on it.
                stats.frames_without_pose += 1
                continue
            stats.frames_detected += 1

            inference_started = time.monotonic()
            detected = detector.predict_image(obs.data).detections
            stats.inference_s.append(time.monotonic() - inference_started)
            stats.raw_detections += len(detected)

            # The detector is constructed with the same threshold, so this is
            # normally a no-op; keeping it explicit means the confidence gate
            # is ours and stays visible in the funnel.
            detections = [d for d in detected if float(d.confidence) >= params.conf]
            stats.below_conf += len(detected) - len(detections)

            nearby = list(lidar.at(obs.ts, tolerance=params.lidar_tolerance_s))
            if not nearby:
                # These detections were already counted as raw, and nothing can
                # place them: without a sweep there is no geometry. Counting them
                # into their own bucket is what keeps the funnel's arithmetic
                # (raw - drops == surviving) true on a recording with a gap.
                stats.frames_without_lidar += 1
                stats.no_lidar_match += len(detections)
                continue
            cloud = min(nearby, key=lambda o: abs(o.ts - obs.ts))
            stats.lidar_dt_ms.append((cloud.ts - obs.ts) * 1000.0)
            uv, depth, world, robot_xyz = projector.project(
                cloud.data.as_numpy()[0].astype(np.float64), pose_tuple
            )

            for detection in detections:
                bbox = (
                    float(detection.bbox[0]),
                    float(detection.bbox[1]),
                    float(detection.bbox[2]),
                    float(detection.bbox[3]),
                )
                mask = detection_mask(detection, camera)
                measured = measure_detection(uv, depth, world, bbox, mask, params)
                if measured is None:
                    stats.no_points_in_detection += 1
                    continue
                if measured.source == "mask":
                    stats.measured_from_mask += 1
                else:
                    stats.measured_from_bbox += 1
                world_xyz = measured.world_xyz
                measurements.append(
                    Measurement(
                        ts=float(obs.ts),
                        label=detection.name,
                        conf=float(detection.confidence),
                        bbox=bbox,
                        mean_gray=mean_gray,
                        robot_xyz=robot_xyz,
                        world_xyz=world_xyz,
                        n_points=measured.n_points,
                        depth_m=measured.depth_median_m,
                        depth_iqr_m=measured.depth_iqr_m,
                        range_m=math.hypot(
                            world_xyz[0] - robot_xyz[0], world_xyz[1] - robot_xyz[1]
                        ),
                        point_source=measured.source,
                        mask_outline=mask_outline(mask if measured.source == "mask" else None),
                        inlier_uv=measured.inlier_uv,
                    )
                )

            if progress and stats.frames_detected % 100 == 0:
                print(
                    f"  {stats.frames_visited} visited / {stats.frames_detected} detected / "
                    f"{len(measurements)} measurements / {time.monotonic() - started:.0f}s",
                    flush=True,
                )

    stats.sweep_s = time.monotonic() - started
    return measurements, stats, detector_info


def apply_gates(
    measurements: list[Measurement], params: GateParams
) -> tuple[list[Measurement], Counter[str]]:
    """Apply the per-measurement gates; the first failing gate claims the drop."""
    kept: list[Measurement] = []
    drops: Counter[str] = Counter()
    for m in measurements:
        if m.n_points < params.min_points:
            drops["min_points"] += 1
        elif m.depth_iqr_m > params.max_depth_iqr_m:
            drops["depth_iqr"] += 1
        elif m.range_m > params.max_range_m:
            drops["envelope_range"] += 1
        elif m.world_xyz[2] > params.max_height_m:
            drops["envelope_height"] += 1
        else:
            kept.append(m)
    return kept, drops


def independent_views(views: list[Measurement], params: GateParams) -> list[Measurement]:
    """Largest greedily-built subset of *views* whose every pair is independent.

    Two views are independent when the robot moved at least ``min_baseline_m``
    between them. Independence is **spatial only, deliberately**: what a second
    view is supposed to add is a second line of sight, and a ground robot that
    sat still for a minute has one line of sight however many frames it took.
    An earlier version also accepted "at least a second elapsed", which counted
    a static re-observation as verification and so overstated what two views
    prove -- and on ``go2_bigoffice`` it never mattered: rerunning the whole
    sweep with that branch removed produced a byte-identical ``refs.jsonl`` and
    an identical funnel (26 labels qualified either way, 250 clusters dropped
    for too few independent views either way). It was a claim the table did not
    need, so it is gone.

    On ``go2_short`` it does bite, once, and on exactly the row it was removed
    to catch: ``beauty salon`` qualified under the old rule on two views whose
    baseline was **4.4 mm**, spanning 1.1 s of the robot standing still. It is
    absent from that dataset's 14 references, and review had independently
    dropped it as class-ambiguous, so nothing downstream moves.

    Greedy in timestamp order is a lower bound on the true maximum clique, which
    keeps the gate conservative.
    """
    selected: list[Measurement] = []
    for view in sorted(views, key=lambda m: m.ts):
        if all(_is_independent(view, other, params) for other in selected):
            selected.append(view)
    return selected


def _is_independent(a: Measurement, b: Measurement, params: GateParams) -> bool:
    return math.dist(a.robot_xyz, b.robot_xyz) >= params.min_baseline_m


def _cluster_by_position(
    measurements: list[Measurement], radius_m: float
) -> list[list[Measurement]]:
    """Greedily group *measurements* whose positions fall within *radius_m*.

    Each measurement joins the cluster whose running median position is nearest,
    if that distance is within the radius, and starts a new cluster otherwise.
    Consumed in timestamp order, so the grouping is deterministic.
    """
    clusters: list[list[Measurement]] = []
    for m in sorted(measurements, key=lambda m: m.ts):
        position = np.array(m.world_xyz)
        nearest: list[Measurement] | None = None
        nearest_distance = float("inf")
        for cluster in clusters:
            center = np.median(np.array([c.world_xyz for c in cluster]), axis=0)
            distance = float(np.linalg.norm(position - center))
            if distance < nearest_distance:
                nearest, nearest_distance = cluster, distance
        if nearest is not None and nearest_distance <= radius_m:
            nearest.append(m)
        else:
            clusters.append([m])
    return clusters


def _one_view_per_frame(cluster: list[Measurement]) -> list[Measurement]:
    """Collapse a cluster to one measurement per frame, keeping the best-supported."""
    per_ts: dict[float, Measurement] = {}
    for m in cluster:
        best = per_ts.get(m.ts)
        if best is None or m.n_points > best.n_points:
            per_ts[m.ts] = m
    return list(per_ts.values())


def _xy_spread(cluster: list[Measurement]) -> float:
    """Largest pairwise XY distance among a cluster's positions."""
    xy = [m.world_xyz[:2] for m in cluster]
    if len(xy) < 2:
        return 0.0
    return max(math.dist(xy[i], xy[j]) for i in range(len(xy)) for j in range(i + 1, len(xy)))


@dataclass
class QualifyDiagnostics:
    """Why labels did or did not make it into the table — reported in the manifest."""

    clusters_formed: int = 0
    cluster_drops: Counter[str] = field(default_factory=Counter)
    label_verdicts: Counter[str] = field(default_factory=Counter)


def qualify(
    measurements: list[Measurement], params: GateParams, dataset: str
) -> tuple[list[Reference], QualifyDiagnostics, dict[str, list[Measurement]]]:
    """Cluster per label and keep the labels that resolve to exactly one object.

    Returns the references sorted by label, the cluster- and label-level
    diagnostics, and the contributing measurements per emitted label (which
    ``--crops`` turns into review images).
    """
    by_label: dict[str, list[Measurement]] = defaultdict(list)
    for m in measurements:
        by_label[m.label].append(m)

    references: list[Reference] = []
    members: dict[str, list[Measurement]] = {}
    diagnostics = QualifyDiagnostics()

    for label in sorted(by_label):
        qualified: list[tuple[list[Measurement], list[Measurement], list[Measurement], float]] = []
        for cluster in _cluster_by_position(by_label[label], params.cluster_radius_m):
            diagnostics.clusters_formed += 1
            views = _one_view_per_frame(cluster)
            independent = independent_views(views, params)
            if len(independent) < params.min_independent_views:
                diagnostics.cluster_drops["insufficient_independent_views"] += 1
                continue
            spread = _xy_spread(cluster)
            if spread > params.max_spread_m:
                diagnostics.cluster_drops["spread_too_large"] += 1
                continue
            qualified.append((cluster, views, independent, spread))

        if not qualified:
            diagnostics.label_verdicts["no_qualified_cluster"] += 1
            continue
        if len(qualified) > 1:
            # Two qualified clusters: the scene holds instances a question could
            # not tell apart, so the label is unusable rather than merely hard.
            diagnostics.label_verdicts["ambiguous_multi_instance"] += 1
            continue

        cluster, views, independent, spread = qualified[0]
        diagnostics.label_verdicts["qualified"] += 1
        members[label] = cluster
        references.append(_build_reference(dataset, label, cluster, views, independent, spread))

    return references, diagnostics, members


def _build_reference(
    dataset: str,
    label: str,
    cluster: list[Measurement],
    views: list[Measurement],
    independent: list[Measurement],
    spread: float,
) -> Reference:
    position = np.median(np.array([m.world_xyz for m in cluster]), axis=0)
    baselines = [
        math.dist(independent[i].robot_xyz, independent[j].robot_xyz)
        for i in range(len(independent))
        for j in range(i + 1, len(independent))
    ]
    ordered = sorted(cluster, key=lambda m: m.ts)
    return Reference(
        dataset=dataset,
        raw_label=label,
        location_group="",  # filled in by assign_location_groups once all labels exist
        x=round(float(position[0]), 4),
        y=round(float(position[1]), 4),
        z=round(float(position[2]), 4),
        n_views_raw=len(views),
        n_independent_views=len(independent),
        n_detections=len(cluster),
        spread_m=round(spread, 4),
        min_baseline_m=round(min(baselines), 4) if baselines else 0.0,
        max_baseline_m=round(max(baselines), 4) if baselines else 0.0,
        depth_median_m=round(float(statistics.median([m.depth_m for m in cluster])), 4),
        mean_conf=round(float(np.mean([m.conf for m in cluster])), 4),
        frame_ts=[round(m.ts, 3) for m in ordered],
        robot_poses=[[round(v, 4) for v in m.robot_xyz] for m in ordered],
    )


def assign_location_groups(
    references: list[Reference], radius_m: float
) -> tuple[list[Reference], list[dict[str, Any]]]:
    """Link references that landed on the same spot, whatever they were called.

    Qualification runs per label, so it cannot see that one physical object
    collected several names — an open-vocabulary detector will happily call the
    same shelf bay ``shelf``, ``bookstore`` and ``organization``. Left alone
    that corrupts scoring twice over: the same place is asked about repeatedly,
    and an agent that drives to it is marked wrong for whichever names it did
    not match. References within *radius_m* of each other (transitively) are
    therefore tagged with a shared ``location_group``, and review has to decide
    which single name survives — ``questions.py`` refuses to emit two questions
    from one group.

    Distance here is **map-frame XY**, matching how the group is used: the
    question is "go to X", the robot drives on a plane, and two labels stacked
    vertically — a shelf and the boxes on it — are one destination however far
    apart they are in height. Linking in 3-D would let the shelf's height split
    a group and let both names become questions with the same right answer.

    Returns the references with the field filled in, plus the group table for
    the manifest.
    """
    ordered = sorted(references, key=lambda r: r.raw_label)
    parent = list(range(len(ordered)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(ordered):
        for j in range(i + 1, len(ordered)):
            b = ordered[j]
            if math.dist((a.x, a.y), (b.x, b.y)) <= radius_m:
                parent[find(i)] = find(j)

    members: dict[int, list[int]] = defaultdict(list)
    for i in range(len(ordered)):
        members[find(i)].append(i)

    # Number the groups by their alphabetically first label so the ids are
    # stable across runs and readable in a diff.
    groups = sorted(members.values(), key=lambda idx: ordered[idx[0]].raw_label)
    tagged: list[Reference] = []
    table: list[dict[str, Any]] = []
    for n, idx in enumerate(groups, 1):
        group_id = f"loc-{n:02d}"
        rows = [ordered[i] for i in idx]
        for row in rows:
            tagged.append(replace(row, location_group=group_id))
        table.append(
            {
                "location_group": group_id,
                "raw_labels": [row.raw_label for row in rows],
                "centroid": [
                    round(float(np.mean([row.x for row in rows])), 4),
                    round(float(np.mean([row.y for row in rows])), 4),
                    round(float(np.mean([row.z for row in rows])), 4),
                ],
                # XY, i.e. the distance the linking decision was actually made
                # on; a 3-D number here would not explain the grouping.
                "max_pairwise_m": round(
                    max(
                        (math.dist((a.x, a.y), (b.x, b.y)) for a in rows for b in rows),
                        default=0.0,
                    ),
                    4,
                ),
            }
        )
    return sorted(tagged, key=lambda r: r.raw_label), table


def write_refs(references: list[Reference], path: Path) -> None:
    """Write one reference per line, ordered by label so diffs stay readable."""
    ordered = sorted(references, key=lambda r: r.raw_label)
    path.write_text("".join(f"{r.to_json()}\n" for r in ordered))


def measurement_drops(stats: ScanStats, gate_drops: Counter[str]) -> dict[str, int]:
    """Every counted detection that never became a measurement, by reason.

    The keys of this mapping *are* the funnel's detection-level drop buckets, and
    the identity a reader relies on is ``raw_detections`` minus their sum equals
    ``surviving_measurements``. Keeping them in one place is what makes that hold
    by construction: a new discard path adds a key here, and both the funnel and
    its consistency check follow. ``dropped_no_lidar_match`` is the bucket that
    was missing -- it stays 0 on a recording whose LiDAR never gaps, so the
    arithmetic looked right until one does.
    """
    return {
        "dropped_below_conf": stats.below_conf,
        "dropped_no_lidar_match": stats.no_lidar_match,
        "dropped_no_points_in_detection": stats.no_points_in_detection,
        "dropped_min_points": gate_drops["min_points"],
        "dropped_depth_iqr": gate_drops["depth_iqr"],
        "dropped_envelope_range": gate_drops["envelope_range"],
        "dropped_envelope_height": gate_drops["envelope_height"],
    }


def build_funnel(
    stats: ScanStats,
    gate_drops: Counter[str],
    diagnostics: QualifyDiagnostics,
    n_location_groups: int,
) -> dict[str, int]:
    """The sweep's counts, as the manifest reports them.

    Detection-level drops come from :func:`measurement_drops` and settle
    ``surviving_measurements``; the cluster- and label-level numbers below them
    describe what happened to those survivors afterwards and are deliberately
    *not* part of that subtraction.
    """
    dropped = measurement_drops(stats, gate_drops)
    return {
        "frames_visited": stats.frames_visited,
        "frames_too_dark": stats.frames_too_dark,
        "frames_detected": stats.frames_detected,
        "frames_without_pose": stats.frames_without_pose,
        "frames_without_lidar": stats.frames_without_lidar,
        "raw_detections": stats.raw_detections,
        **dropped,
        "surviving_measurements": stats.raw_detections - sum(dropped.values()),
        "measured_from_mask": stats.measured_from_mask,
        "measured_from_bbox": stats.measured_from_bbox,
        "clusters_formed": diagnostics.clusters_formed,
        "dropped_clusters_insufficient_independent_views": diagnostics.cluster_drops[
            "insufficient_independent_views"
        ],
        "dropped_clusters_spread_too_large": diagnostics.cluster_drops["spread_too_large"],
        "labels_no_qualified_cluster": diagnostics.label_verdicts["no_qualified_cluster"],
        "labels_ambiguous_multi_instance": diagnostics.label_verdicts["ambiguous_multi_instance"],
        "labels_qualified": diagnostics.label_verdicts["qualified"],
        "location_groups": n_location_groups,
    }


def build_manifest(
    dataset: str,
    params: GateParams,
    detector_info: dict[str, Any],
    stats: ScanStats,
    gate_drops: Counter[str],
    diagnostics: QualifyDiagnostics,
    references: list[Reference],
    location_groups: list[dict[str, Any]],
    wallclock_s: float,
    note: str,
) -> dict[str, Any]:
    """Assemble the provenance record that ships next to ``refs.jsonl``."""
    camera = load_camera()
    transform = load_base_to_optical()
    inference_ms = [1000.0 * s for s in stats.inference_s]
    lidar_dt_ms = [abs(dt) for dt in stats.lidar_dt_ms]
    return {
        "pipeline_version": PIPELINE_VERSION,
        "generator": "dimos/agents/evals/teacher.py",
        "generated_utc": datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds"),
        "dataset": dataset,
        "note": note,
        "detector": detector_info,
        "camera": {
            "source": camera.source,
            "distortion_model": "equidistant",
            "width": camera.width,
            "height": camera.height,
            "K": [round(v, 6) for v in camera.matrix.reshape(-1).tolist()],
            "D": [round(v, 6) for v in camera.distortion.reshape(-1).tolist()],
        },
        "extrinsics": {
            "source": "dimos.robot.unitree.go2.connection.BASE_TO_OPTICAL",
            "frame": f"{transform.frame_id} -> {transform.child_frame_id}",
            "translation": [round(float(v), 6) for v in transform.translation.to_numpy()],
            "rotation_xyzw": [
                round(float(transform.rotation.x), 6),
                round(float(transform.rotation.y), 6),
                round(float(transform.rotation.z), 6),
                round(float(transform.rotation.w), 6),
            ],
        },
        "gates": asdict(params),
        "funnel": build_funnel(stats, gate_drops, diagnostics, len(location_groups)),
        "timing": {
            "wallclock_s": round(wallclock_s, 2),
            "detection_sweep_s": round(stats.sweep_s, 2),
            "yoloe_inference_ms_median": (
                round(statistics.median(inference_ms), 1) if inference_ms else None
            ),
            "lidar_dt_ms_max_abs": round(max(lidar_dt_ms), 1) if lidar_dt_ms else None,
        },
        "n_references": len(references),
        "raw_labels": [r.raw_label for r in references],
        "location_groups": location_groups,
    }


def _build_parser() -> argparse.ArgumentParser:
    defaults = GateParams()
    parser = argparse.ArgumentParser(
        prog="python -m dimos.agents.evals.teacher",
        description=(
            "Derive a reference table of object positions from a recorded replay. "
            "Flag defaults are the frozen pipeline; every value used is echoed "
            "into manifest.json."
        ),
    )
    parser.add_argument("--dataset", default=DEFAULT_DATASET, help="dataset name or .db/.mcap path")
    parser.add_argument(
        "--out-dir", required=True, help="directory to write refs.jsonl and manifest.json into"
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=defaults.sample_hz,
        help="frame sampling rate, measured on the recorded timestamp",
    )
    parser.add_argument(
        "--min-gray",
        type=float,
        default=defaults.min_gray,
        help="skip frames darker than this mean gray level",
    )
    parser.add_argument(
        "--conf", type=float, default=defaults.conf, help="detector confidence threshold"
    )
    parser.add_argument(
        "--yoloe-model",
        default=DEFAULT_YOLOE_MODEL,
        help="YOLOe weights file (prompt-free/LRPC vocabulary)",
    )
    parser.add_argument(
        "--lidar-tolerance",
        type=float,
        default=defaults.lidar_tolerance_s,
        help="max |dt| (s) between a frame and the LiDAR message paired with it",
    )
    parser.add_argument(
        "--point-selection",
        choices=POINT_SELECTIONS,
        default=defaults.point_selection,
        help=(
            "which pixels of a detection may contribute points: its segmentation "
            "mask, or the shrunk bbox (the fallback a detection without a mask "
            "gets either way)"
        ),
    )
    parser.add_argument(
        "--bbox-shrink",
        type=float,
        default=defaults.bbox_shrink,
        help="fraction each bbox is shrunk toward its center before gathering points",
    )
    parser.add_argument(
        "--front-z",
        type=float,
        default=defaults.front_z_m,
        help="min optical-frame depth (m) for a point to count as in front of the camera",
    )
    parser.add_argument(
        "--ground-z",
        type=float,
        default=defaults.ground_z_m,
        help="reject in-detection points below this map-frame height (m); applied first",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=defaults.min_points,
        help="min non-ground points on the detection (mask or shrunk bbox)",
    )
    parser.add_argument(
        "--max-iqr",
        type=float,
        default=defaults.max_depth_iqr_m,
        help="max in-detection depth inter-quartile range (m)",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=defaults.max_range_m,
        help="envelope: max horizontal robot-to-object distance (m)",
    )
    parser.add_argument(
        "--max-height",
        type=float,
        default=defaults.max_height_m,
        help="envelope: max map-frame object height (m)",
    )
    parser.add_argument(
        "--cluster-radius",
        type=float,
        default=defaults.cluster_radius_m,
        help="max distance (m) from a cluster's median position to join it",
    )
    parser.add_argument(
        "--min-baseline",
        type=float,
        default=defaults.min_baseline_m,
        help="robot displacement (m) that makes two views independent",
    )
    parser.add_argument(
        "--min-independent-views",
        type=int,
        default=defaults.min_independent_views,
        help="independent views a cluster needs to qualify",
    )
    parser.add_argument(
        "--max-spread",
        type=float,
        default=defaults.max_spread_m,
        help="max pairwise XY spread (m) within a qualifying cluster",
    )
    parser.add_argument(
        "--link-radius",
        type=float,
        default=defaults.link_radius_m,
        help="map-frame XY distance (m) within which two labels are one location",
    )
    parser.add_argument(
        "--crops",
        action="store_true",
        help=(
            "also write the two review images per reference into <out-dir>/crops: "
            "<question-id>.jpg (sharpest LiDAR-confirmed frame, for identifying the "
            "object) and <question-id>_measurement.jpg (the detection frame, for "
            "checking the geometry)"
        ),
    )
    parser.add_argument(
        "--note",
        default="",
        help="free-text provenance note recorded in manifest.json",
    )
    parser.add_argument("--quiet", action="store_true", help="suppress sweep progress lines")
    return parser


def main(argv: list[str]) -> int:
    args = _build_parser().parse_args(argv)
    params = GateParams(
        sample_hz=args.sample_hz,
        min_gray=args.min_gray,
        conf=args.conf,
        lidar_tolerance_s=args.lidar_tolerance,
        point_selection=args.point_selection,
        bbox_shrink=args.bbox_shrink,
        front_z_m=args.front_z,
        ground_z_m=args.ground_z,
        min_points=args.min_points,
        max_depth_iqr_m=args.max_iqr,
        max_range_m=args.max_range,
        max_height_m=args.max_height,
        cluster_radius_m=args.cluster_radius,
        min_baseline_m=args.min_baseline,
        min_independent_views=args.min_independent_views,
        max_spread_m=args.max_spread,
        link_radius_m=args.link_radius,
    )
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()

    measurements, stats, detector_info = scan(
        args.dataset, params, args.yoloe_model, progress=not args.quiet
    )
    kept, gate_drops = apply_gates(measurements, params)
    references, diagnostics, members = qualify(kept, params, args.dataset)
    references, location_groups = assign_location_groups(references, params.link_radius_m)

    write_refs(references, out_dir / "refs.jsonl")
    manifest = build_manifest(
        args.dataset,
        params,
        detector_info,
        stats,
        gate_drops,
        diagnostics,
        references,
        location_groups,
        time.monotonic() - started,
        args.note,
    )
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    if args.crops:
        # Deferred, and one-way: `crops` imports this module for the projection
        # and the gates, and nothing here may depend on it -- the review images
        # are presentation, and the table must be identical without them.
        from dimos.agents.evals.crops import write_crops

        # The same gates the measurement ran with -- a crop rendered under
        # different projection or sweep tolerance would show frames the table
        # was not built from.
        crops = write_crops(args.dataset, members, references, out_dir / "crops", params)
        print(f"crops:     {crops.written} -> {out_dir / 'crops'}")
        # Printed, never swallowed: every note is a reference whose review images
        # are weaker than the picker intends, which a reviewer has to know before
        # reading them.
        for note in crops.notes:
            print(f"  note:    {note}")

    funnel = manifest["funnel"]
    print(
        f"frames:    {stats.frames_visited} visited, {stats.frames_too_dark} too dark, "
        f"{stats.frames_detected} detected on\n"
        f"detections:{stats.raw_detections} raw -> "
        f"{funnel['measured_from_mask'] + funnel['measured_from_bbox']} placed "
        f"({funnel['measured_from_mask']} by mask, {funnel['measured_from_bbox']} by bbox) -> "
        f"{funnel['surviving_measurements']} past the gates -> "
        f"{funnel['clusters_formed']} clusters\n"
        f"references:{len(references)} qualified of {len({m.label for m in kept})} labels, "
        f"on {len(location_groups)} distinct locations\n"
        f"wallclock: {manifest['timing']['wallclock_s']}s "
        f"(sweep {manifest['timing']['detection_sweep_s']}s)\n"
        f"wrote:     {out_dir / 'refs.jsonl'}, {out_dir / 'manifest.json'}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
