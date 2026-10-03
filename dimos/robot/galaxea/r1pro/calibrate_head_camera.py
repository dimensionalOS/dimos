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

"""Calibrate the R1 Pro left head camera against the Mid-360, cross-checked with stereo depth.

Given a recording with `head_left_color`, `head_stereo_depth`, `lidar_raw` and `tf`, this refines
fx, fy, cx, cy of the left head camera and a 6-DoF correction to camera<-lidar, by jointly
  - pulling lidar depth discontinuities onto image edges (distance transform of Canny edges), and
  - matching lidar inverse depth to stereo inverse depth, with the stereo's own scale / disparity
    offset as nuisance terms (which double as a check of fx * baseline and the right camera's yaw).
Before/after metrics are computed on held-out frames.

    python -m dimos.robot.galaxea.r1pro.calibrate_head_camera rec.mcap --output calib.yaml --overlay overlay.png
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

STEREO_BASELINE_M = 0.120195
CAMERA_FRAME = "camera_head_left_link"
LIDAR_FRAME = "lidar_pointlio_link"
EDGE_CLIP_PX = 30.0
EDGE_SIGMA_PX = 3.0
INVERSE_DEPTH_SIGMA = 0.02
SCAN_PERIOD_S = 0.1  # Mid-360 at 10 Hz
MIN_RANGE_M = 0.5
MAX_RANGE_M = 20.0
STEREO_MAX_RANGE_M = (
    8.0  # beyond this the decimated stereo is a few pixels of disparity and mostly smoothing
)

EMPTY = np.zeros((0, 3))


@dataclass(frozen=True)
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray
    width: int
    height: int

    @property
    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def project(self, points: np.ndarray) -> np.ndarray:
        """Camera-frame points (N,3) to pixels (N,2) through the 8-coefficient rational model."""
        import cv2

        if len(points) == 0:
            return np.zeros((0, 2))
        pixels, _ = cv2.projectPoints(
            points, np.zeros(3), np.zeros(3), self.matrix, self.distortion
        )
        return pixels.reshape(-1, 2)

    def max_ray_slope(self) -> float:
        """Largest |ray / z| that still lands in the image; the rational model folds back beyond it."""
        import cv2

        corners = np.array(
            [[0, 0], [self.width, 0], [0, self.height], [self.width, self.height]], float
        )
        rays = cv2.undistortPoints(corners[:, None], self.matrix, self.distortion).reshape(-1, 2)
        return float(np.linalg.norm(rays, axis=1).max() * 1.05)

    def with_offsets(self, offsets: np.ndarray) -> Camera:
        return replace(
            self,
            fx=self.fx + offsets[0],
            fy=self.fy + offsets[1],
            cx=self.cx + offsets[2],
            cy=self.cy + offsets[3],
        )


@dataclass
class Frame:
    time: float
    gray: np.ndarray  # full-resolution left image
    stereo_depth: np.ndarray  # metres, NaN where the matcher gave up
    stereo_camera: Camera  # pinhole intrinsics of the depth grid
    lidar_points: np.ndarray  # motion-compensated into the lidar frame at `time`
    cam_from_lidar: np.ndarray  # 4x4 prior from tf
    # Filled in by `split_lidar_points`:
    edge_distance: np.ndarray = field(default_factory=lambda: EMPTY)  # px to the nearest image edge
    edge_points: np.ndarray = field(
        default_factory=lambda: EMPTY
    )  # lidar points just inside a silhouette
    edge_outward: np.ndarray = field(default_factory=lambda: EMPTY)  # image direction out of it
    surface_points: np.ndarray = field(
        default_factory=lambda: EMPTY
    )  # lidar points away from silhouettes
    stereo_inverse_depth: np.ndarray = field(
        default_factory=lambda: EMPTY
    )  # NaN at stereo discontinuities


def matrix_from(translation: Any, quaternion_xyzw: Any) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    return np.asarray(points @ matrix[:3, :3].T + matrix[:3, 3])


def pose_from_vector(vector: np.ndarray) -> np.ndarray:
    """[rotvec(3), translation(3)] -> 4x4."""
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(vector[:3]).as_matrix()
    matrix[:3, 3] = vector[3:6]
    return matrix


class TransformHistory:
    """A minimal tf buffer: every edge is interpolated in time, chains are walked up to the root."""

    def __init__(self) -> None:
        self._samples: dict[str, list[tuple[float, Any, Any]]] = defaultdict(list)
        self._parent: dict[str, str] = {}
        self._edges: dict[str, tuple[np.ndarray, np.ndarray, Rotation]] = {}

    def add(
        self, parent: str, child: str, time: float, translation: Any, quaternion_xyzw: Any
    ) -> None:
        self._parent[child] = parent
        self._samples[child].append((time, translation, quaternion_xyzw))

    def freeze(self) -> None:
        for child, samples in self._samples.items():
            samples.sort(key=lambda sample: sample[0])
            times = np.array([sample[0] for sample in samples])
            self._edges[child] = (
                times,
                np.array([s[1] for s in samples]),
                Rotation.from_quat([s[2] for s in samples]),
            )

    def _parent_from_child(self, child: str, time: float) -> np.ndarray:
        times, translations, rotations = self._edges[child]
        upper = int(np.clip(np.searchsorted(times, time), 1, max(len(times) - 1, 1)))
        lower = upper - 1 if len(times) > 1 else 0
        span = times[upper] - times[lower] if len(times) > 1 else 0.0
        alpha = float(np.clip((time - times[lower]) / span, 0, 1)) if span > 0 else 0.0
        relative = (rotations[lower].inv() * rotations[upper]).as_rotvec()
        rotation = rotations[lower] * Rotation.from_rotvec(alpha * relative)
        translation = (1 - alpha) * translations[lower] + alpha * translations[upper]
        return matrix_from(translation, rotation.as_quat())

    def root_from(self, frame: str, time: float) -> np.ndarray:
        matrix = np.eye(4)
        while frame in self._parent:
            matrix = self._parent_from_child(frame, time) @ matrix
            frame = self._parent[frame]
        return matrix

    def parent(self, frame: str) -> str:
        return self._parent[frame]

    def time_range(self, frame: str) -> tuple[float, float]:
        times = self._edges[frame][0]
        return float(times[0]), float(times[-1])

    def target_from_source(
        self, target: str, source: str, target_time: float, source_time: float | None = None
    ) -> np.ndarray:
        source_time = target_time if source_time is None else source_time
        target_to_root = np.linalg.inv(self.root_from(target, target_time))
        return np.asarray(target_to_root @ self.root_from(source, source_time))


def stamp_seconds(header: Any) -> float:
    return float(header.stamp.sec + header.stamp.nanosec * 1e-9)


def camera_from_info(info: Any) -> Camera:
    k = np.array(info.k).reshape(3, 3)
    distortion = np.zeros(8)
    # 8 coefficients = rational model, whatever `distortion_model` says.
    distortion[: len(info.d)] = info.d
    return Camera(k[0, 0], k[1, 1], k[0, 2], k[1, 2], distortion, int(info.width), int(info.height))


def load_frames(
    path: Path,
    frame_count: int,
    window_s: float,
    raw_latency_s: float,
    start_s: float,
    end_s: float,
) -> tuple[Camera, tuple[str, np.ndarray], list[Frame]]:
    """Two passes: small topics first to pick frames, then only the images and scans those frames need."""
    import cv2

    # Only the recording reader needs these, so the math above imports without them.
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    decoders: dict[int, Any] = {}
    factory = DecoderFactory()

    def decode(schema: Any, channel: Any, message: Any) -> Any:
        if channel.id not in decoders:
            decoders[channel.id] = factory.decoder_for(channel.message_encoding, schema)
        return decoders[channel.id](message.data)

    transforms = TransformHistory()
    depth_by_time: dict[float, np.ndarray] = {}
    left_camera = depth_camera = None
    with open(path, "rb") as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        assert summary is not None and summary.statistics is not None, f"{path} has no summary"
        recording_start = summary.statistics.message_start_time * 1e-9
        for schema, channel, message in reader.iter_messages(
            topics=["tf", "head_left_info", "head_stereo_depth", "head_stereo_depth_info"]
        ):
            decoded = decode(schema, channel, message)
            if channel.topic == "tf":
                for tf in decoded.transforms:
                    t, q = tf.transform.translation, tf.transform.rotation
                    transforms.add(
                        tf.header.frame_id,
                        tf.child_frame_id,
                        stamp_seconds(tf.header),
                        [t.x, t.y, t.z],
                        [q.x, q.y, q.z, q.w],
                    )
            elif channel.topic == "head_left_info" and left_camera is None:
                left_camera = camera_from_info(decoded)
            elif channel.topic == "head_stereo_depth_info":
                depth_camera = camera_from_info(decoded)
            elif channel.topic == "head_stereo_depth" and decoded.encoding == "32FC1":
                depth = (
                    np.frombuffer(bytes(decoded.data), "<f4")
                    .reshape(decoded.height, decoded.width)
                    .copy()
                )
                depth[~(depth > 0)] = np.nan
                depth_by_time[stamp_seconds(decoded.header)] = depth
        transforms.freeze()
        odom_start, odom_end = transforms.time_range(LIDAR_FRAME)
        candidates = sorted(
            t
            for t in depth_by_time
            if odom_start + window_s < t < odom_end - window_s
            and start_s <= t - recording_start <= end_s
        )
        if not candidates:
            raise SystemExit("no stereo depth frames overlap the Point-LIO odometry")
        picks = np.unique(np.linspace(0, len(candidates) - 1, frame_count).round().astype(int))
        chosen = [candidates[i] for i in picks]
        chosen_set = set(chosen)

        images: dict[float, np.ndarray] = {}
        scans: list[np.ndarray] = []  # rows of [x, y, z, time]
        for schema, channel, message in reader.iter_messages(
            topics=["head_left_color", "lidar_raw"]
        ):
            log_time = message.log_time * 1e-9
            if channel.topic == "lidar_raw":
                # The raw header is on the Livox uptime clock; arrival minus a fixed latency is the scan's end.
                scan_end = log_time - raw_latency_s
                if not any(
                    abs(scan_end - SCAN_PERIOD_S / 2 - t) < window_s + SCAN_PERIOD_S for t in chosen
                ):
                    continue
                cloud = decode(schema, channel, message)
                raw = np.frombuffer(
                    bytes(cloud.data),
                    dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("offset", "<u4")],
                )
                offsets = raw["offset"] * 1e-9
                point_times = scan_end - offsets.max() + offsets
                scans.append(np.c_[raw["x"], raw["y"], raw["z"], point_times])
            elif chosen[0] - 1.0 < log_time < chosen[-1] + 1.0:
                image = decode(schema, channel, message)
                time = stamp_seconds(image.header)
                if time in chosen_set:
                    gray = cv2.imdecode(
                        np.frombuffer(bytes(image.data), np.uint8), cv2.IMREAD_GRAYSCALE
                    )
                    if gray is not None:
                        images[time] = np.asarray(gray)

    if left_camera is None or depth_camera is None:
        raise SystemExit("recording lacks head_left_info or head_stereo_depth_info")
    frames = []
    for time in chosen:
        if time not in images:
            continue
        stamped = np.concatenate(scans) if scans else np.zeros((0, 4))
        stamped = stamped[np.abs(stamped[:, 3] - time) < window_s]
        ranges = np.linalg.norm(stamped[:, :3], axis=1)
        stamped = stamped[(ranges > MIN_RANGE_M) & (ranges < MAX_RANGE_M)]
        frames.append(
            Frame(
                time=time,
                gray=images[time],
                stereo_depth=depth_by_time[time],
                stereo_camera=depth_camera,
                lidar_points=motion_compensate(stamped, transforms, time),
                cam_from_lidar=transforms.target_from_source(CAMERA_FRAME, LIDAR_FRAME, time),
            )
        )
    mount_parent = transforms.parent(CAMERA_FRAME)
    return (
        left_camera,
        (mount_parent, transforms.target_from_source(mount_parent, CAMERA_FRAME, chosen[0])),
        frames,
    )


def motion_compensate(
    stamped: np.ndarray, transforms: TransformHistory, time: float, bins: int = 20
) -> np.ndarray:
    """Move each point from the lidar pose at its own time to the lidar pose at `time`, via odom."""
    if len(stamped) == 0:
        return np.zeros((0, 3))
    edges = np.linspace(stamped[:, 3].min(), stamped[:, 3].max() + 1e-9, bins + 1)
    slot = np.clip(np.digitize(stamped[:, 3], edges) - 1, 0, bins - 1)
    out = np.empty((len(stamped), 3))
    for index in np.unique(slot):
        mask = slot == index
        mid = 0.5 * (edges[index] + edges[index + 1])
        out[mask] = transform_points(
            transforms.target_from_source(LIDAR_FRAME, LIDAR_FRAME, time, mid), stamped[mask, :3]
        )
    return out


def image_edge_distance(gray: np.ndarray) -> np.ndarray:
    """Per-pixel distance (px) to the nearest Canny edge."""
    import cv2

    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 1.5), 40, 120)
    return np.asarray(
        cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 5), np.float32
    )


def in_view(points_cam: np.ndarray, camera: Camera) -> np.ndarray:
    slope = np.linalg.norm(points_cam[:, :2], axis=1) / np.maximum(points_cam[:, 2], 1e-6)
    return np.asarray((points_cam[:, 2] > MIN_RANGE_M) & (slope < camera.max_ray_slope()))


def front_depth_grid(
    cells: tuple[np.ndarray, np.ndarray],
    depth: np.ndarray,
    grid_shape: tuple[int, int],
    empty: float,
) -> np.ndarray:
    """Per image cell, the nearest lidar depth that landed in it; `empty` where none did."""
    nearest = np.full(grid_shape, np.inf, np.float32)
    np.minimum.at(nearest, cells, depth)
    nearest[np.isinf(nearest)] = empty
    return nearest


def split_lidar_points(
    frame: Frame,
    camera: Camera,
    cell_px: int = 6,
    jump: float = 0.3,
    max_points: int = 4000,
    seed: int = 0,
) -> None:
    """Pick, under the prior, lidar points on silhouettes (with their outward image direction) and on clean surfaces."""
    import cv2

    rng = np.random.default_rng(seed)
    points_cam = transform_points(frame.cam_from_lidar, frame.lidar_points)
    visible = in_view(points_cam, camera)
    points, points_cam = frame.lidar_points[visible], points_cam[visible]
    pixels = camera.project(points_cam)
    inside = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < camera.width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < camera.height)
    )
    points, points_cam, pixels = points[inside], points_cam[inside], pixels[inside]
    cells = (pixels[:, 1] // cell_px).astype(int), (pixels[:, 0] // cell_px).astype(int)
    grid_shape = (camera.height // cell_px + 1, camera.width // cell_px + 1)
    # Drop what the lidar sees from its own viewpoint but the camera cannot: points far behind a neighbouring front.
    hidden_front = cv2.erode(
        front_depth_grid(cells, points_cam[:, 2], grid_shape, np.inf), np.ones((3, 3), np.uint8)
    )
    visible = points_cam[:, 2] <= hidden_front[cells] * (1 + jump)
    points, points_cam, cells = (
        points[visible],
        points_cam[visible],
        (cells[0][visible], cells[1][visible]),
    )
    z = points_cam[:, 2]
    # Within two cells: which neighbouring fronts are much farther (their mean offset points out of the silhouette) or nearer.
    padded_far = np.pad(front_depth_grid(cells, z, grid_shape, 0.0), 2)
    padded_near = np.pad(front_depth_grid(cells, z, grid_shape, np.inf), 2, constant_values=np.inf)
    outward, nearer = np.zeros((len(z), 2)), np.zeros(len(z), bool)
    for dy in range(-2, 3):
        for dx in range(-2, 3):
            far = padded_far[cells[0] + 2 + dy, cells[1] + 2 + dx] > z * (1 + jump)
            outward += far[:, None] * np.array([dx, dy])
            nearer |= padded_near[cells[0] + 2 + dy, cells[1] + 2 + dx] < z / (1 + jump)
    length = np.linalg.norm(outward, axis=1)
    edge = rng.permutation(np.flatnonzero((length > 0) & ~nearer))[:max_points]
    surface = rng.permutation(np.flatnonzero((length == 0) & ~nearer & (z < STEREO_MAX_RANGE_M)))[
        :max_points
    ]
    frame.edge_points, frame.edge_outward = points[edge], outward[edge] / length[edge, None]
    frame.surface_points = points[surface]
    frame.edge_distance = image_edge_distance(frame.gray)
    frame.stereo_inverse_depth = smooth_inverse_depth(frame.stereo_depth)


def smooth_inverse_depth(depth: np.ndarray, jump: float = 0.1) -> np.ndarray:
    """1/depth, NaN next to a stereo discontinuity so bilinear lookups never blend two surfaces."""
    import cv2

    inverse = (1.0 / depth).astype(np.float32)
    filled = np.nan_to_num(inverse, nan=0.0)
    spread = cv2.dilate(filled, np.ones((3, 3), np.uint8)) - cv2.erode(
        np.nan_to_num(inverse, nan=np.inf), np.ones((3, 3), np.uint8)
    )
    inverse[~(spread <= jump * inverse)] = np.nan
    return inverse


def sample(image: np.ndarray, pixels: np.ndarray, fill: float) -> np.ndarray:
    """Bilinear lookup at (u, v); `fill` off the image or where a neighbour is NaN."""
    return np.asarray(
        map_coordinates(image, [pixels[:, 1], pixels[:, 0]], order=1, mode="constant", cval=fill)
    )


def edge_residuals(
    frame: Frame, camera: Camera, cam_from_lidar: np.ndarray, inset_px: float = 0.0
) -> np.ndarray:
    """Image-edge distance at each silhouette point, pushed `inset_px` outward (the last lidar hit lies inside the true outline)."""
    pixels = (
        camera.project(transform_points(cam_from_lidar, frame.edge_points))
        + inset_px * frame.edge_outward
    )
    return np.asarray(np.minimum(sample(frame.edge_distance, pixels, EDGE_CLIP_PX), EDGE_CLIP_PX))


def stereo_residuals(
    frame: Frame, cam_from_lidar: np.ndarray, stereo_gain: float, stereo_offset: float
) -> np.ndarray:
    """Lidar inverse depth minus (corrected) stereo inverse depth; 0 where the stereo has no depth."""
    # Only as many as there are silhouette points, so the smoother, denser stereo term cannot drown out the edges.
    points_cam = transform_points(
        cam_from_lidar, frame.surface_points[: max(len(frame.edge_points), 200)]
    )
    inverse_stereo = sample(
        frame.stereo_inverse_depth, frame.stereo_camera.project(points_cam), np.nan
    )
    residual = 1.0 / points_cam[:, 2] - (stereo_gain * inverse_stereo + stereo_offset)
    return np.nan_to_num(residual, nan=0.0)


def unpack(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    """[rotvec, translation] correction in the camera frame, [dfx, dfy, dcx, dcy], stereo inverse-depth gain and offset, silhouette inset."""
    return (
        pose_from_vector(parameters[:6]),
        parameters[6:10],
        parameters[10],
        parameters[11],
        parameters[12],
    )


def all_residuals(parameters: np.ndarray, frames: list[Frame], camera: Camera) -> np.ndarray:
    correction, intrinsic_offsets, gain, offset, inset = unpack(parameters)
    refined_camera = camera.with_offsets(intrinsic_offsets)
    blocks = []
    for frame in frames:
        cam_from_lidar = correction @ frame.cam_from_lidar
        blocks.append(edge_residuals(frame, refined_camera, cam_from_lidar, inset) / EDGE_SIGMA_PX)
        blocks.append(stereo_residuals(frame, cam_from_lidar, gain, offset) / INVERSE_DEPTH_SIGMA)
    return np.concatenate(blocks)


def robust_cost(parameters: np.ndarray, frames: list[Frame], camera: Camera) -> float:
    return float(np.log1p(all_residuals(parameters, frames, camera) ** 2).sum())


def coarse_search(
    parameters: np.ndarray,
    frames: list[Frame],
    camera: Camera,
    rotation_span_deg: float = 6.0,
    translation_span_m: float = 0.06,
) -> np.ndarray:
    """Coordinate descent over the 6 pose terms on shrinking grids; the edge cost is too rough for least squares to start far off."""
    parameters = parameters.copy()
    for shrink in (1.0, 0.25, 0.0625):
        for axis in range(6):
            span = shrink * (np.radians(rotation_span_deg) if axis < 3 else translation_span_m)
            candidates = parameters[axis] + np.linspace(-span, span, 13)
            costs = []
            for value in candidates:
                trial = parameters.copy()
                trial[axis] = value
                costs.append(robust_cost(trial, frames, camera))
            parameters[axis] = candidates[int(np.argmin(costs))]
    return parameters


def refine(frames: list[Frame], camera: Camera, fit_intrinsics: bool = True) -> np.ndarray:
    """Robust least squares over the correction, intrinsics, stereo nuisance terms and silhouette inset."""
    initial = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0.0, 0.0])
    free = np.ones(len(initial), bool)
    free[6:10] = fit_intrinsics

    def residuals(x: np.ndarray) -> np.ndarray:
        full = initial.copy()
        full[free] = x
        return all_residuals(full, frames, camera)

    initial = coarse_search(initial, frames, camera)
    steps = np.array([1e-4] * 3 + [1e-3] * 3 + [0.5] * 4 + [1e-3, 1e-4, 0.5])[free]
    result = least_squares(residuals, initial[free], x_scale=steps, loss="cauchy", f_scale=1.0)
    parameters = initial.copy()
    parameters[free] = result.x
    return parameters


def summarize(parameters: np.ndarray, camera: Camera) -> dict[str, Any]:
    correction, intrinsic_offsets, gain, offset, inset = unpack(parameters)
    refined = camera.with_offsets(intrinsic_offsets)
    return {
        "rotation_deg": [round(float(v), 4) for v in np.degrees(parameters[:3])],
        "translation_m": [round(float(v), 4) for v in parameters[3:6]],
        **{name: round(float(getattr(refined, name)), 2) for name in ("fx", "fy", "cx", "cy")},
        "stereo_gain": round(float(gain), 4),
    }


def evaluate(
    frames: list[Frame],
    camera: Camera,
    correction: np.ndarray,
    gain: float = 1.0,
    offset: float = 0.0,
) -> dict[str, Any]:
    """Median |lidar - stereo| depth at shared stereo pixels, and image-edge distance of lidar silhouette points."""
    depth_errors, edge_distances = [], []
    for frame in frames:
        cam_from_lidar = correction @ frame.cam_from_lidar
        edge_distances.append(edge_residuals(frame, camera, cam_from_lidar))
        points_cam = transform_points(cam_from_lidar, frame.surface_points)
        stereo = 1.0 / (
            gain
            / sample(frame.stereo_depth, frame.stereo_camera.project(points_cam).round(), np.nan)
            + offset
        )
        valid = np.isfinite(stereo)
        depth_errors.append(np.abs(points_cam[valid, 2] - stereo[valid]))
    depth_errors, edge_distances = np.concatenate(depth_errors), np.concatenate(edge_distances)
    # A recording without stereo depth has nothing to cross-check; the edge metric still applies.
    has_depth = len(depth_errors) > 0
    return {
        "median_abs_depth_error_m": float(np.median(depth_errors)) if has_depth else None,
        "depth_error_p90_m": float(np.percentile(depth_errors, 90)) if has_depth else None,
        "depth_pixels": len(depth_errors),
        "median_edge_distance_px": float(np.median(edge_distances)),
        "edge_within_3px_fraction": float(np.mean(edge_distances <= 3)),
        "edge_points": len(edge_distances),
    }


def describe_pose(matrix: np.ndarray) -> dict[str, Any]:
    rotation = Rotation.from_matrix(matrix[:3, :3])
    return {
        "translation_m": [round(float(v), 5) for v in matrix[:3, 3]],
        "quaternion_xyzw": [round(float(v), 6) for v in rotation.as_quat()],
        "rpy_rad": [round(float(v), 5) for v in rotation.as_euler("xyz")],
    }


def render_overlay(
    frames: list[Frame],
    camera: Camera,
    refined_camera: Camera,
    correction: np.ndarray,
    path: Path,
    count: int = 3,
) -> None:
    """Each row: lidar coloured by depth over the image, prior on the left, refined on the right."""
    import cv2

    rows = []
    for frame in frames[:: max(len(frames) // count, 1)][:count]:
        panels = []
        for cam, cam_from_lidar in (
            (camera, frame.cam_from_lidar),
            (refined_camera, correction @ frame.cam_from_lidar),
        ):
            points_cam = transform_points(cam_from_lidar, frame.lidar_points)
            points_cam = points_cam[in_view(points_cam, cam)]
            pixels = cam.project(points_cam).astype(int)
            canvas = cv2.cvtColor(frame.gray, cv2.COLOR_GRAY2BGR)
            canvas[frame.edge_distance < 1] = (0, 255, 0)
            hue = np.clip(np.log(points_cam[:, 2]) / np.log(MAX_RANGE_M) * 170, 0, 170).astype(
                np.uint8
            )
            colors = cv2.cvtColor(
                np.stack([hue, np.full_like(hue, 255), np.full_like(hue, 255)], 1)[None],
                cv2.COLOR_HSV2BGR,
            )[0]
            for (u, v), color in zip(pixels, colors, strict=False):
                cv2.circle(canvas, (int(u), int(v)), 2, tuple(int(c) for c in color), -1)
            panels.append(cv2.resize(canvas, (camera.width // 2, camera.height // 2)))
        rows.append(np.hstack(panels))
    cv2.imwrite(str(path), np.vstack(rows))


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", type=Path)
    parser.add_argument(
        "--frames", type=int, default=45, help="frames sampled evenly across the recording"
    )
    parser.add_argument(
        "--holdout-every",
        type=int,
        default=3,
        help="every Nth sampled frame is held out of the fit",
    )
    parser.add_argument(
        "--window", type=float, default=0.3, help="seconds of lidar either side of each image"
    )
    parser.add_argument(
        "--raw-latency", type=float, default=0.05, help="lidar_raw arrival minus scan end, seconds"
    )
    parser.add_argument(
        "--start", type=float, default=0.0, help="skip this many seconds from the start"
    )
    parser.add_argument(
        "--end",
        type=float,
        default=np.inf,
        help="ignore frames after this many seconds (e.g. odom blow-up)",
    )
    parser.add_argument(
        "--extrinsic-only", action="store_true", help="keep fx, fy, cx, cy at the recorded values"
    )
    parser.add_argument("--output", type=Path, default=Path("head_camera_calibration.yaml"))
    parser.add_argument(
        "--overlay", type=Path, help="PNG of lidar over held-out images, before | after"
    )
    args = parser.parse_args()

    camera, (mount_parent, parent_from_camera), frames = load_frames(
        args.recording, args.frames, args.window, args.raw_latency, args.start, args.end
    )
    for frame in frames:
        split_lidar_points(frame, camera)
    frames = [f for f in frames if len(f.edge_points) > 50 and len(f.surface_points) > 50]
    holdout = frames[:: args.holdout_every]
    training = [f for i, f in enumerate(frames) if i % args.holdout_every]
    if len(training) < 3 or not holdout:
        raise SystemExit(f"only {len(frames)} usable frames; need lidar + stereo + tf overlap")
    print(f"{len(training)} training / {len(holdout)} held-out frames")

    fit_intrinsics = not args.extrinsic_only
    parameters = refine(training, camera, fit_intrinsics)
    correction, intrinsic_offsets, gain, offset, inset = unpack(parameters)
    refined = camera.with_offsets(intrinsic_offsets)
    reference = holdout[len(holdout) // 2]
    report = {
        "recording": str(args.recording),
        "frames": {"training": len(training), "held_out": len(holdout)},
        "intrinsics": {
            "width": camera.width,
            "height": camera.height,
            "K": [round(float(v), 4) for v in refined.matrix.ravel()],
            "D_rational_unchanged": [float(v) for v in camera.distortion],
            "change_px": {
                k: round(float(v), 3)
                for k, v in zip(("fx", "fy", "cx", "cy"), intrinsic_offsets, strict=True)
            },
        },
        "camera_from_lidar": {
            "prior": describe_pose(reference.cam_from_lidar),
            "refined": describe_pose(correction @ reference.cam_from_lidar),
            "correction_in_camera_frame": {
                **describe_pose(correction),
                "angle_deg": round(float(np.degrees(np.linalg.norm(parameters[:3]))), 4),
            },
            f"{mount_parent}_from_{CAMERA_FRAME}": {
                "prior": describe_pose(parent_from_camera),
                "refined": describe_pose(parent_from_camera @ np.linalg.inv(correction)),
            },
            "note": f"prior/refined at t={reference.time:.3f} (torso joints move it); the correction C is constant and blamed on the camera mount",
        },
        "stereo_check": {
            "inverse_depth_gain": round(float(gain), 5),
            "inverse_depth_offset_per_m": round(float(offset), 6),
            "implied_disparity_offset_px": round(float(offset * refined.fx * STEREO_BASELINE_M), 3),
            "note": "1/Z_lidar = gain/Z_stereo + offset; gain != 1 is an fx*baseline scale error, the offset a right-camera yaw error",
        },
        "silhouette_inset_px": round(float(inset), 3),
        "split_half_consistency": [
            summarize(refine(training[half::2], camera, fit_intrinsics), camera) for half in (0, 1)
        ],
        "held_out_metrics": {
            "before": evaluate(holdout, camera, np.eye(4)),
            "after": evaluate(holdout, refined, correction),
            "after_with_stereo_correction": evaluate(holdout, refined, correction, gain, offset),
        },
    }
    args.output.write_text(yaml.safe_dump(report, sort_keys=False))
    print(yaml.safe_dump(report, sort_keys=False))
    if args.overlay:
        render_overlay(holdout, camera, refined, correction, args.overlay)
        print(f"overlay: {args.overlay}")


if __name__ == "__main__":
    main()
