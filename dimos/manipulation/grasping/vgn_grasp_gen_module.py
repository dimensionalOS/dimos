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

from __future__ import annotations

from collections.abc import Sequence
import importlib.util
import os
from pathlib import Path
import time
from typing import Protocol, cast

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseArray import PoseArray
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.grasping_msgs.TargetBounds import TargetBounds
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.reconstruction.tsdf_debug_export import export_tsdf_debug_files
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VGNGraspGenModuleConfig(ModuleConfig):
    model_path: str | None = None
    model_path_env: str = "DIMOS_VGN_MODEL_PATH"
    output_frame: str = "world"
    default_jaw_width: float = 0.08
    quality_threshold: float = 0.90
    width_filter_min_voxels: float = 1.33
    width_filter_max_voxels: float = 9.33
    filter_candidates_to_target_bounds: bool = True
    min_score: float = 0.0
    auto_generate_on_tsdf: bool = False
    auto_generate_min_interval: float = 1.0
    debug_export_dir: str | None = None


class _VGNDetector(Protocol):
    def __call__(self, state: _VGNState) -> tuple[Sequence[object], Sequence[float], float]: ...


class _TorchNetwork(Protocol):
    def __call__(self, tensor: object) -> tuple[object, object, object]: ...


class _TorchTensor(Protocol):
    def cpu(self) -> _TorchTensor: ...

    def squeeze(self) -> _TorchTensor: ...

    def numpy(self) -> np.ndarray: ...


class _VGNTSDFAdapter:
    def __init__(self, tsdf: TSDFGrid) -> None:
        self.voxel_size = tsdf.voxel_size
        self._grid = tsdf.distances

    def get_grid(self) -> np.ndarray:
        return self._grid


class _VGNState:
    def __init__(self, tsdf: TSDFGrid) -> None:
        self.tsdf = _VGNTSDFAdapter(tsdf)


class VGNGraspGenModule(Module):
    """Generate grasp candidates from TSDF grids using the VGN backend."""

    dedicated_worker = True

    config: VGNGraspGenModuleConfig

    tsdf: In[TSDFGrid]

    grasp_candidates: Out[GraspCandidateArray]
    grasp_poses: Out[PoseArray]
    grasp_target_bounds: Out[TargetBounds]
    target_masked_tsdf: Out[TSDFGrid]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._latest_tsdf: TSDFGrid | None = None
        self._detector: _VGNDetector | None = None
        self._last_auto_generation_ts: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.tsdf.subscribe(self._on_tsdf)))

    @rpc
    def generate_grasps_from_tsdf(self, tsdf: TSDFGrid) -> GraspCandidateArray | None:
        """Generate grasp candidates from an explicit TSDF grid."""
        result = self._generate_grasps_from_tsdf(tsdf)
        if result is not None:
            self.grasp_candidates.publish(result)
            self.grasp_poses.publish(result.to_pose_array())
        return result

    def _generate_grasps_from_tsdf(self, tsdf: TSDFGrid) -> GraspCandidateArray | None:
        self._validate_tsdf(tsdf)
        try:
            grasps, scores, _toc = self._get_detector()(_VGNState(tsdf))
        except Exception as exc:
            raise RuntimeError(f"VGN grasp generation failed: {exc}") from exc
        return self._convert_grasps(tsdf, grasps, scores)

    @rpc
    def generate_latest_grasps(self) -> str:
        """Generate grasp candidates from the latest TSDF received on the stream."""
        if self._latest_tsdf is None:
            empty = _make_candidate_array([], self.config.output_frame)
            self.grasp_candidates.publish(empty)
            return "No TSDF available for grasp generation"
        result = self.generate_grasps_from_tsdf(self._latest_tsdf)
        if result is None:
            return "Failed to generate world-frame grasp candidates"
        if len(result) == 0:
            return "VGN returned no grasp candidates"
        return f"Generated {len(result)} grasp candidates"

    @rpc
    def generate_grasps_for_target_bounds(
        self,
        target_center: Vector3,
        target_size: Vector3,
        target_frame_id: str,
        target_ts: float,
        cushion_m: float = 0.03,
    ) -> GraspCandidateArray | None:
        """Generate candidates from the latest TSDF constrained to target bounds."""
        if self._latest_tsdf is None:
            empty = _make_candidate_array([], self.config.output_frame)
            self.grasp_candidates.publish(empty)
            return None

        bounds = TargetBounds(
            center=target_center,
            size=target_size,
            frame_id=target_frame_id,
            ts=target_ts,
            label="grasp target",
        )
        masked_tsdf, bounds_in_tsdf = self._target_masked_tsdf(
            self._latest_tsdf,
            bounds,
            cushion_m,
        )
        if masked_tsdf is None or bounds_in_tsdf is None:
            empty = _make_candidate_array([], self.config.output_frame, self._latest_tsdf.ts)
            self.grasp_candidates.publish(empty)
            return None

        self.grasp_target_bounds.publish(bounds_in_tsdf)
        self.target_masked_tsdf.publish(masked_tsdf)
        self._export_debug_tsdfs(self._latest_tsdf, masked_tsdf)
        result = self._generate_grasps_from_tsdf(masked_tsdf)
        if result is None:
            return None
        filtered = (
            self._filter_candidates_to_bounds(result, bounds, cushion_m)
            if self.config.filter_candidates_to_target_bounds
            else result
        )
        self.grasp_candidates.publish(filtered)
        self.grasp_poses.publish(filtered.to_pose_array())
        return filtered

    def _on_tsdf(self, tsdf: TSDFGrid) -> None:
        self._latest_tsdf = tsdf
        if not self.config.auto_generate_on_tsdf:
            return
        if not self._should_auto_generate(tsdf.ts):
            return
        self._last_auto_generation_ts = tsdf.ts
        try:
            self.generate_grasps_from_tsdf(tsdf)
        except RuntimeError as exc:
            logger.warning("Automatic VGN generation failed: %s", exc)
            empty = _make_candidate_array([], self.config.output_frame, tsdf.ts)
            self.grasp_candidates.publish(empty)

    def _export_debug_tsdfs(self, full_tsdf: TSDFGrid, masked_tsdf: TSDFGrid) -> None:
        if self.config.debug_export_dir is None:
            return
        ts = f"{time.time():.3f}".replace(".", "_")
        try:
            full_paths = export_tsdf_debug_files(
                full_tsdf, self.config.debug_export_dir, f"{ts}_full_tsdf"
            )
            masked_paths = export_tsdf_debug_files(
                masked_tsdf, self.config.debug_export_dir, f"{ts}_target_masked_tsdf"
            )
        except Exception as exc:
            logger.warning("Failed to export TSDF debug files", error=str(exc))
            return
        logger.info(
            "Exported TSDF debug files",
            full=[str(path) for path in full_paths],
            masked=[str(path) for path in masked_paths],
        )

    def _should_auto_generate(self, ts: float) -> bool:
        if self._last_auto_generation_ts is None:
            return True
        return ts - self._last_auto_generation_ts >= self.config.auto_generate_min_interval

    def _validate_tsdf(self, tsdf: TSDFGrid) -> None:
        if tsdf.distances.shape != (1, 40, 40, 40):
            raise ValueError(f"VGN expects TSDF shape (1, 40, 40, 40), got {tsdf.distances.shape}")
        if tsdf.voxel_size <= 0:
            raise ValueError("TSDF voxel_size must be positive")

    def _get_detector(self) -> _VGNDetector:
        if self._detector is None:
            model_path = self._resolve_model_path()
            if importlib.util.find_spec("vgn") is None:
                raise RuntimeError(
                    "VGN is not installed. Install the grasp optional dependencies "
                    "with `uv sync --extra grasp` before using VGNGraspGenModule."
                )
            try:
                from vgn.detection import VGN  # type: ignore[import-not-found]

                self._detector = cast("_VGNDetector", VGN(model_path, rviz=False))
            except ModuleNotFoundError as exc:
                if exc.name not in {"rospy", "sensor_msgs", "visualization_msgs"}:
                    raise RuntimeError(
                        "VGN is not installed. Install the grasp optional dependencies "
                        "with `uv sync --extra grasp` before using VGNGraspGenModule."
                    ) from exc
                logger.warning(
                    "VGN ROS visualization dependency %s is unavailable; using headless VGN detector",
                    exc.name,
                )
                self._detector = _HeadlessVGNDetector(
                    model_path,
                    quality_threshold=self.config.quality_threshold,
                    width_filter_min_voxels=self.config.width_filter_min_voxels,
                    width_filter_max_voxels=self.config.width_filter_max_voxels,
                )
        return self._detector

    def _resolve_model_path(self) -> str:
        model_path = self.config.model_path or os.environ.get(self.config.model_path_env)
        if not model_path:
            raise RuntimeError(
                f"VGN model path is required. Set `{self.config.model_path_env}` "
                "or pass `model_path=` to VGNGraspGenModule.blueprint()."
            )
        path = Path(model_path).expanduser()
        if not path.exists():
            raise RuntimeError(f"VGN model file does not exist: {path}")
        return str(path)

    def _convert_grasps(
        self,
        tsdf: TSDFGrid,
        grasps: Sequence[object],
        scores: Sequence[float],
    ) -> GraspCandidateArray | None:
        if len(grasps) == 0:
            return _make_candidate_array([], self.config.output_frame, tsdf.ts)

        output_from_target = self._output_from_target_matrix(tsdf)
        if output_from_target is None:
            logger.warning(
                "Cannot transform VGN grasps from %s to %s",
                tsdf.frame_id,
                self.config.output_frame,
            )
            return None

        candidates: list[GraspCandidate] = []
        origin = np.array([tsdf.origin.x, tsdf.origin.y, tsdf.origin.z], dtype=np.float64)
        for index, grasp in enumerate(grasps):
            score = float(scores[index]) if index < len(scores) else 0.0
            if score < self.config.min_score:
                continue
            target_from_grasp = _matrix_from_vgn_grasp(grasp)
            target_from_grasp[:3, 3] += origin
            output_from_grasp = output_from_target @ target_from_grasp
            candidates.append(
                GraspCandidate(
                    pose=_pose_from_matrix(output_from_grasp),
                    jaw_width=_jaw_width(grasp, self.config.default_jaw_width),
                    score=score,
                    id=f"vgn-{index}",
                )
            )

        return _make_candidate_array(candidates, self.config.output_frame, tsdf.ts)

    def _output_from_target_matrix(self, tsdf: TSDFGrid) -> np.ndarray | None:
        if tsdf.frame_id == self.config.output_frame:
            return np.eye(4, dtype=np.float64)
        transform = self.tf.get(self.config.output_frame, tsdf.frame_id, tsdf.ts, 0.1)
        if transform is None:
            return None
        return transform.to_matrix().astype(np.float64)

    def _target_masked_tsdf(
        self,
        tsdf: TSDFGrid,
        bounds: TargetBounds,
        cushion_m: float,
    ) -> tuple[TSDFGrid | None, TargetBounds | None]:
        tsdf_from_bounds = self._frame_transform_matrix(tsdf.frame_id, bounds.frame_id, bounds.ts)
        if tsdf_from_bounds is None:
            logger.warning(
                "Cannot transform target bounds from %s to %s", bounds.frame_id, tsdf.frame_id
            )
            return None, None

        expanded = bounds.expanded(cushion_m)
        min_corner, max_corner = _transformed_aabb(expanded, tsdf_from_bounds)
        centers = _voxel_positions(tsdf)
        inside = np.logical_and(centers >= min_corner, centers <= max_corner).all(axis=-1)
        masked_distances = np.array(tsdf.distances, copy=True)
        masked_distances[0, np.logical_not(inside)] = 1.0
        masked_weights = np.array(tsdf.weights, copy=True) if tsdf.weights is not None else None
        if masked_weights is not None:
            if masked_weights.ndim == 4:
                masked_weights[0, np.logical_not(inside)] = 0.0
            else:
                masked_weights[np.logical_not(inside)] = 0.0

        masked_tsdf = TSDFGrid(
            distances=masked_distances,
            voxel_size=tsdf.voxel_size,
            truncation_distance=tsdf.truncation_distance,
            origin=tsdf.origin,
            weights=masked_weights,
            frame_id=tsdf.frame_id,
            ts=tsdf.ts,
        )
        bounds_center = (min_corner + max_corner) / 2.0
        bounds_size = max_corner - min_corner
        return masked_tsdf, TargetBounds(
            center=Vector3(bounds_center),
            size=Vector3(bounds_size),
            frame_id=tsdf.frame_id,
            ts=tsdf.ts,
            label=expanded.label,
        )

    def _frame_transform_matrix(
        self,
        parent_frame: str,
        child_frame: str,
        ts: float,
    ) -> np.ndarray | None:
        if parent_frame == child_frame:
            return np.eye(4, dtype=np.float64)
        transform = self.tf.get(parent_frame, child_frame, ts, 0.1)
        if transform is None:
            return None
        return transform.to_matrix().astype(np.float64)

    def _filter_candidates_to_bounds(
        self,
        candidates: GraspCandidateArray,
        bounds: TargetBounds,
        cushion_m: float,
    ) -> GraspCandidateArray:
        output_from_bounds = self._frame_transform_matrix(
            self.config.output_frame,
            bounds.frame_id,
            bounds.ts,
        )
        if output_from_bounds is None:
            return candidates
        min_corner, max_corner = _transformed_aabb(bounds.expanded(cushion_m), output_from_bounds)
        filtered = [
            candidate
            for candidate in candidates
            if _point_inside_bounds(candidate.pose.position, min_corner, max_corner)
        ]
        return _make_candidate_array(filtered, candidates.frame_id, candidates.ts)


class _HeadlessVGNDetector:
    """Run pinned VGN inference without importing ROS-backed `vgn.vis`."""

    def __init__(
        self,
        model_path: str,
        *,
        quality_threshold: float = 0.90,
        width_filter_min_voxels: float = 1.33,
        width_filter_max_voxels: float = 9.33,
    ) -> None:
        import torch  # type: ignore[import-not-found]
        from vgn.networks import get_network  # type: ignore[import-not-found]

        self._torch = torch
        self._device = _select_torch_device(torch)
        self._net = _load_vgn_network(torch, get_network, Path(model_path), self._device)
        self._quality_threshold = quality_threshold
        self._width_filter_min_voxels = width_filter_min_voxels
        self._width_filter_max_voxels = width_filter_max_voxels
        logger.info("Loaded headless VGN detector", device=str(self._device))

    def __call__(self, state: _VGNState) -> tuple[Sequence[object], Sequence[float], float]:
        from scipy import ndimage  # type: ignore[import-not-found]
        from vgn.grasp import Grasp, from_voxel_coordinates  # type: ignore[import-not-found]
        from vgn.utils.transform import Rotation, Transform  # type: ignore[import-not-found]

        tsdf_volume = state.tsdf.get_grid()
        voxel_size = state.tsdf.voxel_size
        started_at = time.time()

        quality_volume, rotation_volume, width_volume = self._predict(tsdf_volume)
        quality_volume = ndimage.gaussian_filter(quality_volume, sigma=1.0, mode="nearest")

        squeezed_tsdf = tsdf_volume.squeeze()
        outside_voxels = squeezed_tsdf > 0.5
        inside_voxels = np.logical_and(1e-3 < squeezed_tsdf, squeezed_tsdf < 0.5)
        valid_voxels = ndimage.binary_dilation(
            outside_voxels,
            iterations=2,
            mask=np.logical_not(inside_voxels),
        )
        quality_volume[np.logical_not(valid_voxels)] = 0.0
        quality_volume[
            np.logical_or(
                width_volume < self._width_filter_min_voxels,
                width_volume > self._width_filter_max_voxels,
            )
        ] = 0.0

        logger.info(
            "VGN quality stats before threshold",
            max_quality=float(quality_volume.max()) if quality_volume.size else 0.0,
            threshold=self._quality_threshold,
            width_min=float(width_volume.min()) if width_volume.size else 0.0,
            width_max=float(width_volume.max()) if width_volume.size else 0.0,
        )
        quality_volume[quality_volume < self._quality_threshold] = 0.0
        max_volume = ndimage.maximum_filter(quality_volume, size=4)
        quality_volume = np.where(quality_volume == max_volume, quality_volume, 0.0)

        grasps: list[object] = []
        scores: list[float] = []
        for index in np.argwhere(np.where(quality_volume, 1.0, 0.0)):
            i, j, k = index
            score = float(quality_volume[i, j, k])
            rotation = Rotation.from_quat(rotation_volume[:, i, j, k])
            translation = np.array([i, j, k], dtype=np.float64)
            grasp = Grasp(Transform(rotation, translation), float(width_volume[i, j, k]))
            grasps.append(from_voxel_coordinates(grasp, voxel_size))
            scores.append(score)

        return grasps, scores, time.time() - started_at

    def _predict(self, tsdf_volume: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if tsdf_volume.shape != (1, 40, 40, 40):
            raise ValueError(f"VGN expects TSDF shape (1, 40, 40, 40), got {tsdf_volume.shape}")

        tsdf_tensor = self._torch.from_numpy(tsdf_volume).unsqueeze(0).to(self._device)
        with self._torch.no_grad():
            quality, rotation, width = cast("_TorchNetwork", self._net)(tsdf_tensor)
        quality_tensor = cast("_TorchTensor", quality)
        rotation_tensor = cast("_TorchTensor", rotation)
        width_tensor = cast("_TorchTensor", width)
        return (
            quality_tensor.cpu().squeeze().numpy(),
            rotation_tensor.cpu().squeeze().numpy(),
            width_tensor.cpu().squeeze().numpy(),
        )


def _matrix_from_vgn_grasp(grasp: object) -> np.ndarray:
    pose = getattr(grasp, "pose", grasp)
    if isinstance(pose, np.ndarray):
        return _validate_matrix(pose)
    as_matrix = getattr(pose, "as_matrix", None)
    if callable(as_matrix):
        return _validate_matrix(np.asarray(as_matrix(), dtype=np.float64))

    rotation = getattr(pose, "rotation", None)
    translation = getattr(pose, "translation", None)
    if rotation is None or translation is None:
        raise ValueError(f"Unsupported VGN grasp pose type: {type(pose).__name__}")

    rotation_matrix = _rotation_matrix(rotation)
    translation_vector = _translation_vector(translation)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation_matrix
    matrix[:3, 3] = translation_vector
    return matrix


def _validate_matrix(matrix: np.ndarray) -> np.ndarray:
    if matrix.shape != (4, 4):
        raise ValueError(f"VGN grasp matrix must be 4x4, got {matrix.shape}")
    return matrix.astype(np.float64)


def _rotation_matrix(rotation: object) -> np.ndarray:
    as_matrix = getattr(rotation, "as_matrix", None)
    if callable(as_matrix):
        matrix = np.asarray(as_matrix(), dtype=np.float64)
    else:
        matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f"VGN grasp rotation must be 3x3, got {matrix.shape}")
    return matrix


def _translation_vector(translation: object) -> np.ndarray:
    vector = np.asarray(translation, dtype=np.float64)
    if vector.shape != (3,):
        raise ValueError(f"VGN grasp translation must have shape (3,), got {vector.shape}")
    return vector


def _pose_from_matrix(matrix: np.ndarray) -> Pose:
    pose = Pose()  # type: ignore[call-arg]
    pose.position = Vector3(matrix[:3, 3])
    pose.orientation = Quaternion.from_rotation_matrix(matrix[:3, :3])
    return pose


def _make_candidate_array(
    candidates: list[GraspCandidate],
    frame_id: str,
    ts: float | None = None,
) -> GraspCandidateArray:
    timestamp = ts if ts is not None else time.time()
    header = Header(frame_id)
    sec = int(timestamp)
    header.stamp.sec = sec
    header.stamp.nsec = int((timestamp - sec) * 1_000_000_000)
    header.frame_id = frame_id
    return GraspCandidateArray(
        header=header,
        candidates=candidates,
    )


def _voxel_positions(tsdf: TSDFGrid) -> np.ndarray:
    x, y, z = tsdf.resolution
    indices = np.indices((x, y, z), dtype=np.float64)
    origin = np.array([tsdf.origin.x, tsdf.origin.y, tsdf.origin.z], dtype=np.float64)
    return np.moveaxis(indices, 0, -1) * tsdf.voxel_size + origin


def _transformed_aabb(bounds: TargetBounds, transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = np.array([bounds.center.x, bounds.center.y, bounds.center.z], dtype=np.float64)
    half = np.array([bounds.size.x, bounds.size.y, bounds.size.z], dtype=np.float64) / 2.0
    corners = np.array(
        [
            [center[0] + sx * half[0], center[1] + sy * half[1], center[2] + sz * half[2], 1.0]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    transformed = (transform @ corners.T).T[:, :3]
    return transformed.min(axis=0), transformed.max(axis=0)


def _point_inside_bounds(point: Vector3, min_corner: np.ndarray, max_corner: np.ndarray) -> bool:
    values = np.array([point.x, point.y, point.z], dtype=np.float64)
    return bool(np.logical_and(values >= min_corner, values <= max_corner).all())


def _select_torch_device(torch: object) -> object:
    cuda = torch.cuda  # type: ignore[attr-defined]
    if not cuda.is_available() or cuda.device_count() <= 0:
        return torch.device("cpu")  # type: ignore[attr-defined]
    try:
        torch.empty(1, device="cuda:0")  # type: ignore[attr-defined]
        cuda.synchronize()
    except Exception as exc:
        logger.warning(
            "CUDA is reported available but failed a smoke allocation; using CPU", error=str(exc)
        )
        return torch.device("cpu")  # type: ignore[attr-defined]
    return torch.device("cuda:0")  # type: ignore[attr-defined]


def _load_vgn_network(
    torch: object,
    get_network: object,
    model_path: Path,
    device: object,
) -> object:
    model_name = model_path.stem.split("_")[1]
    net = get_network(model_name)  # type: ignore[operator]
    state_dict = torch.load(model_path, map_location="cpu")  # type: ignore[attr-defined]
    net.load_state_dict(state_dict)
    net = net.to(device)
    net.eval()
    return net


def _jaw_width(grasp: object, default_width: float) -> float:
    width = getattr(grasp, "width", default_width)
    if isinstance(width, int | float | np.floating):
        return float(width)
    return float(default_width)
