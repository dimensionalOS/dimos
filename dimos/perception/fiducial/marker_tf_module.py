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

"""ArUco marker detection with distorted-camera pose and TF publication.

Default dictionary aligns with ``dimos apriltag`` family ``tag36h11``.

Publishes ``world -> markers`` (identity) and ``markers -> marker_{id}`` so composed
lookups match marker poses in ``world``. Requires ``CameraInfo`` (``plumb_bob`` or
empty distortion supported best; refine intrinsics on hardware when needed).
Camera calibration runbook: ``docs/usage/camera_calibration.md``.

The pose chain is ``base_link -> <optical> -> marker`` where ``<optical>`` is
``Image.frame_id`` when set, else ``CameraInfo.frame_id``, else ``camera_optical``.
That matches the frame the pixels live in. Then ``world -> base_link`` is applied
before publishing the marker namespace.

OpenCV 4.7+ uses ``ArucoDetector``; pose uses ``solvePnP`` (``estimatePoseSingleMarkers``
was removed in newer OpenCV builds).

Compose with a camera publisher (e.g. Go2) via matching stream names::

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.perception.fiducial.marker_tf_module import MarkerTfModule
    from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

    unitree_go2_with_markers = autoconnect(
        unitree_go2_basic,
        MarkerTfModule.blueprint(marker_length_m=0.18),
    )
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.spec.perception import Camera
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

try:
    import cv2.aruco
except (ImportError, AttributeError) as e:
    raise ImportError(
        "dimos.perception.fiducial requires cv2.aruco. Install with: "
        "uv sync --inexact --extra apriltag"
    ) from e

logger = setup_logger()

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy


def camera_info_to_cv_matrices(camera_info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    """Build OpenCV ``cameraMatrix`` and ``distCoeffs`` from ``CameraInfo``."""
    k = np.array(camera_info.K, dtype=np.float64).reshape(3, 3)
    d = np.array(camera_info.D if camera_info.D else [], dtype=np.float64).reshape(-1, 1)
    return k, d


def _camera_optical_frame_id(image: Image, camera_info: CameraInfo) -> str:
    """Frame in which image pixels and intrinsics apply (optical convention in ROS).

    Prefer ``Image.frame_id`` so TF lookups match the stream that produced the
    pixels. Fall back to ``CameraInfo.frame_id``, then a conventional default.
    """
    for fid in (image.frame_id, camera_info.frame_id):
        if fid and fid.strip():
            return fid.strip()
    return "camera_optical"


def _aruco_marker_object_points(marker_length_m: float) -> np.ndarray:
    """Corner order matches OpenCV ArUco / solvePnP convention (planar square, Z=0)."""
    h = marker_length_m / 2.0
    return np.array(
        [
            [-h, h, 0.0],
            [h, h, 0.0],
            [h, -h, 0.0],
            [-h, -h, 0.0],
        ],
        dtype=np.float32,
    )


def estimate_marker_pose(
    corners_px: np.ndarray,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Return ``(rvec, tvec)`` for camera optical <- marker from undistorted solvePnP."""
    obj = _aruco_marker_object_points(marker_length_m)
    img = corners_px.reshape(4, 1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None
    return rvec, tvec


def rvec_tvec_to_transform(
    rvec: np.ndarray,
    tvec: np.ndarray,
    *,
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> Transform:
    """Build ``Transform`` for ``frame_id`` <- ``child_frame_id`` (camera <- marker)."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    quat = Quaternion.from_rotation_matrix(rot_mat)
    tx, ty, tz = tvec.reshape(3)
    return Transform(
        translation=Vector3(float(tx), float(ty), float(tz)),
        rotation=quat,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        ts=ts,
    )


def create_aruco_detector(dictionary_name: str) -> cv2.aruco.ArucoDetector:
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary {dictionary_name!r}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)


class MarkerTfModuleConfig(ModuleConfig):
    """Configuration for :class:`MarkerTfModule`.

    ``marker_length_m`` is the physical edge length of the printed square marker
    in meters (required; no default).
    """

    world_frame: str = "world"
    base_frame: str = "base_link"
    markers_frame: str = "markers"
    marker_namespace_prefix: str | None = None
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    marker_length_m: float = Field(
        ..., gt=0.0, description="Physical square marker edge length in meters."
    )
    max_freq: float = 15.0
    tf_lookup_tolerance: float = 0.5


class MarkerTfModule(Module):
    """Subscribe to ``color_image`` + ``camera_info``, publish marker poses on ``self.tf``."""

    config: MarkerTfModuleConfig

    color_image: In[Image]
    camera_info: In[CameraInfo]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._latest_camera_info: CameraInfo | None = None
        self._warned_distortion_model = False
        self._detector = create_aruco_detector(self.config.aruco_dictionary)

    def _markers_parent_frame(self) -> str:
        p = self.config.marker_namespace_prefix
        base = self.config.markers_frame
        return f"{p}/{base}" if p else base

    def _marker_child_frame(self, marker_id: int) -> str:
        p = self.config.marker_namespace_prefix
        name = f"marker_{marker_id}"
        return f"{p}/{name}" if p else name

    def _maybe_warn_distortion(self, camera_info: CameraInfo) -> None:
        model = (camera_info.distortion_model or "").strip().lower()
        if model in ("", "plumb_bob"):
            return
        if not self._warned_distortion_model:
            logger.warning(
                "MarkerTfModule: distortion_model=%r may be unsupported; using D as-is.",
                camera_info.distortion_model,
            )
            self._warned_distortion_model = True

    def _process_color_image(self, image: Image) -> None:
        info = self._latest_camera_info
        if info is None:
            logger.debug("MarkerTfModule: no CameraInfo yet; skipping frame")
            return

        self._maybe_warn_distortion(info)

        h, w = image.height, image.width
        if info.height and info.width and (info.height != h or info.width != w):
            logger.debug(
                "MarkerTfModule: image size %sx%s != CameraInfo %sx%s; skip",
                w,
                h,
                info.width,
                info.height,
            )
            return

        gray = image.to_grayscale().as_numpy()
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        cam_mtx, dist = camera_info_to_cv_matrices(info)
        optical = _camera_optical_frame_id(image, info)
        t_world_base = self.tf.get(
            self.config.world_frame,
            self.config.base_frame,
            image.ts,
            self.config.tf_lookup_tolerance,
        )
        if t_world_base is None:
            logger.debug(
                "MarkerTfModule: no TF %s -> %s at ts=%s",
                self.config.world_frame,
                self.config.base_frame,
                image.ts,
            )
            return

        t_base_optical = self.tf.get(
            self.config.base_frame,
            optical,
            image.ts,
            self.config.tf_lookup_tolerance,
        )
        if t_base_optical is None:
            logger.debug(
                "MarkerTfModule: no TF %s -> %s at ts=%s",
                self.config.base_frame,
                optical,
                image.ts,
            )
            return

        markers_parent = self._markers_parent_frame()
        ts = image.ts
        out: list[Transform] = [
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id=self.config.world_frame,
                child_frame_id=markers_parent,
                ts=ts,
            )
        ]

        for corner_set, mid_arr in zip(corners, ids, strict=True):
            mid = int(mid_arr[0])
            pose = estimate_marker_pose(
                corner_set,
                self.config.marker_length_m,
                cam_mtx,
                dist,
            )
            if pose is None:
                continue
            rvec, tvec = pose
            t_optical_marker = rvec_tvec_to_transform(
                rvec,
                tvec,
                frame_id=optical,
                child_frame_id="__marker_tmp",
                ts=ts,
            )
            t_base_marker = t_base_optical + t_optical_marker
            t_world_marker = t_world_base + t_base_marker
            out.append(
                Transform(
                    translation=t_world_marker.translation,
                    rotation=t_world_marker.rotation,
                    frame_id=markers_parent,
                    child_frame_id=self._marker_child_frame(mid),
                    ts=ts,
                )
            )

        if len(out) > 1:
            self.tf.publish(*out)

    @simple_mcache
    def sharp_image_stream(self) -> Observable[Image]:
        return backpressure(
            self.color_image.pure_observable().pipe(
                sharpness_barrier(self.config.max_freq),
            )
        )

    @rpc
    def start(self) -> None:
        super().start()

        def on_camera_info(msg: CameraInfo) -> None:
            self._latest_camera_info = msg

        unsub_info = self.camera_info.subscribe(on_camera_info)
        self.register_disposable(Disposable(unsub_info) if callable(unsub_info) else unsub_info)
        self.register_disposable(self.sharp_image_stream().subscribe(self._process_color_image))

    @rpc
    def stop(self) -> None:
        super().stop()


def deploy(
    dimos: ModuleCoordinator,
    camera: Camera,
    prefix: str = "/marker_tf",
    **kwargs: Any,
) -> ModuleProxy:
    """Wire :class:`MarkerTfModule` inputs from a :class:`~dimos.spec.perception.Camera`.

    Registers the module via :meth:`~dimos.core.coordination.module_coordinator.ModuleCoordinator.deploy`
    so lifecycle and deployment routing match other perception modules.

    ``prefix`` maps to :attr:`MarkerTfModuleConfig.marker_namespace_prefix` (leading ``/`` stripped)
    unless ``marker_namespace_prefix`` is passed explicitly in ``kwargs``.
    """
    deploy_kwargs = dict(kwargs)
    if "marker_namespace_prefix" not in deploy_kwargs:
        stripped = prefix.lstrip("/")
        deploy_kwargs["marker_namespace_prefix"] = stripped if stripped else None

    mod = dimos.deploy(MarkerTfModule, **deploy_kwargs)
    mod.color_image.connect(camera.color_image)
    mod.camera_info.connect(camera.camera_info)
    mod.start()
    return mod
