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

"""Experimental SixNG RGB-D adapter for the existing Sim2 CameraRenderer seam."""

from collections.abc import Callable
from dataclasses import dataclass
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import threading
import time
from typing import Any, cast

import cbor2
from dimos_lcm.sim_msgs import WorldState
import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
import trimesh

from dimos.experimental.gta_sim2.world_objects import ObjectStateStore
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.assets import resolve_usd_assets
from dimos.sim2.description import scene_revision
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.geometry.poses import poses_by_id
from dimos.sim2.geometry.query import MujocoGeometry
from dimos.sim2.sensors.camera.config import CameraConfig

RAGE_TO_RENDER = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)


def projection(camera: CameraConfig, near: float = 0.01, far: float = 5000) -> NDArray[np.float64]:
    """WGPU reversed-Z projection, including asymmetric focal lengths and centre."""
    return np.array(
        [
            [2 * camera.fx / camera.width, 0, 1 - 2 * camera.cx / camera.width, 0],
            [0, 2 * camera.fy / camera.height, 2 * camera.cy / camera.height - 1, 0],
            [0, 0, near / (far - near), far * near / (far - near)],
            [0, 0, -1, 0],
        ]
    )


def renderer_transform(
    position: NDArray[np.float64], rotation: NDArray[np.float64], origin: NDArray[np.float64]
) -> NDArray[np.float64]:
    matrix = np.eye(4)
    matrix[:3, :3] = RAGE_TO_RENDER @ rotation
    matrix[:3, 3] = RAGE_TO_RENDER @ (position + origin)
    return matrix


def object_visual(state: dict[str, Any], origin: NDArray[np.float64]) -> dict[str, Any]:
    """Convert a physics pivot pose into the authored GLB model's placement."""
    visual = state["visual"]
    position = np.asarray(state["position"], dtype=np.float64)
    quaternion = np.asarray(state["quaternion_wxyz"], dtype=np.float64)
    pivot = np.asarray(visual["pivot"], dtype=np.float64)
    scale = float(visual["scale"])
    if (
        position.shape != (3,)
        or pivot.shape != (3,)
        or quaternion.shape != (4,)
        or not np.isfinite(np.r_[position, pivot, quaternion, scale]).all()
        or scale <= 0
        or not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-5)
    ):
        raise ValueError("Invalid dynamic object pose/scale")
    rotation = Rotation.from_quat(quaternion, scalar_first=True).as_matrix()
    matrix = np.eye(4)
    matrix[:3, :3] = RAGE_TO_RENDER @ rotation @ RAGE_TO_RENDER.T * scale
    matrix[:3, 3] = RAGE_TO_RENDER @ (position + origin - rotation @ (pivot * scale))
    return {
        "id": state["identity"],
        "source_map": visual["source_map"],
        "source_model": visual["source_model"],
        "source_position": visual["source_position"],
        "transform": matrix.T.ravel().tolist(),
    }


@dataclass(frozen=True)
class ObserverOutput:
    """Independent browser frames; these are never published as sensor acquisitions."""

    directory: Path
    control_port: int
    width: int = 960
    height: int = 540


class SixNGRenderer:
    """Own a separate renderer process and match each response to its acquisition.

    Depth covers opaque and alpha-tested map/robot surfaces. Transparent water
    and glass returns are not simulated. Static map visuals are not collision
    meshes, so optical and contact coverage need separate validation.
    """

    def __init__(
        self,
        executable: Path,
        game: Path,
        work: Path,
        runtime: Path,
        origin: tuple[float, float, float],
        gpu: str | None = None,
        observer: ObserverOutput | None = None,
        object_store: Path | None = None,
    ) -> None:
        self.executable, self.game, self.work, self.runtime = executable, game, work, runtime
        self.origin = np.asarray(origin, dtype=np.float64)
        self._gpu = gpu
        self._observer = observer
        self._object_store = object_store
        self._saved_objects: dict[str, dict[str, Any]] = {}
        self._objects_read_at = 0.0
        self._objects_center = np.full(3, np.inf)
        self._process: subprocess.Popen[bytes] | None = None
        self._temporary: tempfile.TemporaryDirectory[str] | None = None
        self._log: Any = None
        self._cancelled = threading.Event()
        self._sequence = 0
        self._cameras: dict[str, CameraConfig] = {}
        self._actor_ids: list[str] = []

    def load(self, scene: UsdModel, cameras: tuple[CameraConfig, ...]) -> None:
        if (
            self._process is not None
            or not cameras
            or len({c.frame_id for c in cameras}) != len(cameras)
        ):
            raise ValueError("Renderer already loaded or invalid camera set")
        self.runtime.mkdir(parents=True, exist_ok=True)
        self._temporary = tempfile.TemporaryDirectory(prefix="sensor-", dir=self.runtime)
        self.directory = Path(self._temporary.name)
        self._cancelled.clear()
        try:
            self.geometry = MujocoGeometry(
                build_scene(scene, kinematic=True).compile(), scene_revision(scene)
            )
            if not {c.frame_id for c in cameras} <= set(self.geometry.body_ids):
                raise ValueError("Camera frames missing from composed scene")
            self._cameras = {c.frame_id: c for c in cameras}
            self._export_actors()
            root = str(scene.source_path.parent) if scene.source_path else ""
            (self.runtime / "composed.usda").write_text(resolve_usd_assets(scene.usd, root))
            env = dict(
                os.environ,
                SIXNG_GAME=str(self.game),
                SIXNG_WORK=str(self.work),
                SIXNG_SIM2_DIR=str(self.directory),
                SIXNG_MAP_NO_CELLS="1",
                SIXNG_MAP_BATCHES="1",
                RAYON_NUM_THREADS="8",
                WGPU_BACKEND="vulkan",
            )
            env.pop("SIXNG_WEB_DIR", None)
            env.pop("SIXNG_SIM2_OBSERVER", None)
            if self._observer:
                env.update(
                    SIXNG_WEB_DIR=str(self._observer.directory),
                    SIXNG_WEB_CONTROL_PORT=str(self._observer.control_port),
                    SIXNG_SIM2_OBSERVER="1",
                )
            self._log = (self.runtime / "sixng.log").open("wb")
            self._process = subprocess.Popen(
                [
                    str(self.executable),
                    "--tab",
                    "map3d",
                    *(["--gpu", self._gpu] if self._gpu else []),
                    "--size",
                    f"{self._observer.width}x{self._observer.height}"
                    if self._observer
                    else "640x480",
                    "--center",
                    f"{self.origin[0]},{self.origin[1]}",
                    "--height",
                    "2",
                    "--quality",
                    "high",
                    "--max-tex",
                    "1024",
                    "--ram-cap",
                    "20",
                    "--tod",
                    "14",
                    "--no-terrain",
                    "--no-windows",
                    "--no-vsync",
                ],
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except BaseException:
            self.close()
            raise

    def _export_actors(self) -> None:
        model = self.geometry.model
        assets = []
        for body in range(1, model.nbody):
            parts = []
            for geom in np.flatnonzero(model.geom_bodyid == body):
                if (
                    model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH
                    or model.geom_group[geom] >= 3
                ):
                    continue
                mesh = int(model.geom_dataid[geom])
                va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                fa, fn = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                part = trimesh.Trimesh(
                    vertices=model.mesh_vert[va : va + vn].copy(),
                    faces=model.mesh_face[fa : fa + fn].copy(),
                    process=False,
                )
                transform = np.eye(4)
                transform[:3, :3] = Rotation.from_quat(
                    model.geom_quat[geom], scalar_first=True
                ).as_matrix()
                transform[:3, 3] = model.geom_pos[geom]
                part.apply_transform(transform)
                part.visual = trimesh.visual.ColorVisuals(
                    mesh=part,
                    face_colors=np.tile(
                        (np.clip(model.geom_rgba[geom], 0, 1) * 255).astype(np.uint8),
                        (len(part.faces), 1),
                    ),
                )
                parts.append(part)
            if parts:
                asset = self.directory / f"body-{body}.glb"
                export_glb = cast(
                    "Callable[[trimesh.Scene], bytes]", trimesh.exchange.gltf.export_glb
                )
                asset.write_bytes(export_glb(trimesh.Scene(parts)))
                name = model.body(body).name
                self._actor_ids.append(name)
                assets.append({"id": name, "path": str(asset)})
        (self.directory / "assets.json").write_text(json.dumps(assets))

    def _make_request(self, sample: WorldState, name: str) -> dict[str, Any]:
        poses = poses_by_id(sample)
        try:
            self.geometry.set_poses(sample)
        except ValueError as error:
            raise ValueError(
                f"Observer scene mismatch: sample revision={sample.model_revision}, "
                f"observer revision={self.geometry.revision}, "
                f"missing={set(self.geometry.body_ids) - set(poses)}, "
                f"extra={set(poses) - set(self.geometry.body_ids)}"
            ) from error
        actors = []
        for body in self._actor_ids:
            p = poses[body]
            rotation = Rotation.from_quat(
                [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            ).as_matrix()
            matrix = renderer_transform(
                np.array([p.position.x, p.position.y, p.position.z]), rotation, self.origin
            )
            actors.append({"id": body, "transform": matrix.T.ravel().tolist()})
        camera = self._cameras[name]
        p = poses[name]
        rotation = Rotation.from_quat(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        ).as_matrix()
        objects: dict[str, dict[str, Any]] = {}
        if self._object_store is not None:
            center = np.array([p.position.x, p.position.y, p.position.z])
            if self._observer:
                try:
                    status = json.loads((self._observer.directory / "status.json").read_text())
                    center = np.asarray(status["position"], dtype=np.float64) - self.origin
                except (OSError, ValueError, KeyError):
                    pass
            now = time.monotonic()
            if (
                now - self._objects_read_at > 0.25
                or np.linalg.norm(center - self._objects_center) > 10
            ):
                self._saved_objects = ObjectStateStore.visual_nearby(
                    self._object_store, center, 750
                )
                self._objects_read_at, self._objects_center = now, center.copy()
            objects.update(self._saved_objects)
        for entry in sample.cbor_observations:
            if entry.entity_id == "world" and entry.key == "gta/objects":
                values = cbor2.loads(bytes(entry.value))["objects"]
                for value in values:
                    objects[value["identity"]] = value
        if len(objects) > 4096:
            raise MemoryError("Visible dynamic object budget exceeded")
        self._sequence += 1
        return {
            "version": 1,
            "id": self._sequence,
            "size": [camera.width, camera.height],
            "clock_id": sample.header.clock_id,
            "epoch": sample.header.epoch,
            "time_ns": sample.header.time_ns,
            "model_revision": sample.model_revision,
            "eye": (
                RAGE_TO_RENDER
                @ (np.array([p.position.x, p.position.y, p.position.z]) + self.origin)
            ).tolist(),
            "forward": (RAGE_TO_RENDER @ rotation[:, 2]).tolist(),
            "up": (-RAGE_TO_RENDER @ rotation[:, 1]).tolist(),
            "projection": projection(camera).T.ravel().tolist(),
            "actors": actors,
            "objects": [object_visual(value, self.origin) for value in objects.values()],
        }

    def _send(self, request: dict[str, Any]) -> None:
        if self._process is None or self._process.poll() is not None:
            raise RuntimeError(f"SixNG exited; see {self.runtime / 'sixng.log'}")
        error = self.directory / "error.txt"
        if error.exists():
            raise RuntimeError(error.read_text())
        pending = self.directory / "request.tmp"
        pending.write_text(json.dumps(request))
        pending.replace(self.directory / "request.json")

    def publish_observer(self, sample: WorldState, focus_body: str) -> None:
        """Publish latest poses without blocking physics on browser frame readback."""
        if self._observer is None:
            raise RuntimeError("Observer output is not configured")
        request = self._make_request(sample, next(iter(self._cameras)))
        p = poses_by_id(sample)[focus_body].position
        request["focus"] = (RAGE_TO_RENDER @ (np.array([p.x, p.y, p.z]) + self.origin)).tolist()
        self._send(request)

    def render(
        self, sample: WorldState, camera_ids: tuple[str, ...]
    ) -> dict[str, tuple[NDArray[np.uint8], NDArray[np.float32]]]:
        if self._observer is not None:
            raise RuntimeError("Observer frames cannot be used as sensor acquisitions")
        if len(set(camera_ids)) != len(camera_ids) or not set(camera_ids) <= self._cameras.keys():
            raise ValueError("Unknown or duplicate camera IDs")
        images = {}
        for name in camera_ids:
            camera = self._cameras[name]
            self._send(self._make_request(sample, name))
            pixels = self._receive(60 if self._sequence == 1 else 10)
            _, sequence, width, height = struct.unpack_from("<8sQII", pixels)
            if (
                pixels[:8] != b"SIM2RGBD"
                or sequence != self._sequence
                or (width, height) != (camera.width, camera.height)
                or len(pixels) != 24 + width * height * 8
            ):
                raise ValueError("Renderer returned invalid acquisition or dimensions")
            color = (
                np.frombuffer(pixels, np.uint8, width * height * 4, 24)
                .reshape(height, width, 4)[..., :3]
                .copy()
            )
            depth = (
                np.frombuffer(pixels, "<f4", width * height, 24 + width * height * 4)
                .reshape(height, width)
                .copy()
            )
            images[name] = color, depth
        return images

    def _receive(self, timeout: float) -> bytes:
        end = time.monotonic() + timeout
        while not self._cancelled.is_set() and time.monotonic() < end:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError(f"SixNG exited; see {self.runtime / 'sixng.log'}")
            error = self.directory / "error.txt"
            if error.exists():
                raise RuntimeError(error.read_text())
            response = self.directory / "response.bin"
            if response.exists():
                pixels = response.read_bytes()
                if len(pixels) >= 24 and struct.unpack_from("<Q", pixels, 8)[0] == self._sequence:
                    return pixels
            self._cancelled.wait(0.002)
        raise TimeoutError("SixNG capture cancelled or deadline exceeded")

    def cancel(self) -> None:
        self._cancelled.set()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()

    def close(self) -> None:
        self.cancel()
        if self._process is not None:
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=5)
            self._process = None
        if self._log is not None:
            self._log.close()
            self._log = None
        if self._temporary is not None:
            self._temporary.cleanup()
            self._temporary = None
        self._actor_ids.clear()
        self._cameras.clear()
        self._saved_objects.clear()
        self._objects_read_at = 0.0
