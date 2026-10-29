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

"""One native Unreal camera worker attached to the authoritative physics world."""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import queue
import tempfile
import threading
import time
from typing import Any
from uuid import uuid4

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.messages import FrameMetadata, StateFrame
from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.reader import WorldReader
from dimos.sim2.sensors.spec import Mount
from dimos.sim2.unreal.assets import camera_parameters, camera_pose, export_scene, geom_poses
from dimos.sim2.unreal.client import RenderedFrame, RendererProcess
from dimos.sim2.world.spec import SimulationSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class UnrealCameraConfig(ModuleConfig):
    """Executable and one camera binding, independent of the robot definition."""

    executable: Path
    project: Path | None = None
    robot_id: str | None = None
    sensor: Camera
    startup_timeout: float = Field(default=180.0, gt=0, allow_inf_nan=False)
    capture_timeout: float = Field(default=15.0, gt=0, allow_inf_nan=False)
    log_path: Path | None = None
    offscreen: bool = True

    @model_validator(mode="after")
    def validate_camera(self) -> UnrealCameraConfig:
        sensor = self.sensor
        if not 0 < sensor.width <= 4096 or not 0 < sensor.height <= 4096:
            raise ValueError("Unreal image dimensions must be between 1 and 4096")
        if sensor.fovy is not None and not 0 < sensor.fovy < 180:
            raise ValueError("camera fovy must be between 0 and 180 degrees")
        if sensor.latency:
            raise ValueError("Unreal camera latency is measured; modeled latency is not supported")
        if not sensor.depth or sensor.pointcloud or not sensor.shadows:
            raise ValueError(
                "Unreal camera currently produces RGB-D with shadows and no pointcloud"
            )
        return self


def camera_view(
    reader: WorldReader, sensor: Camera, robot_id: str | None
) -> tuple[dict[str, Any], NDArray[np.float64], NDArray[np.float64]]:
    """Resolve the existing camera or mount into calibration and optical TF."""
    prefix = reader.model_prefix(robot_id)
    if isinstance(sensor.camera, str):
        index = reader.model.camera(prefix + sensor.camera).id
        calibration = camera_parameters(
            reader.model, reader.data, index, sensor.width, sensor.height
        )
        position = reader.data.cam_xpos[index].copy()
        rotation = reader.data.cam_xmat[index].reshape(3, 3).copy()
    else:
        mount: Mount = sensor.camera
        if robot_id is None and mount.link == "world":
            origin = np.zeros(3)
            parent = np.eye(3)
        else:
            name = prefix + mount.link
            try:
                index = reader.model.body(name).id
                origin = reader.data.xpos[index]
                parent = reader.data.xmat[index].reshape(3, 3)
            except KeyError:
                index = reader.model.site(name).id
                origin = reader.data.site_xpos[index]
                parent = reader.data.site_xmat[index].reshape(3, 3)
        position = origin + parent @ np.asarray(mount.xyz)
        rotation = parent @ Rotation.from_euler("xyz", mount.rpy).as_matrix()
        focal = sensor.height / (2 * math.tan(math.radians(sensor.fovy or 60.0) / 2))
        calibration = {
            **camera_pose(position, rotation),
            "width": sensor.width,
            "height": sensor.height,
            "fx": focal,
            "fy": focal,
            "cx": (sensor.width - 1) / 2,
            "cy": (sensor.height - 1) / 2,
        }
    optical = Rotation.from_matrix(rotation @ np.diag([1, -1, -1])).as_quat()
    return calibration, position, optical


def frame_messages(
    rendered: RenderedFrame,
    calibration: dict[str, Any],
    position: NDArray[np.float64],
    optical: NDArray[np.float64],
    frame_id: str,
) -> tuple[Image, Image, CameraInfo, TFMessage]:
    """Publish aligned channels with acquisition time, never delivery time."""
    timestamp = float(rendered.header["timestamp"])
    color = Image(
        data=np.ascontiguousarray(rendered.rgba[:, :, :3]),
        format=ImageFormat.RGB,
        frame_id=frame_id,
        ts=timestamp,
    )
    depth = Image(data=rendered.depth, format=ImageFormat.DEPTH, frame_id=frame_id, ts=timestamp)
    info = CameraInfo.from_intrinsics(
        calibration["fx"],
        calibration["fy"],
        calibration["cx"],
        calibration["cy"],
        calibration["width"],
        calibration["height"],
        frame_id=frame_id,
    )
    info.ts = timestamp
    transform = Transform(
        translation=Vector3(*position),
        rotation=Quaternion(*optical),
        frame_id="world",
        child_frame_id=frame_id,
        ts=timestamp,
    )
    return color, depth, info, TFMessage(transform)


class UnrealCameraModule(Module):
    """Capture snapshots on a separate worker; never advance or block physics."""

    config: UnrealCameraConfig
    dedicated_worker = True
    _simulation: SimulationSpec
    snapshots: In[StateFrame]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    depth_camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    frame_metadata: Out[FrameMetadata]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self._ready: queue.Queue[Exception | None] = queue.Queue(maxsize=1)
        self._process: RendererProcess | None = None
        self._status: dict[str, Any] = {
            "running": False,
            "frames": 0,
            "discarded": 0,
            "error": None,
        }
        self.reader: WorldReader

    @rpc
    def start(self) -> None:
        super().start()
        self.reader = WorldReader(self._simulation.describe())
        self.register_disposable(Disposable(self.snapshots.subscribe(self._on_snapshot)))
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="unreal-camera", daemon=True)
        self._thread.start()
        error = self._ready.get(timeout=self.config.startup_timeout + 10)
        if error is not None:
            raise error

    def _on_snapshot(self, snapshot: StateFrame) -> None:
        # Serialize reset notification with the final publication check.
        with self._lock:
            self.reader.push(snapshot)

    def _publish_frame(
        self,
        metadata: FrameMetadata,
        rendered: RenderedFrame,
        calibration: dict[str, Any],
        position: NDArray[np.float64],
        optical: NDArray[np.float64],
    ) -> bool:
        frame_id = f"{self.config.robot_id or 'world'}/{self.config.sensor.name}_optical"
        color, depth, info, transforms = frame_messages(
            rendered,
            calibration,
            position,
            optical,
            frame_id,
        )
        with self._lock:
            if self._stop.is_set() or self.reader.latest_episode != metadata.episode_id:
                self._status["discarded"] += 1
                return False
            self.color_image.publish(color)
            self.depth_image.publish(depth)
            self.camera_info.publish(info)
            self.depth_camera_info.publish(info)
            self.tf.publish(transforms)
            self.frame_metadata.publish(metadata)
            self._status.update(
                frames=self._status["frames"] + 1,
                metadata=asdict(metadata),
                capture_ms=rendered.header["capture_ms"],
                roundtrip_ms=1000 * rendered.roundtrip_seconds,
                acquisition_age_ms=1000 * (time.time() - rendered.header["timestamp"]),
            )
            return True

    def _run(self) -> None:
        ready = False
        try:
            log = self.config.log_path or STATE_DIR / "unreal" / f"{uuid4().hex}.log"
            process = RendererProcess(
                self.config.executable,
                log,
                project=self.config.project,
                startup_timeout=self.config.startup_timeout,
                capture_timeout=self.config.capture_timeout,
                offscreen=self.config.offscreen,
            )
            with self._lock:
                self._process = process
                self._status["log_path"] = str(log)
                if self._stop.is_set():
                    process.cancel()
            with (
                tempfile.TemporaryDirectory(prefix="dimos-unreal-") as directory,
                process as client,
            ):
                with self._lock:
                    self._status["running"] = True
                model: Any = None
                episode = -1
                geom_ids: list[int] = []
                last_tick = -1
                sensor = self.config.sensor
                while not self._stop.is_set():
                    started = time.monotonic()
                    if not self.reader.update():
                        self._stop.wait(0.005)
                        continue
                    metadata = self.reader.metadata
                    assert metadata is not None
                    if self.reader.model is not model:
                        manifest = export_scene(self.reader.model, self.reader.data)
                        path = Path(directory) / "scene.json"
                        path.write_text(
                            json.dumps(manifest, allow_nan=False, separators=(",", ":"))
                        )
                        client.load(path)
                        geom_ids = [geom["id"] for geom in manifest["geoms"]]
                        model = self.reader.model
                        with self._lock:
                            self._status.update(geoms=len(geom_ids), meshes=len(manifest["meshes"]))
                    if episode != metadata.episode_id:
                        client.reset(metadata.episode_id)
                        episode = metadata.episode_id
                        last_tick = -1
                    if metadata.physics_tick == last_tick:
                        self._stop.wait(0.005)
                        continue
                    calibration, position, optical = camera_view(
                        self.reader, sensor, self.config.robot_id
                    )
                    rendered = client.capture(
                        {
                            "episode": metadata.episode_id,
                            "sequence": metadata.sequence,
                            "physics_tick": metadata.physics_tick,
                            "sim_time": metadata.sim_time,
                            "timestamp": self.reader.timestamp,
                            "poses": geom_poses(self.reader.data, geom_ids),
                            "camera": calibration,
                        }
                    )
                    last_tick = metadata.physics_tick
                    # Reset can arrive while GPU work is in flight. That response
                    # is valid for its old snapshot, but must never reach the new run.
                    published = self._publish_frame(
                        metadata, rendered, calibration, position, optical
                    )
                    if published and not ready:
                        self._ready.put(None)
                        ready = True
                    self._stop.wait(max(0.0, 1 / sensor.rate_hz - (time.monotonic() - started)))
        except Exception as error:
            if not self._stop.is_set():
                with self._lock:
                    self._status["error"] = f"{type(error).__name__}: {error}"
                logger.exception("Unreal camera worker failed", error=str(error))
            if not ready:
                self._ready.put(error)
        finally:
            with self._lock:
                self._status["running"] = False
                self._process = None

    @rpc
    def renderer_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    @rpc
    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            process = self._process
        if process is not None:
            process.cancel()
        if self._thread is not None:
            self._thread.join(timeout=15)
            if self._thread.is_alive():
                raise RuntimeError("Unreal camera worker did not stop")
        super().stop()
