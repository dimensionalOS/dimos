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

"""Render synchronized, occluded measurements for each robot on a private scene copy."""

import math
import threading
import time
from collections.abc import Callable
from copy import copy

import mujoco
import numpy as np
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.pollen.microduck.sim_module import LIDAR_CAMERA_SPECS
from dimos.simulation.engines.mujoco_engine import camera_ray_directions
from dimos.utils.logging_config import setup_logger
from microduck_world.camera import HEAD_CAMERA
from microduck_world.football import Scoreboard
from microduck_world.robot_io import RobotVision
from numpy.typing import NDArray

logger = setup_logger()
SENSOR_SIZE = (640, 360)
SENSOR_FPS = 3.0
LIDAR_SIZE = (96, 48)


class RobotSensors:
    def __init__(self, model: mujoco.MjModel, publish: Callable[[str, RobotVision], None]) -> None:
        self._source = model
        self._publish = publish
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._snapshot: (
            tuple[NDArray[np.float64], dict[str, tuple[str, tuple[float, float]]], list[int]] | None
        ) = None
        self._thread = threading.Thread(target=self._run, name="robot-sensors", daemon=True)
        self._error: str | None = None

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def start(self) -> None:
        self._thread.start()

    def snapshot(
        self,
        qpos: NDArray[np.float64],
        assignments: dict[str, tuple[str, tuple[float, float]]],
        lit: list[int] | None = None,
    ) -> None:
        with self._lock:
            self._snapshot = (qpos.copy(), dict(assignments), list(lit or []))

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            raise RuntimeError("Robot sensor renderer did not stop")

    def _run(self) -> None:
        renderer: mujoco.Renderer | None = None
        try:
            model = copy(self._source)
            scoreboard = Scoreboard(model)
            data = mujoco.MjData(model)
            renderer = mujoco.Renderer(model, height=SENSOR_SIZE[1], width=SENSOR_SIZE[0])
            option = mujoco.MjvOption()
            option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
            directions = camera_ray_directions(*LIDAR_SIZE, 140.0)
            while not self._stop.is_set():
                started = time.monotonic()
                with self._lock:
                    current = self._snapshot
                if current is None:
                    self._stop.wait(0.05)
                    continue
                qpos, assignments, lit = current
                scoreboard.apply(model, lit)
                data.qpos[:] = qpos
                mujoco.mj_forward(model, data)
                for id, (generation, origin) in assignments.items():
                    if self._stop.is_set():
                        break
                    prefix = "" if id == "duck1" else id + "_"
                    camera = model.camera(prefix + HEAD_CAMERA).id
                    ts = time.time()
                    offset = np.array([*origin, 0.0])
                    position = data.cam_xpos[camera].copy() - offset
                    optical = data.cam_xmat[camera].reshape(3, 3) @ np.diag([1, -1, -1])
                    orientation = Quaternion.from_rotation_matrix(optical)
                    pose = PoseStamped(
                        position=position.tolist(), orientation=orientation, frame_id="world", ts=ts
                    )
                    width, height = SENSOR_SIZE
                    focal = height / (2 * math.tan(math.radians(float(model.cam_fovy[camera])) / 2))
                    info = CameraInfo.from_intrinsics(
                        focal,
                        focal,
                        width / 2,
                        height / 2,
                        width,
                        height,
                        frame_id="camera_optical",
                    )
                    info.ts = ts
                    renderer.update_scene(data, camera=camera, scene_option=option)
                    image = renderer.render().copy()
                    renderer.enable_depth_rendering()
                    depth = renderer.render().copy()
                    renderer.disable_depth_rendering()
                    points = self._range_points(model, data, prefix, directions) - offset
                    tf = TFMessage(
                        Transform(
                            translation=Vector3(*position),
                            rotation=orientation,
                            frame_id="world",
                            child_frame_id="camera_optical",
                            ts=ts,
                        )
                    )
                    self._publish(
                        id,
                        RobotVision(
                            generation,
                            Image(
                                data=image, format=ImageFormat.RGB, frame_id="camera_optical", ts=ts
                            ),
                            Image(
                                data=depth,
                                format=ImageFormat.DEPTH,
                                frame_id="camera_optical",
                                ts=ts,
                            ),
                            info,
                            pose,
                            tf,
                            points.astype(np.float32),
                        ),
                    )
                self._stop.wait(max(0.001, 1 / SENSOR_FPS - (time.monotonic() - started)))
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            logger.exception("Robot sensor pipeline failed")
        finally:
            if renderer is not None:
                renderer.close()

    @staticmethod
    def _range_points(
        model: mujoco.MjModel, data: mujoco.MjData, prefix: str, directions: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        root = model.body(prefix + "trunk_base").id
        own = []
        for geom in range(model.ngeom):
            ancestor = int(model.geom_bodyid[geom])
            while ancestor and ancestor != root:
                ancestor = int(model.body_parentid[ancestor])
            if ancestor == root:
                own.append(geom)
        groups = model.geom_group[own].copy()
        model.geom_group[own] = 5
        hits = []
        try:
            for name, _ in LIDAR_CAMERA_SPECS:
                camera = model.camera(prefix + name).id
                origin = data.cam_xpos[camera].copy()
                rays = directions @ data.cam_xmat[camera].reshape(3, 3).T
                geom_ids = np.full(len(rays), -1, dtype=np.int32)
                distance = np.full(len(rays), -1.0, dtype=np.float64)
                mujoco.mj_multiRay(
                    model,
                    data,
                    origin,
                    rays.ravel(),
                    np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8),
                    1,
                    -1,
                    geom_ids,
                    distance,
                    None,
                    len(rays),
                    6.0,
                )
                valid = (distance >= 0.05) & (distance <= 6.0)
                valid &= np.abs(directions[:, 1] * distance) <= 0.6
                hits.append(origin + rays[valid] * distance[valid, None])
        finally:
            model.geom_group[own] = groups
        return np.vstack(hits)
