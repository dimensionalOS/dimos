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

"""One bounded, demand-driven MuJoCo render shared by JPEG comparison viewers."""

import math
import threading
import time
from collections.abc import Callable
from copy import copy
from typing import Any

import mujoco
import numpy as np
import requests
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from microduck_world.football import Scoreboard
from numpy.typing import NDArray

COMPARE_CHANNEL = "world_compare_image"
COMPARE_SIZE = (640, 360)
COMPARE_FPS = 12.0
COMPARE_OFFSET = (0.9, -1.2, 0.8)
COMPARE_FOV = 42.0


def comparison_requested(stats: Any) -> bool:
    """The relay's aggregate subscriptions already handle multiple viewers/disconnects."""
    if not isinstance(stats, dict) or not isinstance(stats.get("perRobot"), dict):
        return False
    return any(
        isinstance(robot, dict)
        and isinstance(robot.get("subs"), list)
        and COMPARE_CHANNEL in robot["subs"]
        for robot in stats["perRobot"].values()
    )


def comparison_camera(data: mujoco.MjData, focus_body: int) -> mujoco.MjvCamera:
    """Match the Three.js Follow duck view, shared across comparison viewers."""
    x, y, z = COMPARE_OFFSET
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = data.xpos[focus_body]
    camera.distance = math.sqrt(x * x + y * y + z * z)
    camera.azimuth = math.degrees(math.atan2(-y, -x))
    camera.elevation = -math.degrees(math.atan2(z, math.hypot(x, y)))
    return camera


class JpegComparison:
    """Own render data and GL context on a worker; never step or lock the live simulation."""

    def __init__(
        self, model: mujoco.MjModel, stats_url: str, publish: Callable[[Image], None]
    ) -> None:
        # A private model also isolates visual settings from the agent camera.
        self._model = model
        self._stats_url = stats_url
        self._publish = publish
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._qpos: tuple[NDArray[np.float64], list[int]] | None = None
        self._active = False
        self._frames = 0
        self._render_ms = 0.0
        self._error: str | None = None
        self._thread = threading.Thread(target=self._run, name="world-jpeg-comparison", daemon=True)
        self._thread.start()

    def snapshot(self, qpos: NDArray[np.float64], lit: list[int] | None = None) -> None:
        with self._lock:
            if self._active:
                self._qpos = (qpos.copy(), list(lit or []))

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": self._active,
                "frames": self._frames,
                "renderMs": round(self._render_ms, 2),
                "error": self._error,
            }

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError("JPEG comparison renderer did not stop")

    def _run(self) -> None:
        renderer: mujoco.Renderer | None = None
        model: mujoco.MjModel | None = None
        data: mujoco.MjData | None = None
        focus = 0
        scoreboard: Scoreboard | None = None
        next_probe = 0.0
        next_frame = 0.0
        option = mujoco.MjvOption()
        option.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        try:
            with requests.Session() as http:
                http.trust_env = False
                while not self._stop.is_set():
                    now = time.monotonic()
                    if now >= next_probe:
                        wanted = False
                        try:
                            response = http.get(self._stats_url, timeout=0.5)
                            response.raise_for_status()
                            wanted = comparison_requested(response.json())
                        except (requests.RequestException, ValueError):
                            pass  # Fail closed: no relay means no comparison rendering.
                        with self._lock:
                            self._active = wanted
                            if not wanted:
                                self._qpos = None
                        next_probe = time.monotonic() + 1.0
                    with self._lock:
                        active = self._active
                        qpos = self._qpos
                    if not active:
                        if renderer is not None:
                            renderer.close()
                            renderer = None
                            data = None
                            model = None
                        self._stop.wait(max(0.01, next_probe - time.monotonic()))
                        continue
                    if qpos is None or now < next_frame:
                        self._stop.wait(0.01 if qpos is None else min(0.05, next_frame - now))
                        continue
                    try:
                        started = time.monotonic()
                        if renderer is None:
                            model = copy(self._model)
                            scoreboard = Scoreboard(model)
                            model.vis.global_.fovy = COMPARE_FOV
                            model.vis.global_.offwidth = max(
                                model.vis.global_.offwidth, COMPARE_SIZE[0]
                            )
                            model.vis.global_.offheight = max(
                                model.vis.global_.offheight, COMPARE_SIZE[1]
                            )
                            data = mujoco.MjData(model)
                            focus = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
                            renderer = mujoco.Renderer(
                                model, height=COMPARE_SIZE[1], width=COMPARE_SIZE[0]
                            )
                        assert model is not None and data is not None
                        data.qpos[:] = qpos[0]
                        assert scoreboard is not None
                        scoreboard.apply(model, qpos[1])
                        mujoco.mj_forward(model, data)
                        renderer.update_scene(
                            data, camera=comparison_camera(data, focus), scene_option=option
                        )
                        rgb = renderer.render()
                        self._publish(
                            Image(
                                data=rgb,
                                format=ImageFormat.RGB,
                                frame_id="comparison",
                                ts=time.time(),
                            )
                        )
                        with self._lock:
                            self._frames += 1
                            self._render_ms = (time.monotonic() - started) * 1000
                            self._error = None
                    except Exception as exc:
                        with self._lock:
                            self._error = str(exc)[:160]
                        if renderer is not None:
                            renderer.close()
                            renderer = None
                        self._stop.wait(1.0)
                    next_frame = max(started + 1.0 / COMPARE_FPS, time.monotonic())
        finally:
            if renderer is not None:
                renderer.close()
            with self._lock:
                self._active = False
