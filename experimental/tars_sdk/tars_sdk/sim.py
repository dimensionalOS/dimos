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

"""MuJoCo backend: physics, joint-level PD servos, and sensors. Knows nothing about gaits."""

from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

from tars_sdk.kinematics import heading, quat_to_mat
from tars_sdk.model.generate import mjcf_xml
from tars_sdk.model.params import Params
from tars_sdk.types import JOINT_NAMES, N_SLABS, CameraFrame, JointTargets, Measurement, Odometry

Controller = Callable[[float, Measurement], JointTargets]


def build_model(
    params: Params, scene: Path | None = None, spawn: tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> tuple[mujoco.MjModel, str]:
    """Compile TARS alone (on a checker floor) or attached into `scene` at spawn (x, y, yaw).

    Returns the model and the name prefix of TARS elements ("" standalone, "tars/" in a scene).
    Deterministic, so a viewer process can rebuild the identical model from the same args.
    """
    if scene is None:
        return mujoco.MjModel.from_xml_string(mjcf_xml(params)), ""
    spec = mujoco.MjSpec.from_file(str(scene))
    robot = mujoco.MjSpec.from_string(mjcf_xml(params, standalone=False))
    spec.option.timestep = robot.option.timestep
    spec.option.integrator = robot.option.integrator
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, 1920)
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, 1440)
    x, y, yaw = spawn
    frame = spec.worldbody.add_frame(
        pos=[x, y, 0.0], quat=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    )
    spec.attach(robot, prefix="tars/", frame=frame)
    return spec.compile(), "tars/"


class SimBackend:
    """Steps MuJoCo and runs a controller callback at `control_hz`.

    Threaded (`start()`) runs in real time; otherwise call `advance(seconds)` yourself.
    Hold `lock` while touching `model`/`data` from another thread (e.g. a viewer).
    Odometry frame: world axes, origin at the spawn point.
    """

    def __init__(
        self,
        params: Params,
        scene: Path | None = None,
        spawn: tuple[float, float, float] = (0.0, 0.0, 0.0),
        control_hz: float = 100.0,
    ) -> None:
        self.params, self.scene, self.spawn = params, scene, spawn
        self.model, self.prefix = build_model(params, scene, spawn)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.RLock()
        self.control_dt = 1.0 / control_hz
        self._substeps = max(1, round(self.control_dt / self.model.opt.timestep))
        self._controller: Controller | None = None
        self._targets = JointTargets()
        self._thread: threading.Thread | None = None
        self._running = False
        self._renderer: mujoco.Renderer | None = None
        self._render_data = mujoco.MjData(self.model)
        self._step_hooks: list[Callable[[], None]] = []

        m, n = self.model, self._name
        self._qadr = np.array([m.joint(n(j)).qposadr[0] for j in JOINT_NAMES])
        self._vadr = np.array([m.joint(n(j)).dofadr[0] for j in JOINT_NAMES])
        self._act = np.array([m.actuator(n(j)).id for j in JOINT_NAMES])
        self._hub = m.body(n("base_link")).id
        root = m.joint(n("root"))
        self._root_q, self._root_v = int(root.qposadr[0]), int(root.dofadr[0])
        self._effort = m.actuator_ctrlrange[self._act, 1].copy()

        def sens(name: str) -> slice:
            s = m.sensor(n(name))
            return slice(s.adr[0], s.adr[0] + s.dim[0])

        self._s_quat, self._s_gyro, self._s_acc = (
            sens("imu_quat"),
            sens("imu_gyro"),
            sens("imu_acc"),
        )
        self._s_touch = [sens(f"slab_{i}_foot_touch") for i in range(1, N_SLABS + 1)]

        # start seated: every slab straight down, slides retracted, hub at the spawn point
        mujoco.mj_resetData(m, self.data)
        mujoco.mj_forward(m, self.data)
        self._origin = self.data.qpos[self._root_q : self._root_q + 2].copy()

    def _name(self, name: str) -> str:
        return self.prefix + name

    def add_step_hook(self, hook: Callable[[], None]) -> None:
        """Called (under `lock`) after every control step, e.g. to mirror state to a viewer."""
        self._step_hooks.append(hook)

    # ------------------------------------------------------------ control
    def set_controller(self, controller: Controller) -> None:
        self._controller = controller

    def measure(self) -> Measurement:
        d = self.data
        return Measurement(
            time=float(d.time),
            joint_q=d.qpos[self._qadr].copy(),
            joint_dq=d.qvel[self._vadr].copy(),
            joint_tau=d.ctrl[self._act].copy(),
            imu_quat=d.sensordata[self._s_quat].copy(),
            imu_gyro=d.sensordata[self._s_gyro].copy(),
            imu_acc=d.sensordata[self._s_acc].copy(),
            foot_force=np.array([d.sensordata[s][0] for s in self._s_touch]),
        )

    def ground_truth(self) -> Odometry:
        d, a, b = self.data, self._root_q, self._root_v
        q = d.qpos[a + 3 : a + 7].copy()
        yaw = heading(quat_to_mat(q))  # axle-based: valid when the hub is upside down (roll)
        v = d.qvel[b : b + 3]
        cy, sy = np.cos(yaw), np.sin(yaw)
        return Odometry(
            x=float(d.qpos[a] - self._origin[0]),
            y=float(d.qpos[a + 1] - self._origin[1]),
            z=float(d.qpos[a + 2]),
            yaw=yaw,
            vx=float(cy * v[0] + sy * v[1]),
            vy=float(-sy * v[0] + cy * v[1]),
            wz=float((quat_to_mat(q) @ d.qvel[b + 3 : b + 6])[2]),
            quat=q,
        )

    def set_hub_wrench(self, wrench: np.ndarray) -> None:
        """External world-frame [force, torque] on the hub (sim assist). Persists until changed."""
        self.data.xfrc_applied[self._hub] = wrench

    def hub_velocity_world(self) -> tuple[np.ndarray, np.ndarray]:
        """Ground-truth hub (linear, angular) velocity in the world frame."""
        d = self.data
        R = d.xmat[self._hub].reshape(3, 3)
        b = self._root_v
        return d.qvel[b : b + 3].copy(), R @ d.qvel[b + 3 : b + 6]

    def _apply_pd(self) -> None:
        d, t = self.data, self._targets
        q, dq = d.qpos[self._qadr], d.qvel[self._vadr]
        tau = t.kp * (t.q - q) + t.kd * (t.dq - dq) + t.tau
        d.ctrl[self._act] = np.clip(tau, -self._effort, self._effort)

    def _control_step(self) -> None:
        with self.lock:
            if self._controller is not None:
                self._targets = self._controller(self.control_dt, self.measure())
            for _ in range(self._substeps):
                self._apply_pd()
                mujoco.mj_step(self.model, self.data)
            for hook in self._step_hooks:
                hook()

    def advance(self, seconds: float) -> None:
        for _ in range(max(1, round(seconds / self.control_dt))):
            self._control_step()

    # ------------------------------------------------------------ threading
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="tars-sim", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        next_t = time.perf_counter()
        while self._running:
            self._control_step()
            next_t += self.control_dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.25:  # fell far behind: drop the backlog instead of racing
                next_t = time.perf_counter()

    # ------------------------------------------------------------ camera
    def render_camera(
        self, width: int, height: int, depth: bool, camera: str = "front_camera"
    ) -> CameraFrame:
        camera = self._name(camera)
        """Render from the calling thread using a snapshot of the current state."""
        with self.lock:
            self._render_data.qpos[:] = self.data.qpos
            self._render_data.time = self.data.time
        mujoco.mj_kinematics(self.model, self._render_data)
        mujoco.mj_camlight(self.model, self._render_data)
        if self._renderer is None or (self._renderer.width, self._renderer.height) != (
            width,
            height,
        ):
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        r = self._renderer
        r.update_scene(self._render_data, camera=camera)
        rgb = r.render().copy()
        d = None
        if depth:
            r.enable_depth_rendering()
            r.update_scene(self._render_data, camera=camera)
            d = r.render().copy()
            r.disable_depth_rendering()
        fovy = float(self.model.cam_fovy[self.model.camera(camera).id])
        return CameraFrame(rgb=rgb, depth=d, fovy_deg=fovy, time=float(self._render_data.time))
