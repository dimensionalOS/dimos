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

"""Experimental static collision residency, preserving the Sim2 dynamic schema.

Only world-attached geometry changes. A background compiler prepares the next
model; the physics owner transfers the complete integration state at a tick
boundary. This is static streaming, not dynamic-object LOD or a city-wide claim.
"""

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray
from pxr import Sdf, Usd

from dimos.robot.assets.model import UsdModel
from dimos.sim2.geometry.mujoco import build_scene
from dimos.sim2.physics.mujoco import MujocoBackend


class CollisionCatalog:
    def __init__(self, path: Path) -> None:
        with np.load(path) as archive:
            self.triangles: NDArray[np.float64] = archive["triangles"].astype(np.float64)
            self.coverage: NDArray[np.float64] = archive["coverage_bounds"].astype(np.float64)
        if self.triangles.ndim != 3 or self.triangles.shape[1:] != (3, 3):
            raise ValueError("Collision catalog must contain triangles")
        if not np.isfinite(self.triangles).all():
            raise ValueError("Nonfinite collision catalog")
        if (
            self.coverage.shape != (2, 3)
            or not np.isfinite(self.coverage).all()
            or np.any(self.coverage[0] >= self.coverage[1])
        ):
            raise ValueError("Invalid collision coverage bounds")
        self.digest = hashlib.sha256(path.read_bytes()).hexdigest()
        self._lo = self.triangles.min(axis=1)
        self._hi = self.triangles.max(axis=1)

    def select(self, center: NDArray[np.float64], half_extent: float) -> NDArray[np.int64]:
        # Include a prism-thickness margin. Keep whole, uniquely indexed source
        # triangles: clipping at tile edges would change surfaces near contacts.
        return np.flatnonzero(
            np.all(self._hi[:, :2] >= center - half_extent - 0.04, axis=1)
            & np.all(self._lo[:, :2] <= center + half_extent + 0.04, axis=1)
        )


@dataclass
class PreparedWindow:
    center: NDArray[np.float64]
    indices: NDArray[np.int64]
    model: mujoco.MjModel
    data: mujoco.MjData
    build_s: float


def compile_window(
    base: mujoco.MjSpec,
    catalog: CollisionCatalog,
    center: NDArray[np.float64],
    half_extent: float,
) -> PreparedWindow:
    start = time.monotonic()
    spec = base.copy()
    indices = catalog.select(center, half_extent)
    faces = [0, 1, 2, 5, 4, 3, 0, 3, 4, 0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 3, 2, 3, 0]
    for index in indices:
        tri = catalog.triangles[index]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-8:
            raise ValueError(f"Degenerate source triangle {index}")
        normal /= norm
        if normal[2] < 0:
            tri, normal = tri[[0, 2, 1]], -normal
        name = f"gta_stream/triangle_{index}"
        spec.add_mesh(
            name=name, uservert=np.concatenate((tri, tri - normal * 0.04)).ravel(), userface=faces
        )
        spec.worldbody.add_geom(
            name=name,
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname=name,
            contype=1,
            conaffinity=1,
            group=3,
            friction=[0.8, 0.005, 0.0001],
        )
    model = spec.compile()
    return PreparedWindow(center, indices, model, mujoco.MjData(model), time.monotonic() - start)


def collision_base(
    scene: UsdModel, dt_ns: int, gravity_world: tuple[float, float, float]
) -> mujoco.MjSpec:
    base = build_scene(scene, dt_ns, gravity_world=gravity_world)
    for geom in list(base.geoms):
        if geom.name.startswith("/world/gta_patch/"):
            base.delete(geom)
    for mesh in list(base.meshes):
        if mesh.name.startswith("/world/gta_patch/"):
            base.delete(mesh)
    return base


def transfer_state(
    old_model: mujoco.MjModel,
    old_data: mujoco.MjData,
    new_model: mujoco.MjModel,
    new_data: mujoco.MjData,
) -> float:
    """Copy all integration state, rejecting incompatible dynamic schemas.

    The returned error is measured before forward dynamics rebuilds derived state.
    Active contact geometry must exist by stable name in the replacement model.
    """
    for kind, count in (("body", "nbody"), ("joint", "njnt"), ("actuator", "nu")):
        if getattr(old_model, count) != getattr(new_model, count):
            raise ValueError(f"Streaming changed {kind} count")
        if any(
            getattr(old_model, kind)(i).name != getattr(new_model, kind)(i).name
            for i in range(getattr(old_model, count))
        ):
            raise ValueError(f"Streaming changed {kind} identity/order")
    for field in (
        "body_parentid",
        "body_mass",
        "body_inertia",
        "jnt_type",
        "jnt_qposadr",
        "jnt_dofadr",
        "actuator_trnid",
    ):
        if not np.array_equal(getattr(old_model, field), getattr(new_model, field)):
            raise ValueError(f"Streaming changed dynamic field {field}")
    for contact in old_data.contact:
        for geom in contact.geom:
            if geom >= 0:
                try:
                    new_model.geom(old_model.geom(int(geom)).name)
                except KeyError as missing:
                    raise ValueError("Refusing to unload a contacting collider") from missing
    signature = mujoco.mjtState.mjSTATE_INTEGRATION
    size = mujoco.mj_stateSize(old_model, signature)
    if size != mujoco.mj_stateSize(new_model, signature):
        raise ValueError("Streaming changed integration-state layout")
    state = np.empty(size)
    mujoco.mj_getState(old_model, old_data, state, signature)
    # Sim2's motor actions mutate these arrays, not only data.ctrl.
    new_model.actuator_gainprm[:] = old_model.actuator_gainprm
    new_model.actuator_biasprm[:] = old_model.actuator_biasprm
    mujoco.mj_setState(new_model, new_data, state, signature)
    check = np.empty_like(state)
    mujoco.mj_getState(new_model, new_data, check, signature)
    error = float(np.max(np.abs(check - state)))
    if error != 0 or not np.isfinite(check).all():
        raise ValueError("Integration-state transfer was not exact")
    mujoco.mj_forward(new_model, new_data)
    return error


class StreamingMujocoBackend(MujocoBackend):
    def __init__(
        self,
        catalog: Path,
        focus_body: str,
        metrics: Path,
        tile_size: float = 8,
        half_extent: float = 12,
    ) -> None:
        if not 0 < tile_size <= half_extent - 3:
            raise ValueError("Window requires at least a three-metre protected margin")
        self._catalog_path, self._focus_body, self._metrics_path = catalog, focus_body, metrics
        self._tile_size, self._half_extent = tile_size, half_extent
        self._executor: ThreadPoolExecutor | None = None
        self._compiler: subprocess.Popen[str] | None = None
        self._scratch: tempfile.TemporaryDirectory[str] | None = None
        self._pending: Future[PreparedWindow] | None = None
        self._events: list[dict[str, Any]] = []
        self._tick = 0
        self._acceleration_error = 0.0

    def load(
        self, scene: UsdModel, dt_ns: int, gravity_world: tuple[float, float, float] = (0, 0, -9.81)
    ) -> None:
        self._catalog = CollisionCatalog(self._catalog_path)
        layer = Sdf.Layer.CreateAnonymous()
        layer.ImportFromString(scene.usd)
        stage = Usd.Stage.Open(layer)
        identity = (
            stage.GetPrimAtPath("/world/gta_patch")
            .GetAttribute("gta:collision_catalog_sha256")
            .Get()
        )
        if identity != self._catalog.digest:
            raise ValueError("Collision catalog does not match composed scene identity")
        # Optical geometry stays in the sensor scene, never changes revision,
        # and is omitted from the dynamics compiler's collision-only copy.
        base = collision_base(scene, dt_ns, gravity_world)
        self._metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self._scratch = tempfile.TemporaryDirectory(
            prefix="collision-", dir=self._metrics_path.parent
        )
        directory = Path(self._scratch.name)
        base_path = directory / "scene.usda"
        base_path.write_text(scene.usd)
        super().load(scene, dt_ns, gravity_world)
        if self.root_ids != (self._focus_body,):
            raise ValueError("This streaming prototype supports one floating robot only")
        center = self._center(self.data.xpos[self.model.body(self._focus_body).id, :2])
        self._window = compile_window(base, self._catalog, center, self._half_extent)
        transfer_state(self.model, self.data, self._window.model, self._window.data)
        self.model, self.data = self._window.model, self._window.data
        # DimOS workers are multiprocessing daemons. Own an explicit child
        # with pipes and cleanup instead of changing their process contract.
        self._compiler = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "dimos.experimental.gta_sim2.demo_collision_worker",
                "--scene",
                str(base_path),
                "--catalog",
                str(self._catalog_path),
                "--catalog-sha256",
                self._catalog.digest,
                "--dt-ns",
                str(dt_ns),
                "--gravity",
                *map(str, gravity_world),
                "--output",
                str(directory / "window.mjb"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="collision-io")
        self._record("initial", self._window, 0, 0)

    def _compile_process(self, center: NDArray[np.float64]) -> PreparedWindow:
        assert (
            self._compiler is not None
            and self._compiler.stdin is not None
            and self._compiler.stdout is not None
        )
        started = time.monotonic()
        self._compiler.stdin.write(
            json.dumps({"center": center.tolist(), "half_extent": self._half_extent}) + "\n"
        )
        self._compiler.stdin.flush()
        for line in self._compiler.stdout:
            if not line.startswith('{"window":'):
                continue
            result = json.loads(line)
            if "error" in result:
                raise RuntimeError("Collision compiler failed: " + result["error"])
            model = mujoco.MjModel.from_binary_path(result["window"])
            return PreparedWindow(
                center,
                np.asarray(result["indices"], dtype=np.int64),
                model,
                mujoco.MjData(model),
                time.monotonic() - started,
            )
        raise RuntimeError("Collision compiler exited before producing a window")

    def _center(self, xy: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.floor(xy / self._tile_size + 0.5) * self._tile_size

    def _record(self, event: str, window: PreparedWindow, error: float, swap_s: float) -> None:
        self._events.append(
            {
                "event": event,
                "time_s": self.data.time,
                "center": window.center.tolist(),
                "resident_triangles": len(window.indices),
                "build_s": window.build_s,
                "state_error": error,
                "swap_s": swap_s,
                "contacts": self.data.ncon,
                "acceleration_error": self._acceleration_error,
            }
        )
        self._metrics_path.parent.mkdir(parents=True, exist_ok=True)
        pending = self._metrics_path.with_suffix(".tmp")
        pending.write_text(
            json.dumps(
                {
                    "catalog_triangles": len(self._catalog.triangles),
                    "half_extent_m": self._half_extent,
                    "events": self._events,
                },
                indent=2,
            )
        )
        pending.replace(self._metrics_path)

    def step(self) -> None:
        xy = self.data.xpos[self.model.body(self._focus_body).id, :2].copy()
        if np.any(xy <= self._catalog.coverage[0, :2] + 2) or np.any(
            xy >= self._catalog.coverage[1, :2] - 2
        ):
            raise RuntimeError(
                "Reached the indexed collision coverage boundary; simulation stopped"
            )
        if self._pending is not None and self._pending.done():
            ready = self._pending.result()
            self._pending = None
            protected = self._catalog.select(xy, 2)
            if (
                np.all(np.isin(protected, ready.indices))
                and np.max(np.abs(xy - ready.center)) < self._half_extent - 3
            ):
                start = time.monotonic()
                # SimModule may have just changed motor gains/targets. Compare
                # accelerations under the same current controls, not the last
                # step's cached forward result.
                mujoco.mj_forward(self.model, self.data)
                old_acceleration = self.data.qacc.copy()
                error = transfer_state(self.model, self.data, ready.model, ready.data)
                self._acceleration_error = float(np.max(np.abs(ready.data.qacc - old_acceleration)))
                self.model, self.data, self._window = ready.model, ready.data, ready
                self._record("swap", ready, error, time.monotonic() - start)
        if np.max(np.abs(xy - self._window.center)) >= self._half_extent - 2:
            raise RuntimeError(
                "Collision prefetch did not finish before the protected boundary; simulation stopped"
            )
        if self._tick % 10 == 0 and self._pending is None:
            center = self._center(xy)
            if not np.array_equal(center, self._window.center):
                assert self._executor is not None
                self._pending = self._executor.submit(self._compile_process, center)
        self._tick += 1
        super().step()

    def close(self) -> None:
        if self._compiler is not None:
            self._compiler.terminate()
            try:
                self._compiler.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._compiler.kill()
                self._compiler.wait()
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._executor = None
        if self._compiler is not None:
            if self._compiler.stdin is not None:
                self._compiler.stdin.close()
            if self._compiler.stdout is not None:
                self._compiler.stdout.close()
            self._compiler = None
        if self._scratch is not None:
            self._scratch.cleanup()
            self._scratch = None
        super().close()
