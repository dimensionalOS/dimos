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

"""World-scale residency around one robot and its awake contact islands.

Robot USD/body identities remain fixed. Environment objects use the explicit
world-scoped CBOR extension; their stable identities belong to the catalog.
"""

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any

import cbor2
from dimos_lcm.control_msgs import CborCommand, JointCommand
from dimos_lcm.sim_msgs import CborObservation
import mujoco
import numpy as np
from numpy.typing import NDArray
from pxr import Sdf, Usd

from dimos.experimental.gta_sim2.prefetch import predict_position
from dimos.experimental.gta_sim2.streaming import StreamingMujocoBackend, collision_base
from dimos.experimental.gta_sim2.world_compile import compile_world_window
from dimos.experimental.gta_sim2.world_geometry import WorldCatalog
from dimos.experimental.gta_sim2.world_objects import (
    ObjectStateStore,
    awake_objects,
    object_snapshot,
    restore_object,
    transfer_named_state,
)
from dimos.robot.assets.model import UsdModel
from dimos.sim2.physics.mujoco import MujocoBackend


@dataclass
class WorldWindow:
    center: NDArray[np.float64]
    model: mujoco.MjModel
    data: mujoco.MjData
    metadata: dict[str, Any]
    build_s: float


class WorldStreamingMujocoBackend(MujocoBackend):
    def __init__(
        self,
        catalog: Path,
        focus_body: str,
        metrics: Path,
        tile_size: float = 8,
        half_extent: float = 12,
        compiler_delay_s: float = 0,
        state_path: Path | None = None,
    ) -> None:
        if not 0 < tile_size <= half_extent - 4 or not 0 <= compiler_delay_s <= 10:
            raise ValueError("Invalid streaming grid or compiler delay")
        self._catalog_path, self._focus_body, self._metrics_path = catalog, focus_body, metrics
        self._tile_size, self._half_extent = tile_size, half_extent
        self._delay, self._state_path = compiler_delay_s, state_path
        self._compiler: subprocess.Popen[str] | None = None
        self._io: ThreadPoolExecutor | None = None
        self._maintenance: ThreadPoolExecutor | None = None
        self._scratch: tempfile.TemporaryDirectory[str] | None = None
        self._pending: Future[WorldWindow] | None = None
        self._pending_started = 0.0
        self._store: ObjectStateStore | None = None
        self._events: deque[dict[str, Any]] = deque(maxlen=128)
        self._latencies: deque[float] = deque([0.5], maxlen=16)
        self._settled: dict[str, float] = {}
        self._actors: dict[str, dict[str, Any]] = {}
        self._route: NDArray[np.float64] | None = None
        self._route_until = 0.0
        self._load_error = ""
        self._retry_at = 0.0
        self._tick = self._swaps = self._rejections = 0
        self._ground_clearance: float | None = None
        self._metrics_future: Future[None] | None = None

    def load(
        self, scene: UsdModel, dt_ns: int, gravity_world: tuple[float, float, float] = (0, 0, -9.81)
    ) -> None:
        self._catalog = WorldCatalog(self._catalog_path)
        layer = Sdf.Layer.CreateAnonymous()
        layer.ImportFromString(scene.usd)
        stage = Usd.Stage.Open(layer)
        if (
            stage.GetPrimAtPath("/world/gta_patch")
            .GetAttribute("gta:collision_catalog_sha256")
            .Get()
            != self._catalog.digest
        ):
            raise ValueError("World catalog does not match the composed scene")
        self._metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self._scratch = tempfile.TemporaryDirectory(
            prefix="world-collision-", dir=self._metrics_path.parent
        )
        directory = Path(self._scratch.name)
        scene_path = directory / "scene.usda"
        scene_path.write_text(scene.usd)
        state_key = hashlib.sha256(str(self._metrics_path.resolve()).encode()).hexdigest()[:16]
        self._state_path = self._state_path or self._catalog_path.parent / "state" / (
            state_key + ".sqlite"
        )
        self._state_path = self._state_path.resolve()
        self._store = ObjectStateStore(self._state_path, self._catalog.digest)
        super().load(scene, dt_ns, gravity_world)
        if self.root_ids != (self._focus_body,):
            raise ValueError("World streaming currently supports one robot root")
        self._robot_nv = self.model.nv
        base = collision_base(scene, dt_ns, gravity_world)
        center = self._center(self._position())
        started = time.monotonic()
        model, data, _indices, metadata = compile_world_window(
            base, self._catalog, center, self._half_extent, self._state_path
        )
        self._window = WorldWindow(center, model, data, metadata, time.monotonic() - started)
        transfer_named_state(self.model, self.data, model, data, set())
        self.model, self.data = model, data
        self._actors = metadata.get("actors", {})
        self._catalog.close()
        self._compiler = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "dimos.experimental.gta_sim2.demo_world_worker",
                "--scene",
                str(scene_path),
                "--catalog",
                str(self._catalog_path),
                "--digest",
                self._catalog.digest,
                "--state",
                str(self._state_path),
                "--directory",
                str(directory / "models"),
                "--dt-ns",
                str(dt_ns),
                "--gravity",
                *map(str, gravity_world),
                "--delay-s",
                str(self._delay),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self._io = ThreadPoolExecutor(max_workers=1, thread_name_prefix="world-compiler-io")
        self._maintenance = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="world-maintenance"
        )
        self._record("initial", 0, 0, 0)

    def _position(self) -> NDArray[np.float64]:
        return np.asarray(
            self.data.xpos[self.model.body(self._focus_body).id], dtype=np.float64
        ).copy()

    def _velocity(self) -> NDArray[np.float64]:
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(
            self.model,
            self.data,
            mujoco.mjtObj.mjOBJ_XBODY,
            self.model.body(self._focus_body).id,
            velocity,
            0,
        )
        return velocity[3:].copy()

    def _center(self, position: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.floor(position / self._tile_size + 0.5) * self._tile_size

    def _latency(self) -> float:
        elapsed = time.monotonic() - self._pending_started if self._pending is not None else 0
        return max(0.5, elapsed, float(np.quantile(self._latencies, 0.95))) + 0.25

    def _snapshots(self) -> dict[str, dict[str, Any]]:
        return {
            identity: object_snapshot(
                self.model, self.data, identity, actor["body"], actor["radius"]
            )
            for identity, actor in self._actors.items()
        }

    def _compile_process(
        self, center: NDArray[np.float64], pinned: dict[str, dict[str, Any]]
    ) -> WorldWindow:
        assert (
            self._compiler is not None
            and self._compiler.stdin is not None
            and self._compiler.stdout is not None
        )
        started = time.monotonic()
        self._compiler.stdin.write(
            json.dumps(
                {"center": center.tolist(), "half_extent": self._half_extent, "pinned": pinned}
            )
            + "\n"
        )
        self._compiler.stdin.flush()
        for line in self._compiler.stdout:
            if not line.startswith('{"window":'):
                continue
            result = json.loads(line)
            if result.get("error"):
                raise RuntimeError(result["error"])
            model = mujoco.MjModel.from_binary_path(result["window"])
            data = mujoco.MjData(model)
            metadata = result["metadata"]
            metadata.update(cache_hit=result["cache_hit"], compiled_cache=result["compiled_cache"])
            metadata["object_regions"] = tuple(
                sorted(
                    (key, tuple(np.floor(np.asarray(value["position"]) / 4 + 0.5)))
                    for key, value in pinned.items()
                )
            )
            for identity, actor in metadata.get("actors", {}).items():
                actor["state"] = result["states"].get(identity)
                if actor["state"]:
                    restore_object(model, data, actor["state"])
            return WorldWindow(center, model, data, metadata, time.monotonic() - started)
        raise RuntimeError("World compiler exited")

    def _write_metrics(self, snapshot: dict[str, Any], event: dict[str, Any]) -> None:
        pending = self._metrics_path.with_suffix(".tmp")
        pending.write_text(json.dumps(snapshot, indent=2))
        pending.replace(self._metrics_path)
        log = self._metrics_path.with_suffix(".events.jsonl")
        if log.exists() and log.stat().st_size > 8 << 20:
            log.replace(log.with_suffix(".previous.jsonl"))
        with log.open("a") as stream:
            stream.write(json.dumps(event) + "\n")

    def _record(self, event: str, error: float, acceleration_error: float, swap_s: float) -> None:
        metadata = self._window.metadata
        row = {
            "event": event,
            "time_s": self.data.time,
            "center": self._window.center.tolist(),
            "resident_triangles": metadata["triangle_prisms"],
            "resident_primitives": metadata["analytic_shapes"],
            "active_objects": len(self._actors),
            "build_s": self._window.build_s,
            "state_error": error,
            "acceleration_error": acceleration_error,
            "swap_s": swap_s,
            "cache_hit": metadata.get("cache_hit", False),
            "compiled_cache": metadata.get("compiled_cache", {}),
            "geometry_cache": metadata["cache"],
            "unresolved_instances": metadata["unresolved_instances"],
        }
        self._events.append(row)
        snapshot = {
            "catalog": "spatial_world",
            "half_extent_m": self._half_extent,
            "swaps": self._swaps,
            "rejections": self._rejections,
            "counts": self._catalog.manifest["counts"],
            "events": list(self._events),
        }
        assert self._maintenance is not None
        self._metrics_future = self._maintenance.submit(self._write_metrics, snapshot, row)

    def _adopt(self, ready: WorldWindow, position: NDArray[np.float64]) -> bool:
        if np.max(np.abs(position - ready.center)) >= self._half_extent - 3:
            return False
        assert self._store is not None
        awake = awake_objects(self.model, self.data, self._actors, self._settled)
        incoming = ready.metadata.get("actors", {})
        if not awake <= incoming.keys():
            return False
        snapshots = self._snapshots()
        removed = self._actors.keys() - incoming.keys()
        sleeping = {self._actors[identity]["body"] for identity in removed}
        for identity, actor in incoming.items():
            if identity not in self._actors:
                state = self._store.latest(identity, actor.get("state"))
                if state:
                    restore_object(ready.model, ready.data, state)
        started = time.monotonic()
        mujoco.mj_forward(self.model, self.data)
        old_acceleration = self.data.qacc[: self._robot_nv].copy()
        try:
            error = transfer_named_state(self.model, self.data, ready.model, ready.data, sleeping)
        except ValueError as failure:
            if "contact" in str(failure).lower() or "awake" in str(failure).lower():
                return False
            raise
        acceleration_error = float(
            np.max(np.abs(ready.data.qacc[: self._robot_nv] - old_acceleration))
        )
        self._store.save(list(snapshots.values()))
        retired = [self._window]
        self.model, self.data, self._window = ready.model, ready.data, ready
        self._actors = incoming
        self._settled = {key: value for key, value in self._settled.items() if key in incoming}
        self._active_object_regions = ready.metadata.get("object_regions", ())
        self._swaps += 1
        duration = time.monotonic() - started
        assert self._maintenance is not None
        # The list owns the old window. Clearing it transfers the final decref
        # to this worker even if it runs before the owner returns from _adopt.
        self._maintenance.submit(list.clear, retired)
        self._record("swap", error, acceleration_error, duration)
        return True

    def step(self) -> None:
        position = self._position()
        if self._pending is not None and self._pending.done():
            ready: WorldWindow | None = None
            try:
                ready = self._pending.result()
                self._latencies.append(ready.build_s)
                if not self._adopt(ready, position):
                    self._rejections += 1
                self._load_error = ""
            except (RuntimeError, MemoryError, ValueError, OSError) as error:
                self._load_error = str(error)
                self._retry_at = self.data.time + 1.0
            completed = [self._pending]
            self._pending = None
            ready = None
            if self._maintenance is not None:
                self._maintenance.submit(list.clear, completed)
        # Governor begins braking at half_extent-3; hard fault has a further 2 m reserve.
        if np.max(np.abs(position - self._window.center)) >= self._half_extent - 1:
            raise RuntimeError("Robot crossed the protected resident boundary despite braking")
        if np.any(position <= self._catalog.coverage[0] + 1) or np.any(
            position >= self._catalog.coverage[1] - 1
        ):
            raise RuntimeError("Robot reached the catalog data extent")
        if self._tick % 10 == 0 and self._pending is None and self.data.time >= self._retry_at:
            route = self._route if self.data.time < self._route_until else None
            predicted = predict_position(
                position, self._velocity(), self._latency(), route, self._half_extent - 4
            )
            center = self._center(predicted)
            if np.max(np.abs(position - center)) >= self._half_extent - 3:
                center = self._center(position)
            # Replace terrain around awake objects too, even if the robot is parked.
            awake = awake_objects(self.model, self.data, self._actors, self._settled)
            snapshots = self._snapshots()
            pinned = {key: snapshots[key] for key in awake}
            object_regions = tuple(
                sorted(
                    (key, tuple(np.floor(np.asarray(value["position"]) / 4 + 0.5)))
                    for key, value in pinned.items()
                )
            )
            if not np.array_equal(center, self._window.center) or object_regions != getattr(
                self, "_active_object_regions", ()
            ):
                assert self._io is not None
                self._pending_started = time.monotonic()
                self._pending = self._io.submit(self._compile_process, center, pinned)
        if self._tick % 100 == 0 and self._actors:
            assert self._store is not None
            self._store.save(list(self._snapshots().values()))
        if self._tick % 20 == 0:
            group = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
            geom = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(
                self.model,
                self.data,
                position,
                np.array([0.0, 0.0, -1.0]),
                group,
                1,
                self.model.body(self._focus_body).id,
                geom,
            )
            self._ground_clearance = float(distance) if distance >= 0 else None
        self._tick += 1
        super().step()

    def read_cbor_observations(self) -> list[CborObservation]:
        position = self._position()
        low = np.maximum(
            self._window.center[:2] - self._half_extent + 3, self._catalog.coverage[0, :2] + 3
        )
        high = np.minimum(
            self._window.center[:2] + self._half_extent - 3, self._catalog.coverage[1, :2] - 3
        )
        state = {
            "position": position.tolist(),
            "velocity": self._velocity().tolist(),
            "safe_bounds": [low.tolist(), high.tolist()],
            "latency_s": self._latency(),
            "loading": self._pending is not None,
            "load_error": self._load_error,
            "clearance_m": self._ground_clearance,
            "swaps": self._swaps,
            "time_s": self.data.time,
        }
        payload = cbor2.dumps(state)
        result = [
            CborObservation(
                entity_id="world", key="gta/streaming", value_length=len(payload), value=payload
            )
        ]
        if self._actors:
            actors = [
                {
                    "identity": identity,
                    "position": self.data.xpos[self.model.body(actor["body"]).id].tolist(),
                    "quaternion_wxyz": self.data.xquat[self.model.body(actor["body"]).id].tolist(),
                }
                for identity, actor in self._actors.items()
            ]
            payload = cbor2.dumps({"time_s": self.data.time, "objects": actors})
            result.append(
                CborObservation(
                    entity_id="world", key="gta/objects", value_length=len(payload), value=payload
                )
            )
        return result

    def apply_action(
        self, joint_commands: list[JointCommand], cbor_commands: list[CborCommand]
    ) -> None:
        route, until = self._route, self._route_until
        for command in cbor_commands:
            if command.entity_id != "world" or command.key != "gta/route":
                raise ValueError("Unsupported world-streaming command")
            try:
                value = cbor2.loads(bytes(command.value))
                points = np.asarray(value["points"], dtype=np.float64)
                expires = float(value["expires_at_s"])
            except (ValueError, TypeError, KeyError, cbor2.CBORDecodeError) as failure:
                raise ValueError("Malformed route prediction") from failure
            if (
                points.ndim != 2
                or points.shape[1:] != (3,)
                or not 1 <= len(points) <= 256
                or not np.isfinite(points).all()
                or not self.data.time < expires <= self.data.time + 5
            ):
                raise ValueError("Invalid or expired route prediction")
            route, until = points, expires
        super().apply_action(joint_commands, [])
        self._route, self._route_until = route, until

    def close(self) -> None:
        if self._store is not None and hasattr(self, "data"):
            self._store.save(list(self._snapshots().values()))
        if self._compiler is not None:
            try:
                os.killpg(self._compiler.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._compiler.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self._compiler.pid, signal.SIGKILL)
                self._compiler.wait()
        if self._io is not None:
            self._io.shutdown(wait=True, cancel_futures=True)
        if self._compiler is not None:
            if self._compiler.stdin:
                self._compiler.stdin.close()
            if self._compiler.stdout:
                self._compiler.stdout.close()
            self._compiler = None
        if self._maintenance is not None:
            self._maintenance.shutdown(wait=True)
        if self._store is not None:
            self._store.close()
            self._store = None
        if hasattr(self, "_catalog"):
            self._catalog.close()
        if self._scratch is not None:
            self._scratch.cleanup()
            self._scratch = None
        super().close()


def streaming_backend(
    catalog: Path, focus_body: str, metrics: Path, compiler_delay_s: float = 0
) -> MujocoBackend:
    cls = WorldStreamingMujocoBackend if catalog.suffix == ".json" else StreamingMujocoBackend
    if cls is WorldStreamingMujocoBackend:
        return cls(
            catalog=catalog,
            focus_body=focus_body,
            metrics=metrics,
            compiler_delay_s=compiler_delay_s,
        )
    if compiler_delay_s:
        raise ValueError("Compiler delay injection requires a world catalog")
    return cls(catalog=catalog, focus_body=focus_body, metrics=metrics)
