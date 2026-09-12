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

"""Native selected-object observation stream and measured ACT outcomes."""

import copy
import json
from pathlib import Path
import threading
import time
from typing import Any
from uuid import uuid4

import numpy as np
from pydantic import Field, TypeAdapter
from reactivex.disposable import Disposable
from threadpoolctl import threadpool_limits  # type: ignore[import-untyped]

from dimos.constants import DIMOS_PROJECT_ROOT, RECORDINGS_DIR
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.imitation.observation import VectorObservation
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.galaxea.r1pro.grasping_sim import VIRTUAL_BASE_JOINTS
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_packing_scene import (
    ObjectLayout,
    prepare_object_scene,
    sample_layout,
)
from dimos.robot.galaxea.r1pro.object_packing_state import ObjectPackingState
from dimos.robot.galaxea.r1pro.object_packing_task import ObjectPackingTask
from dimos.robot.galaxea.r1pro.object_recovery import plan_object_recovery
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


class R1ProObjectPackingSimConfig(MujocoSimModuleConfig):
    generate_scene: bool = False
    seed: int = Field(default=210000, ge=0)
    scene_package: Path = DIMOS_PROJECT_ROOT / "dimos/data/scene_packages/hssd_102344115"
    output: Path = Field(default_factory=lambda: RECORDINGS_DIR / "r1pro-object-sim" / uuid4().hex)


class R1ProObjectPackingSim(MujocoSimModule):
    config: R1ProObjectPackingSimConfig
    right_wrist: Out[Image]
    object_goal: Out[VectorObservation]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._preparation_lock = threading.Lock()
        self._session: dict[str, Any] | None = None
        self._thread_limits: threadpool_limits | None = None
        self._layout: ObjectLayout | None = None
        self._state: ObjectPackingState | None = None
        self._initial: list[dict[str, Any]] | None = None
        self._last_check = 0.0
        self._error: str | None = None

    @rpc
    def prepare_object_session(self) -> dict[str, Any]:
        """Prepare once; control and simulation share the scene and calibrated joints."""
        with self._preparation_lock:
            if self._session is not None:
                return dict(self._session)
            self._thread_limits = threadpool_limits(limits=1, user_api="blas")
            if self.config.generate_scene:
                package = (
                    Path(global_config.scene_package or self.config.scene_package)
                    .expanduser()
                    .resolve()
                )
                if not package.is_dir():
                    raise FileNotFoundError(
                        f"House scene package missing: {package}; set --scene-package"
                    )
                output = self.config.output.expanduser().resolve()
                if (output / "scene.xml").exists():
                    raise FileExistsError(
                        f"Session already exists: {output}; choose a new --output"
                    )
                self._layout = sample_layout(self.config.seed)
                scene = prepare_object_scene(
                    output / "scene.xml", self._layout, scene_package=package
                )
                scene.with_suffix(".objects.json").write_text(json.dumps(self._layout.to_dict()))
                self.config.address = scene
            scene = Path(self.config.address).expanduser().resolve()
            self._layout = TypeAdapter(ObjectLayout).validate_json(
                scene.with_suffix(".objects.json").read_text()
            )
            with ObjectPackingTask(scene, self._layout, images=False) as task:
                joints = (*R1PRO_PICK_PLACE_JOINTS, *VIRTUAL_BASE_JOINTS)
                self.config.reset_joint_positions = task.home.tolist() + [0.0] * 3
                limits = [task.model.joint(n).range.tolist() for n in joints]
            self._session = dict(
                scene=str(scene), output=str(scene.parent), seed=self._layout.seed, limits=limits
            )
            return dict(self._session)

    @rpc
    def build(self) -> None:
        self.prepare_object_session()
        super().build()

    @rpc
    def start(self) -> None:
        self.register_disposable(Disposable(self.color_image.subscribe(self._publish_goal)))
        super().start()

    def _ensure_state(self, engine: MujocoEngine) -> ObjectPackingState:
        if self._state is None:
            assert self._layout is not None
            if self.config.reset_joint_positions is None:
                raise RuntimeError("Native object simulation requires its calibrated home pose")
            self._state = ObjectPackingState(
                engine.model,
                engine.data,
                self._layout,
                np.asarray(self.config.reset_joint_positions[:20]),
            )
        return self._state

    def _publish_goal(self, frame: Image) -> None:
        engine = self._engine
        if engine is None:
            return
        with engine._lock:
            if self._initial is None:
                return
            goal = self._ensure_state(engine).goal()
        self.object_goal.publish(VectorObservation(ts=frame.ts, values=tuple(map(float, goal))))

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        super()._publish_shm_and_lcm(engine)
        with engine._lock:
            state = self._ensure_state(engine)
            if self._initial is None:
                return
            state.observe()
            if engine.data.time - self._last_check >= 0.05:
                self._last_check = float(engine.data.time)
                try:
                    state.validate(self._initial)
                except RuntimeError as exc:
                    self._error = str(exc)

    @rpc
    def is_simulation_running(self) -> bool:
        """Check the native physics loop, including viewer closure."""
        engine = self._engine
        return bool(engine and engine._sim_thread and engine._sim_thread.is_alive())

    @rpc
    def select_object(self, index: int, grasp_only: bool = False) -> dict[str, Any]:
        """Assign an object and available tray slot while ACT is stopped at home."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            state = self._ensure_state(engine)
            if self._error is not None:
                raise RuntimeError("Reset the scene after the failed manipulation")
            if (
                engine.data.time < 0.6
                or np.max(np.abs(engine.data.qpos[state.qids] - state.home)) >= 0.015
            ):
                raise RuntimeError("Wait for the open gripper to return home before selecting")
            if not state.select_object(index, grasp_only=grasp_only):
                self._initial = None
                return {"selected": False, "reason": "tray_full"}
            self._initial = state.inventory()
            state.validate(self._initial)
            return {
                "selected": True,
                "object": state.layout.objects[index].name,
                "goal": state.target.tolist(),
            }

    @rpc
    def prepare_object_place(self) -> dict[str, Any]:
        """Plan free tray space for the currently held object without resetting grasp history."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            state = self._ensure_state(engine)
            if self._initial is None or not state.holding():
                raise RuntimeError("Pick and hold an object before placing")
            state.validate(self._initial)
            target = state.placement_target(state.selected)
            if target is None:
                return {"selected": False, "reason": "tray_full; object remains held"}
            state.target = target
            self._error = None
            return {
                "selected": True,
                "object": state.layout.objects[state.selected].name,
                "goal": target.tolist(),
            }

    @rpc
    def object_state(self) -> dict[str, Any]:
        """Read current inventory, selection, collision evidence and completion."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            state = self._ensure_state(engine)
            base = engine.data.body("base_link")
            rows = state.inventory()
            for i, row in enumerate(rows):
                relative = base.xmat.reshape(3, 3).T @ (np.asarray(row["position"]) - base.xpos)
                row.update(
                    index=i,
                    rgba=list(state.layout.objects[i].rgba),
                    id=f"object_{i + 1}",
                    forward_m=float(relative[0]),
                    left_m=float(relative[1]),
                    distance_m=float(np.linalg.norm(relative[:2])),
                )
            return {
                "seed": state.layout.seed,
                "supported_arms": ["right"],
                "source": "simulator_ground_truth",
                "holding": self._initial is not None and state.holding(),
                "held_object": f"object_{state.selected + 1}"
                if self._initial is not None and state.holding()
                else None,
                "objects": rows,
                "selected": state.selected if self._initial is not None else None,
                "result": state.result().to_dict() if self._initial is not None else None,
                "pick_complete": self._initial is not None
                and self._error is None
                and state.pick_complete(),
                "at_home": bool(np.max(np.abs(engine.data.qpos[state.qids] - state.home)) < 0.015),
                "error": self._error,
                "robot_obstacles": state.guard.collisions(engine.data, ignore_cargo=True),
                "sim_time": float(engine.data.time),
                "wall_time": time.time(),
            }

    @rpc
    def reset(self) -> bool:
        """Explicitly reset this generated scene after stopping all control tasks."""
        applied = super().reset()
        if applied and self._engine is not None:
            with self._engine._lock:
                self._state = None
                self._initial = None
                self._error = None
                self._last_check = 0.0
        return applied

    @rpc
    def plan_object_recovery(self) -> list[dict[str, Any]]:
        """Plan a supported release and empty-hand retreat without teleporting objects."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            state = self._ensure_state(engine)
            snapshot = copy.copy(engine.data)
            layout, home = state.layout, state.home.copy()
            self._initial = None
        return plan_object_recovery(engine.model, snapshot, layout, home)

    @rpc
    def finish_object_recovery(self) -> dict[str, Any]:
        """Clear failed-rollout metadata only after measured open-handed return home."""
        engine = self._engine
        if engine is None:
            raise RuntimeError("Simulation has not started")
        with engine._lock:
            state = self._ensure_state(engine)
            rows = state.inventory()
            if np.max(np.abs(engine.data.qpos[state.qids] - state.home)) >= 0.015:
                raise RuntimeError("Recovery has not reached the calibrated home posture")
            if any(
                not row["released"] or not row["upright"] or not row["support_geoms"]
                for row in rows
            ):
                raise RuntimeError("An object is unsupported, tipped or still held")
            if state.guard.collisions(engine.data, ignore_cargo=True):
                raise RuntimeError("Robot remains in contact with the environment")
            self._initial, self._error = None, None
            return self.object_state()

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            if self._thread_limits is not None:
                self._thread_limits.restore_original_limits()
                self._thread_limits = None
