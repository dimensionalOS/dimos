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

import json
from pathlib import Path
import time
from typing import Any

import numpy as np
from pydantic import TypeAdapter
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.imitation.observation import VectorObservation
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.galaxea.r1pro.object_packing_scene import ObjectLayout
from dimos.robot.galaxea.r1pro.object_packing_state import ObjectPackingState
from dimos.simulation.engines.mujoco_engine import MujocoEngine
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


class R1ProObjectPackingSim(MujocoSimModule):
    right_wrist: Out[Image]
    object_goal: Out[VectorObservation]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._layout: ObjectLayout | None = None
        self._state: ObjectPackingState | None = None
        self._initial: list[dict[str, Any]] | None = None
        self._last_check = 0.0
        self._error: str | None = None

    @rpc
    def build(self) -> None:
        metadata = Path(self.config.address).with_suffix(".objects.json")
        self._layout = TypeAdapter(ObjectLayout).validate_python(json.loads(metadata.read_text()))
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
    def select_object(self, index: int) -> dict[str, Any]:
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
            if not state.select_object(index):
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
                    id=f"object_{i + 1}",
                    forward_m=float(relative[0]),
                    left_m=float(relative[1]),
                    distance_m=float(np.linalg.norm(relative[:2])),
                )
            return {
                "seed": state.layout.seed,
                "objects": rows,
                "selected": state.selected if self._initial is not None else None,
                "result": state.result().to_dict() if self._initial is not None else None,
                "pick_complete": self._initial is not None
                and self._error is None
                and state.pick_complete(),
                "at_home": bool(np.max(np.abs(engine.data.qpos[state.qids] - state.home)) < 0.015),
                "error": self._error,
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
