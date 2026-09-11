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

"""Goal-conditioned bottle packing through the standard DimOS coordinator."""

from pathlib import Path
import time
from typing import Any, ClassVar

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.imitation.observation import VectorObservation
from dimos.imitation.policy.lerobot.module import R1ProPackingPolicy
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.galaxea.r1pro.grasping_blueprint import R1ProGraspingSim, build_r1pro_manipulation
from dimos.robot.galaxea.r1pro.learning import R1PRO_PACKING_TASK
from dimos.robot.galaxea.r1pro.packing import clear_pick_order
from dimos.robot.galaxea.r1pro.packing_sim import PACKING_BODIES
from dimos.robot.galaxea.r1pro.packing_state import (
    PackingMonitor,
    open_gripper_at_home,
    plan_bottle_goal,
)
from dimos.simulation.engines.mujoco_engine import MujocoEngine


class R1ProPackingSim(R1ProGraspingSim):
    """Publish an explicit simulator goal alongside each head camera observation."""

    packing_goal: Out[VectorObservation]
    cargo_bodies: ClassVar[tuple[str, ...]] = PACKING_BODIES

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._packing_monitor: PackingMonitor | None = None
        self._selected: int | None = None
        self._goal: tuple[float, ...] | None = None

    @rpc
    def start(self) -> None:
        self.register_disposable(Disposable(self.color_image.subscribe(self._publish_goal)))
        super().start()
        assert self._engine is not None
        with self._engine._lock:
            if self._packing_monitor is None:
                self._packing_monitor = PackingMonitor(self._engine.model, self._engine.data)

    @rpc
    def reset(self) -> bool:
        applied = super().reset()
        if applied and self._engine is not None:
            with self._engine._lock:
                self._packing_monitor = PackingMonitor(self._engine.model, self._engine.data)
                self._selected = None
                self._goal = None
        return applied

    def _publish_goal(self, frame: Image) -> None:
        if self._engine is None:
            return
        with self._engine._lock:
            goal = self._goal
        if goal is not None:
            self.packing_goal.publish(VectorObservation(ts=frame.ts, values=goal))

    def _publish_shm_and_lcm(self, engine: MujocoEngine) -> None:
        super()._publish_shm_and_lcm(engine)
        with engine._lock:
            if self._packing_monitor is None:
                self._packing_monitor = PackingMonitor(engine.model, engine.data)
            self._packing_monitor.observe()

    @rpc
    def packing_order(self, seed: int | None = None) -> list[int]:
        """Choose accessible sources left to right, or randomize with a seed."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            positions = tuple(
                (
                    float(self._engine.data.body(name).xpos[0]),
                    float(self._engine.data.body(name).xpos[1]),
                )
                for name in PACKING_BODIES
            )
        return clear_pick_order(
            positions,
            tuple(map(int, np.random.default_rng(seed).permutation(5)))
            if seed is not None
            else None,
        )

    @rpc
    def select_bottle(self, index: int) -> dict[str, Any]:
        """Set the next object/slot goal while the open gripper is at home."""
        if self._engine is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            data = self._engine.data
            if data.time < 0.6:
                raise RuntimeError("Wait for the scene to settle before selecting a bottle")
            if not open_gripper_at_home(data):
                raise RuntimeError(
                    "Stop the policy with the open gripper at home before selecting a bottle"
                )
            goal = plan_bottle_goal(data, index)
            if goal is None:
                self._goal = None
                return {"selected": False, "reason": "tray_full"}
            self._selected = index
            self._goal = tuple(map(float, goal))
            return {"selected": True, "bottle": index + 1, "goal": self._goal}

    @rpc
    def task_state(self) -> dict[str, Any]:
        """Expose all five bottles to the shared physical tray-delivery runner."""
        if self._engine is None or self._packing_monitor is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            state = super().task_state()
            # Packing retains lift/grasp evidence per bottle, not in the
            # single-bottle monitor inherited by the shared transport state.
            for key in ("peak_lift_m", "bilateral_grasp", "bottle_position"):
                state.pop(key)
            cargo = self._packing_monitor.report()
            return {
                **state,
                **cargo,
                **{
                    key: all(bottle[key] for bottle in cargo["bottles"])
                    for key in ("inside_bin", "released", "settled", "upright")
                },
            }

    @rpc
    def packing_state(self) -> dict[str, Any]:
        """Read all-bottle evidence and selected-pick completion."""
        if self._engine is None or self._packing_monitor is None:
            raise RuntimeError("Simulation has not started")
        with self._engine._lock:
            return {
                **self._packing_monitor.report(),
                "wall_time": time.time(),
                "right_tcp": self._engine.data.site("right_tcp").xpos.tolist(),
                "ready_for_pick": bool(self._engine.data.time >= 0.6),
                "selected": self._packing_monitor.bottle_state(self._selected)
                if self._selected is not None
                else None,
            }


def build_r1pro_packing(
    *, scene_path: Path, artifact: str, device: str = "cuda", headless: bool = False
) -> Blueprint:
    """Use ACT trajectories for each pick and explicit geometry for empty slots."""
    return build_r1pro_manipulation(
        scene_path=scene_path,
        artifact=artifact,
        device=device,
        headless=headless,
        simulator=R1ProPackingSim,
        policy_module=R1ProPackingPolicy,
        task_description=R1PRO_PACKING_TASK,
        background_camera_rendering=True,
        viewer_lookat=(0.2, -0.25, 1.0),
        viewer_distance=1.8,
        viewer_azimuth=225.0,
        viewer_elevation=-30.0,
    )
