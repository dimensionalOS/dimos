# Copyright 2025-2026 Dimensional Inc.
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

"""Robot-side connection module for the FOPDT twist-base sim plant.

Mirrors :class:`dimos.robot.unitree.go2.connection.GO2Connection`'s shape
so the rest of the stack (ControlCoordinator + ``transport_lcm`` adapter
+ tasks) is identical between sim and hw. The only thing that differs is
which connection module the operator brings up:

    sim:  dimos run coordinator-sim-fopdt
    hw:   dimos run unitree-go2-webrtc-keyboard-teleop

This module:
- subscribes ``cmd_vel: In[Twist]`` from the bus,
- integrates a :class:`TwistBasePlantSim` under the latest command at a
  fixed tick rate (ZOH between callbacks),
- publishes ``odom: Out[PoseStamped]`` from the integrated unicycle pose.
"""

from __future__ import annotations

from threading import Event, Thread
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.benchmarking.plant import (
    GO2_PLANT_FITTED,
    TwistBasePlantParams,
    TwistBasePlantSim,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FopdtPlantConnectionConfig(ModuleConfig):
    """Sim plant runtime config.

    ``plant_params`` defaults to the vendored Go2 fit so a bare blueprint
    works out of the box. ``tick_rate_hz`` controls how often the plant
    integrates and republishes odom — matches the coordinator's tick
    rate by convention so the sim ticks at the same cadence as control.
    ``frame_id`` is stamped on published PoseStamped messages.
    """

    plant_params: TwistBasePlantParams = GO2_PLANT_FITTED
    tick_rate_hz: float = 10.0
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_yaw: float = 0.0
    frame_id: str = "odom"


class FopdtPlantConnection(Module):
    """In-process FOPDT twist-base sim posing as a real robot connection.

    Wire shape (LCM topics) is identical to a real twist-base bring-up:
    consume Twist on ``cmd_vel``, publish PoseStamped on ``odom``. The
    coordinator on the other side uses ``transport_lcm`` exactly as it
    does against hw — there is no per-mode adapter.
    """

    config: FopdtPlantConnectionConfig
    cmd_vel: In[Twist]
    odom: Out[PoseStamped]

    _plant: TwistBasePlantSim
    _cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    _stop_event: Event
    _thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._plant = TwistBasePlantSim(self.config.plant_params)
        self._stop_event = Event()

    @rpc
    def start(self) -> None:
        super().start()
        dt = 1.0 / self.config.tick_rate_hz
        self._plant.reset(self.config.initial_x, self.config.initial_y, self.config.initial_yaw, dt)
        self._cmd = (0.0, 0.0, 0.0)
        self._stop_event.clear()

        unsub = self.cmd_vel.subscribe(self._on_cmd_vel)
        self.register_disposable(Disposable(unsub))

        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(
            f"FopdtPlantConnection started @ {self.config.tick_rate_hz:g} Hz "
            f"(initial pose=({self.config.initial_x:.2f}, {self.config.initial_y:.2f}, "
            f"{self.config.initial_yaw:.2f}))"
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._thread = None
        super().stop()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))

    def _run(self) -> None:
        period = 1.0 / self.config.tick_rate_hz
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            vx, vy, wz = self._cmd
            self._plant.step(vx, vy, wz, period)

            pose = PoseStamped(
                ts=time.time(),
                frame_id=self.config.frame_id,
                position=Vector3(self._plant.x, self._plant.y, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, self._plant.yaw)),
            )
            self.odom.publish(pose)

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                self._stop_event.wait(sleep_for)
            else:
                next_tick = time.perf_counter()


__all__ = ["FopdtPlantConnection", "FopdtPlantConnectionConfig"]
