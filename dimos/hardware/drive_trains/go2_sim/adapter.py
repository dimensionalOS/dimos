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

"""Layer 1 sim TwistBase adapter for the Unitree Go2.

Wraps :class:`~dimos.utils.benchmarking.plant.Go2PlantSim` and presents
the standard :class:`TwistBaseAdapter` protocol so any controller / task /
coordinator that talks to a real Go2 base can be exercised in pure-Python
sim with no hardware.

Plant integration is wall-clock driven: each :meth:`write_velocities`
call advances the plant by ``time.perf_counter()`` delta since the last
write. The ControlCoordinator's tick loop calls write_velocities once
per tick, so the plant ticks at the coordinator's tick_rate.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.utils.benchmarking.plant import GO2_PLANT_FITTED, Go2PlantParams, Go2PlantSim

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry


class Go2SimTwistBaseAdapter:
    """FOPDT + unicycle sim posing as a real Go2 twist base.

    Implements :class:`TwistBaseAdapter`. ``dof`` is fixed at 3 for the
    holonomic Go2 model (vx, vy, wz); commanding non-zero vy is a sim
    artifact in the sim ground truth (the real Go2 strafes — characterize
    real vy via ``go2_characterization --mode hw``).
    """

    def __init__(
        self,
        dof: int = 3,
        params: Go2PlantParams | None = None,
        initial_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        nominal_dt: float = 0.1,
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"Go2SimTwistBaseAdapter requires dof=3, got {dof}")
        self._dof = dof
        self._params = params if params is not None else GO2_PLANT_FITTED
        self._plant = Go2PlantSim(self._params)
        self._initial_pose = initial_pose
        self._nominal_dt = nominal_dt

        self._cmd: list[float] = [0.0, 0.0, 0.0]
        self._last_step_time: float | None = None
        self._enabled = False
        self._connected = False

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        self._plant.reset(*self._initial_pose, dt=self._nominal_dt)
        self._last_step_time = None
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    # =========================================================================
    # Info
    # =========================================================================

    def get_dof(self) -> int:
        return self._dof

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_velocities(self) -> list[float]:
        """Return plant's actual filtered velocities (m/s, m/s, rad/s)."""
        return [self._plant.vx, self._plant.vy, self._plant.wz]

    def read_odometry(self) -> list[float] | None:
        """Return plant's integrated pose [x (m), y (m), yaw (rad)]."""
        return [self._plant.x, self._plant.y, self._plant.yaw]

    # =========================================================================
    # Control
    # =========================================================================

    def write_velocities(self, velocities: list[float]) -> bool:
        """Advance plant under ZOH of the prior cmd, then latch new cmd."""
        if len(velocities) != self._dof:
            return False
        now = time.perf_counter()
        if self._last_step_time is not None:
            dt = now - self._last_step_time
            if dt > 0:
                self._plant.step(self._cmd[0], self._cmd[1], self._cmd[2], dt)
        self._cmd = list(velocities)
        self._last_step_time = now
        return True

    def write_stop(self) -> bool:
        return self.write_velocities([0.0, 0.0, 0.0])

    # =========================================================================
    # Enable/Disable
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled

    # =========================================================================
    # Sim helpers (not part of Protocol)
    # =========================================================================

    @property
    def plant(self) -> Go2PlantSim:
        """Direct access to the underlying plant (for inspection / tests)."""
        return self._plant

    def set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        """Override the start pose. Takes effect on next :meth:`connect`."""
        self._initial_pose = (x, y, yaw)


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the registry under ``go2_sim_twist_base``."""
    registry.register("go2_sim_twist_base", Go2SimTwistBaseAdapter)


__all__ = ["Go2SimTwistBaseAdapter"]
