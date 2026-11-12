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

from collections.abc import Callable
from itertools import pairwise
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalStatus
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.orchestration.demo.demo_orchestrator import demo_orchestrator
from dimos.orchestration.loader import load_and_check
from dimos.utils.testing.waiting import wait_until

PATROL = Path(__file__).parent / "patrol.toml"

# The delays of the demo, cut down so that a route loop and a charge cycle fit in a few seconds.
FAST = {
    "fakebase": {"arrive_s": 0.05, "spin_s": 0.05},
    "fakebattery": {"period_s": 0.05, "drain_per_s": 6.0, "charge_per_s": 400.0},
    "fakeestop": {"period_s": 0.05},
}


class Recorder(Module):
    """Keeps what the fake base and the fake battery publish."""

    status: In[GoalStatus]
    percent: In[Float32]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._arrived: list[str] = []
        self._levels: list[float] = []

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.status.subscribe(self._on_status)))
        self.register_disposable(Disposable(self.percent.subscribe(self._on_percent)))

    def _on_status(self, status: GoalStatus) -> None:
        self._arrived.append(status.goal_id.id)

    def _on_percent(self, percent: Float32) -> None:
        self._levels.append(percent.data)

    @rpc
    def arrived(self) -> list[str]:
        return list(self._arrived)

    @rpc
    def levels(self) -> list[float]:
        return list(self._levels)


def _charge_cycle(levels: list[float], low_battery: float, full_battery: float) -> bool:
    """Whether the battery ran low, was charged to full, and is draining again."""
    low = next((i for i, level in enumerate(levels) if level < low_battery), None)
    if low is None:
        return False
    full = next((i for i in range(low, len(levels)) if levels[i] >= full_battery), None)
    return full is not None and any(later < earlier for earlier, later in pairwise(levels[full:]))


def test_the_demo_runs_a_route_loop_and_a_charge_cycle(
    tmp_path: Path, build: Callable[..., ModuleCoordinator]
) -> None:
    constants = load_and_check(PATROL).constants
    low, full = constants["low_battery"], constants["full_battery"]
    assert isinstance(low, float) and isinstance(full, float)
    text = PATROL.read_text()
    assert text.count("dwell_s = 2.0") == 1
    patrol = tmp_path / "patrol.toml"
    patrol.write_text(text.replace("dwell_s = 2.0", "dwell_s = 0.15"))

    blueprint = autoconnect(demo_orchestrator, Recorder.blueprint())
    coordinator = build(blueprint, {"orchestrator": {"file": str(patrol)}, **FAST})
    recorder = coordinator.get_instance(Recorder)
    wait_until(
        lambda: _charge_cycle(recorder.levels(), low, full),
        timeout=30.0,
        message="the battery never went through a charge cycle",
    )
    arrived: list[str] = recorder.arrived()
    levels: list[float] = recorder.levels()

    # The second goal raises, so no status carries its id, and the patrol carries on.
    assert arrived[:3] == ["goal-1", "goal-3", "goal-4"]
    assert _charge_cycle(levels, low, full)
