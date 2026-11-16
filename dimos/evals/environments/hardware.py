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

"""A real robot driven through ``dimos run``, with an operator in the loop.

The operator readies the scene before each case and confirms it; the code never
moves the robot to reset it. When a case ends the robot is halted in place, then
dimos stops the way ``dimos stop`` does, so each module's stop() runs (a Go2 lies down).
"""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.evals.environments.live import LiveEnvironment, LiveEnvironmentConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Sequence

    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.evals.agents.base import EvalSubject
    from dimos.evals.types import RunningEnvironment

logger = setup_logger()


class HardwareEnvironmentConfig(LiveEnvironmentConfig):
    task: str
    """The case's instruction, shown to the operator before the robot starts."""
    stop_tool: str | None = None
    """Skill that halts motion before the robot disconnects, e.g. ``cancel``."""
    stop_calls: tuple[tuple[str, dict[str, str]], ...] = ()
    """Skills with arguments run in order after ``stop_tool``, e.g. to park an arm
    whose motors go limp on disconnect."""
    parked_presets: dict[str, str] = Field(default_factory=dict)
    """Planning group to the joint preset it must be at once ``stop_calls`` ran, e.g.
    ``{"left_manipulator": "init"}``. If a group is not there, or a stop call failed,
    the operator is asked to hold the arm before dimos stops."""
    parked_tolerance_rad: float = 0.05
    module_env: dict[str, str] = Field(default_factory=dict)
    launch_timeout_s: float = 300.0
    graceful_stop: bool = True


class HardwareEnvironment(LiveEnvironment):
    """A real robot; the operator resets the scene and confirms it is ready."""

    config: HardwareEnvironmentConfig

    def preflight(self, agent: EvalSubject) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("hardware evals need an operator at an interactive terminal")
        super().preflight(agent)

    def start(self, modules: Sequence[str], *, requires_mcp: bool = False) -> RunningEnvironment:
        answer = input(
            f"\nNext eval task: {self.config.task}\n"
            "Is the scene reset and the robot ready, with the area clear? [y/N] "
        )
        if not answer.strip().lower().startswith("y"):
            raise RuntimeError("operator did not confirm the robot is ready")
        return super().start(modules, requires_mcp=requires_mcp)

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = None  # the real robot: no --simulation
        proc.extra_env.update(self.config.module_env)

    def stop(self) -> None:
        try:
            # A recording means start() finished and the agent may have moved the robot.
            if self.config.stop_tool is not None and self._recording is not None:
                problem: str | None = None
                try:
                    # The client loads OpenCV; only a case that ran needs it.
                    from dimos.porcelain.dimos import Dimos

                    app = Dimos.connect(timeout=5.0)
                    try:
                        getattr(app.skills, self.config.stop_tool)()
                        for tool, kwargs in self.config.stop_calls:
                            result = getattr(app.skills, tool)(**kwargs)
                            logger.info("stop call", tool=tool, args=kwargs, result=str(result))
                        problem = self._unparked(app)
                    finally:
                        app.stop()
                except Exception as e:
                    problem = f"halt or park failed: {e}"
                    logger.error(
                        "HALT FAILED: the robot may still be moving, use the e-stop",
                        tool=self.config.stop_tool,
                        error=e,
                    )
                if problem is not None and (self.config.stop_calls or self.config.parked_presets):
                    # Stopping dimos may turn the motors off, and an arm left up would drop.
                    logger.error("ROBOT NOT PARKED", problem=problem)
                    input(
                        f"\nROBOT NOT PARKED: {problem}\n"
                        "Hold or support the arm (or use the e-stop), then press Enter to stop dimos. "
                    )
        finally:
            super().stop()

    def _unparked(self, app: object) -> str | None:
        """Why a group is not at its parked preset, or None when all are."""
        if not self.config.parked_presets:
            return None
        groups = app.ManipulationModule.get_state().groups  # type: ignore[attr-defined]
        for group, preset in self.config.parked_presets.items():
            state = groups.get(group)
            target = None if state is None else state.joint_presets.get(preset)
            if state is None or target is None or state.joints is None:
                return f"{group} has no {preset!r} preset or joint reading"
            now = dict(zip(state.joints.name, state.joints.position, strict=True))
            off = max(
                abs(now[name] - goal)
                for name, goal in zip(target.name, target.position, strict=True)
            )
            if off > self.config.parked_tolerance_rad:
                return f"{group} is {off:.2f} rad from {preset!r}"
            logger.info("parked", group=group, preset=preset, off_rad=round(off, 3))
        return None
