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

"""MuJoCo launch, readiness and settling for live manipulation evals."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import TYPE_CHECKING, cast

from pydantic import Field

from dimos.evals.environments.lib.recorded_poses import last_body_transform
from dimos.evals.environments.sim import Sim, SimConfig

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


_READY_STREAMS = ("color_image", "coordinator_joint_state")
_RECORDED_TOPICS = ("color_image", "camera_info", "coordinator_joint_state", "tf", "odom")


class MujocoEnvironmentConfig(SimConfig):
    tracked_bodies: tuple[str, ...] = ()
    at_rest_rad_s: float = 0.02
    module_env: dict[str, str] = Field(default_factory=dict)
    scene: Path | None = None


class MujocoEnvironment(Sim):
    """Run agent evaluations in a MuJoCo scene, with ground-truth object poses recorded on tf."""

    config: MujocoEnvironmentConfig

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "mujoco"
        proc.global_args = ["--record-topics", ",".join(_RECORDED_TOPICS)]
        proc.extra_env.update(self.config.module_env)
        proc.extra_env.setdefault(
            "MUJOCOSIMMODULE__HEADLESS", os.environ.get("MUJOCOSIMMODULE__HEADLESS", "true")
        )
        if self.config.tracked_bodies:
            proc.extra_env["MUJOCOSIMMODULE__TRACKED_BODIES"] = json.dumps(
                list(self.config.tracked_bodies)
            )
        if self.config.scene is not None:
            proc.extra_env["MUJOCOSIMMODULE__SCENE_XML"] = str(self.config.scene.resolve())

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        self.wait_ready(recording, deadline=deadline)
        return {}

    def wait_ready(self, recording: Store, *, deadline: float) -> None:
        """Wait for fresh samples on every ready stream and a pose for every tracked body."""
        while time.monotonic() < deadline:
            try:
                ages = [
                    time.time() - getattr(recording.streams, name).last().data.ts
                    for name in _READY_STREAMS
                    if name in recording.streams
                ]
                for body in self.config.tracked_bodies:
                    last_body_transform(recording, body)
            except (LookupError, AttributeError):
                ages = []
            if len(ages) == len(_READY_STREAMS) and all(age < 10.0 for age in ages):
                return
            time.sleep(0.1)
        raise TimeoutError(
            f"MuJoCo did not publish fresh {', '.join(_READY_STREAMS)}"
            + (
                f" and poses for {', '.join(self.config.tracked_bodies)}"
                if self.config.tracked_bodies
                else ""
            )
        )

    def latest_pose(self, recording: Store) -> PoseStamped:
        """Base pose from ``odom`` for floating-base robots; a fixed-base arm has none."""
        if "odom" not in recording.streams:
            raise LookupError("No MuJoCo odometry recorded")
        return cast("PoseStamped", recording.streams.odom.last().data)

    def settle(self, budget_s: float) -> None:
        """At rest once every joint is slower than ``at_rest_rad_s`` for ``at_rest_s``."""
        recording = self._recording
        if recording is None or "coordinator_joint_state" not in recording.streams:
            super().settle(budget_s)
            return
        rest_since: float | None = None
        deadline = time.monotonic() + budget_s
        while time.monotonic() < deadline:
            try:
                state = recording.streams.coordinator_joint_state.last().data
            except LookupError:
                return
            now = time.monotonic()
            if any(abs(v) > self.config.at_rest_rad_s for v in state.velocity):
                rest_since = None
            elif rest_since is None:
                rest_since = now
            elif now - rest_since >= self.config.at_rest_s:
                return
            time.sleep(self.config.settle_poll_s)
