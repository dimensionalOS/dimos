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
from typing import TYPE_CHECKING

from pydantic import Field

from dimos.evals.environments.lib.recorded_poses import last_body_transform
from dimos.evals.environments.sim import Sim, SimConfig

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.memory.store.base import Store


class MujocoEnvironmentConfig(SimConfig):
    tracked_bodies: tuple[str, ...] = ()
    settle_on_joints: bool = True
    module_env: dict[str, str] = Field(default_factory=dict)
    ready_streams: tuple[str, ...] = ("color_image", "coordinator_joint_state")
    recorded_topics: tuple[str, ...] = (
        "color_image",
        "camera_info",
        "coordinator_joint_state",
        "tf",
        "body_poses",
        "odom",
    )
    # Full xArm MJCF for the MujocoSimModule; the coordinator's sim adapter attaches to it.
    scene: Path | None = None
    base_height: float | None = Field(default=None, ge=0, allow_inf_nan=False)


class MujocoEnvironment(Sim):
    """Run agent evaluations in a MuJoCo scene, with ground-truth object poses recorded on body_poses."""

    config: MujocoEnvironmentConfig

    def configure_launch(self, proc: DimosCliCall) -> None:
        proc.simulator = "mujoco"
        proc.global_args = ["--record-topics", ",".join(self.config.recorded_topics)]
        if self.config.scene is not None:
            proc.extra_env["MUJOCOSIMMODULE__ADDRESS"] = str(self.config.scene.resolve())
        proc.extra_env.update(self.config.module_env)
        if self.config.base_height is not None:
            proc.extra_env["MANIPULATIONMODULE__MODEL__BASE_POSE"] = json.dumps(
                {"frame_id": "world", "position": [0.0, 0.0, self.config.base_height]}
            )
        proc.extra_env.setdefault(
            "MUJOCOSIMMODULE__HEADLESS", os.environ.get("MUJOCOSIMMODULE__HEADLESS", "true")
        )
        if self.config.tracked_bodies:
            proc.extra_env["MUJOCOSIMMODULE__TRACKED_BODIES"] = json.dumps(
                list(self.config.tracked_bodies)
            )

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        self.wait_ready(recording, deadline=deadline)
        return {}

    def wait_ready(self, recording: Store, *, deadline: float) -> None:
        """Wait for fresh samples on every ready stream and a pose for every tracked body."""
        while time.monotonic() < deadline:
            try:
                ages = [
                    time.time() - getattr(recording.streams, name).last().data.ts
                    for name in self.config.ready_streams
                    if name in recording.streams
                ]
                for body in self.config.tracked_bodies:
                    last_body_transform(recording, body)
            except (LookupError, AttributeError):
                ages = []
            if len(ages) == len(self.config.ready_streams) and all(age < 10.0 for age in ages):
                return
            time.sleep(0.1)
        raise TimeoutError(
            f"MuJoCo did not publish fresh {', '.join(self.config.ready_streams)}"
            + (
                f" and poses for {', '.join(self.config.tracked_bodies)}"
                if self.config.tracked_bodies
                else ""
            )
        )
