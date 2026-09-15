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

"""Habitat scene/launch settings for a caller-supplied dimos composition."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

from pydantic import model_validator

from dimos.evals.environments.sim import Sim, SimConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if TYPE_CHECKING:
    from dimos.e2e_tests.dimos_cli_call import DimosCliCall
    from dimos.evals.agents.base import Agent
    from dimos.memory.store.base import Store
    from dimos.simulation.habitat.connection import HabitatConnectionConfig


class HabitatEnvironmentConfig(SimConfig):
    """Scene overrides; None preserves HabitatConnectionConfig's current default.

    Paths must resolve in the Habitat subprocess. scene_id is the dataset's
    handle or a supported asset path, not a DimSim scene name. Scene selection
    belongs to the case alongside its references, rather than a global override
    of an existing DimSim QA suite.
    """

    scene_dataset_config: str | None = None
    scene_id: str | None = None
    seed: int = 0
    start_yaw_deg: float = 90.0
    # ROS world-frame floor position in meters.
    # None retains the existing seeded navigable-point selection. An explicit
    # invalid position must fail, not silently move to an unrelated room.
    start_position_ros: tuple[float, float, float] | None = None
    executable: str | None = None

    @model_validator(mode="after")
    def finite_spawn(self) -> HabitatEnvironmentConfig:
        values = (*(self.start_position_ros or ()), self.start_yaw_deg)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Habitat spawn and yaw must be finite")
        if self.attach:
            raise ValueError("Habitat evals require fresh launches; attach is not supported")
        return self


class HabitatEnvironment(Sim):
    """Habitat scene settings; Sim owns processes, MCP, recordings and cleanup."""

    config: HabitatEnvironmentConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._spawn: PoseStamped | None = None

    def preflight(self, agent: Agent) -> None:
        """Check scene inputs and the existing native build/install prerequisites.

        Reuse HabitatConnection's build flow. Do not require the separate
        habitat-py39 viewer environment or download datasets during grading.
        """
        if self.config.scene_dataset_config is not None:
            path = Path(self.config.scene_dataset_config).expanduser()
            if str(path) != "default" and not path.is_file():
                raise FileNotFoundError(path)
        if self.config.executable is not None and not Path(self.config.executable).is_file():
            raise FileNotFoundError(self.config.executable)
        super().preflight(agent)

    def connection_config(self) -> HabitatConnectionConfig:
        """Resolve overrides against existing sensor defaults, without semantic access."""
        from dimos.simulation.habitat.connection import HabitatConnectionConfig

        fields = HabitatEnvironmentConfig.model_fields.keys() - SimConfig.model_fields.keys()
        overrides = self.config.model_dump(include=fields, exclude_none=True)
        if "scene_dataset_config" in overrides and overrides["scene_dataset_config"] != "default":
            overrides["scene_dataset_config"] = str(
                Path(overrides["scene_dataset_config"]).expanduser().resolve()
            )
        return HabitatConnectionConfig(**overrides, publish_semantic=False)

    def configure_launch(self, proc: DimosCliCall) -> None:
        """Supply scene overrides and recording selection, not a skill or planner API.

        The case owns the robot/navigation/skill composition, as for DimSim.
        HabitatConnection consumes its Twist commands and publishes observations.
        """
        config = self.connection_config()
        fields = (
            *sorted(HabitatEnvironmentConfig.model_fields.keys() - SimConfig.model_fields.keys()),
            "publish_semantic",
        )
        environment = [("DIMOS_TRANSPORT", "zenoh")]
        for name in fields:
            value = getattr(config, name)
            environment.append(
                (
                    f"HABITATCONNECTION__{name.upper()}",
                    value if isinstance(value, str) else json.dumps(value),
                )
            )
        proc.simulator = None
        # Keep RGB, the derived point cloud, pose, maps and navigation
        # traces. Navigation topics come from the composed blueprint, not
        # HabitatConnection itself. Raw depth is used internally, not recorded.
        proc.global_args = [
            "--record-topics",
            ",".join(
                (
                    "color_image",
                    "camera_info",
                    "habitat_scan",
                    "odometry",
                    "tf",
                    "local_map",
                    "global_map",
                    "goal",
                    "path",
                    "cmd_vel",
                    "stop_movement",
                    "goal_reached",
                )
            ),
        ]
        proc.extra_env.update(dict(environment))

    def prepare_recording(self, recording: Store, path: Path, deadline: float) -> dict[str, Path]:
        self.wait_ready(recording, deadline=deadline)
        metadata = path.parent / "habitat_episode.json"
        metadata.write_text(json.dumps(self.episode_metadata(), indent=2))
        return {"episode": metadata}

    def wait_ready(self, recording: Store, *, deadline: float) -> None:
        """Wait for fresh observations; Sim separately checks the MCP endpoint."""
        # Message timestamps are Unix wall-clock seconds. Use wall time for
        # freshness and monotonic time only for the elapsed launch deadline.
        while time.monotonic() < deadline:
            try:
                pose = self.latest_pose(recording)
                image = recording.streams.color_image.last().data
                if time.time() - min(pose.ts, image.ts) < 10.0:
                    self._spawn = pose
                    return
            except (LookupError, AttributeError):
                pass
            time.sleep(0.1)
        raise TimeoutError("Habitat did not publish fresh RGB and odometry")

    def latest_pose(self, recording: Store) -> PoseStamped:
        """Adapt Habitat's odometry: Odometry to the settling pose contract.

        Do not assume the DimSim odom: PoseStamped stream or read commanded pose
        instead of achieved pose. Preserve the message's frame and timestamp.
        """
        if "odometry" not in recording.streams:
            raise LookupError("No Habitat odometry recorded")
        odom = recording.streams.odometry.last().data
        return PoseStamped(
            ts=odom.ts, frame_id=odom.frame_id, position=odom.position, orientation=odom.orientation
        )

    def episode_metadata(self) -> dict[str, object]:
        """Record the launch overrides and initial observed pose.

        Sensor settings belong to the caller's blueprint. Do not report fresh
        HabitatConnectionConfig defaults as if they were observed runtime settings.
        """
        config = self.connection_config()
        return {
            "backend": "habitat",
            "connection_overrides": config.model_dump(
                mode="json",
                include={
                    *(HabitatEnvironmentConfig.model_fields.keys() - SimConfig.model_fields.keys()),
                    "publish_semantic",
                },
            ),
            # Describes the scan generation method, not publication state.
            "point_cloud_source": "depth_unprojection",
            "initial_observed_position_ros": list(self._spawn.position) if self._spawn else None,
        }
