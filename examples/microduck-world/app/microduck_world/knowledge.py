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

"""Private, observation-backed place and object knowledge for one robot runtime."""

import json
import math
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from dimos.agents.annotation import skill
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.robot.pollen.microduck.skills import (
    MicroduckSkillContainer,
    MicroduckSkillContainerConfig,
)
from microduck_world.robot_io import Observation
from reactivex.disposable import Disposable


@dataclass(frozen=True)
class CameraView:
    id: str
    observation: Observation

    def agent_encode(self) -> list[dict[str, Any]]:
        image = self.observation.image
        return [
            {
                "type": "text",
                "text": json.dumps(
                    {
                        "observation_id": self.id,
                        "width": self.observation.camera_info.width,
                        "height": self.observation.camera_info.height,
                        "coordinates": "Pixels: x from left, y from top.",
                    }
                ),
            },
            *image.agent_encode(),
        ]


class KnowledgeConfig(MicroduckSkillContainerConfig):
    knowledge_dir: str


def locate_pixel(observation: Observation, x: int, y: int) -> tuple[float, float, float]:
    info = observation.camera_info
    if not 0 <= x < info.width or not 0 <= y < info.height:
        raise ValueError("Pixel is outside this camera image")
    depth = float(observation.depth.data[y, x])
    if not math.isfinite(depth) or not 0.05 <= depth <= 6:
        raise ValueError("This pixel has no usable measured depth")
    point = np.array(
        [(x - info.K[2]) * depth / info.K[0], (y - info.K[5]) * depth / info.K[4], depth]
    )
    pose = observation.camera_pose
    world = pose.orientation.to_rotation_matrix() @ point + pose.position.to_numpy()
    return float(world[0]), float(world[1]), float(world[2])


class DuckKnowledge(MicroduckSkillContainer):
    config: KnowledgeConfig
    observation: In[Observation]
    global_costmap: In[OccupancyGrid]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._observations_lock = threading.Lock()
        self._latest: Observation | None = None
        self._seen: OrderedDict[str, Observation] = OrderedDict()
        self._known_cells = 0
        self._known_area = 0.0
        self._journal: list[dict[str, Any]] = []

    @rpc
    def start(self) -> None:
        super().start()
        directory = Path(self.config.knowledge_dir)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / "observations.json"
        if path.exists():
            self._journal = json.loads(path.read_text())[-100:]
        self.register_disposable(Disposable(self.observation.subscribe(self._on_observation)))
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_map)))

    def _on_observation(self, observation: Observation) -> None:
        with self._observations_lock:
            self._latest = observation

    def _on_map(self, grid: OccupancyGrid) -> None:
        with self._observations_lock:
            self._known_cells = int(np.count_nonzero(grid.grid >= 0))
            self._known_area = self._known_cells * grid.resolution**2

    @skill
    def observe(self) -> CameraView | SkillResult:
        """Look through your own camera.

        Returns an image and observation_id for recording objects or observations.
        """
        with self._observations_lock:
            observation = self._latest
            if observation is None or time.time() - observation.image.ts > 5:
                return SkillResult.fail("EXECUTION_TIMEOUT", "No recent camera observation")
            id = f"{observation.image.ts:.6f}"
            self._seen[id] = observation
            while len(self._seen) > 8:
                self._seen.popitem(last=False)
            return CameraView(id, observation)

    def _seen_observation(self, id: str) -> Observation:
        with self._observations_lock:
            observation = self._seen.get(id)
        if observation is None:
            raise ValueError("Unknown observation. Call observe and use its observation_id.")
        return observation

    def _record(self, observation: Observation, description: str) -> str:
        description = description.strip()
        if not description or len(description) > 1000:
            raise ValueError("Description must contain 1–1000 characters")
        directory = Path(self.config.knowledge_dir)
        image_name = f"seen-{observation.image.ts:.6f}.jpg"
        if not cv2.imwrite(str(directory / image_name), observation.image.data[:, :, ::-1]):
            raise ValueError("Could not save camera evidence")
        entry = {
            "description": description,
            "observed_at": observation.image.ts,
            "image": image_name,
            "camera_position": observation.camera_pose.position.to_numpy().tolist(),
        }
        with self._observations_lock:
            self._journal = [*self._journal, entry][-100:]
            (directory / "observations.json").write_text(json.dumps(self._journal, indent=2))
        return image_name

    @skill
    def remember_object(
        self, name: str, observation_id: str, pixel_x: int, pixel_y: int, description: str = ""
    ) -> str:
        """Annotate a visible object using your camera and measured depth.

        Args:
            name: Your name for the visible object.
            observation_id: ID returned by observe.
            pixel_x: Horizontal pixel on the object, measured from the image left edge.
            pixel_y: Vertical pixel on the object, measured from the image top edge.
            description: What you observed; mention uncertainty when appropriate.
        """
        try:
            name = name.strip()
            if not name or len(name) > 100:
                raise ValueError("Object name must contain 1–100 characters")
            observation = self._seen_observation(observation_id)
            x, y, z = locate_pixel(observation, pixel_x, pixel_y)
            image = self._record(observation, description or name)
            self._places_memory().add(
                name,
                x,
                y,
                kind="object",
                metadata={
                    "source": "camera-depth",
                    "observation_id": observation_id,
                    "description": description,
                    "image": image,
                    "z": z,
                    "pixel": [pixel_x, pixel_y],
                },
            )
            self._publish_places()
            return f"Remembered {name} at ({x:.2f}, {y:.2f}) from my camera observation."
        except ValueError as exc:
            return f"Could not annotate object: {exc}"

    @skill
    def record_observation(self, observation_id: str, description: str) -> str:
        """Remember what you learned from one of your own camera observations.

        Args:
            observation_id: ID returned by observe.
            description: What you saw or inferred, explicitly noting uncertainty.
        """
        try:
            self._record(self._seen_observation(observation_id), description)
            return "Saved this observation in my private memory."
        except ValueError as exc:
            return f"Could not remember observation: {exc}"

    @skill
    def understanding(self) -> str:
        """Summarize your measured map coverage, known places and recorded observations."""
        with self._observations_lock:
            coverage = {
                "observed_cells": self._known_cells,
                "observed_area_m2": round(self._known_area, 2),
            }
            journal = list(self._journal[-20:])
        return json.dumps({"map": coverage, "places": self.list_places(), "observations": journal})
