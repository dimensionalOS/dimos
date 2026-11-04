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

"""Physical scene: shared robot descriptions, placements and actuator parameters."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from functools import cached_property
import hashlib
import json

from dimos.core.shared_config import ConfigSenders
from dimos.robot.assets.model import LoadedRobotModel, RobotModel


@dataclass(frozen=True)
class Drive:
    joint: str
    kp: float = 200.0
    kd: float = 20.0
    initial: float = 0.0
    armature: float = 0.0  # reflected rotor inertia, kg m² (rotary) or kg (linear)
    friction: float = 0.0  # Coulomb joint friction, Nm or N


@dataclass(frozen=True)
class Instance:
    id: str
    model: RobotModel | LoadedRobotModel | None = None
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    floating: bool = False
    drives: tuple[Drive, ...] = ()
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def description(self) -> LoadedRobotModel:
        if self.model is None:
            raise ValueError(f"model {self.id} has not been resolved")
        return self.model.load() if isinstance(self.model, RobotModel) else self.model


@dataclass(frozen=True)
class SceneConfig:
    instances: tuple[Instance, ...]
    gravity_world: tuple[float, float, float] = (0.0, 0.0, -9.81)

    def resolve(self, config_senders: ConfigSenders) -> SceneConfig:
        """Bind instance IDs to shared models before any consumer builds or starts."""
        models: dict[str, LoadedRobotModel] = {}
        for sender, config in config_senders:
            contribution = config.get("robot_models", {})
            if not isinstance(contribution, dict):
                raise TypeError(f"{sender}: robot_models must be a mapping")
            for name, model in contribution.items():
                if not isinstance(name, str) or not isinstance(model, LoadedRobotModel):
                    raise TypeError(f"{sender}: robot_models must map IDs to LoadedRobotModel")
                if name in models:
                    raise ValueError(f"duplicate shared model {name} from {sender}")
                models[name] = model
        instances = []
        for instance in self.instances:
            model = models.get(instance.id)
            if model is None:
                model = instance.description
            elif instance.model is not None and instance.description.xml != model.xml:
                raise ValueError(f"conflicting model definitions for {instance.id}")
            instances.append(replace(instance, model=model))
        return replace(self, instances=tuple(instances))

    @cached_property
    def revision(self) -> str:
        """Identity of the materialized description; not a native solver checkpoint."""
        payload = {
            "gravity_world": self.gravity_world,
            "instances": [
                {
                    "id": i.id,
                    "xml": i.description.xml,
                    "xyz": i.xyz,
                    "rpy": i.rpy,
                    "floating": i.floating,
                    "drives": [asdict(d) for d in i.drives],
                }
                for i in self.instances
            ],
        }
        return hashlib.sha256(
            json.dumps(json.loads(json.dumps(payload), parse_int=float), sort_keys=True).encode()
        ).hexdigest()
