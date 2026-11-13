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

"""Scene selection follows UsdManager: get_scene(), then get_asset() for its files."""

from pathlib import Path

from pydantic import Field, field_validator, model_validator

from dimos.core.module import ModuleConfig
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUTS, RPCSpec
from dimos.robot.assets.client import fetch_scene
from dimos.robot.assets.model import UsdModel
from dimos.robot.usd_manager import Mount


class SceneConfig(ModuleConfig):
    scene: UsdModel | Path | None = None
    scene_manager: Mount | None = None
    rpc_timeouts: dict[str, float] = Field(
        default_factory=lambda: {**DEFAULT_RPC_TIMEOUTS, "get_scene": 120.0}
    )

    @field_validator("scene_manager", mode="before")
    @classmethod
    def normalize_scene_manager(cls, value: object) -> object:
        return None if value is None else Mount.of(value)

    @model_validator(mode="after")
    def one_source(self) -> "SceneConfig":
        if (self.scene is None) == (self.scene_manager is None):
            raise ValueError("provide exactly one scene or scene_manager")
        return self

    def load_scene(self, rpc: RPCSpec) -> UsdModel:
        if self.scene_manager is not None:
            return fetch_scene(self.scene_manager.remote_name, rpc)
        if isinstance(self.scene, Path):
            return UsdModel.from_file(self.scene)
        assert self.scene is not None
        return self.scene
