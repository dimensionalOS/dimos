# Copyright 2025 Dimensional Inc.
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

from functools import cached_property
from pydantic_settings import BaseSettings, SettingsConfigDict


class GlobalConfig(BaseSettings):
    robot_ip: str | None = None
    use_simulation: bool = False
    use_replay: bool = False
    n_dask_workers: int = 2

    model_config = SettingsConfigDict(
        env_prefix="DIMOS_",
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        frozen=True,
    )

    @cached_property
    def unitree_connection_type(self) -> str:
        if self.use_replay:
            return "fake"
        if self.use_simulation:
            return "mujoco"
        return "webrtc"
