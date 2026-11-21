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

"""Stack composition, process management, and shared transport settings.

Module and subsystem settings belong to their owners' configuration classes.
"""

import os

from pydantic import ValidationInfo, field_validator
from pydantic_settings import SettingsConfigDict

from dimos.core.global_config_fields import (
    GlobalConfigFields,
    TransportBackend as TransportBackend,
    ZenohMode as ZenohMode,
    ZenohProcessMode as ZenohProcessMode,
)

# pytest exports PYTEST_VERSION to the whole process tree. Tests must not pick up
# a developer's .env (ROBOT_IP, SIMULATION, ...); dimos/conftest.py exports the
# LLM API keys itself.
ENV_FILE = None if "PYTEST_VERSION" in os.environ else ".env"


class GlobalConfig(GlobalConfigFields):
    model_config = SettingsConfigDict(
        env_file=ENV_FILE,
        env_file_encoding="utf-8",
        extra="ignore",
        validate_assignment=True,
    )

    @field_validator("record_engine")
    @classmethod
    def _validate_record_engine(cls, value: str, info: ValidationInfo) -> str:
        if info.data.get("record") == "mcap" and value != "rust":
            raise ValueError("MCAP recording requires --record-engine rust")
        return value

    @field_validator("record_encoding_threads")
    @classmethod
    def _validate_record_encoding_threads(
        cls, value: int | None, info: ValidationInfo
    ) -> int | None:
        if value is not None and info.data.get("record_engine") != "rust":
            raise ValueError("--record-encoding-threads is valid only with --record-engine rust")
        return value

    def update(self, **kwargs: object) -> None:
        """Update config fields in place."""
        for key, value in kwargs.items():
            if key not in type(self).model_fields:
                raise AttributeError(f"GlobalConfig has no field '{key}'")
            setattr(self, key, value)

    @property
    def parsed_robot_ips(self) -> tuple[str, ...]:
        """Parse robot IPs without requiring them during blueprint discovery."""
        return tuple(x.strip() for x in (self.robot_ips or "").split(",") if x.strip())

    @property
    def processed_robot_ips(self) -> tuple[str, ...]:
        ips = self.parsed_robot_ips
        if error := require_robot_ips(ips):
            raise ValueError(error)
        return ips


global_config = GlobalConfig()


def require_robot_ips(ips: tuple[str, ...]) -> str | None:
    """Validate the IPs used to compose a blueprint, independent of runtime globals."""
    if not ips:
        return "No robot IPs specified. Set ROBOT_IPS or --robot-ips to at least one IP."
    return None
