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

from abc import ABC
from typing import Any, get_type_hints

from pydantic import BaseModel


class BaseConfig(BaseModel):
    model_config = {"arbitrary_types_allowed": True, "extra": "forbid"}


class Configurable:
    config: BaseConfig

    def __init__(self, **kwargs: Any) -> None:
        config_type = get_type_hints(type(self))["config"]
        self.config = config_type(**kwargs)


class Service(Configurable, ABC):
    def start(self) -> None:
        if hasattr(super(), "start"):
            super().start()  # type: ignore[misc]

    def stop(self) -> None:
        if hasattr(super(), "stop"):
            super().stop()  # type: ignore[misc]
