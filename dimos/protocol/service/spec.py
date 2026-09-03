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
import sys
from typing import Any, ClassVar, get_type_hints

from pydantic import BaseModel

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from dimos.core.global_config import TransportBackend


class BaseConfig(BaseModel):
    model_config = {"arbitrary_types_allowed": True, "extra": "forbid"}


class SessionConfig(BaseConfig):
    """One transport's session settings, as opposed to a module's own config."""

    transport: ClassVar[TransportBackend]

    def to_wire(self) -> dict[str, Any]:
        """This session as the JSON object a native module reads on stdin."""
        return {}

    def rebased(self) -> Self:
        """This config's explicit fields over the current global config defaults."""
        return type(self)(**{name: getattr(self, name) for name in self.model_fields_set})


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
