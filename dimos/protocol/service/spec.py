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
from typing import Any, Generic, TypeVar, get_args

from pydantic import BaseModel


class BaseConfig(BaseModel):
    model_config = {"arbitrary_types_allowed": True, "extra": "forbid"}


# Generic type for service configuration
ConfigT = TypeVar("ConfigT", bound=BaseConfig)


class Configurable(Generic[ConfigT]):
    config: ConfigT

    @classmethod
    def _resolve_config_type(cls) -> type[Any]:
        """Walk the MRO to find the concrete config type from generic params.

        Returns the first concrete BaseConfig subclass found, or the first
        TypeVar default — whichever comes first in MRO order.
        """
        for mrocls in cls.__mro__:
            for base in getattr(mrocls, "__orig_bases__", ()):
                args = get_args(base)
                if not args:
                    continue
                arg = args[0]
                if isinstance(arg, type) and issubclass(arg, BaseConfig):
                    return arg
                default = getattr(arg, "__default__", None)
                if isinstance(default, type) and issubclass(default, BaseConfig):
                    return default
        raise TypeError(f"{cls.__name__} has no concrete config type in its generic bases")

    def __init__(self, **kwargs: Any) -> None:
        self.config = self._resolve_config_type()(**kwargs)


class Service(Configurable[ConfigT], ABC):
    def start(self) -> None:
        # Only call super().start() if it exists
        if hasattr(super(), "start"):
            super().start()  # type: ignore[misc]

    def stop(self) -> None:
        # Only call super().stop() if it exists
        if hasattr(super(), "stop"):
            super().stop()  # type: ignore[misc]
