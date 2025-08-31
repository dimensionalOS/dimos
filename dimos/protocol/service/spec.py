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

from abc import ABC, abstractmethod
from typing import Any, Dict, Generic, Type, TypeVar

# Generic type for service configuration
ConfigT = TypeVar("ConfigT")


class ConfigBase:
    """Base class for configuration that supports inheritance and field overrides."""

    def __init__(self, **kwargs):
        # Get all class attributes that aren't methods or special attributes
        for key, value in self.__class__.__dict__.items():
            if not key.startswith("_") and not callable(value):
                setattr(self, key, value)

        # Override with any provided kwargs
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __repr__(self):
        attrs = {k: v for k, v in self.__dict__.items() if not k.startswith("_")}
        return f"{self.__class__.__name__}({', '.join(f'{k}={v!r}' for k, v in attrs.items())})"


class Configurable(Generic[ConfigT]):
    default_config: Type[ConfigT]

    def __init__(self, **kwargs) -> None:
        self.config: ConfigT = self.default_config(**kwargs)


class Service(Configurable[ConfigT], ABC):
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...
