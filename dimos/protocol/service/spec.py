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
from typing import Generic, Type, TypeVar

# Generic type for service configuration
ConfigT = TypeVar("ConfigT")


class Service(ABC, Generic[ConfigT]):
    default_config: Type[ConfigT]

    def __init__(self, **kwargs) -> None:
        self.config: ConfigT = self.default_config(**kwargs)

    @abstractmethod
    def start(self) -> None:
        """Start the service."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Stop the service."""
        ...
