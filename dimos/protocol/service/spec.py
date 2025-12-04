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

from abc import ABC
from typing import Generic, TypeVar

# Generic type for service configuration
ConfigT = TypeVar("ConfigT")


class Configurable(Generic[ConfigT]):
    default_config: type[ConfigT]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.config: ConfigT = self.default_config(**kwargs)


class Service(Configurable[ConfigT], ABC):
    def start(self) -> None:
        # Only call super().start() if it exists
        if hasattr(super(), "start"):
            super().start()  # type: ignore[misc]

    def stop(self) -> None:
        # Only call super().stop() if it exists
        if hasattr(super(), "stop"):
            super().stop()  # type: ignore[misc]
