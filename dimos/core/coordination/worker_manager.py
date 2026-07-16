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
from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxyProtocol

logger = setup_logger()


class WorkerManager(ABC):
    deployment_identifier: str

    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol: ...

    @abstractmethod
    def deploy_parallel(
        self,
        specs: Sequence[ModuleSpec],
        blueprint_args: Mapping[str, Mapping[str, Any]],
    ) -> list[ModuleProxyProtocol]: ...

    @abstractmethod
    def deploy_fresh(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol: ...

    @abstractmethod
    def undeploy(self, proxy: ModuleProxyProtocol) -> None: ...

    def prepare_for_load(self, n_extra: int, has_modules: bool) -> None:
        """Give a manager an opportunity to scale before a blueprint is loaded."""
        return None

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def health_check(self) -> bool: ...

    @abstractmethod
    def suppress_console(self) -> None: ...
