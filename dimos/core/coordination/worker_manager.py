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

from collections.abc import Hashable, Sequence
from typing import TYPE_CHECKING, Any, Protocol

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxyProtocol

logger = setup_logger()


class WorkerManager(Protocol):
    deployment_identifier: str

    def __init__(self, g: GlobalConfig) -> None: ...

    def start(self) -> None: ...

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol: ...

    def deploy_parallel(
        self,
        specs: Sequence[ModuleSpec],
        groups: Sequence["Hashable | None"] | None = None,
    ) -> list[ModuleProxyProtocol]:
        """Deploy every spec, keeping specs that share a group in one worker.

        Args:
            specs: Module class, global config and kwargs, per module.
            groups: Optional group key per spec, parallel to ``specs``. Specs
                with the same key land in one worker process, in first-seen
                order; ``None`` leaves a spec on the usual least-loaded choice.
        """
        ...

    def stop(self) -> None: ...

    def health_check(self) -> bool: ...

    def suppress_console(self) -> None: ...
