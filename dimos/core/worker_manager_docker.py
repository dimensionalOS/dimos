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
from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING, Any

from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.utils.logging_config import setup_logger
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.docker_module import DockerModuleProxy
    from dimos.core.rpc_client import ModuleProxyProtocol

logger = setup_logger()


class WorkerManagerDocker:
    """Manages deployment of Docker-backed modules."""

    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g
        self._deployed: list[DockerModuleProxy] = []

    def should_manage(self, module_class: type) -> bool:
        # inlined to prevent circular dependency
        from dimos.core.docker_module import is_docker_module

        return is_docker_module(module_class)

    def start(self) -> None:
        """No-op — Docker manager has no persistent workers."""

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        # inlined to prevent circular dependency
        from dimos.core.docker_module import DockerModuleProxy

        mod = DockerModuleProxy(module_class, g=global_config, **kwargs)  # type: ignore[arg-type]
        mod.build()
        self._deployed.append(mod)
        return mod

    def deploy_parallel(self, specs: list[ModuleSpec]) -> list[ModuleProxyProtocol]:
        # inlined to prevent circular dependency
        from dimos.core.docker_module import DockerModuleProxy

        def _on_errors(
            _outcomes: list[Any], successes: list[DockerModuleProxy], errors: list[Exception]
        ) -> None:
            for mod in successes:
                with suppress(Exception):
                    mod.stop()
            raise ExceptionGroup("docker deploy_parallel failed", errors)

        def _deploy_one(spec: ModuleSpec) -> DockerModuleProxy:
            mod = DockerModuleProxy(spec[0], g=spec[1], **spec[2])  # type: ignore[arg-type]
            mod.build()
            return mod

        results = safe_thread_map(specs, _deploy_one, _on_errors)
        self._deployed.extend(results)
        return results  # type: ignore[return-value]

    def stop(self) -> None:
        for mod in reversed(self._deployed):
            with suppress(Exception):
                mod.stop()
        self._deployed.clear()

    def health_check(self) -> bool:
        # TODO: in the future decide on what a meaninful health check would be
        return True

    def suppress_console(self) -> None:
        """No-op — Docker containers manage their own stdio."""
