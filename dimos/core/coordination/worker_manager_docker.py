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
from dimos.utils.safe_thread_map import safe_thread_map

if TYPE_CHECKING:
    from dimos.core.docker_module import DockerModuleProxy
    from dimos.core.rpc_client import ModuleProxyProtocol

logger = setup_logger()


class WorkerManagerDocker:
    deployment_identifier: str = "docker"

    def __init__(self, g: GlobalConfig) -> None:
        self._cfg = g
        self._deployed: list[DockerModuleProxy] = []

    def start(self) -> None:
        pass

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        # inlined to prevent circular dependency
        from dimos.core.docker_module import DockerModuleProxy

        mod = DockerModuleProxy(module_class, g=global_config, **kwargs)
        self._deployed.append(mod)
        return mod

    def deploy_parallel(self, specs: list[ModuleSpec]) -> list[ModuleProxyProtocol]:
        # inlined to prevent circular dependency
        from dimos.core.docker_module import DockerModuleProxy

        def _deploy(spec: ModuleSpec) -> DockerModuleProxy:
            # spec = (module_class, global_config, kwargs)
            mod = DockerModuleProxy(spec[0], g=spec[1], **spec[2])
            self._deployed.append(mod)
            return mod

        try:
            return safe_thread_map(specs, _deploy)
        except:
            self.stop()
            raise

    def stop(self) -> None:
        for mod in reversed(self._deployed):
            with suppress(Exception):
                mod.stop()
        self._deployed.clear()

    def health_check(self) -> bool:
        for mod in self._deployed:
            if not mod.is_running():
                logger.error(
                    "Docker container not running",
                    module=getattr(mod, "_module_name", "unknown"),
                )
                return False
        return True

    def suppress_console(self) -> None:
        # already suppressed by default
        pass
