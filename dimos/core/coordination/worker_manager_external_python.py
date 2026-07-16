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

from collections.abc import Mapping, Sequence
from typing import Any, cast

from dimos.core.coordination.external_python_worker import ExternalPythonWorker
from dimos.core.coordination.worker_manager import WorkerManager
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleBase, ModuleSpec
from dimos.core.rpc_client import ModuleProxyProtocol, RPCClient
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _merge_config_kwargs(kwargs: dict[str, Any], overrides: Mapping[str, Any]) -> dict[str, Any]:
    merged = dict(kwargs)
    merged.update(overrides)
    return merged


class WorkerManagerExternalPython(WorkerManager):
    deployment_identifier = "external-python"

    def __init__(self, g: GlobalConfig) -> None:
        super().__init__(g)
        self._workers: dict[ModuleProxyProtocol, ExternalPythonWorker] = {}
        self._module_constructor_kwargs: dict[ModuleProxyProtocol, dict[str, Any]] = {}
        self._closed = False

    def start(self) -> None:
        return

    def deploy(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")
        constructor_kwargs = dict(kwargs)
        constructor_kwargs["g"] = global_config
        worker = ExternalPythonWorker(module_class, constructor_kwargs)
        try:
            worker.start()
        except BaseException:
            # A worker is not registered until startup succeeds, so clean up
            # any process resources it may have created before re-raising.
            try:
                worker.stop()
            except Exception:
                logger.exception("Failed to clean up external Python worker startup")
            raise
        proxy = cast("ModuleProxyProtocol", RPCClient.remote(module_class))
        self._workers[proxy] = worker
        self._module_constructor_kwargs[proxy] = dict(kwargs)
        return proxy

    def deploy_parallel(
        self, specs: Sequence[ModuleSpec], blueprint_args: Mapping[str, Mapping[str, Any]]
    ) -> list[ModuleProxyProtocol]:
        return [
            self.deploy(
                module_class,
                global_config,
                _merge_config_kwargs(kwargs, blueprint_args.get(module_class.name, {})),
            )
            for module_class, global_config, kwargs in specs
        ]

    def deploy_fresh(
        self,
        module_class: type[ModuleBase],
        global_config: GlobalConfig,
        kwargs: dict[str, Any],
    ) -> ModuleProxyProtocol:
        return self.deploy(module_class, global_config, kwargs)

    def undeploy(self, proxy: ModuleProxyProtocol) -> None:
        worker = self._workers.pop(proxy, None)
        if worker is None:
            raise ValueError("Proxy is not managed by external Python workers")
        worker.stop()
        self._module_constructor_kwargs.pop(proxy, None)

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        for worker in self._workers.values():
            worker.stop()
        self._workers.clear()
        self._module_constructor_kwargs.clear()

    def health_check(self) -> bool:
        healthy = True
        for _proxy, worker in self._workers.items():
            if worker.pid is not None:
                continue
            module_name = worker.declaration.__name__
            try:
                diagnostics = worker.diagnostics()[:4096]
            except Exception as error:
                diagnostics = f"diagnostics unavailable: {error!r}"
            logger.error(
                "External Python worker failed health check",
                module=module_name,
                diagnostics=diagnostics,
            )
            healthy = False
        return healthy

    def suppress_console(self) -> None:
        return
