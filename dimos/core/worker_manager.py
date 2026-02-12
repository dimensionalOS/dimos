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

from typing import Any

from dimos.core.module import ModuleT
from dimos.core.rpc_client import RPCClient
from dimos.core.worker import Worker
from dimos.utils.actor_registry import ActorRegistry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WorkerManager:
    def __init__(self) -> None:
        self._workers: list[Worker] = []
        self._closed = False

    def deploy(self, module_class: type[ModuleT], *args: Any, **kwargs: Any) -> RPCClient:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        worker = Worker(module_class, args=args, kwargs=kwargs)
        worker.deploy()
        self._workers.append(worker)
        return worker.get_instance()

    def deploy_parallel(
        self, module_specs: list[tuple[type[ModuleT], tuple[Any, ...], dict[Any, Any]]]
    ) -> list[RPCClient]:
        if self._closed:
            raise RuntimeError("WorkerManager is closed")

        workers: list[Worker] = []
        for module_class, args, kwargs in module_specs:
            worker = Worker(module_class, args=args, kwargs=kwargs)
            worker.start_process()
            workers.append(worker)

        for worker in workers:
            worker.wait_until_ready()
            self._workers.append(worker)

        return [worker.get_instance() for worker in workers]

    def close_all(self) -> None:
        if self._closed:
            return
        self._closed = True

        logger.info("Shutting down all workers...")

        for worker in reversed(self._workers):
            try:
                worker.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down worker: {e}", exc_info=True)

        self._workers.clear()
        ActorRegistry.clear()

        logger.info("All workers shut down")
