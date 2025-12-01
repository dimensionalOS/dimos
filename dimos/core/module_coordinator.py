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

import time
from typing import TypeVar

from dimos import core
from dimos.core import DimosCluster, Module
from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource

T = TypeVar("T", bound="Module")


class ModuleCoordinator(Resource):
    _client: DimosCluster | None = None
    _n: int | None = None
    _memory_limit: str = "auto"
    _deployed_modules: dict[type[Module], Module] = {}

    def __init__(
        self,
        n: int | None = None,
        memory_limit: str = "auto",
        global_config: GlobalConfig | None = None,
    ) -> None:
        """Initialize a ModuleCoordinator for managing distributed module lifecycle.

        ModuleCoordinator orchestrates three core responsibilities: cluster lifecycle
        management, type-indexed module registry (one instance per module class), and
        clean shutdown coordination in reverse deployment order.

        Args:
            n: Number of Dask worker processes. If None, uses global_config.n_dask_workers.
            memory_limit: Memory limit per worker (e.g., "4GB", "500MB", "auto").
            global_config: System-wide settings (robot IP, simulation mode, worker count).
                If None, creates default GlobalConfig instance.

        The coordinator initializes in "Initialized" state with no active cluster.
        Configuration cascades: n parameter → global_config.n_dask_workers → defaults.

        Examples:
            Standard workflow with explicit configuration:

            >>> from dimos.core import Module
            >>> class CameraModule(Module):
            ...     def __init__(self, resolution=1080):
            ...         super().__init__()
            ...         self.resolution = resolution
            >>> coordinator = ModuleCoordinator(n=2, memory_limit="4GB")
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: 4GB
            >>> camera = coordinator.deploy(CameraModule, resolution=1080)
            >>> coordinator.start_all_modules()
            >>> coordinator.stop()

            Using global configuration object:

            >>> config = GlobalConfig(n_dask_workers=2, robot_ip="192.168.1.1")
            >>> coordinator = ModuleCoordinator(global_config=config)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> coordinator.stop()
        """
        cfg = global_config or GlobalConfig()
        self._n = n if n is not None else cfg.n_dask_workers
        self._memory_limit = memory_limit

    def start(self) -> None:
        """Start the underlying Dask cluster with configured parameters.

        Initializes a Dask LocalCluster with the specified number of workers and
        memory limits, installs signal handlers for graceful shutdown, and
        transitions the coordinator from Initialized to Started state.

        Preconditions:
            - `start()` has not been called yet
            - Worker count and memory limit configured via `__init__`

        Side effects:
            - Spawns the configured number of worker processes via Dask LocalCluster
            - Allocates system resources (CPU, memory) for workers
            - Installs signal handlers for graceful shutdown
            - Creates shared memory segments for IPC
            - Initializes ActorRegistry for distributed actors

        Raises:
            RuntimeError: If called when the cluster is already running.
            Exception: Any error from `core.start()` propagates.

        Examples:
            Full deployment workflow:

            >>> from dimos.core import Module
            >>> class CameraModule(Module):
            ...     pass
            >>> class ProcessorModule(Module):
            ...     pass
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> camera = coordinator.deploy(CameraModule)
            >>> processor = coordinator.deploy(ProcessorModule)
            >>> coordinator.start_all_modules()
            >>> coordinator.stop()
        """
        self._client = core.start(self._n, self._memory_limit)

    def stop(self) -> None:
        """Perform orderly shutdown of all modules and the cluster.

        Stops all deployed modules in reverse deployment order and closes the Dask
        cluster. This method is idempotent and can be safely called multiple times.

        Side Effects:
            - Calls `stop()` on each deployed module in reverse order
            - Closes Dask cluster, cleaning up worker processes, shared memory,
              signal handlers, event loops, and CUDA resources if applicable
            - If any module's `stop()` raises an exception, the error is logged
              but shutdown continues for best-effort cleanup

        Examples:
            Manual cleanup after work:

            >>> from dimos.core import Module
            >>> class CameraModule(Module):
            ...     pass
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> camera = coordinator.deploy(CameraModule)
            >>> coordinator.start_all_modules()
            >>> # ... do work ...
            >>> coordinator.stop()  # Clean shutdown

            Idempotent operation is safe:

            >>> coordinator.stop()
            >>> coordinator.stop()  # Safe to call again
        """
        for module in reversed(self._deployed_modules.values()):
            module.stop()

        self._client.close_all()  # type: ignore[union-attr]

    def deploy(self, module_class: type[T], *args, **kwargs) -> T:  # type: ignore[no-untyped-def]
        """Deploy a Module class as a distributed actor on the Dask cluster.

        The coordinator maintains at most one instance per module class. If the same
        class is deployed twice, the second deployment overwrites the first in the
        registry, though the original actor remains active in the cluster.

        Preconditions:
            - Coordinator must be in Started state (`start()` has been called)
            - `module_class` must be a concrete subclass of Module

        Postconditions:
            - Actor created and registered, retrievable via `get_instance(module_class)`

              >>> from dimos.core import Module
              >>> class TestMod(Module): pass
              >>> coord = ModuleCoordinator(n=2)
              >>> coord.start()  # doctest: +ELLIPSIS
              Initialized...
              >>> inst = coord.deploy(TestMod)  # doctest: +ELLIPSIS
              deployed: ...
              >>> assert coord.get_instance(TestMod) is inst
              >>> coord.stop()

            - Previous deployment of the same class is overwritten if exists

        Args:
            module_class: The Module subclass to deploy.
            *args: Positional arguments forwarded to `module_class.__init__`
            **kwargs: Keyword arguments forwarded to `module_class.__init__`

        Returns:
            RPCClient proxy to the deployed actor, usable for remote method calls.

        Raises:
            ValueError: If `start()` has not been called
            TypeError: If `module_class` is not a Module subclass
            Exception: Any error from `module_class.__init__` or Dask deployment

        Side Effects:
            - Submits actor creation task to Dask scheduler
            - Registers actor reference in distributed ActorRegistry
            - Replaced actors remain in memory but are no longer tracked

        Examples:
            Deploying and connecting multiple modules:

            >>> from dimos.core import Module, In, Out
            >>> class DataSource(Module):
            ...     output: Out[dict] = None
            >>> class DataProcessor(Module):
            ...     input: In[dict] = None
            ...     def __init__(self, buffer_size=100):
            ...         super().__init__()
            ...         self.buffer_size = buffer_size
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> source = coordinator.deploy(DataSource)
            >>> processor = coordinator.deploy(DataProcessor, buffer_size=100)
            >>> processor.input.connect(source.output)
            >>> coordinator.start_all_modules()
            >>> coordinator.stop()

            Deployment replacement (second deployment overwrites first):

            >>> from dimos.core import Module
            >>> class CameraModule(Module):
            ...     def __init__(self, resolution=720):
            ...         super().__init__()
            ...         self.resolution = resolution
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> cam1 = coordinator.deploy(CameraModule, resolution=720)
            >>> cam2 = coordinator.deploy(CameraModule, resolution=1080)
            >>> # cam2 overwrites cam1 in the registry
            >>> assert coordinator.get_instance(CameraModule) is cam2
            >>> # cam1 actor still exists in cluster but is no longer tracked
            >>> coordinator.stop()

        Notes:
            - Deployment must occur after `start()` and before `start_all_modules()`
              in the typical lifecycle: `start()` → `deploy()` → `start_all_modules()`
            - The returned proxy is **not** the actual module instance, but an RPC wrapper
              that forwards method calls to the remote actor.
            - Module dependencies should be deployed in topological order (dependencies
              before dependents) to ensure proper initialization.
        """
        if not self._client:
            raise ValueError("Not started")

        module = self._client.deploy(module_class, *args, **kwargs)  # type: ignore[attr-defined]
        self._deployed_modules[module_class] = module
        return module  # type: ignore[no-any-return]

    def start_all_modules(self) -> None:
        """Initialize all deployed modules by calling their start() methods.

        Calls each module's start() method in deployment order to initialize RPC
        servers, event loops, and background tasks. After completion, all modules
        are ready to process messages.

        Preconditions:
            - At least one module has been deployed via deploy()
            - Module dependencies are satisfied (caller's responsibility to deploy
              modules in the correct order)

        Raises:
            Exception: If any module's start() method raises an exception, that
                exception propagates to the caller. No rollback occurs - modules
                started before the failure remain in started state.

        Examples:
            Basic usage with multiple modules:

            >>> from dimos.core import Module
            >>> class CameraModule(Module):
            ...     pass
            >>> class ProcessorModule(Module):
            ...     pass
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> camera = coordinator.deploy(CameraModule)
            >>> processor = coordinator.deploy(ProcessorModule)
            >>> coordinator.start_all_modules()
            >>> # Both modules now have active RPC servers
            >>> coordinator.stop()
        """
        for module in self._deployed_modules.values():
            module.start()

    def get_instance(self, module: type[T]) -> T | None:
        """Retrieve a previously deployed module instance by its class.

        This method provides type-safe lookup of deployed modules from the
        coordinator's registry. The registry is indexed by module class, maintaining
        at most one instance per module type.

        Args:
            module: The module class to look up.

        Returns:
            The deployed module instance (an RPCClient proxy) if the class has been
            deployed, or None if not found.

        Examples:
            Check deployment before use:

            >>> from dimos.core import Module
            >>> class PerceptionModule(Module):
            ...     pass
            >>> coordinator = ModuleCoordinator(n=2)
            >>> coordinator.start()
            Initialized dimos local cluster with 2 workers, memory limit: auto
            >>> instance = coordinator.get_instance(PerceptionModule)
            >>> instance is None
            True
            >>> perception = coordinator.deploy(PerceptionModule)  # doctest: +ELLIPSIS
            deployed: ...
            >>> coordinator.get_instance(PerceptionModule) is not None
            True
            >>> coordinator.stop()
        """
        return self._deployed_modules.get(module)  # type: ignore[return-value]

    def loop(self) -> None:
        """Run the coordinator's event loop until interrupted, then perform clean shutdown.

        Blocks the calling thread indefinitely, providing the primary execution
        mechanism for long-running DimOS applications. When interrupted via Ctrl+C
        (SIGINT) or other signals, performs graceful shutdown via `stop()`.

        Side Effects:
            - Blocks the calling thread indefinitely
            - Handles KeyboardInterrupt (Ctrl+C) gracefully
            - Calls `stop()` in finally block for guaranteed cleanup (see `stop()` for details)

        Examples:
            Typical deployment pattern with graceful shutdown:

            ```python
            from dimos.core import Module, ModuleCoordinator

            class CameraModule(Module):
                pass

            class ProcessorModule(Module):
                pass

            coordinator = ModuleCoordinator(n=4)
            coordinator.start()
            camera = coordinator.deploy(CameraModule)
            processor = coordinator.deploy(ProcessorModule)
            coordinator.start_all_modules()
            coordinator.loop()  # Blocks until Ctrl+C, then stops automatically
            ```

            Using blueprint composition:

            ```python
            from dimos.core.blueprints import autoconnect
            from dimos.blueprints.unitree import connection, spatial_memory

            blueprint = autoconnect(connection(), spatial_memory())
            coordinator = blueprint.build()  # Already starts coordinator and modules
            coordinator.loop()  # Blocks until Ctrl+C
            ```
        """
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            return
        finally:
            self.stop()
