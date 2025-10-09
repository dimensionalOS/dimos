from __future__ import annotations

import multiprocessing as mp
from typing import Optional

from dask.distributed import Client, LocalCluster
from rich.console import Console

import dimos.core.colors as colors
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleBase, ModuleConfig
from dimos.core.stream import In, Out, RemoteIn, RemoteOut, Transport
from dimos.utils.actor_registry import ActorRegistry
from dimos.core.transport import (
    LCMTransport,
    SHMTransport,
    ZenohTransport,
    pLCMTransport,
    pSHMTransport,
)
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPCSpec
from dimos.protocol.tf import LCMTF, TF, PubSubTF, TFConfig, TFSpec

__all__ = ["TF", "LCMTF", "PubSubTF", "TFSpec", "TFConfig"]


def patch_actor(actor, cls): ...


class RPCClient:
    def __init__(self, actor_instance, actor_class):
        self.rpc = LCMRPC()
        self.actor_class = actor_class
        self.remote_name = actor_class.__name__
        self.actor_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()
        self.rpc.start()
        self._unsub_fns = []

    def stop_client(self):
        for unsub in self._unsub_fns:
            try:
                unsub()
            except Exception:
                pass

        self._unsub_fns = []

        if self.rpc:
            self.rpc.stop()
            self.rpc = None

    def __reduce__(self):
        # Return the class and the arguments needed to reconstruct the object
        return (
            self.__class__,
            (self.actor_instance, self.actor_class),
        )

    # passthrough
    def __getattr__(self, name: str):
        # Check if accessing a known safe attribute to avoid recursion
        if name in {
            "__class__",
            "__init__",
            "__dict__",
            "__getattr__",
            "rpcs",
            "remote_name",
            "remote_instance",
            "actor_instance",
        }:
            raise AttributeError(f"{name} is not found.")

        if name in self.rpcs:
            # Get the original method to preserve its docstring
            original_method = getattr(self.actor_class, name, None)

            def rpc_call(*args, **kwargs):
                # For stop/close/shutdown, use call_nowait to avoid deadlock
                # (the remote side stops its RPC service before responding)
                if name in ("stop", "close", "shutdown"):
                    if self.rpc:
                        self.rpc.call_nowait(f"{self.remote_name}/{name}", (args, kwargs))
                    self.stop_client()
                    return None

                result, unsub_fn = self.rpc.call_sync(f"{self.remote_name}/{name}", (args, kwargs))
                self._unsub_fns.append(unsub_fn)
                return result

            # Copy docstring and other attributes from original method
            if original_method:
                rpc_call.__doc__ = original_method.__doc__
                rpc_call.__name__ = original_method.__name__
                rpc_call.__qualname__ = f"{self.__class__.__name__}.{original_method.__name__}"

            return rpc_call

        # return super().__getattr__(name)
        # Try to avoid recursion by directly accessing attributes that are known
        return self.actor_instance.__getattr__(name)


DimosCluster = Client


def patchdask(dask_client: Client, local_cluster: LocalCluster) -> DimosCluster:
    def deploy(
        actor_class,
        *args,
        **kwargs,
    ):
        console = Console()
        with console.status(f"deploying [green]{actor_class.__name__}", spinner="arc"):
            actor = dask_client.submit(
                actor_class,
                *args,
                **kwargs,
                actor=True,
            ).result()

            worker = actor.set_ref(actor).result()
            print((f"deployed: {colors.green(actor)} @ {colors.blue('worker ' + str(worker))}"))

            # Register actor deployment in shared memory
            ActorRegistry.update(str(actor), str(worker))

            return RPCClient(actor, actor_class)

    def check_worker_memory():
        """Check memory usage of all workers."""
        info = dask_client.scheduler_info()
        console = Console()
        total_workers = len(info.get("workers", {}))
        total_memory_used = 0
        total_memory_limit = 0

        for worker_addr, worker_info in info.get("workers", {}).items():
            metrics = worker_info.get("metrics", {})
            memory_used = metrics.get("memory", 0)
            memory_limit = worker_info.get("memory_limit", 0)

            cpu_percent = metrics.get("cpu", 0)
            managed_bytes = metrics.get("managed_bytes", 0)
            spilled = metrics.get("spilled_bytes", {}).get("memory", 0)
            worker_status = worker_info.get("status", "unknown")
            worker_id = worker_info.get("id", "?")

            memory_used_gb = memory_used / 1e9
            memory_limit_gb = memory_limit / 1e9
            managed_gb = managed_bytes / 1e9
            spilled_gb = spilled / 1e9

            total_memory_used += memory_used
            total_memory_limit += memory_limit

            percentage = (memory_used_gb / memory_limit_gb * 100) if memory_limit_gb > 0 else 0

            if worker_status == "paused":
                status = "[red]PAUSED"
            elif percentage >= 95:
                status = "[red]CRITICAL"
            elif percentage >= 80:
                status = "[yellow]WARNING"
            else:
                status = "[green]OK"

            console.print(
                f"Worker-{worker_id} {worker_addr}: "
                f"{memory_used_gb:.2f}/{memory_limit_gb:.2f}GB ({percentage:.1f}%) "
                f"CPU:{cpu_percent:.0f}% Managed:{managed_gb:.2f}GB "
                f"{status}"
            )

        if total_workers > 0:
            total_used_gb = total_memory_used / 1e9
            total_limit_gb = total_memory_limit / 1e9
            total_percentage = (total_used_gb / total_limit_gb * 100) if total_limit_gb > 0 else 0
            console.print(
                f"[bold]Total: {total_used_gb:.2f}/{total_limit_gb:.2f}GB ({total_percentage:.1f}%) across {total_workers} workers[/bold]"
            )

    def close_all():
        import time

        # Get the event loop before shutting down
        loop = dask_client.loop

        # Close cluster and client
        ActorRegistry.clear()
        local_cluster.close()
        dask_client.close()

        # Stop the Tornado IOLoop to clean up IO loop and Profile threads
        if loop and hasattr(loop, "add_callback") and hasattr(loop, "stop"):
            try:
                loop.add_callback(loop.stop)
            except Exception:
                pass

        # Shutdown the Dask offload thread pool
        try:
            from distributed.utils import _offload_executor

            if _offload_executor:
                _offload_executor.shutdown(wait=False)
        except Exception:
            pass

        # Give threads a moment to clean up
        time.sleep(0.1)

    dask_client.deploy = deploy
    dask_client.check_worker_memory = check_worker_memory
    dask_client.stop = lambda: dask_client.close()
    dask_client.close_all = close_all
    return dask_client


def start(n: Optional[int] = None, memory_limit: str = "auto") -> Client:
    """Start a Dask LocalCluster with specified workers and memory limits.

    Args:
        n: Number of workers (defaults to CPU count)
        memory_limit: Memory limit per worker (e.g., '4GB', '2GiB', or 'auto' for Dask's default)
    """
    console = Console()
    if not n:
        n = mp.cpu_count()
    with console.status(
        f"[green]Initializing dimos local cluster with [bright_blue]{n} workers", spinner="arc"
    ):
        cluster = LocalCluster(
            n_workers=n,
            threads_per_worker=4,
            memory_limit=memory_limit,
        )
        client = Client(cluster)

    console.print(
        f"[green]Initialized dimos local cluster with [bright_blue]{n} workers, memory limit: {memory_limit}"
    )
    return patchdask(client, cluster)
