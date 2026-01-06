from __future__ import annotations

import multiprocessing as mp
import signal
import time

from dask.distributed import Client, LocalCluster
from rich.console import Console

import dimos.core.colors as colors
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleBase, ModuleConfig, ModuleConfigT
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out, RemoteIn, RemoteOut, Transport
from dimos.core.transport import (
    LCMTransport,
    SHMTransport,
    ZenohTransport,
    pLCMTransport,
    pSHMTransport,
)
from dimos.protocol.rpc import LCMRPC
from dimos.protocol.rpc.spec import RPCSpec
from dimos.protocol.tf import LCMTF, TF, PubSubTF, TFConfig, TFSpec
from dimos.utils.actor_registry import ActorRegistry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

__all__ = [
    "LCMRPC",
    "LCMTF",
    "TF",
    "DimosCluster",
    "In",
    "LCMTransport",
    "Module",
    "ModuleBase",
    "ModuleConfig",
    "ModuleConfigT",
    "Out",
    "PubSubTF",
    "RPCSpec",
    "RemoteIn",
    "RemoteOut",
    "SHMTransport",
    "TFConfig",
    "TFSpec",
    "Transport",
    "ZenohTransport",
    "pLCMTransport",
    "pSHMTransport",
    "rpc",
    "start",
]


class CudaCleanupPlugin:
    """Dask worker plugin to cleanup CUDA resources on shutdown."""

    def setup(self, worker) -> None:  # type: ignore[no-untyped-def]
        """Called when worker starts."""
        pass

    def teardown(self, worker) -> None:  # type: ignore[no-untyped-def]
        """Clean up CUDA resources when worker shuts down."""
        try:
            import sys

            if "cupy" in sys.modules:
                import cupy as cp  # type: ignore[import-not-found]

                # Clear memory pools
                mempool = cp.get_default_memory_pool()
                pinned_mempool = cp.get_default_pinned_memory_pool()
                mempool.free_all_blocks()
                pinned_mempool.free_all_blocks()
                cp.cuda.Stream.null.synchronize()
                mempool.free_all_blocks()
                pinned_mempool.free_all_blocks()
        except Exception:
            pass


def patch_actor(actor, cls) -> None: ...  # type: ignore[no-untyped-def]


DimosCluster = Client


def patchdask(dask_client: Client, local_cluster: LocalCluster) -> DimosCluster:
    def deploy(  # type: ignore[no-untyped-def]
        actor_class,
        *args,
        **kwargs,
    ):
        logger.info("Deploying module.", module=actor_class.__name__)
        actor = dask_client.submit(  # type: ignore[no-untyped-call]
            actor_class,
            *args,
            **kwargs,
            actor=True,
        ).result()

        worker = actor.set_ref(actor).result()
        logger.info("Deployed module.", module=actor._cls.__name__, worker_id=worker)

        # Register actor deployment in shared memory
        ActorRegistry.update(str(actor), str(worker))

        return RPCClient(actor, actor_class)

    def check_worker_memory() -> None:
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
            spilled / 1e9

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

    def close_all() -> None:
        # Prevents multiple calls to close_all
        if hasattr(dask_client, "_closed") and dask_client._closed:
            return
        dask_client._closed = True  # type: ignore[attr-defined]

        # Stop all SharedMemory transports before closing Dask
        # This prevents the "leaked shared_memory objects" warning and hangs
        try:
            import gc

            from dimos.protocol.pubsub import shmpubsub

            for obj in gc.get_objects():
                if isinstance(obj, shmpubsub.SharedMemory | shmpubsub.PickleSharedMemory):
                    try:
                        obj.stop()
                    except Exception:
                        pass
        except Exception:
            pass

        # Get the event loop before shutting down
        loop = dask_client.loop

        # Clear the actor registry
        ActorRegistry.clear()

        # Close cluster and client with reasonable timeout
        # The CudaCleanupPlugin will handle CUDA cleanup on each worker
        try:
            local_cluster.close(timeout=5)
        except Exception:
            pass

        try:
            dask_client.close(timeout=5)  # type: ignore[no-untyped-call]
        except Exception:
            pass

        if loop and hasattr(loop, "add_callback") and hasattr(loop, "stop"):
            try:
                loop.add_callback(loop.stop)
            except Exception:
                pass

        # Note: We do NOT shutdown the _offload_executor here because it's a global
        # module-level ThreadPoolExecutor shared across all Dask clients in the process.
        # Shutting it down here would break subsequent Dask client usage (e.g., in tests).
        # The executor will be cleaned up when the Python process exits.

        # Give threads time to clean up
        # Dask's IO loop and Profile threads are daemon threads
        # that will be cleaned up when the process exits
        # This is needed, solves race condition in CI thread check
        time.sleep(0.1)

    dask_client.deploy = deploy  # type: ignore[attr-defined]
    dask_client.check_worker_memory = check_worker_memory  # type: ignore[attr-defined]
    dask_client.stop = lambda: dask_client.close()  # type: ignore[attr-defined, no-untyped-call]
    dask_client.close_all = close_all  # type: ignore[attr-defined]
    return dask_client


def start(n: int | None = None, memory_limit: str = "auto") -> DimosCluster:
    """Start a Dask LocalCluster with specified workers and memory limits.

    Args:
        n: Number of workers (defaults to CPU count)
        memory_limit: Memory limit per worker (e.g., '4GB', '2GiB', or 'auto' for Dask's default)

    Returns:
        DimosCluster: A patched Dask client with deploy(), check_worker_memory(), stop(), and close_all() methods
    """

    console = Console()
    if not n:
        n = mp.cpu_count()
    with console.status(
        f"[green]Initializing dimos local cluster with [bright_blue]{n} workers", spinner="arc"
    ):
        cluster = LocalCluster(  # type: ignore[no-untyped-call]
            n_workers=n,
            threads_per_worker=4,
            memory_limit=memory_limit,
            plugins=[CudaCleanupPlugin()],  # Register CUDA cleanup plugin
        )
        client = Client(cluster)  # type: ignore[no-untyped-call]

    console.print(
        f"[green]Initialized dimos local cluster with [bright_blue]{n} workers, memory limit: {memory_limit}"
    )

    patched_client = patchdask(client, cluster)
    patched_client._shutting_down = False  # type: ignore[attr-defined]

    # Signal handler with proper exit handling
    def signal_handler(sig, frame) -> None:  # type: ignore[no-untyped-def]
        # If already shutting down, force exit
        if patched_client._shutting_down:  # type: ignore[attr-defined]
            import os

            console.print("[red]Force exit!")
            os._exit(1)

        patched_client._shutting_down = True  # type: ignore[attr-defined]
        console.print(f"[yellow]Shutting down (signal {sig})...")

        try:
            patched_client.close_all()  # type: ignore[attr-defined]
        except Exception:
            pass

        import sys

        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    return patched_client


def wait_exit() -> None:
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("exiting...")
            return
