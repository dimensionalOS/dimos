from __future__ import annotations

import multiprocessing as mp
from typing import Optional

from dask.distributed import Client, LocalCluster
from rich.console import Console

import dimos.core.colors as colors
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleBase
from dimos.core.stream import In, Out, RemoteIn, RemoteOut, Transport
from dimos.core.transport import LCMTransport, ZenohTransport, pLCMTransport
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPC
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
            return lambda *args, **kwargs: self.rpc.call_sync(
                f"{self.remote_name}/{name}", (args, kwargs)
            )

        # return super().__getattr__(name)
        # Try to avoid recursion by directly accessing attributes that are known
        return self.actor_instance.__getattr__(name)


def patchdask(dask_client: Client):
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

            return RPCClient(actor, actor_class)

    dask_client.deploy = deploy
    return dask_client


def start(n: Optional[int] = None) -> Client:
    console = Console()
    if not n:
        n = mp.cpu_count()
    with console.status(
        f"[green]Initializing dimos local cluster with [bright_blue]{n} workers", spinner="arc"
    ):
        cluster = LocalCluster(
            n_workers=n,
            threads_per_worker=4,
        )
        client = Client(cluster)

    console.print(f"[green]Initialized dimos local cluster with [bright_blue]{n} workers")
    return patchdask(client)


# this needs to go away
# client.shutdown() is the correct shutdown method
def stop(client: Client):
    client.close()
    client.cluster.close()
