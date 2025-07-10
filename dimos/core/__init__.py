from __future__ import annotations

import multiprocessing as mp
import time
from typing import Optional

import pytest
from dask.distributed import Client, LocalCluster
from rich.console import Console

import dimos.core.colors as colors
from dimos.core.core import In, Out, RemoteOut, rpc
from dimos.core.module import Module, ModuleBase
from dimos.core.transport import LCMTransport, ZenohTransport, pLCMTransport
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPC


def patch_actor(actor, cls): ...


class RPCClient:
    def __init__(self, rpc, actor_instance, actor_class):
        self.rpc = rpc()
        self.remote_name = actor_class.__name__
        self.remote_instance = actor_instance
        self.rpcs = actor_class.rpcs.keys()

        self.rpc.start()

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
            return lambda *args: self.rpc.call_sync(f"{self.remote_name}/{name}", args)

        # Try to avoid recursion by directly accessing attributes that are known
        attribute = object.__getattribute__(self.actor_instance, name)
        return attribute


def patchdask(dask_client: Client):
    def deploy(
        actor_class,
        rpc: RPC = LCMRPC,
        *args,
        **kwargs,
    ):
        console = Console()
        with console.status(f"deploying [green]{actor_class.__name__}", spinner="arc"):
            actor = dask_client.submit(
                actor_class,
                *args,
                **kwargs,
                rpc=rpc,
                actor=True,
            ).result()

            worker = actor.set_ref(actor).result()
            print((f"deployed: {colors.green(actor)} @ {colors.blue('worker ' + str(worker))}"))

            return RPCClient(rpc, actor, actor_class)

    dask_client.deploy = deploy
    return dask_client


@pytest.fixture
def dimos():
    process_count = 3  # we chill
    client = start(process_count)
    yield client
    stop(client)


def start(n: Optional[int] = None) -> Client:
    console = Console()
    if not n:
        n = mp.cpu_count()
    with console.status(
        f"[green]Initializing dimos local cluster with [bright_blue]{n} workers", spinner="arc"
    ) as status:
        cluster = LocalCluster(
            n_workers=n,
            threads_per_worker=4,
        )
        client = Client(cluster)

    console.print(f"[green]Initialized dimos local cluster with [bright_blue]{n} workers")
    return patchdask(client)


def stop(client: Client):
    client.close()
    client.cluster.close()
