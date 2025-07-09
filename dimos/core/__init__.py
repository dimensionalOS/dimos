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
from dimos.protocol.rpc.spec import RPC


def patch_actor(actor, cls): ...


def patchdask(dask_client: Client):
    def deploy(
        actor_class,
        rpc: RPC = None,
        *args,
        **kwargs,
    ):
        console = Console()
        with console.status(f"deploying [green]{actor_class.__name__}", spinner="arc"):
            actor = dask_client.submit(
                actor_class,
                *args,
                **kwargs,
                rpc=RPC,
                actor=True,
            ).result()

            worker = actor.set_ref(actor).result()
            print((f"deployed: {colors.green(actor)} @ {colors.blue('worker ' + str(worker))}"))

            if rpc:
                rpc = RPC()
                rpc.serve_module_rpc(actor)

            for name, rpc in actor_class.rpcs.items():
                print(f"binding rpc on {actor_class}, {name} to {rpc}")
                setattr(actor, name, lambda: print("RPC CALLED", name, actor_class))

            return actor

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
