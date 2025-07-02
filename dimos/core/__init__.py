import multiprocessing as mp

import pytest
from dask.distributed import Client, LocalCluster

import dimos.core.colors as colors
from dimos.core.core import In, Out, RemoteOut, rpc
from dimos.core.module_dask import Module
from dimos.core.transport import LCMTransport, ZenohTransport, pLCMTransport


def patchdask(dask_client: Client):
    def deploy(actor_class, *args, **kwargs):
        actor = dask_client.submit(
            actor_class,
            *args,
            **kwargs,
            actor=True,
        ).result()

        actor.set_ref(actor).result()
        print(colors.green(f"Subsystem deployed: {actor}"))
        return actor

    dask_client.deploy = deploy
    return dask_client


@pytest.fixture
def dimos():
    process_count = 3  # we chill
    client = start(process_count)
    yield client
    stop(client)


def start(n):
    if not n:
        n = mp.cpu_count()
    print(colors.green(f"Initializing dimos local cluster with {n} workers"))
    cluster = LocalCluster(
        n_workers=n,
        threads_per_worker=3,
    )
    client = Client(cluster)
    return patchdask(client)


def stop(client: Client):
    client.close()
    client.cluster.close()
