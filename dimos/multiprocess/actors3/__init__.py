import pytest
from dask.distributed import Client, LocalCluster

from dimos.multiprocess.actors3.base import In, LCMTransport, Out, RemoteOut, rpc
from dimos.multiprocess.actors3.module_dask import Module


def patchdask(dask_client: Client):
    def deploy(actor_class, *args, **kwargs):
        actor = dask_client.submit(
            actor_class,
            *args,
            **kwargs,
            actor=True,
        ).result()

        actor.set_ref(actor).result()
        print(f"\033[32msubsystem deployed: [{actor}]\033[0m")
        return actor

    dask_client.deploy = deploy
    return dask_client


@pytest.fixture
def dimos():
    process_count = 3  # we chill
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=3)
    client = Client(cluster)
    yield patchdask(client)
    client.close()
    cluster.close()
