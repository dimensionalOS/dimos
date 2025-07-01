#!/usr/bin/env python3

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

# import multiprocessing as mp
import pytest
from dask.distributed import Client, LocalCluster


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
    #    process_count = mp.cpu_count()
    process_count = 3  # we chill
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=1)
    client = Client(cluster)
    yield patchdask(client)
    client.close()
    cluster.close()


def initialize(n=2):
    cluster = LocalCluster(n_workers=n, threads_per_worker=3)
    client = Client(cluster)
    return patchdask(client)
