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

import multiprocessing as mp

import pytest
from dask.distributed import Client, LocalCluster

from dimos.multiprocess.experiments.streamsActors import FrameProcessor, camera_loop


@pytest.fixture
def dask_client():
    process_count = mp.cpu_count()
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=1)
    client = Client(cluster)
    yield client
    client.close()
    cluster.close()


@pytest.mark.asyncio
async def test_frame_processing(dask_client):
    # Create two frame processors
    actor_a = dask_client.submit(FrameProcessor, "A", actor=True).result()
    actor_b = dask_client.submit(FrameProcessor, "B", actor=True).result()

    print(f"\nActor A: {actor_a}, Actor B: {actor_b}")
    camera_loop(100, actor_a, actor_b)
    print(f"Latency A: {actor_a.latency}, Latency B: {actor_b.latency}")
