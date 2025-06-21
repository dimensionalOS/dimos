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

import asyncio
import multiprocessing as mp

import pytest
from dask.distributed import Client, LocalCluster

from dimos.multiprocess.actors import (
    CameraActor,
    LatencyActor,
    VideoActor,
    deploy_actor,
)


@pytest.fixture
def dask_client():
    process_count = mp.cpu_count()
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=1)
    client = Client(cluster)
    yield client
    client.close()
    cluster.close()


@pytest.mark.asyncio
async def test_api(dask_client):
    print("Deploying actors")
    camera_actor = deploy_actor(dask_client, VideoActor, camera_index=0)
    frame_actor = deploy_actor(dask_client, LatencyActor, name="LatencyActor", verbose=True)

    print(f"Camera actor: {camera_actor}")
    print(f"Frame actor: {frame_actor}")

    camera_actor.add_processor(frame_actor)
    camera_actor.run(70).result()
    print("Camera actor run finished")

    await asyncio.sleep(2)
    # print(f"Frame actor received {frame_actor.frame_count} frames")
