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
from datetime import datetime
from typing import Any, Callable, Tuple

import pytest
from dask.distributed import Client, LocalCluster

from dimos.multiprocess.experiments.streamsActors import CameraLoop, FrameProcessor


def time_call(func: Callable, *args, **kwargs) -> Tuple[Any, float]:
    """
    Time any function call and return both the result and execution time.

    Args:
        func: The function to call
        *args: Positional arguments to pass to the function
        **kwargs: Keyword arguments to pass to the function

    Returns:
        Tuple of (result, execution_time_in_seconds)
    """
    start_time = datetime.now()
    func(*args, **kwargs)
    execution_time = (datetime.now() - start_time).total_seconds() * 1_000
    return execution_time


@pytest.fixture
def dask_client():
    process_count = mp.cpu_count()
    cluster = LocalCluster(n_workers=process_count, threads_per_worker=1)
    client = Client(cluster)
    yield client
    client.close()
    cluster.close()


@pytest.mark.asyncio
async def test_frame_processing_actor_latency(dask_client):
    # Create two frame processors as actors
    actor_a = dask_client.submit(FrameProcessor, "A", actor=True).result()
    actor_b = dask_client.submit(FrameProcessor, "B", actor=True).result()

    # Create camera loop as an actor
    camera_actor = dask_client.submit(CameraLoop, fps=60, actor=True).result()

    print(f"\nActor A: {actor_a}, Actor B: {actor_b}, Camera: {camera_actor}")

    camera_actor.add_processors(actor_a, actor_b)

    # Run the camera loop actor
    camera_actor.run(50).result()
    # we are not awaiting but calling result() in order to block while this is executing

    print(f"Attribute access latency {time_call(lambda: actor_a.avg_latency)}ms")
    print(f"Function call latency {time_call(lambda: actor_a.get_latency().result())}ms")

    # Check latencies
    print(
        f"Messages received by actor A: {actor_a.frame_count}, Average latency A: {actor_a.avg_latency}"
    )
    print(
        f"Messages received by actor B: {actor_b.frame_count}, Average latency B: {actor_b.avg_latency}"
    )

    assert actor_a.frame_count == 50
    assert actor_b.frame_count == 50
    assert 0 < actor_a.avg_latency < 10
    assert 0 < actor_b.avg_latency < 10


@pytest.mark.asyncio
async def _test_actor_api_design(dask_client):
    camera = CameraLoop(fps=60)
    objects = FrameProcessor(name="Objects", input=camera)
    people = FrameProcessor(name="People", input=camera.stream_main)

    joined = JoinStreams(objects, people)
    joined.sink(print)
