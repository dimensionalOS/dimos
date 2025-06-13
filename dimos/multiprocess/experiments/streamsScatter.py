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

import time
from datetime import datetime, timedelta

from dask.distributed import Client, LocalCluster, Queue, Worker, get_client, get_worker
from streamz import Stream


def inc(x):
    time.sleep(1)  # simulate actual work
    return x + 1


def camera_loop(stream):
    """Fake camera – emits a timestamp every 0.2 s"""
    n = 0
    while True:
        stream.emit((n, time.time()))
        n += 1
        time.sleep(0.01)


def main():
    cluster = LocalCluster(n_workers=12, threads_per_worker=1)
    client = Client(cluster)
    print("Dashboard:", client.dashboard_link)

    source = Stream(asynchronous=True)
    source.scatter().map(inc).buffer(8).gather().sink(print)

    for i in range(3):
        source.emit(i)

    raw = Stream()
    dasked = raw.scatter()

    def checklatency(frame):
        (n, timestamp) = frame
        time_diff = (datetime.now() - datetime.fromtimestamp(timestamp)).total_seconds() * 1_000
        return (n, timestamp, time_diff)

    branch_a = dasked.map(checklatency).gather().sink(lambda x: print("A", x))

    # branch B – run on worker 1
    branch_b = dasked.map(checklatency).gather().sink(lambda x: print("B", x))

    camera_loop(raw)  # this blocks


if __name__ == "__main__":
    main()
