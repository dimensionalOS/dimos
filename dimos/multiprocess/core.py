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

import cv2
from dask.distributed import Client, LocalCluster, Queue, Worker, get_client, get_worker


def main():
    # 1. Spin-up scheduler + workers
    cluster = LocalCluster(n_workers=4, threads_per_worker=1)
    client = Client(cluster)
    print("Dashboard:", client.dashboard_link)

    # 2. Create a queue for frame communication
    frame_queue = Queue("camera_frames", client=client)

    # 3. Make one CameraAgent *inside* a worker ----------------------
    cam_actor = client.submit(CameraAgent, "/dev/video0", 30, frame_queue, actor=True).result()
    # cam_actor is just a tiny proxy object in the driver process

    # 4. Start its loop (runs on the worker, returns a Future) -------
    cam_loop_future = cam_actor.loop()  # non-blocking here!

    # 5. Show that we can get frames from the queue ------------------
    try:
        for i in range(10):
            frame = frame_queue.get(timeout=5)  # 5 second timeout
            print(f"Got frame {i + 1}: {frame.shape}")
    except Exception as e:
        print(f"Error getting frames: {e}")

    # 6. Clean shutdown ---------------------------------------------
    try:
        cam_actor.stop().result()  # ask remote agent to exit its loop
    except Exception as e:
        print(f"Error stopping camera actor: {e}")
    finally:
        client.shutdown()


if __name__ == "__main__":
    main()
