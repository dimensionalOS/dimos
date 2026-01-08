# Copyright 2025-2026 Dimensional Inc.
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

import argparse, time, numpy as np
from dask.distributed import Client, LocalCluster

from dimos.utils.ipc_factory import make_frame_channel, CPU_IPC_Factory, CUDA_IPC_Factory
from dimos.utils.multirate import MultiRateProcessor

from dimos.models.depth.metric3d import Metric3D

try:
    import cupy as cp  # only used when mode=cuda and device backend is CUDA
except Exception:
    cp = None

# ============ Actors that live on Dask workers ============


class SourceActor:
    """Owns the FrameChannel (CPU SHM or CUDA IPC) and accepts publish() calls."""

    def __init__(self, shape, dtype="uint8", prefer="auto", device_id=0):
        self.shape = tuple(shape)
        self.dtype = np.dtype(dtype)
        self.channel = make_frame_channel(
            self.shape, dtype=self.dtype, prefer=prefer, device=device_id
        )

    def descriptor(self):
        return self.channel.descriptor()

    def device(self):
        return self.channel.device  # "cpu" | "cuda"

    def publish(self, frame):
        # frame can be NumPy (CPU) or CuPy (CUDA)
        self.channel.publish(frame)

    def close(self):
        self.channel.close()


class ModelActor:
    """
    Attaches to a FrameChannel descriptor and runs a single model at its own FPS
    via MultiRateProcessor. Supply handler per model below.
    """

    def __init__(self, desc, model_name, fps):
        # Attach to the channel
        kind = desc.get("kind")
        if kind == "cuda":
            self.slot = CUDA_IPC_Factory.attach(desc)
            backend = "cuda"
        elif kind == "cpu":
            self.slot = CPU_IPC_Factory.attach(desc)
            backend = "cpu"
        else:
            raise ValueError(f"Unknown channel kind: {kind}")

        self.metric3d = Metric3D()

        # ------ Handlers ------
        # TODO: replace these with camera calls
        def depth_handler(img, meta):
            start = time.time()
            print(f"depth_handler called. Time: {start}")
            # img: np.ndarray on CPU, cupy.ndarray on CUDA
            x = self.metric3d.infer_depth(img)  # cheap placeholder op
            end = time.time()
            print(f"x returned: {x}. Time: {end}. Time taken: {end - start:.4f} seconds")

        def detector_handler(img, meta):
            print("detector_handler called")
            _ = img.mean()

        def segmenter_handler(img, meta):
            print("segmenter_handler called")
            _ = img.mean()

        handlers = {
            "depth": depth_handler,
            "detector": detector_handler,
            "segmenter": segmenter_handler,
        }

        targets = {model_name: float(fps)}
        self.proc = MultiRateProcessor(
            channel=self.slot,
            target_fps_by_model=targets,
            handlers={model_name: handlers[model_name]},
            max_age_s=0.25,
        )
        self.proc.start()

    def stop(self):
        self.proc.stop()
        self.slot.close()


# ============ Cluster bring-up (CPU or CUDA) ============


def make_cluster(mode: str):
    """
    mode="cpu": LocalCluster (process workers); POSIX SHM works across processes on same node.
    mode="cuda": LocalCUDACluster if available; falls back to LocalCluster if dask-cuda missing.
    """
    if mode == "cuda":
        try:
            from dask_cuda import LocalCUDACluster

            cluster = LocalCUDACluster(protocol="tcp")  # enable GPU-aware comms
            client = Client(cluster)
            return cluster, client, True
        except Exception as e:
            print(
                "[warn] CUDA cluster requested but dask-cuda/UCX not available; falling back to CPU LocalCluster."
            )
    # CPU cluster
    cluster = LocalCluster(
        n_workers=2,
        threads_per_worker=1,
        processes=True,
        nanny=False,  # <- important: keep actors stable
        memory_limit="0",  # <- no nanny-based restarts from memory
    )
    client = Client(cluster)
    return cluster, client, False


# ============ Demo harness ============


def main(mode: str):
    cluster, client, cuda_cluster = make_cluster(mode)
    info = client.scheduler_info()
    workers = list(info["workers"])
    print("Workers:", workers)

    H, W, C = 1080, 1920, 3
    shape = (H, W, C)

    prefer = "cuda" if mode == "cuda" and cuda_cluster else "cpu"

    # Create one SourceActor on first worker
    src_fut = client.submit(
        SourceActor, shape, "uint8", prefer, 0, actor=True, workers=[workers[0]]
    )
    src = src_fut.result()
    desc = src.descriptor().result()
    backend = src.device().result()
    print("Channel backend:", backend)

    # Start a couple of model loops on the same worker
    m1 = client.submit(ModelActor, desc, "depth", 15, actor=True, workers=[workers[0]]).result()
    m2 = client.submit(ModelActor, desc, "detector", 15, actor=True, workers=[workers[0]]).result()

    # Publisher in the client (driver) for demo purposes.
    # In production you might run the camera inside SourceActor to avoid network hops.
    rng = np.random.default_rng(0)
    period, next_t = 1 / 30, time.monotonic()
    for _ in range(120):  # ~4 seconds
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
        frame = rng.integers(0, 256, size=shape, dtype=np.uint8)
        if backend == "cuda" and cp is not None:
            src.publish(cp.asarray(frame))  # H2D on worker; D2D if already on GPU
        else:
            src.publish(frame)
        next_t += period

    print("cleanup time. stopping m1 / m2")
    # Clean up
    try:
        # Stop model actors first
        m1.stop().result(timeout=10)
        m2.stop().result(timeout=10)

        # If you have a camera thread:
        # src.stop_camera().result(timeout=10)

        # Close the source (unlinks SHM etc.)
        src.close().result(timeout=10)

    finally:
        client.close()
        cluster.close()


# Run:
#   python dask_frame_cluster.py --mode cpu
#   python dask_frame_cluster.py --mode cuda

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["cpu", "cuda"], default="cpu")
    args = ap.parse_args()
    main(args.mode)
