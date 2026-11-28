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

"""Native thread-pool hygiene, for `dimos run` and for tests alike.

Every OpenMP/BLAS runtime sizes its pool to all cores and spin-waits at barriers
(libgomp: ~300k pause iterations per region; libomp: KMP_BLOCKTIME of active
spinning after each region). With ~10 worker processes each owning core-sized
pools this oversubscribes ~10x: measured 12% of all cycles spinning in
gomp_barrier_wait_end and load avg 30-50 at ~50% CPU. PASSIVE wait + small pools
cut a 10 Hz open3d workload 3.7x (1.10 -> 0.30 cores) and ~50 threads/worker ->
~10. Must be set before any library loads its OpenMP runtime (libgomp reads env
at load, OpenBLAS spawns its pool at import); workers inherit via the forkserver.
User env wins.
"""

import os

NATIVE_THREAD_POOLS = {
    "KMP_BLOCKTIME": "0",
    "OMP_WAIT_POLICY": "PASSIVE",
    "GOMP_SPINCOUNT": "0",
    "OMP_NUM_THREADS": "4",
    "OPENBLAS_NUM_THREADS": "2",
    "MKL_NUM_THREADS": "2",
    "NUMEXPR_NUM_THREADS": "2",
    # cv2 reads this at import for its parallel_for pool; replaces the eager
    # `import cv2; cv2.setNumThreads(2)` the worker entrypoint used to do.
    "OPENCV_FOR_THREADS_NUM": "2",
}


def limit_native_thread_pools() -> None:
    """Set each pool's default in the environment, unless the user set it."""
    for name, value in NATIVE_THREAD_POOLS.items():
        os.environ.setdefault(name, value)
