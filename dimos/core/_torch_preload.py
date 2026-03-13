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

"""
Dask worker preload script: preloads PyTorch shared libraries before the TLS
block fills up.

On Linux (aarch64 in particular), PyTorch's libc10.so uses static TLS storage.
If it is loaded lazily via dlopen() inside a spawned/forked worker process after
other libraries have already consumed the TLS block, the linker raises:

    cannot allocate memory in static TLS block

Registering this module as a Dask worker preload (via LocalCluster worker_kwargs)
ensures the libraries are loaded at worker startup — before any user code runs —
giving them first access to the TLS slots they need.
"""

import ctypes
import importlib.util
import os


def dask_setup(worker):  # type: ignore[no-untyped-def]  # noqa: ANN001
    """Called by Dask at worker startup before any tasks run."""
    try:
        spec = importlib.util.find_spec("torch")
        if spec and spec.origin:
            lib_dir = os.path.join(os.path.dirname(spec.origin), "lib")
            for lib_name in ["libc10.so", "libtorch.so", "libtorch_cpu.so"]:
                lib_path = os.path.join(lib_dir, lib_name)
                if os.path.exists(lib_path):
                    ctypes.CDLL(lib_path)
    except Exception:
        pass
