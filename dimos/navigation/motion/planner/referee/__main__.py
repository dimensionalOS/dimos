# Copyright 2026 Dimensional Inc.
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

"""Canonical CLI entry: pins the BLAS thread pools BEFORE numpy loads.

avoid_ms is measured with time.process_time(), which sums CPU across all
threads of the process. numpy/OpenBLAS keeps a spinning worker pool, and
whenever a spin window overlaps a timed plan() call, that spin time is
charged straight into the candidate's score. Measured on this battery:
pinning the pools took identical-code score spread from 0.334 to 0.0069.

The env vars only work if they are set before numpy is first imported,
which is why this file sets them before importing anything else — run the
battery as `python -m dimos.navigation.motion.planner.autoresearch`.
"""

import os

for _v in (
    "OPENBLAS_NUM_THREADS",
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
):
    os.environ.setdefault(_v, "1")

if __name__ == "__main__":
    # The guard matters: multiprocessing's spawn re-imports the launching
    # module in every worker (as __mp_main__); without it, --jobs would
    # recursively re-run the whole CLI in each child. The import is relative
    # so a copy of this package runs under any name (e.g. `python -m referee`
    # in an exported lab).
    from .sim import main

    main()
