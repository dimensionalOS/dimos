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

"""The planner runner: pick a candidate, score it against the referee.

    python -m dimos.navigation.motion.planner --score --gen 40
    python -m dimos.navigation.motion.planner --score --planner gold
    python -m dimos.navigation.motion.planner --score --planner ml.candidate:make

Every candidate — the production crate, an autoresearch lab's, a learned one —
is a ``--planner`` name resolved by :mod:`~...planner.referee.planners.base`,
and all of them are judged by the same referee. ``referee/__main__.py`` is the
same CLI from inside the copyable unit, which is what exported labs run as
``python -m referee``.

The BLAS pinning below is repeated in both entry points on purpose: it only
works if it happens before numpy is first imported, so whichever module is
`-m`'d has to do it itself. avoid_ms is time.process_time(), which sums CPU
across all threads, and OpenBLAS's spinning pool otherwise bills its spin time
to the candidate (measured: identical-code score spread 0.334 -> 0.0069).
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
    # recursively re-run the whole CLI in each child.
    from .referee.sim import main

    main()
