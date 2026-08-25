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

"""SPACE route retracing — retrace the demonstrated route (upstream: the walkthrough is the shortest path).

One episode per environment, scored with the benchmark's own SPL. The
bevimage cases run anywhere (the discrete-map world is pure Python); the
ego cases render observations online and need habitat-sim — their preflight
says so on machines without it::

    dimos evals run dimos.evals.suites.space.route_retracing --tags bevimage --limit 5 --system-prompt ""

--blind withholds the walkthrough, leaving the model to navigate an
environment it has never seen — the guessing floor for an episode.
"""

from __future__ import annotations

from dimos.evals.suites.space._bench import nav_suite
from dimos.evals.types import Suite

SUITE: Suite = nav_suite("shortestpath", "route_retracing")
