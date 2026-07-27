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

"""Pure-package test isolation for the T15/T9 run db.

Live rim sessions capture decisions whenever health is on (T9 Q3), and the
debug/health run-db resolution falls back to the repo run-log dir when no
``DIMOS_RUN_LOG_DIR`` is set — so without isolation every test's
``<module>.decisions/.health/.config`` streams would pour into the user's
``logs/debug.db`` (their real run data).

Pinned to a tmp dir HERE, at conftest IMPORT time (during collection, before any
test runs and before the coordination forkserver spawns) — the same mechanism
``dimos/conftest.py`` uses for ``LCM_DEFAULT_URL``/``XDG_STATE_HOME``: an env var
so the legacy e2e's forkserver WORKER processes inherit it too (a per-test
fixture can't — the forkserver caches its env at first spawn). An explicit
``DIMOS_RUN_LOG_DIR`` in the environment is respected (opts out of isolation);
``test_per_run_logs`` manages the var itself via ``monkeypatch.delenv``.
"""

import os
import tempfile

if "DIMOS_RUN_LOG_DIR" not in os.environ:
    os.environ["DIMOS_RUN_LOG_DIR"] = tempfile.mkdtemp(prefix="dimos-pure-run-log-")
