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

import pytest

from dimos.diagnostics.runtime import Runtime


@pytest.fixture
def trace_runtime(tmp_path, monkeypatch):
    runtime = Runtime("run", tmp_path / "run")
    monkeypatch.setattr("dimos.diagnostics.runtime._runtime", runtime)
    monkeypatch.setattr("dimos.diagnostics.runtime._mode", "trace")
    yield runtime
    runtime.close()
