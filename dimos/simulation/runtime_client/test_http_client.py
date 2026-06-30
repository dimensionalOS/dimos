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

"""Tests for the synchronous runtime sidecar HTTP client."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
PROTOCOL_SRC = REPO_ROOT / "packages" / "dimos-runtime-protocol" / "src"
sys.path.insert(0, str(PROTOCOL_SRC))

from dimos.simulation.runtime_client.http_client import RuntimeSidecarClient


def test_wait_until_healthy_times_out_for_unavailable_sidecar() -> None:
    client = RuntimeSidecarClient("http://127.0.0.1:9", timeout_s=0.01)

    with pytest.raises(TimeoutError):
        client.wait_until_healthy(timeout_s=0.02, poll_s=0.001)
