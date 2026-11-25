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

"""Transport fixtures exercise real wire delivery between independent processes."""

import pytest

from dimos.diagnostics.conformance import run_backend


@pytest.mark.parametrize("backend", ["zenoh", "lcm", "ros", "dds", "webrtc", "cloudflare"])
def test_cross_process_identity_fanout_and_reordering(backend):
    result = run_backend(backend)
    if result["status"] == "skipped":
        pytest.skip(result["reason"])
    assert result["deliveries"] == (24 if backend in ("webrtc", "cloudflare") else 18)
    assert result["sequence_order"] == [1, 3, 2, 3, 4]
