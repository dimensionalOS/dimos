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

"""Live smoke test against a real RTAB-Map install.

Tool-marked so it skips on dev machines without the binary. The intent is to
spin up the module with the (future) :class:`SubprocessRtabRunner` and verify
it emits at least one ``corrected_odometry`` within a short window.

Currently the subprocess runner is a placeholder; this test will skip until
the real bridge lands.
"""

from __future__ import annotations

import shutil

import pytest

from dimos.navigation.nav_stack.modules.rtab_map._rtab_runner import (
    SubprocessRtabRunner,
)

pytestmark = [pytest.mark.tool]


def _rtabmap_on_path() -> bool:
    return shutil.which("rtabmap") is not None


@pytest.mark.skipif(not _rtabmap_on_path(), reason="rtabmap binary not installed")
def test_subprocess_runner_smoke() -> None:
    """When the real bridge lands, this drives a dummy publisher into the
    module and asserts a corrected_odometry message appears within 10s.

    For now it just confirms the placeholder loudly refuses to be used,
    which is the contract we have.
    """
    with pytest.raises(NotImplementedError):
        SubprocessRtabRunner()
