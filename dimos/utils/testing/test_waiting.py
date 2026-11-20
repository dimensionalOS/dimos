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

import pytest

from dimos.utils.testing import waiting


def test_wait_until_observes_condition_after_late_wakeup(mocker):
    clock = mocker.patch.object(waiting, "time")
    now = 0.0

    def wake_late(_interval: float) -> None:
        nonlocal now
        now = 3.0

    clock.monotonic.side_effect = lambda: now
    clock.sleep.side_effect = wake_late

    # The condition becomes true at t=1, but the next poll only runs at t=3.
    waiting.wait_until(lambda: now >= 1.0, timeout=2.0)
    clock.sleep.assert_called_once()


def test_wait_until_times_out_when_condition_stays_false():
    with pytest.raises(TimeoutError, match="missing feedback"):
        waiting.wait_until(lambda: False, timeout=0, message="missing feedback")
