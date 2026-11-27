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

"""Shared helpers for replay tests."""

from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from reactivex import merge

if TYPE_CHECKING:
    from unittest.mock import MagicMock

# How long a test waits before declaring it hung. A healthy wait returns as soon as its
# event fires, so this only decides how fast a broken test fails: far above any healthy
# wait, far below pytest's 600 s --timeout, whose thread method kills the whole worker
# with a stack dump instead of the test's own assertion message.
HUNG_TEST_TIMEOUT = 30.0


def playback_finished(add: MagicMock) -> threading.Event:
    """Set once every stream that a ``mocker.spy(Playback, "add")`` saw has completed.

    Call it after the module's ``start()`` added its streams, before it plays.
    """
    finished = threading.Event()
    merge(*add.spy_return_list).subscribe(on_completed=finished.set)
    return finished
