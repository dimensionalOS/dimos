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

"""Built-in `PolicyBackend` implementations.

Importing this package registers ``"test"`` and ``"lerobot"`` with
`dimos.manipulation.policy.registry`. The LeRobot backend's heavy
dependencies are NOT imported here; they load lazily on the first
``create_backend("lerobot", ...)`` call.
"""

from __future__ import annotations

from dimos.manipulation.policy.backend import PolicyBackend
from dimos.manipulation.policy.registry import register_backend


def _make_test_backend(**kwargs: object) -> PolicyBackend:
    from dimos.manipulation.policy.backends.test import TestPolicy

    return TestPolicy(**kwargs)  # type: ignore[arg-type]


def _make_lerobot_backend(**kwargs: object) -> PolicyBackend:
    from dimos.manipulation.policy.backends.lerobot import LeRobotBackend

    return LeRobotBackend(**kwargs)  # type: ignore[arg-type]


register_backend("test", _make_test_backend)
register_backend("lerobot", _make_lerobot_backend)


__all__: list[str] = []
