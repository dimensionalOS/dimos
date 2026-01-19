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

"""Mock backend for testing manipulator drivers without hardware.

Usage:
    >>> from dimos.hardware.manipulators.xarm import XArm
    >>> from dimos.hardware.manipulators.mock import MockBackend
    >>> arm = XArm(backend=MockBackend())
    >>> arm.start()  # No hardware needed!
    >>> arm.move_joint([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    >>> assert arm.backend.read_joint_positions() == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
"""

from dimos.hardware.manipulators.mock.backend import MockBackend

__all__ = ["MockBackend"]
