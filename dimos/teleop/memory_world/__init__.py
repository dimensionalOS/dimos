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

"""First-person VR exploration of a recorded robot memory."""

from typing import Any

__all__ = ["MemoryWorldConfig", "MemoryWorldModule"]


def __getattr__(name: str) -> Any:
    # Lazily: the module pulls in torch, FastAPI and cv2, and a sibling such as
    # recording.py is imported by subprocesses that must start fast.
    if name in __all__:
        from dimos.teleop.memory_world import module

        return getattr(module, name)
    raise AttributeError(name)
