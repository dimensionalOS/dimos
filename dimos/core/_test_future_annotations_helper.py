# Copyright 2025 Dimensional Inc.
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

"""
Helper module for testing blueprint handling with PEP 563 (future annotations).

This file exists because `from __future__ import annotations` affects the entire file.
"""

from __future__ import annotations

from dimos.core.module import Module
from dimos.core.stream import In, Out  # noqa


class FutureData:
    pass


class FutureModuleOut(Module):
    data: Out[FutureData] = None  # type: ignore[assignment]


class FutureModuleIn(Module):
    data: In[FutureData] = None  # type: ignore[assignment]
