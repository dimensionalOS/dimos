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

"""Unify typing compatibility across multiple Python versions."""

from __future__ import annotations

from collections.abc import Sequence
import sys

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

if sys.version_info < (3, 11):

    class ExceptionGroup(Exception):  # type: ignore[no-redef]  # noqa: N818
        """Minimal ExceptionGroup polyfill for Python 3.10."""

        exceptions: tuple[BaseException, ...]

        def __init__(self, message: str, exceptions: Sequence[BaseException]) -> None:
            super().__init__(message)
            self.exceptions = tuple(exceptions)
else:
    import builtins

    ExceptionGroup = builtins.ExceptionGroup  # type: ignore[misc]

__all__ = [
    "ExceptionGroup",
    "TypeVar",
]
