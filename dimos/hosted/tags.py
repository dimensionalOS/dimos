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

"""Placement tags a Host detects about itself. A tagger is a module in taggers/ with ``tags()``."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from dimos.hosted.plugins import load
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

Tagger = Callable[[], "set[str] | None"]


def taggers() -> list[Tagger]:
    return [module.tags for module in load("dimos.hosted.taggers")]


def auto_tags(fns: Iterable[Tagger] | None = None) -> set[str]:
    """Union of every tagger's tags; a tagger that raises is logged and skipped."""
    tags: set[str] = set()
    for fn in taggers() if fns is None else fns:
        try:
            tags |= set(fn() or ())
        except Exception:
            logger.warning("Host tagger failed", tagger=fn.__module__, exc_info=True)
    return tags
