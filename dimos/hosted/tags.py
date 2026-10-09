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

"""Host tags: key/value facts a Host advertises, e.g. go2, gpu=RTX 2070, ram_gb=31.

A bare tag is a key with an empty value. A tagger is a module in taggers/ whose
``tags()`` returns a dict, or a set of bare keys. A placement requirement ``key``
needs the key, ``key=value`` needs that exact value.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping

from dimos.hosted.plugins import load
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TagSource = Mapping[str, str] | Iterable[str]
Tagger = Callable[[], "TagSource | None"]


def as_tags(source: TagSource | None) -> dict[str, str]:
    """A dict of tags from a dict, or from ``key`` / ``key=value`` strings."""
    if source is None:
        return {}
    if isinstance(source, Mapping):
        return {str(k): str(v) for k, v in source.items()}
    return dict(item.partition("=")[::2] for item in source)


def format_tags(tags: TagSource) -> str:
    return ",".join(k if not v else f"{k}={v}" for k, v in sorted(as_tags(tags).items()))


def missing(tags: TagSource, required: Iterable[str]) -> list[str]:
    """The requirements ``tags`` does not meet."""
    have = as_tags(tags)
    out = []
    for requirement in required:
        key, sep, value = requirement.partition("=")
        if key not in have or (sep and have[key] != value):
            out.append(requirement)
    return sorted(out)


def taggers() -> list[Tagger]:
    return [module.tags for module in load("dimos.hosted.taggers")]


def auto_tags(fns: Iterable[Tagger] | None = None) -> dict[str, str]:
    """Every tagger's tags merged; a tagger that raises is logged and skipped."""
    tags: dict[str, str] = {}
    for fn in taggers() if fns is None else fns:
        try:
            tags.update(as_tags(fn()))
        except Exception:
            logger.warning("Host tagger failed", tagger=fn.__module__, exc_info=True)
    return tags
