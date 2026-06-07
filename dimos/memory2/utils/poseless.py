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

from __future__ import annotations

from typing import TYPE_CHECKING, TypeVar

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation, PoseTuple

logger = setup_logger()

# Warn once after this many leading pose-less observations.
_POSE_WARN_AFTER = 10

T = TypeVar("T")


def posed_only(
    upstream: Iterator[Observation[T]], label: str
) -> Iterator[tuple[Observation[T], PoseTuple]]:
    """Yield each posed observation paired with its pose tuple, dropping pose-less ones.

    Warns once if many leading observations arrive without a pose before anything
    has been yielded, which usually points at a misconfigured stream name or
    alignment tolerance.
    """
    skipped = 0
    yielded = 0
    warned = False
    for obs in upstream:
        if obs.pose_tuple is None:
            skipped += 1
            if not warned and yielded == 0 and skipped >= _POSE_WARN_AFTER:
                logger.warning(
                    "%s: %d observations had no pose; check the lidar/odom "
                    "stream names and alignment tolerance",
                    label,
                    skipped,
                )
                warned = True
            logger.debug("%s: obs %s has no pose; skipping", label, obs.id)
            continue
        yielded += 1
        yield obs, obs.pose_tuple
