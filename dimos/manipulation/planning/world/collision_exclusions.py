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

"""Common model self-collision policy for planning backends."""

from __future__ import annotations

from collections.abc import Iterable, Set as AbstractSet
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig


def srdf_collision_exclusions(root: ET.Element) -> list[tuple[str, str]]:
    return [
        (first, second)
        for element in root.iter()
        if element.tag.rsplit("}", 1)[-1] == "disable_collisions"
        if (first := element.get("link1")) is not None
        if (second := element.get("link2")) is not None
    ]


def collision_exclusion_pairs(
    config: RobotModelConfig,
    links: AbstractSet[str],
    adjacent_links: Iterable[tuple[str, str]],
) -> tuple[tuple[str, str], ...]:
    """Merge URDF neighbors, explicit pairs and SRDF pairs as unordered exclusions.

    A pair wholly outside a selected model subtree does not apply. A pair
    with only one surviving link is an error instead of a silent typo.
    """

    def ordered(first: str, second: str) -> tuple[str, str]:
        return (first, second) if first <= second else (second, first)

    pairs = {ordered(first, second) for first, second in adjacent_links}
    configured = list(config.collision_exclusion_pairs)
    if config.srdf_path is not None:
        configured.extend(srdf_collision_exclusions(ET.parse(config.srdf_path).getroot()))
    for first, second in configured:
        if first not in links and second not in links:
            continue
        if first not in links or second not in links:
            raise ValueError(
                f"Model collision exclusion references unknown links: {first} <-> {second}"
            )
        pairs.add(ordered(first, second))
    return tuple(sorted(pairs))
