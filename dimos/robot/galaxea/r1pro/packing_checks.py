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

"""Physical non-target invariants shared by collection and ACT evaluation."""

from math import dist
from typing import Any


def validate_other_bottles(
    initial: list[dict[str, Any]], current: list[dict[str, Any]], selected: int
) -> None:
    """Reject wrong-object picks, tipped neighbors, and disturbed tray contents."""
    before = {row["bottle"]: row for row in initial}
    after = {row["bottle"]: row for row in current}
    if before.keys() != after.keys():
        raise RuntimeError("Bottle inventory changed during ACT execution")
    for number, row in after.items():
        if number == selected + 1:
            continue
        previous = before[number]
        disturbed = not row["upright"] or not row["released"]
        if previous["inside_bin"]:
            disturbed |= not row["inside_bin"]
        else:
            disturbed |= dist(previous["bottle_position"], row["bottle_position"]) > 0.015
        if disturbed:
            raise RuntimeError(f"ACT disturbed unrequested bottle_{number}; stopping the pick")
