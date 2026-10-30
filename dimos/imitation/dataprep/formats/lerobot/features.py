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

"""Fixed DimOS-to-LeRobot feature naming conventions."""

from collections.abc import Collection, Mapping


def canonical_feature_names(
    observations: Mapping[str, str],
    actions: Collection[str],
) -> dict[str, str]:
    """Map feature roles and dtypes to policy-facing keys without changing streams."""
    if set(observations) & set(actions):
        raise ValueError("observation and action feature names must be distinct")
    numeric_count = sum(dtype not in {"image", "video"} for dtype in observations.values())
    names: dict[str, str] = {}
    for key, dtype in observations.items():
        if dtype in {"image", "video"}:
            name = (
                key
                if key.startswith("observation.images.")
                else f"observation.images.{key.removeprefix('observation.')}"
            )
        elif key.startswith("observation."):
            name = key
        else:
            name = "observation.state" if numeric_count == 1 else f"observation.{key}"
        names[key] = name
    for key in actions:
        if key == "action" or key.startswith("action."):
            names[key] = key
        elif len(actions) == 1:
            names[key] = "action"
        else:
            names[key] = f"action.{key}"
    if len(set(names.values())) != len(names):
        raise ValueError(f"LeRobot feature rename collision: {names}")
    return names
