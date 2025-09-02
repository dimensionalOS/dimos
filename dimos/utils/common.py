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

from typing import Union


def map_value(
    value: Union[int, float],
    in_min: Union[int, float],
    in_max: Union[int, float],
    out_min: Union[int, float],
    out_max: Union[int, float],
    clamp: bool = True,
) -> float:
    """
    Map a value from one range to another using linear interpolation.

    Maps an input value from the input range [in_min, in_max] to the output range
    [out_min, out_max]. By default, the input value is clamped to the input range
    before mapping.

    Args:
        value: The input value to map
        in_min: Minimum value of the input range
        in_max: Maximum value of the input range
        out_min: Minimum value of the output range
        out_max: Maximum value of the output range
        clamp: Whether to clamp the input value to [in_min, in_max] (default: True)

    Returns:
        The mapped value in the output range

    """
    # Handle edge case where input range has zero width
    if in_max == in_min:
        raise ValueError(f"Input range has zero width: in_min={in_min}, in_max={in_max}")

    # Clamp input value to input range if requested
    if clamp:
        # Handle inverted ranges properly
        range_min = min(in_min, in_max)
        range_max = max(in_min, in_max)
        value = max(range_min, min(value, range_max))

    # Linear interpolation formula
    normalized = (value - in_min) / (in_max - in_min)
    mapped = out_min + normalized * (out_max - out_min)

    return mapped
