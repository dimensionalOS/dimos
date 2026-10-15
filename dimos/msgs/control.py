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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Construction and boundary validation for generated control interface values."""

from collections.abc import Sequence
import math
import time

from dimos_generated.dimos_msgs.msg import ControlValues
import numpy as np
from numpy.typing import ArrayLike


def validate_control_values(frame: ControlValues) -> None:
    """Reject mismatched arrays or non-finite values at a control boundary."""
    if len(frame.interface_names) != len(frame.values):
        raise ValueError(
            "interface_names and values must be the same length; "
            f"got {len(frame.interface_names)} names and {len(frame.values)} values"
        )
    for name, value in zip(frame.interface_names, frame.values, strict=True):
        if not math.isfinite(value):
            raise ValueError(f"non-finite value for interface {name!r}: {value!r}")


def control_values(
    source: str,
    source_ts: float | None = None,
    epoch: int = 0,
    sequence: int = 0,
    interface_names: Sequence[str] = (),
    values: ArrayLike = (),
) -> ControlValues:
    """Copy a sparse frame, stamping only an absent publisher timestamp."""
    frame = ControlValues(
        source=source,
        source_ts=time.time() if source_ts is None else source_ts,
        epoch=epoch,
        sequence=sequence,
        interface_names=list(interface_names),
        values=np.array(values, dtype=np.float64, copy=True),
    )
    validate_control_values(frame)
    return frame


def control_values_dict(frame: ControlValues) -> dict[str, float]:
    """Pair interface names with values; the contract checks uniqueness."""
    validate_control_values(frame)
    return dict(zip(frame.interface_names, frame.values, strict=True))
