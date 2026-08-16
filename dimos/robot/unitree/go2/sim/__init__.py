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

"""The measured Go2 plant: physics with provenance, and its uncertainty.

Every constant in this package is either a direct measurement with provenance
or a fitted value with a stated range. The published artifact is here (presets
and DR spreads, importable without mujoco or mcap); the pipeline that PRODUCED
it lives in :mod:`~dimos.robot.unitree.go2.sim.sysid` and needs the ``mcap``
extra. ``sysid`` imports the plant; the plant never imports ``sysid``.
"""

from dimos.robot.unitree.go2.sim.ranges import (
    ACCEL,
    BUILTIN_PRESETS,
    DEFAULT_PRESET,
    DR_ACCEL,
    DR_FLOOR,
    DR_PINNED,
    KNOBS,
    MEASURED,
    STOCK,
    Knob,
    Preset,
    load_preset,
)

__all__ = [
    "ACCEL",
    "BUILTIN_PRESETS",
    "DEFAULT_PRESET",
    "DR_ACCEL",
    "DR_FLOOR",
    "DR_PINNED",
    "KNOBS",
    "MEASURED",
    "STOCK",
    "Knob",
    "Preset",
    "load_preset",
]
