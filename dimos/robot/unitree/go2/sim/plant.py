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

"""The Go2 actuator, as measured: joint order, limits, and the speed envelope.

The chain itself (PD demand -> clip to :data:`TORQUE_LIMITS` -> envelope ->
first-order lag) lives in :mod:`dimos.simulation.sysid.plant`; these are the
Go2's numbers for it.
"""

from __future__ import annotations

import numpy as np

from dimos.simulation.sysid.plant import TorqueEnvelope

# Unitree SDK LowCmd.motor_cmd / LowState.motor_state order for the 12 leg
# motors (indices 12-19 are unused on a Go2). Lives here rather than in
# backend/model.py so ingest can map wire order without importing mujoco.
UNITREE_MOTOR_NAMES: tuple[str, ...] = (
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
)

# menagerie unitree_go2 actuator order — the joint order everything here uses.
MUJOCO_ACTUATOR_NAMES: tuple[str, ...] = (
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
)

# unitree_vec[UNITREE_TO_MUJOCO] == mujoco_vec
UNITREE_TO_MUJOCO: tuple[int, ...] = tuple(
    UNITREE_MOTOR_NAMES.index(n) for n in MUJOCO_ACTUATOR_NAMES
)

# Per-joint torque limits (hip, thigh, calf) x 4. Slightly tighter than the
# MJCF ctrlrange, so they bind first.
TORQUE_LIMITS = np.array([23.0, 23.0, 35.0] * 4)


# The measured envelope, as three named variants: the droop's EXISTENCE is
# robust, its SIZE only to ±0.15 at 3-6 rad/s, so the uncertainty ships as
# named plants and every result that uses one must name it. "central" is the
# only variant whose replayed delivered/demanded ratio lands in-band in every
# speed bin of both recordings.
_GAIN_SPEEDS = (0.0, 2.0, 4.5, 8.0, 14.0)
_CEIL_SPEEDS = (0.0, 8.0, 10.0, 30.0)
_NO_CEILING = float(TORQUE_LIMITS.max())
TORQUE_ENVELOPES: dict[str, TorqueEnvelope] = {
    e.name: e
    for e in (
        TorqueEnvelope(
            "conservative",
            _GAIN_SPEEDS,
            (1.0, 1.00, 0.70, 0.60, 0.56),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 6.10, 6.10),
        ),
        TorqueEnvelope(
            "central",
            _GAIN_SPEEDS,
            (1.0, 0.95, 0.55, 0.45, 0.41),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 5.50, 5.50),
        ),
        TorqueEnvelope(
            "aggressive",
            _GAIN_SPEEDS,
            (1.0, 0.80, 0.40, 0.30, 0.26),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 4.90, 4.90),
        ),
        # "central" driving with the braking quadrant measured separately; the
        # measured brake/drive ratio applied to central reproduces
        # "conservative" to within 0.01 at every knot.
        TorqueEnvelope(
            "central-signed",
            _GAIN_SPEEDS,
            (1.0, 0.95, 0.55, 0.45, 0.41),
            _CEIL_SPEEDS,
            (_NO_CEILING, _NO_CEILING, 5.50, 5.50),
            brake_gain=(1.0, 1.00, 0.70, 0.60, 0.56),
        ),
    )
}
