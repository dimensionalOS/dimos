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


"""The Unitree Go2, as measured."""

from __future__ import annotations

from .base import Embodiment

# Baked by the fitted-sim envelope sweep over the governed slow band (stand +
# 0.35 + 0.50 m/s). The sweep and the surface these fold out of are the sim's,
# and live with it (README, "What is not here") -- what is here is its measured
# output, which is what the planner and the judge actually read.
GO2_ENVELOPE: tuple[tuple[float, float, float, float, float], ...] = (
    (0.0, 0.819, 0.416, -0.023, 0.000),
    (26.6, 0.802, 0.436, -0.032, -0.008),
    (45.0, 0.788, 0.472, -0.035, -0.018),
    (63.4, 0.781, 0.500, -0.039, -0.016),
    (90.0, 0.781, 0.507, -0.039, -0.009),
    (116.6, 0.781, 0.497, -0.039, 0.000),
    (135.0, 0.781, 0.463, -0.039, -0.001),
    (153.4, 0.781, 0.422, -0.039, -0.003),
    (180.0, 0.781, 0.416, -0.039, 0.000),
)

# Measured 0.0334 m of extra width per rad/m, residuals <= 12 mm.
GO2_ARC_INFLATE = 0.0334

GO2 = Embodiment(envelope=GO2_ENVELOPE, arc_inflate=GO2_ARC_INFLATE)

# payload adds 8 cm in front: longer body, centre 4 cm further forward.
# No measured envelope of its own: it falls back to the union everywhere.
GO2_PAYLOAD = Embodiment(tag="go2-payload", length=0.963, center_off=0.042, comfort=0.5)
