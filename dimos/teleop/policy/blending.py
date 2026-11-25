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

"""Pure, component-wise blending of world-frame TCP velocities."""

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.policy.takeover import Weights6
from dimos.utils.transform_utils import twist_to_numpy


def normalize_weights(weights: float | Weights6) -> NDArray[np.float64]:
    values = np.asarray(weights, dtype=np.float64)
    if (
        values.shape not in ((), (6,))
        or not np.isfinite(values).all()
        or not np.all((values >= 0) & (values <= 1))
    ):
        raise ValueError("expected one or six finite policy weights between zero and one")
    return np.broadcast_to(values, (6,))


def blend_twist(
    teleop: TwistStamped | None, policy: TwistStamped | None, weights: float | Weights6
) -> TwistStamped:
    """Blend six velocity axes; entirely unused sources may be absent or invalid.

    The output keeps the active sources' common frame and latest timestamp.
    This standalone illustration is not used by the coordinator's joint blender.
    """
    alpha = normalize_weights(weights)
    result = np.zeros(6)
    frame: str | None = None
    stamp = -np.inf
    for name, command, gain in (("teleop", teleop, 1 - alpha), ("policy", policy, alpha)):
        if not np.any(gain):
            continue
        if command is None:
            raise ValueError(f"missing {name} twist")
        if frame is not None and command.frame_id != frame:
            raise ValueError("twists must use the same frame")
        values = twist_to_numpy(command)
        if not np.isfinite(values).all() or not np.isfinite(command.ts):
            raise ValueError(f"nonfinite {name} twist")
        frame = command.frame_id
        stamp = max(stamp, command.ts)
        result += gain * values
    if not np.isfinite(result).all():
        raise ValueError("nonfinite blended twist")
    return TwistStamped(ts=stamp, frame_id=frame, linear=result[:3], angular=result[3:])
