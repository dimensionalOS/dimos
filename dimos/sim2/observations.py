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

"""Generated scene messages and a validated, body-only sensor sample.

Entity IDs are model body/frame names, never joint names or native engine IDs.
Every initial publication contains every body. Losing a publication requires no
replay: the next complete sample is sufficient. This is not a physics checkpoint.
"""

from dataclasses import dataclass
import math
import struct

import cbor2
from dimos_lcm.sim_msgs import Observation, Observations
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.sim2.types import Array, Layout, WorldState


@dataclass(frozen=True)
class SceneSample:
    run_id: str
    revision: str
    epoch: int
    tick: int
    sim_time: float
    wall_time: float
    poses: Array  # world-from-body XYZ / XYZW, in the bound description's body order


def observations(run_id: str, layout: Layout, state: WorldState) -> Observations:
    """Project a local physics sample onto the public pose-only contract."""
    items = []
    for name, row in zip(layout.bodies, state.poses, strict=True):
        payload = Pose(*row).lcm_encode()
        items.append(
            Observation(
                entity_id=name,
                kind=Observation.POSE,
                schema="",
                data_length=len(payload),
                data=payload,
            )
        )
    return Observations(
        run_id=run_id,
        model_revision=layout.revision,
        epoch=state.epoch,
        tick=state.tick,
        sim_time=state.sim_time,
        ts=state.wall_time,
        frame_id="world",
        complete=True,
        observations_length=len(items),
        observations=items,
    )


def read_poses(message: Observations, layout: Layout) -> SceneSample:
    """Validate the entire batch before making any reader state visible.

    Sparse publication is deliberately unsupported by this reader. Optional CBOR
    observations never supply the poses required to render or raycast the scene.
    """
    if message.model_revision != layout.revision:
        raise ValueError("observation/model revision mismatch")
    if not message.complete or message.frame_id != "world":
        raise ValueError("expected a complete world-frame sample")
    if (
        not message.run_id
        or message.epoch < 0
        or message.tick < 0
        or not math.isfinite(message.sim_time)
        or message.sim_time < 0
        or not math.isfinite(message.ts)
    ):
        raise ValueError("invalid observation identity or time")
    if message.observations_length != len(message.observations):
        raise ValueError("observation count mismatch")
    poses: dict[str, tuple[float, ...]] = {}
    for item in message.observations:
        if item.data_length != len(item.data):
            raise ValueError("observation payload length mismatch")
        if item.kind == Observation.POSE:
            if item.entity_id in poses or item.schema:
                raise ValueError("duplicate pose or nonstandard pose schema")
            try:
                pose = Pose.lcm_decode(bytes(item.data))
            except (ValueError, struct.error) as error:
                raise ValueError("invalid encoded pose") from error
            row = (*pose.position.to_tuple(), *pose.orientation.to_tuple())
            if not np.isfinite(row).all() or not np.isclose(np.linalg.norm(row[3:]), 1, atol=1e-6):
                raise ValueError("pose must be finite with a unit quaternion")
            poses[item.entity_id] = row
        elif item.kind == Observation.CBOR:
            if not item.schema or item.entity_id not in layout.bodies:
                raise ValueError("CBOR observations require a known entity and schema")
            try:
                cbor2.loads(item.data)
            except cbor2.CBORDecodeError as error:
                raise ValueError("invalid CBOR payload") from error
        # Unknown optional observations are length-delimited and can be skipped.
    if poses.keys() != set(layout.bodies):
        raise ValueError("complete sample must contain exactly the declared bodies")
    array = np.asarray([poses[name] for name in layout.bodies], dtype=np.float64).reshape(-1, 7)
    array.flags.writeable = False
    return SceneSample(
        message.run_id,
        message.model_revision,
        message.epoch,
        message.tick,
        message.sim_time,
        message.ts,
        array,
    )
