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


from dataclasses import dataclass
from typing import Literal

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.sim2.mujoco.names import resolved

_SITE_SIZE = (0.001, 0.001, 0.001)


@dataclass(frozen=True)
class SensorMount:
    kind: Literal["camera", "lidar", "imu"]
    frame_id: str
    name: str
    fovy: float = 60.0


def mount_sensor(spec: mujoco.MjSpec, sensor: SensorMount) -> None:
    frame = sensor.frame_id
    # A lidar or imu named as a site the world already has reads that site, so there is nothing to place.
    if (
        sensor.kind != "camera"
        and resolved([s.name for s in spec.sites], sensor.name) == sensor.name
    ):
        return
    site, body, pos, quat = None, spec.worldbody, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
    if frame != "world":
        site_name = resolved([s.name for s in spec.sites], frame)
        body_name = resolved([b.name for b in spec.bodies], frame)
        site = spec.site(site_name) if site_name else None
        body = spec.body(body_name) if body_name else None
        if site is not None:
            compiled = spec.copy().compile()
            index = compiled.site(site_name).id
            body, pos, quat = site.parent, compiled.site_pos[index], compiled.site_quat[index]
        elif body is None:
            raise ValueError(
                f"no site or body {frame!r} to mount a {sensor.kind} on; this world has "
                f"{sorted(b.name for b in spec.bodies)[:8]}..."
            )
    name = sensor.name
    if sensor.kind == "camera":
        optical = Rotation.from_quat(quat, scalar_first=True) * Rotation.from_euler("x", np.pi)
        body.add_camera(
            name=name, fovy=sensor.fovy, pos=pos, quat=optical.as_quat(scalar_first=True)
        )
    elif resolved([s.name for s in spec.sites], name) != name:
        body.add_site(name=name, size=_SITE_SIZE, rgba=(0, 0, 0, 0), pos=pos, quat=quat)
