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

"""MuJoCo/robosuite sensor acquisition, behind the physics adapter boundary."""

from __future__ import annotations

import math
from typing import Any

import mujoco
import numpy as np
from robosuite.utils.camera_utils import get_real_depth_map
from robosuite.utils.observables import (
    Observable,
    create_deterministic_delayer,
    sensor as observable_sensor,
)
from scipy.spatial.transform import Rotation

from dimos.sim2.sensors.camera.spec import Camera
from dimos.sim2.sensors.imu.spec import Imu
from dimos.sim2.sensors.observations import observation_key


def create_observables(
    sim: Any, sensor: Camera | Imu, model_prefix: str, robot_id: str | None = None
) -> dict[str, Observable]:
    """Bind native sensors to upstream Observable; the environment updates them.

    Metadata follows actual payload acquisition, so a held image or IMU value
    retains its acquisition time even when the Observable's rate/delay changes.
    The caller may configure the returned Observables' upstream corruption,
    filter and delay functions directly.
    """
    key = observation_key(robot_id, sensor)
    metadata_key = f"{key}/metadata"
    acquired: dict[str, Any] = {}
    functions: dict[str, Any] = {}
    if isinstance(sensor, Camera):
        camera_name = model_prefix + sensor.model_name
        camera_id = sim.model.camera_name2id(camera_name)
        focal = sensor.height / (2 * math.tan(math.radians(sim.model.cam_fovy[camera_id]) / 2))

        def capture(cache: dict[str, Any], field: str) -> Any:
            # RGB and depth may have different upstream sampling rates. Either
            # can acquire a frame; channels sampled together share one render.
            capture_key = f"{key}/capture"
            entry = cache.get(capture_key)
            if entry is None or entry["time"] != sim.data.time or field in entry["readers"]:
                context = sim._render_context_offscreen
                if context is not None:
                    context.vopt.geomgroup[:] = 1
                    context.scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = sensor.shadows
                rendered = sim.render(
                    camera_name=camera_name,
                    width=sensor.width,
                    height=sensor.height,
                    depth=sensor.depth,
                )
                payload = {}
                if sensor.depth:
                    rgb, raw_depth = rendered
                    payload["depth"] = get_real_depth_map(sim, raw_depth[::-1]).astype(np.float32)
                else:
                    rgb = rendered
                payload["image"] = np.ascontiguousarray(rgb[::-1])
                rotation = sim.data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
                entry = {
                    "time": sim.data.time,
                    "payload": payload,
                    "readers": set(),
                    "metadata": np.array(
                        [
                            1.0,
                            sim.data.time,
                            *sim.data.cam_xpos[camera_id],
                            *Rotation.from_matrix(rotation).as_quat(),
                            focal,
                        ]
                    ),
                }
                cache[capture_key] = entry
            entry["readers"].add(field)
            meta_name = "depth_metadata" if field == "depth" else "metadata"
            acquired[f"{key}/{meta_name}"] = entry["metadata"]
            return entry["payload"][field]

        @observable_sensor(modality="image")  # type: ignore[untyped-decorator]
        def image(cache: dict[str, Any]) -> Any:
            return capture(cache, "image")

        functions["image"] = image
        if sensor.depth:

            @observable_sensor(modality="image")  # type: ignore[untyped-decorator]
            def depth(cache: dict[str, Any]) -> Any:
                return capture(cache, "depth")

            functions["depth"] = depth
        metadata_shape = 10
    else:
        prefix = f"{model_prefix}sensor/{sensor.name}"
        gyro = sim.model.sensor_name2id(f"{prefix}/gyro")
        accel = sim.model.sensor_name2id(f"{prefix}/accel")
        gyro_address = int(sim.model.sensor_adr[gyro])
        accel_address = int(sim.model.sensor_adr[accel])

        @observable_sensor(modality="imu")  # type: ignore[untyped-decorator]
        def imu(cache: dict[str, Any]) -> Any:
            acquired[metadata_key] = np.array([1.0, sim.data.time])
            return np.concatenate(
                (
                    sim.data.sensordata[gyro_address : gyro_address + 3],
                    sim.data.sensordata[accel_address : accel_address + 3],
                )
            )

        functions["imu"] = imu
        metadata_shape = 2

    @observable_sensor(modality="sim2_metadata")  # type: ignore[untyped-decorator]
    def metadata(cache: dict[str, Any]) -> Any:
        return acquired.get(metadata_key, np.full(metadata_shape, np.nan))

    functions["metadata"] = metadata
    if isinstance(sensor, Camera) and sensor.depth:

        @observable_sensor(modality="sim2_metadata")  # type: ignore[untyped-decorator]
        def depth_metadata(cache: dict[str, Any]) -> Any:
            return acquired.get(f"{key}/depth_metadata", np.full(metadata_shape, np.nan))

        functions["depth_metadata"] = depth_metadata
    observables = {
        f"{key}/{name}": Observable(
            name=f"{key}/{name}",
            sensor=function,
            sampling_rate=(1 / float(sim.model.opt.timestep))
            if name.endswith("metadata")
            else sensor.rate_hz,
            delayer=create_deterministic_delayer(
                0 if name.endswith("metadata") else sensor.latency
            ),
        )
        for name, function in functions.items()
    }
    # Observable probes sensor({}) during construction to infer its shape.
    # That validation capture is not an acquired observation.
    acquired.clear()
    return observables
