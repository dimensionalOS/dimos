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

"""Camera blueprints generated from policy image-source declarations."""

from __future__ import annotations

from collections.abc import Mapping

from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.profile import ImageSource, PolicyIOProfile

CameraDevice = int | str


def profile_cameras(
    profile: PolicyIOProfile,
    devices: Mapping[str, CameraDevice],
) -> tuple[list[Blueprint], list[tuple[str, str, str]]]:
    """Build cameras and explicit output remappings for a policy profile."""
    image_sources = {
        source.stream: source
        for source in profile.observations.values()
        if isinstance(source, ImageSource)
    }
    missing = sorted(set(image_sources) - set(devices))
    unknown = sorted(set(devices) - set(image_sources))
    if missing or unknown:
        details = []
        if missing:
            details.append(f"missing cameras: {missing}")
        if unknown:
            details.append(f"unknown cameras: {unknown}")
        raise ValueError("; ".join(details))

    blueprints: list[Blueprint] = []
    remappings: list[tuple[str, str, str]] = []
    for stream_name, source in image_sources.items():
        height, width, _channels = source.shape
        instance_name = f"PolicyCamera_{stream_name}"
        blueprints.append(
            CameraModule.blueprint(
                instance_name=instance_name,
                hardware=WebcamConfig(
                    camera_index=devices[stream_name],
                    width=width,
                    height=height,
                    fps=profile.sync.rate_hz,
                    frame_id_prefix=stream_name,
                ),
                frame_id=f"{stream_name}_camera_link",
            )
        )
        remappings.extend(
            [
                (instance_name, "color_image", stream_name),
                (instance_name, "camera_info", f"{stream_name}_camera_info"),
                (instance_name, "tf", f"{stream_name}_tf"),
            ]
        )
    return blueprints, remappings
