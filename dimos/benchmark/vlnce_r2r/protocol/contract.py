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

"""Immutable public negotiation contract shared by both UDS endpoints."""

from typing import Mapping  # noqa: UP035

from . import vlnce_public_v1_pb2 as pb

PROTOCOL_REVISION = "vlnce-public.v1"
WORLD_FRAME = "habitat_world"
BASE_FRAME = "habitat_base"
CAMERA_FRAME = "camera_optical"
RGB_ENCODING = "rgb8"
DEPTH_ENCODING = "32FC1_LE"
CONTROL_PERIOD_SECONDS = 0.1
MAX_LINEAR_X = 0.5
MAX_LINEAR_Y = 0.5
MAX_ANGULAR_Z = 1.0
CAPABILITIES = (
    "planar_control",
    "route_submission",
    "rgb",
    "depth",
    "static_occupancy",
)


def expected_handshake(identity: Mapping[str, str]) -> pb.Handshake:
    """Build the exact public contract from its three public attempt identities."""

    return pb.Handshake(
        attempt_id=identity["attempt_id"],
        case_id=identity["case_id"],
        episode_id=identity["episode_id"],
        protocol_revision=identity["protocol_revision"],
        world_frame=WORLD_FRAME,
        base_frame=BASE_FRAME,
        camera_frame=CAMERA_FRAME,
        rgb_encoding=RGB_ENCODING,
        depth_encoding=DEPTH_ENCODING,
        max_linear_x=MAX_LINEAR_X,
        max_linear_y=MAX_LINEAR_Y,
        max_angular_z=MAX_ANGULAR_Z,
        control_period_seconds=CONTROL_PERIOD_SECONDS,
        capabilities=CAPABILITIES,
    )
