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

"""Blueprints: db replay -> SigLIP 2 patch embeddings -> hyperspace voxel map."""

from __future__ import annotations

from typing import Any

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE, DEFAULT_CAPACITY_DEPTH_IMAGE
from dimos.core.coordination.blueprints import TransportSpec, autoconnect
from dimos.core.stream import Transport
from dimos.core.transport import pSHMTransport
from dimos.models.embedding.base import PatchEmbeddings
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.hyperspace.module import HyperspaceModule
from dimos.perception.hyperspace.replay import D455ReplayModule
from dimos.perception.siglip2.module import SigLIP2Module


def heavy_stream_transports() -> dict[tuple[str, type], TransportSpec | Transport[Any]]:
    """Shared-memory transports for the high-bandwidth streams.

    Raw replayed images saturate LCM UDP multicast and drown the tf stream
    (dropped tf = failed world lookups = skipped frames), so the heavy
    streams ride shared memory instead.
    """
    return {
        ("color_image", Image): pSHMTransport(
            "/color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        ),
        ("depth_image", Image): pSHMTransport(
            "/depth_image", default_capacity=DEFAULT_CAPACITY_DEPTH_IMAGE
        ),
        ("patch_embeddings", PatchEmbeddings): pSHMTransport(
            "/patch_embeddings", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
        ),
        # Mid-360 clouds are ~3 MB at 10 Hz — enough to drown the tf stream on LCM.
        ("lidar", PointCloud2): pSHMTransport(
            "/lidar", default_capacity=DEFAULT_CAPACITY_DEPTH_IMAGE
        ),
    }


hyperspace_replay = autoconnect(
    D455ReplayModule.blueprint(),
    SigLIP2Module.blueprint(),
    HyperspaceModule.blueprint(),
).transports(heavy_stream_transports())
