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

"""The requests ``PointCloud2.agent_encode()`` accepts."""

from dimos.experimental.agent_encode.pointcloud.fields import (  # noqa: F401
    Band,
    Components,
    DistanceField,
    Grid,
    HeightField,
    Resample,
    Select,
    Threshold,
)
from dimos.experimental.agent_encode.pointcloud.handlers.closest import Closest  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.handlers.depth_view import DepthView  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.handlers.field_outputs import (  # noqa: F401
    Map,
    Sample,
    Window,
)
from dimos.experimental.agent_encode.pointcloud.handlers.occupancy_map import (
    OccupancyMap,  # noqa: F401
)
from dimos.experimental.agent_encode.pointcloud.handlers.overlap import Overlap  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.handlers.overview import Overview  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.handlers.pick import (  # noqa: F401
    Pick,
    SelectionRef,
)
from dimos.experimental.agent_encode.pointcloud.handlers.sweep import Sweep  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.render.overlays import Segment  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.runtime.dispatch import EncodeBudget  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder  # noqa: F401
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere  # noqa: F401
