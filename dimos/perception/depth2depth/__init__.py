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

"""Dense metric depth from a sparse stereo one and the colour frame."""

from dimos.perception.depth2depth.fusion import (
    DEPTH_MODEL_NAME,
    Depth2Depth as Depth2DepthFuser,
    FuseConfig,
    Fusion,
    fit_affine,
    fuse,
)

__all__ = [
    "DEPTH_MODEL_NAME",
    "Depth2DepthFuser",
    "FuseConfig",
    "Fusion",
    "fit_affine",
    "fuse",
]
