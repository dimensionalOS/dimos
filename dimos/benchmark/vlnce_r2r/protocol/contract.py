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

"""Fixed public motion and frame contract shared by both UDS endpoints."""

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
