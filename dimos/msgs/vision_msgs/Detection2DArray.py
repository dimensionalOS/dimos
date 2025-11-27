# Copyright 2025 Dimensional Inc.
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
from dimos_lcm.vision_msgs.Detection2DArray import (  # type: ignore[import-untyped]
    Detection2DArray as LCMDetection2DArray,
)

from dimos.types.timestamped import to_timestamp


class Detection2DArray(LCMDetection2DArray):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection2DArray"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection2DArray.__annotations__

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)
