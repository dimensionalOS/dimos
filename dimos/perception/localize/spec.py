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

from typing import Protocol

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.localize.types import Localization
from dimos.spec.utils import Spec


class LocalizationSpec(Spec, Protocol):
    """Objects a localize call found, and the scene around them, as world-frame clouds."""

    def get_object(self, instance_id: str) -> Localization | None: ...

    def scene_cloud(
        self,
        exclude_instance_id: str | None = None,
        start: float = -5.0,
        duration: float = 5.0,
    ) -> PointCloud2 | None: ...
