# Copyright 2025-2026 Dimensional Inc.
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

"""ControlCoordinator carrying the inputs the path-following tasks consume."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.std_msgs.Float32 import Float32


class PathFollowingCoordinator(ControlCoordinator):
    """Adds the ``path`` and ``speed`` ports the follower task cards bind to.

    The base coordinator ships only the generic command inputs; a deployment
    declares whatever else its tasks consume (see ``TASK_CONSUMES``).
    """

    path: In[Path]
    speed: In[Float32]
