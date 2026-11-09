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


"""The office_555 room, converted once from its MJCF and committed beside this file.

The path and nothing else. There used to be a `UsdModel` here as well, which read all 139
thousand lines of the layer at import of this module and was used by nobody: every blueprint
that wants the room builds its own, because it has frames of its own to add. And a `CLEAR_FLOOR`
spot, which the one blueprint that needed it had written out by hand anyway.
"""

from pathlib import Path

SCENE = Path(__file__).parent / "scene.usda"
