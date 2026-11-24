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


from pathlib import Path

# The layer is text, but every mesh it references is Git LFS, so a checkout made with
# GIT_LFS_SKIP_SMUDGE=1 has pointer files here. Nothing fetches them on the way in: UsdManager
# refuses a pointer by name (require_fetched_assets) so the failure says `git lfs checkout`
# rather than arriving as unreadable geometry inside a physics engine.
SCENE = Path(__file__).parent / "scene.usda"
