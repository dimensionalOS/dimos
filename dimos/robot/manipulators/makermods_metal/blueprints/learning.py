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

"""Single Metal, one overhead camera, and the shared learning flow."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.makermods_metal.learning import build_metal_learning

makermods_metal_learning_collection = autoconnect(build_metal_learning())
dreamlabs_makermods_metal_eef = autoconnect(build_metal_learning(collection=False))
