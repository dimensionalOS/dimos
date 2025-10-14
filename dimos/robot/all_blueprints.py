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

from dimos.core.blueprints import ModuleBlueprintSet


# The blueprints are defined as import strings so as not to trigger unnecessary imports.
all_blueprints = {
    "unitree-go2": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:standard",
    "unitree-go2-basic": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:basic",
    "unitree-go2-shm": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:standard_with_shm",
    "demo-osm": "dimos.mapping.osm.demo_osm:demo_osm",
}


def get_blueprint_by_name(name: str) -> ModuleBlueprintSet:
    if name not in all_blueprints:
        raise ValueError(f"Unknown blueprint set name: {name}")
    module_path, attr = all_blueprints[name].split(":")
    module = __import__(module_path, fromlist=[attr])
    return getattr(module, attr)
