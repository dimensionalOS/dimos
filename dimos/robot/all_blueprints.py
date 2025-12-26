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
    "unitree-go2-agentic": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic",
    "demo-osm": "dimos.mapping.osm.demo_osm:demo_osm",
    "demo-remapping": "dimos.robot.unitree_webrtc.demo_remapping:remapping",
    "demo-remapping-transport": "dimos.robot.unitree_webrtc.demo_remapping:remapping_and_transport",
}


all_modules = {
    "astar_planner": "dimos.navigation.global_planner.planner",
    "behavior_tree_navigator": "dimos.navigation.bt_navigator.navigator",
    "connection": "dimos.robot.unitree_webrtc.unitree_go2",
    "depth_module": "dimos.robot.unitree_webrtc.depth_module",
    "detection_2d": "dimos.perception.detection2d.module2D",
    "foxglove_bridge": "dimos.robot.foxglove_bridge",
    "holonomic_local_planner": "dimos.navigation.local_planner.holonomic_local_planner",
    "human_input": "dimos.agents2.cli.human",
    "llm_agent": "dimos.agents2.agent",
    "mapper": "dimos.robot.unitree_webrtc.type.map",
    "navigation_skill": "dimos.agents2.skills.navigation",
    "object_tracking": "dimos.perception.object_tracker",
    "osm_skill": "dimos.agents2.skills.osm.py",
    "spatial_memory": "dimos.perception.spatial_perception",
    "utilization": "dimos.utils.monitoring",
    "wavefront_frontier_explorer": "dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector",
    "websocket_vis": "dimos.web.websocket_vis.websocket_vis_module",
}


def get_blueprint_by_name(name: str) -> ModuleBlueprintSet:
    if name not in all_blueprints:
        raise ValueError(f"Unknown blueprint set name: {name}")
    module_path, attr = all_blueprints[name].split(":")
    module = __import__(module_path, fromlist=[attr])
    return getattr(module, attr)


def get_module_by_name(name: str) -> ModuleBlueprintSet:
    if name not in all_modules:
        raise ValueError(f"Unknown module name: {name}")
    python_module = __import__(all_modules[name], fromlist=[name])
    return getattr(python_module, name)()
