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

from dimos.core.blueprints import ModuleBlueprintSet

# The blueprints are defined as import strings so as not to trigger unnecessary imports.
all_blueprints = {
    "unitree-go2": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:nav",
    "unitree-go2-basic": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:basic",
    "unitree-go2-nav": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:nav",
    "unitree-go2-ros": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:ros",
    "unitree-go2-detection": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:detection",
    "unitree-go2-spatial": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:spatial",
    "unitree-go2-temporal-memory": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:temporal_memory",
    "unitree-go2-agentic": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic",
    "unitree-go2-agentic-mcp": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic_mcp",
    "unitree-go2-agentic-ollama": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic_ollama",
    "unitree-go2-agentic-huggingface": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:agentic_huggingface",
    "unitree-go2-vlm-stream-test": "dimos.robot.unitree_webrtc.unitree_go2_blueprints:vlm_stream_test",
    "unitree-g1": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:standard",
    "unitree-g1-sim": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:standard_sim",
    "unitree-g1-basic": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:basic_ros",
    "unitree-g1-basic-sim": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:basic_sim",
    "unitree-g1-shm": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:standard_with_shm",
    "unitree-g1-agentic": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:agentic",
    "unitree-g1-agentic-sim": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:agentic_sim",
    "unitree-g1-joystick": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:with_joystick",
    "unitree-g1-full": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:full_featured",
    "unitree-g1-detection": "dimos.robot.unitree_webrtc.unitree_g1_blueprints:detection",
    # Control orchestrator blueprints
    "orchestrator-mock": "dimos.control.blueprints:orchestrator_mock",
    "orchestrator-xarm7": "dimos.control.blueprints:orchestrator_xarm7",
    "orchestrator-xarm6": "dimos.control.blueprints:orchestrator_xarm6",
    "orchestrator-piper": "dimos.control.blueprints:orchestrator_piper",
    "orchestrator-dual-mock": "dimos.control.blueprints:orchestrator_dual_mock",
    "orchestrator-dual-xarm": "dimos.control.blueprints:orchestrator_dual_xarm",
    "orchestrator-piper-xarm": "dimos.control.blueprints:orchestrator_piper_xarm",
    # Demo blueprints
    "demo-camera": "dimos.hardware.sensors.camera.module:demo_camera",
    "demo-osm": "dimos.mapping.osm.demo_osm:demo_osm",
    "demo-skill": "dimos.agents.skills.demo_skill:demo_skill",
    "demo-gps-nav": "dimos.agents.skills.demo_gps_nav:demo_gps_nav_skill",
    "demo-google-maps-skill": "dimos.agents.skills.demo_google_maps_skill:demo_google_maps_skill",
    "demo-object-scene-registration": "dimos.perception.demo_object_scene_registration:demo_object_scene_registration",
    "demo-error-on-name-conflicts": "dimos.robot.unitree_webrtc.demo_error_on_name_conflicts:blueprint",
}


all_modules = {
    "replanning_a_star_planner": "dimos.navigation.replanning_a_star.module",
    "camera_module": "dimos.hardware.camera.module",
    "depth_module": "dimos.robot.unitree_webrtc.depth_module",
    "detection_2d": "dimos.perception.detection2d.module2D",
    "foxglove_bridge": "dimos.robot.foxglove_bridge",
    "g1_connection": "dimos.robot.unitree.connection.g1",
    "g1_joystick": "dimos.robot.unitree_webrtc.g1_joystick_module",
    "g1_skills": "dimos.robot.unitree_webrtc.unitree_g1_skill_container",
    "google_maps_skill": "dimos.agents.skills.google_maps_skill_container",
    "gps_nav_skill": "dimos.agents.skills.gps_nav_skill",
    "human_input": "dimos.agents.cli.human",
    "keyboard_teleop": "dimos.robot.unitree_webrtc.keyboard_teleop",
    "llm_agent": "dimos.agents.agent",
    "mapper": "dimos.robot.unitree_webrtc.type.map",
    "navigation_skill": "dimos.agents.skills.navigation",
    "object_tracking": "dimos.perception.object_tracker",
    "osm_skill": "dimos.agents.skills.osm",
    "ros_nav": "dimos.navigation.rosnav",
    "spatial_memory": "dimos.perception.spatial_perception",
    "speak_skill": "dimos.agents.skills.speak_skill",
    "unitree_skills": "dimos.robot.unitree_webrtc.unitree_skill_container",
    "utilization": "dimos.utils.monitoring",
    "wavefront_frontier_explorer": "dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector",
    "websocket_vis": "dimos.web.websocket_vis.websocket_vis_module",
    "web_input": "dimos.agents.cli.web",
    # Control orchestrator module
    "control_orchestrator": "dimos.control.orchestrator",
}


def get_blueprint_by_name(name: str) -> ModuleBlueprintSet:
    if name not in all_blueprints:
        raise ValueError(f"Unknown blueprint set name: {name}")
    module_path, attr = all_blueprints[name].split(":")
    module = __import__(module_path, fromlist=[attr])
    return getattr(module, attr)  # type: ignore[no-any-return]


def get_module_by_name(name: str) -> ModuleBlueprintSet:
    if name not in all_modules:
        raise ValueError(f"Unknown module name: {name}")
    python_module = __import__(all_modules[name], fromlist=[name])
    return getattr(python_module, name)()  # type: ignore[no-any-return]
